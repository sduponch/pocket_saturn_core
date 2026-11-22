// (Copied from MIRLO64 rtl/analogizer/an_jvs.sv, with the `div` input.) A JVS host on the SNAC port: one RS485 node at 115200 8N1, a 512-byte buffer and the registers the runtime drives it with
// (JVS: tools/mirlo_regs.py). The host frames, escapes and checksums what it sends, and unescapes and checks what it receives;
// it can resend the same frame every PERIOD ms by itself, so that the runtime only ever reads the last good answer and never
// waits for the bus.
//
//   ID    00 R   0x4A560001
//   CTRL  04 RW  [0] ON the pins are the JVS's (the adapter must own the port: `own`), [1] POLL resend the last XFER's frame every
//                PERIOD ms, [15:8] PERIOD: ms from a frame's start to the next poll, and the time an answer is waited for (reset 20, 1..255)
//   STAT  08 R   [0] SENSE the SENSE pin's level, [15:8] GOOD answers with a good checksum (wraps), [23:16] ERRORS transactions without one
//   XFER  0C     write: send {[6:0] N} once, now (and the frame POLL resends from then on): N bytes of the buffer from 0, the node then the
//                data; the host adds the sync byte, the length (N: the data and the sum) and the sum.
//                read: [31] BUSY (from the write until its answer or PERIOD ms), [30] TIMEOUT (the last transaction: no frame came),
//                [29] BAD (it came broken: checksum, a byte's stop bit, more than 128 bytes, or not over in PERIOD ms),
//                [7:0] the bytes of the last good answer (node .. sum)
//   PTR   10 RW  [7:0] the buffer's pointer
//   DATA  14 RW  the byte at PTR; each access moves PTR on by one
//
// The buffer as the runtime sees it: 0..127 the frame to send (node, data), 128..255 the last good answer as it came, unescaped
// (node 00, length, status, a report and its data per command, sum). Answers are taken into the other half of a double buffer
// and the halves swap when the checksum is good, so a broken answer never replaces a good one; an answer swapped in while the
// runtime reads it shows up as GOOD moving on (read GOOD, the bytes, GOOD again).
//
// The bus. DIR (the RS485 driver) comes on 10 us before the first start bit and goes off 30 us after the last stop bit (what the
// Sega hosts were measured doing); the receiver listens only while DIR is off. After an answer (or PERIOD ms) the bus is left alone
// 30 us more before the next frame: the answer is taken in the middle of its last stop bit, and the board's driver trails it. A raw E0 always starts a frame; D0 escapes the byte
// that follows (sent as itself minus one). Writing the buffer's frame while POLL resends it can send a torn frame: clear POLL first.
`default_nettype none
module an_jvs #(
    parameter int CLK_HZ = 62_500_000,
    parameter int BAUD   = 115_200
) (
    input  wire         clk, rst,
    // the registers (mirlo_regs.sv): the strobes already decoded for this block
    input  wire         sel_wr, sel_rd,
    input  wire  [3:0]  a,                  // r_addr[5:2]
    input  wire  [31:0] wd,
    output logic [31:0] rd,
    input  wire         own,                // the adapter owns the cartridge port (INTERACT2 bit 31)
    // the pins
    output wire         en,                 // the JVS has the pins (bank 0 an input)
    output wire         tx,                 // the RS485 transceiver's DI (idles high)
    output wire         dir,                // its DE / not-RE: 1 drives the bus
    input  wire         rx,                 // its RO (asynchronous)
    input  wire         sense,              // the SENSE line (asynchronous)
    input  wire  [10:0] div                 // clocks of a bit when the clock's rate changes at run time; 0: DIV (from CLK_HZ)
);

localparam int DIV  = (CLK_HZ + BAUD / 2) / BAUD;              // clocks of a bit
wire  [10:0] dv   = div != 11'd0 ? div : 11'(DIV);            // (the Saturn: clk_sys follows the video mode)
localparam int LEAD = CLK_HZ / 100_000;                         // 10 us
localparam int TAIL = CLK_HZ / 100_000 * 3;                     // 30 us
localparam int MS   = CLK_HZ / 1_000;

typedef enum logic [2:0] { S_IDLE, S_LEAD, S_BYTE, S_TXB, S_TAIL, S_RX, S_GAP } st_t;
st_t         st;

// ---------------------------------------------------------------- registers
logic        on, poll;
logic [7:0]  period;
logic [7:0]  ptr;
logic        pend;                          // a transaction asked for by XFER and not over yet
logic [6:0]  x_n;                           // the frame XFER gave: what is sent (and resent)
logic        x_to, x_bad;
logic [7:0]  r_cnt;                         // the last good answer's bytes
logic [7:0]  n_good;                       // (the Saturn: no ERRORS count, STAT[23:16] reads 0)
logic        cur;                           // the buffer half the last good answer is in
logic        e;
assign e  = on && own;
assign en = e;

// ---------------------------------------------------------------- the pins in
logic        rx1, rx_s, se1, se_s;
always_ff @(posedge clk) begin rx1 <= rx | ~own; rx_s <= rx1; se1 <= sense; se_s <= se1; end

// ---------------------------------------------------------------- time: PM is the ms since the last frame's start (ms_c restarts with it)
logic [15:0] ms_c = '0;
logic [7:0]  pm;
wire         ms_end = ms_c == 16'(MS - 1);

// ---------------------------------------------------------------- the sequencer: the frame out, then the answer in
logic [6:0]  n;                             // the frame being sent: bytes from the buffer
logic [7:0]  k;                             // out: the frame's logical byte (0 node, 1 length, 2..n data, n+1 sum); in: the answer's
logic [7:0]  sum;
logic        esc;                           // out: the second byte of an escape is due (esc_v); in: the last byte was D0
logic [7:0]  esc_v;
logic        first;                         // out: the sync byte is due
logic [10:0] cnt;                           // clocks to the next bit (and the lead and tail times)
logic [3:0]  nb;                            // bits so far
logic [9:0]  sh;                            // out: the byte's 10 bits, LSB first; in: the byte's 8
logic        r_act;                         // in: a byte is being received
logic        rx_stb;                        // in: a byte came (sh[7:0]), one clock
logic        inf;                           // in: a frame's sync byte came
logic        ferr;                          // in: a byte with a bad stop bit came
logic [7:0]  len;

// the buffer: port A the runtime's, port B the sequencer's
(* ramstyle = "M10K, no_rw_check" *) reg [7:0] mem [0:511];
reg [7:0] qa, qb;
integer ii;
initial for (ii = 0; ii < 512; ii = ii + 1) mem[ii] = 8'd0;
wire [8:0]  a_addr = ptr[7] ? {1'b1, cur, ptr[6:0]} : {2'b00, ptr[6:0]};
wire        data_wr = sel_wr && a == 4'd5 && !ptr[7];
always @(posedge clk) begin
    if (data_wr) mem[a_addr] <= wd[7:0];
    qa <= mem[a_addr];
end
wire [7:0]  rv = esc ? sh[7:0] + 8'd1 : sh[7:0];             // the answer's byte unescaped
wire        rx_data = st == S_RX && rx_stb && inf && sh[7:0] != 8'hE0 && sh[7:0] != 8'hD0;
wire [8:0]  b_addr = st == S_RX ? {1'b1, ~cur, k[6:0]} : {2'b00, k == 8'd0 ? 7'd0 : k[6:0] - 7'd1};
always @(posedge clk) begin
    if (rx_data) mem[b_addr] <= rv;
    qb <= mem[b_addr];
end

// out: the frame's logical byte k (qb: the buffer's byte at k, read since k last moved)
wire [7:0]  tv = k == 8'd1 ? {1'b0, n} : k == {1'b0, n} + 8'd1 ? sum : qb;

wire        x_wr = sel_wr && a == 4'd3;
task automatic finish(input logic good);
    st <= S_GAP; cnt <= 11'(TAIL - 1); pend <= 1'b0; r_act <= 1'b0;
    x_to <= !good && !inf && !ferr; x_bad <= !good && (inf || ferr);
    if (good) begin cur <= ~cur; r_cnt <= k + 8'd1; n_good <= n_good + 8'd1; end
endtask

always_ff @(posedge clk) begin
    rx_stb <= 1'b0;
    ms_c <= ms_end ? 16'd0 : ms_c + 16'd1;
    if (ms_end && pm != 8'hFF) pm <= pm + 8'd1;
    if (rst) begin
        st <= S_IDLE; on <= 1'b0; poll <= 1'b0; period <= 8'd20; ptr <= '0; pend <= 1'b0; x_n <= '0; x_to <= 1'b0; x_bad <= 1'b0;
        r_cnt <= '0; n_good <= '0; cur <= 1'b0; r_act <= 1'b0; pm <= 8'hFF;
    end else begin
        // the registers
        if (sel_wr && a == 4'd1) begin on <= wd[0]; poll <= wd[1]; period <= wd[15:8]; end
        if (sel_wr && a == 4'd4) ptr <= wd[7:0];
        else if ((sel_wr || sel_rd) && a == 4'd5) ptr <= ptr + 8'd1;
        if (x_wr) begin pend <= 1'b1; x_n <= wd[6:0]; end

        case (st)
        S_IDLE: begin
            if ((pend || (poll && pm >= period)) && x_n != 7'd0 && e && !x_wr) begin
                n <= x_n; k <= '0; sum <= '0; esc <= 1'b0; first <= 1'b1; cnt <= 11'(LEAD - 1); st <= S_LEAD;
                pm <= '0; ms_c <= '0;
            end else if (pend && !x_wr) begin pend <= 1'b0; x_to <= 1'b1; x_bad <= 1'b0; end      // (no frame, or the pins not the JVS's)
        end
        S_LEAD: if (cnt == '0) st <= S_BYTE; else cnt <= cnt - 11'd1;
        S_BYTE: begin
            cnt <= dv - 11'd1; nb <= '0; st <= S_TXB;
            if (first) begin first <= 1'b0; sh <= {1'b1, 8'hE0, 1'b0}; end
            else if (esc) begin esc <= 1'b0; sh <= {1'b1, esc_v, 1'b0}; end
            else if (k == {1'b0, n} + 8'd2) begin cnt <= 11'(TAIL - 1); st <= S_TAIL; end
            else begin
                k <= k + 8'd1; sum <= sum + tv;
                if (tv == 8'hE0 || tv == 8'hD0) begin sh <= {1'b1, 8'hD0, 1'b0}; esc <= 1'b1; esc_v <= tv - 8'd1; end
                else sh <= {1'b1, tv, 1'b0};
            end
        end
        S_TXB: if (cnt == '0) begin
            cnt <= dv - 11'd1; sh <= {1'b1, sh[9:1]}; nb <= nb + 4'd1;
            if (nb == 4'd9) st <= S_BYTE;
        end else cnt <= cnt - 11'd1;
        S_TAIL: if (cnt == '0) begin st <= S_RX; k <= '0; inf <= 1'b0; ferr <= 1'b0; esc <= 1'b0; r_act <= 1'b0; end
                else cnt <= cnt - 11'd1;
        S_RX: begin
            // a byte: the start bit's middle, then a bit every DIV clocks, the stop bit last
            if (r_act) begin
                if (cnt != '0) cnt <= cnt - 11'd1;
                else begin
                    cnt <= dv - 11'd1; nb <= nb + 4'd1;
                    if (nb == 4'd0) begin if (rx_s) r_act <= 1'b0; end                     // (a glitch, not a start bit)
                    else if (nb != 4'd9) sh[7:0] <= {rx_s, sh[7:1]};
                    else begin
                        r_act <= 1'b0;
                        if (rx_s) rx_stb <= 1'b1;
                        else begin ferr <= 1'b1; inf <= 1'b0; end                           // (the frame is lost: a new sync byte is waited for)
                    end
                end
            end else if (!rx_s) begin r_act <= 1'b1; cnt <= {1'b0, dv[10:1]} - 11'd1; nb <= '0; end
            // the byte into the frame
            if (rx_stb) begin
                if (sh[7:0] == 8'hE0) begin inf <= 1'b1; k <= '0; sum <= '0; esc <= 1'b0; end
                else if (inf && sh[7:0] == 8'hD0) esc <= 1'b1;
                else if (inf) begin
                    esc <= 1'b0;
                    if (k == 8'd1) len <= rv;
                    if (k >= 8'd2 && k == {1'b0, len} + 8'd1) finish(rv == sum);     // the sum: the answer is over
                    else if (k == 8'd127) begin inf <= 1'b0; finish(1'b0); end      // (longer than the buffer's half)
                    else begin k <= k + 8'd1; sum <= sum + rv; end
                end
            end else if (pm >= period) finish(1'b0);                                // (nothing, or not over, in PERIOD ms)
        end
        S_GAP: if (cnt == '0) st <= S_IDLE; else cnt <= cnt - 11'd1;      // (the board's last stop bit and its DE's tail)
        default: st <= S_IDLE;
        endcase
        if (!e && st != S_IDLE) begin st <= S_IDLE; pend <= 1'b0; x_to <= 1'b1; x_bad <= 1'b0; end
    end
end

always_comb begin
    rd = 32'd0;
    case (a)
    4'd0: rd = 32'h4A56_0001;
    4'd1: rd = {16'd0, period, 6'd0, poll, on};
    4'd2: rd = {16'd0, n_good, 7'd0, se_s};
    4'd3: rd = {pend, x_to, x_bad, 21'd0, r_cnt};
    4'd4: rd = {24'd0, ptr};
    4'd5: rd = {24'd0, qa};
    default: ;
    endcase
end

// ---------------------------------------------------------------- the pins out
logic        dir_r, tx_r;
always_ff @(posedge clk) begin
    dir_r <= e && (st == S_LEAD || st == S_BYTE || st == S_TXB || st == S_TAIL);
    tx_r  <= !(e && st == S_TXB) || sh[0];
end
assign dir = dir_r;
assign tx  = tx_r;
endmodule
`default_nettype wire
