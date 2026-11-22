// (Copied from MIRLO64 sim/analogizer/jvs/jvs_dev.sv.)
// A JVS I/O board for the bench (sim/analogizer/jvs): one node on the RS485 bus at its own bit time (`div` clocks of 16 ns), and
// the ways a real one misbehaves. It takes the host's frames (unescaped, checksum checked), answers those for its address (or the
// broadcast address while it has none) after `delay_us`, and keeps SENSE high until it is addressed.
//   commands  F0 D9 reset (no answer), F1 a set the address, 10 identity, 11 12 13 command / JVS / comms revisions, 14 features,
//             20 p b switches (sw: the system byte, then the players'), 22 n analog channels (an: 16 bits each, MSB first),
//             32 n d.. general outputs (kept in last_out); another: report 02 and the rest of the frame is not looked at
//   mode      [0] never answers   [1] the answer's sum is wrong   [2] the answer stops halfway   [3] noise before the answer's sync
//             [4] a byte halfway through the answer with a bad stop bit   [5] an identity longer than the host's 128 bytes
`default_nettype none
module jvs_dev (
    input  wire         clk, rst,
    input  wire         bus,                    // the bus level (the board's receiver)
    output reg          de,                     // the board drives the bus
    output reg          txd,
    output reg          sense,
    input  wire  [15:0] div,
    input  wire  [15:0] delay_us,
    input  wire  [7:0]  mode,
    input  wire  [63:0] sw,                     // byte 0 the system byte, then 7 bytes of switches
    input  wire  [127:0] an,                    // channel 0 in [15:0]
    output reg   [31:0] n_frames,               // frames taken with a good sum
    output reg   [31:0] n_bad,                  // frames with a bad sum
    output reg   [31:0] n_ans,                  // answers sent
    output reg   [7:0]  addr,
    output reg   [7:0]  last_out [0:7],
    output reg   [7:0]  n_out
);
localparam string ID  = "MIRLO64;JVS TEST BOARD;837-00000 ;Ver9.99;26/11";
localparam string IDL = "MIRLO64;JVS TEST BOARD WITH AN IDENTITY FAR LONGER THAN THE HOST KEEPS, A HUNDRED AND FIFTY BYTES OR SO, TO SEE IT DROPPED;Ver9.99";

reg  [7:0]  fb [0:255];                         // the frame being received: node, length, data, sum
reg  [7:0]  ob [0:255];                         // the answer, unescaped
reg  [7:0]  wb [0:511];                         // the answer as sent
reg  [8:0]  sb [0:511];                         // (the stop bit of each)
integer     fi, wn, wi, o, p, i, c, nn, bb, nb_r, cnt_r, cnt_t, nb_t, t_go;
reg         inf, esc, r_act, busy_t, ok;
reg  [7:0]  sh, v, s;
reg  [9:0]  tsh;
reg         b1, b2;
string      ids;

initial begin
    de = 0; txd = 1; sense = 1; n_frames = 0; n_bad = 0; n_ans = 0; addr = 0; n_out = 0;
    inf = 0; esc = 0; r_act = 0; busy_t = 0; fi = 0; b1 = 1; b2 = 1; wn = 0; t_go = 0;
end

task automatic put(input [7:0] x); ob[o] = x; o = o + 1; endtask

// the frame in fb[0 .. fi-1] is complete
task automatic take;
    s = 0;
    for (i = 0; i < fi - 1; i = i + 1) s = s + fb[i];
    ok = s == fb[fi - 1];
    if (!ok) n_bad = n_bad + 1; else n_frames = n_frames + 1;
    o = 3;
    if (!ok) begin
        if (fb[0] == addr && addr != 0) begin ob[2] = 8'h03; build(); end
    end else if ((fb[0] == addr && addr != 0) || fb[0] == 8'hFF) begin
        ob[2] = 8'h01;
        p = 2;
        while (p < fi - 1) begin
            c = fb[p];
            if (c == 8'hF0) p = p + 2;                                              // (D9 checked by nobody: a board resets)
            else if (c == 8'hF1) begin
                if (fb[0] == 8'hFF && addr == 0) begin addr = fb[p + 1]; put(8'h01); end
                p = p + 2;
            end else if (c == 8'h10) begin
                put(8'h01); ids = mode[5] ? IDL : ID;
                for (i = 0; i < ids.len(); i = i + 1) put(ids[i]);
                put(8'h00); p = p + 1;
            end else if (c == 8'h11) begin put(8'h01); put(8'h13); p = p + 1; end
            else if (c == 8'h12) begin put(8'h01); put(8'h30); p = p + 1; end
            else if (c == 8'h13) begin put(8'h01); put(8'h10); p = p + 1; end
            else if (c == 8'h14) begin
                put(8'h01);
                put(8'h01); put(8'h02); put(8'h0C); put(8'h00);                     // 2 players, 12 switches
                put(8'h03); put(8'h08); put(8'h10); put(8'h00);                     // 8 analog channels, 16 bits
                put(8'h12); put(8'h08); put(8'h00); put(8'h00);                     // 8 general outputs
                put(8'h00); p = p + 1;
            end else if (c == 8'h20) begin
                put(8'h01); put(sw[7:0]);
                nn = fb[p + 1] * fb[p + 2];
                for (i = 0; i < nn; i = i + 1) put(i < 7 ? sw[8 * (i + 1) +: 8] : 8'h00);
                p = p + 3;
            end else if (c == 8'h22) begin
                put(8'h01);
                for (i = 0; i < fb[p + 1]; i = i + 1) begin put(an[16 * i + 8 +: 8]); put(an[16 * i +: 8]); end
                p = p + 2;
            end else if (c == 8'h32) begin
                n_out = fb[p + 1];
                for (i = 0; i < fb[p + 1] && i < 8; i = i + 1) last_out[i] = fb[p + 2 + i];
                put(8'h01); p = p + 2 + fb[p + 1];
            end else begin put(8'h02); p = fi; end
        end
        if (fb[0] == 8'hFF && fb[2] == 8'hF0) addr = 0;                             // (a reset: unaddressed again)
        if (o > 3) build();
    end
    sense <= addr == 0;
endtask

// the answer ob[0 .. o-1] (from the status byte at 2) framed and escaped into wb, and the send scheduled
task automatic build;
    ob[0] = 8'h00; ob[1] = 8'(o - 1);
    s = 0;
    for (i = 0; i < o; i = i + 1) s = s + ob[i];
    ob[o] = mode[1] ? s ^ 8'h01 : s;
    wn = 0;
    if (mode[3]) begin wb[wn] = 8'h55; wn = wn + 1; wb[wn] = 8'hD0; wn = wn + 1; end
    wb[wn] = 8'hE0; wn = wn + 1;
    for (i = 0; i <= o; i = i + 1)
        if (ob[i] == 8'hE0 || ob[i] == 8'hD0) begin wb[wn] = 8'hD0; wb[wn + 1] = ob[i] - 8'd1; wn = wn + 2; end
        else begin wb[wn] = ob[i]; wn = wn + 1; end
    for (i = 0; i < wn; i = i + 1) sb[i] = 1;
    if (mode[4]) sb[wn / 2] = 0;
    if (mode[2]) wn = wn / 2;
    if (!mode[0]) begin busy_t = 1; t_go = delay_us * 62 + delay_us / 2; wi = -1; cnt_t = 0; end
endtask

always @(posedge clk) begin
    b1 <= bus; b2 <= b1;
    if (rst) begin
        de <= 0; txd <= 1; sense <= 1; addr = 0; r_act = 0; inf = 0; busy_t = 0;
    end else if (busy_t) begin
        // the answer: delay_us, DE on 2 us before the first start bit, the bytes, DE off 2 us after the last stop bit
        if (t_go > 0) t_go = t_go - 1;
        else if (wi == -1) begin de <= 1; txd <= 1; cnt_t = cnt_t + 1; if (cnt_t == 125) begin wi = 0; nb_t = 0; cnt_t = 0; tsh = {sb[0][0], wb[0], 1'b0}; end end
        else if (wi < wn) begin
            txd <= tsh[0];
            cnt_t = cnt_t + 1;
            if (cnt_t == div) begin
                cnt_t = 0; tsh = {1'b1, tsh[9:1]}; nb_t = nb_t + 1;
                if (nb_t == 10) begin nb_t = 0; wi = wi + 1; if (wi < wn) tsh = {sb[wi][0], wb[wi], 1'b0}; end
            end
        end else begin
            txd <= 1; cnt_t = cnt_t + 1;
            if (cnt_t == 125) begin de <= 0; busy_t = 0; n_ans = n_ans + 1; end
        end
    end else begin
        // the host's bytes
        if (r_act) begin
            cnt_r = cnt_r - 1;
            if (cnt_r == 0) begin
                cnt_r = div; nb_r = nb_r + 1;
                if (nb_r == 1) begin if (b2) r_act = 0; end
                else if (nb_r <= 9) sh = {b2, sh[7:1]};
                else begin
                    r_act = 0;
                    if (!b2) inf = 0;
                    else if (sh == 8'hE0) begin inf = 1; fi = 0; esc = 0; end
                    else if (inf && sh == 8'hD0) esc = 1;
                    else if (inf) begin
                        v = esc ? sh + 8'd1 : sh; esc = 0;
                        fb[fi] = v; fi = fi + 1;
                        if (fi >= 3 && fi == fb[1] + 2) begin inf = 0; take(); end
                        else if (fi == 256) inf = 0;
                    end
                end
            end
        end else if (!b2) begin r_act = 1; cnt_r = div / 2; nb_r = 0; end
    end
end
endmodule
`default_nettype wire
