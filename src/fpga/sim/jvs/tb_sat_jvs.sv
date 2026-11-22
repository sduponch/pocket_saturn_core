// Bench of glue/sat_jvs.sv (and the an_jvs it holds) driven as fw/cdb/smpc_jvs.c drives it, on CDB_HLE's EXT port
// words 6 and 7, against a behavioural I/O board (jvs_dev.sv): the menu switch, the bit time of each video mode, an
// init frame and its answer, the poll resent by itself and its answer read byte by byte (EXT_RE moving PTR on), the
// pads register, and the pins and pads given back when the menu switch goes off. Prints ALL PASS or SOME FAILED.
`timescale 1ns/1ps
`default_nettype none
module tb_sat_jvs;
reg          clk = 0, rst = 1;
always #9.3 clk = ~clk;                         // ~53.76 MHz (clk_sys, NTSC)

reg          on_74 = 0;
reg  [1:0]   mode = 2'b00;
reg          ext_we = 0, ext_re = 0;
reg  [2:0]   ext_a = 0;
reg  [31:0]  ext_wd = 0;
wire [31:0]  ext_rd;
wire         en, tx, dir, rx, sense, pads;
wire [15:0]  joy1, joy2;
reg  [1:0]   guns_74 = 0, recoil_74 = 1;
reg          clk_74a = 0, bridge_wr = 0;
always #6.734 clk_74a = ~clk_74a;               // 74.25 MHz
reg  [31:0]  bridge_addr = 0, bridge_wr_data = 0;
wire [1:0]   gun_on, gun_trig, gun_start;
wire [7:0]   gun1_x, gun1_y, gun2_x, gun2_y;
sat_jvs #(.CLK_HZ(53_748_200)) dut (.clk, .rst, .on_74, .guns_74, .recoil_74, .clk_74a, .bridge_wr, .bridge_addr,
    .bridge_wr_data, .mode, .vres(2'd1), .ext_we, .ext_re, .ext_a, .ext_wd, .ext_rd,
    .en, .tx, .dir, .rx, .sense, .pads, .joy1, .joy2, .gun_on, .gun1_x, .gun1_y, .gun2_x, .gun2_y, .gun_trig, .gun_start);
task automatic bw(input [31:0] ad, input [31:0] d);   // an APF bridge write
    @(posedge clk_74a) #1; bridge_addr = ad; bridge_wr_data = d; bridge_wr = 1;
    @(posedge clk_74a) #1; bridge_wr = 0;
endtask

// the bus: the host's driver while DIR, else the board's, else idle (1)
wire         de, dtx;
wire         bus = dir ? tx : de ? dtx : 1'b1;
assign rx = dir ? 1'b1 : bus;
reg  [15:0]  d_div = 467, d_delay = 40;
reg  [63:0]  d_sw = 64'h0;
wire [31:0]  d_frames, d_bad, d_ans;
wire [7:0]   d_addr, d_nout;
wire [7:0]   d_out [0:7];
jvs_dev dev (.clk, .rst, .bus, .de, .txd(dtx), .sense, .div(d_div), .delay_us(d_delay), .mode(8'd0), .sw(d_sw), .an(128'd0),
             .n_frames(d_frames), .n_bad(d_bad), .n_ans(d_ans), .addr(d_addr), .last_out(d_out), .n_out(d_nout));

integer      fails = 0, checks = 0;
task automatic check(input bit c, input string what);
    checks = checks + 1;
    if (!c) begin fails = fails + 1; $display("FAIL: %s", what); end
endtask

// the firmware's accesses: a write is EXT_WE for one clock; a read is EXT_RE for one clock, EXT_RD taken at its end
task automatic ext_w(input [2:0] a, input [31:0] v);
    @(posedge clk) #1; ext_a = a; ext_wd = v; ext_we = 1;
    @(posedge clk) #1; ext_we = 0;
    repeat (4) @(posedge clk);                  // (the CPU's next access comes a few clocks later)
endtask
task automatic ext_r(input [2:0] a, output [31:0] v);
    @(posedge clk) #1; ext_a = a; ext_re = 1;
    @(negedge clk) v = ext_rd;
    @(posedge clk) #1; ext_re = 0;
    repeat (4) @(posedge clk);
endtask
task automatic jw(input [3:0] r, input [31:0] v); ext_w(6, r); ext_w(7, v); endtask
task automatic jr(input [3:0] r, output [31:0] v); ext_w(6, r); ext_r(7, v); endtask

reg  [31:0]  v, x;
reg  [7:0]   ans [0:31];
integer      n, i, t0, g0;
task automatic send(input integer cnt, input [8*8-1:0] f, input [31:0] ctrl);   // f's top bytes first
    jw(1, ctrl & ~32'd2); jw(4, 0); ext_w(6, 5);                                 // (POLL off while the frame is written)
    for (int k = 0; k < cnt; k++) ext_w(7, f[8*8-1-8*k -: 8]);
    jw(3, cnt);
    if (ctrl[1]) jw(1, ctrl);
endtask
task automatic wait_done(output [31:0] xr);
    t0 = 0;
    do begin jr(3, xr); t0++; end while (xr[31] && t0 < 200000);
endtask
task automatic answer(output integer cnt);
    jr(3, x); cnt = x[7:0]; if (cnt > 32) cnt = 32;
    jw(4, 128); ext_w(6, 5);
    for (int k = 0; k < cnt; k++) begin ext_r(7, v); ans[k] = v[7:0]; end
endtask

// the bit time on the wire: clocks between the sync byte's start bit and its first 0 data bit (E0: bits 0-4 are 0, so
// the start bit and 5 zero bits give one low stretch of 6 bits)
integer      lo_start, lo_len, measuring = 0;
reg          tx_d = 1;
always @(posedge clk) begin
    tx_d <= tx;
    if (measuring == 1 && dir && tx_d && !tx) begin lo_start = $time; measuring = 2; end
    else if (measuring == 2 && !tx_d && tx) begin lo_len = $time - lo_start; measuring = 3; end
end

initial begin
    repeat (20) @(posedge clk); rst = 0;
    repeat (10) @(posedge clk);

    ext_r(6, v);
    check(v[31] == 0 && en == 0, "off at start: ON 0, the pins not the JVS's");
    on_74 = 1; repeat (10) @(posedge clk);
    ext_r(6, v);
    check(v[31] == 1, "the menu's switch reaches ON");
    jr(0, v);
    check(v == 32'h4A56_0001, $sformatf("an_jvs ID through IDX 0 (%08x)", v));

    // ---- set the address (FF F1 01) at each video mode's bit time; the board's receiver at the same bit time
    for (int m = 0; m < 4; m++) begin
        mode = 2'(m);
        d_div = m == 0 ? 467 : m == 1 ? 497 : m == 2 ? 463 : 494;
        repeat (10) @(posedge clk);
        measuring = 1;
        if (m == 0) send(3, {8'hFF, 8'hF1, 8'h01, 40'd0}, 32'h0000_1401);
        else        send(2, {8'h01, 8'h10, 48'd0}, 32'h0000_1401);                 // (identity)
        wait_done(x);
        check(x[30:29] == 2'b00, $sformatf("mode %0d: an answer (XFER %08x)", m, x));
        check(measuring == 3 && lo_len > 6 * d_div * 18.6 - 60 && lo_len < 6 * d_div * 18.6 + 60,
              $sformatf("mode %0d: six bit times of %0d clocks (%0d ns)", m, d_div, lo_len));
        answer(n);
        check(n >= 4 && ans[0] == 8'h00 && ans[2] == 8'h01 && ans[3] == 8'h01, $sformatf("mode %0d: answer 00 .. 01 01", m));
        repeat (600000) @(posedge clk);         // (10 ms between an answer and the next frame)
    end
    check(d_addr == 8'd1, "the board took the address 1");
    check(d_bad == 0, "the board saw no broken frame");

    // ---- the poll, resent by the hardware: 01 20 02 02, every 14 ms
    mode = 2'b00; d_div = 467;
    d_sw = 64'h0000_0000_C3A5_0F80;             // system 80, P1 0F A5, P2 C3 00
    jr(2, v); g0 = v[15:8];
    send(4, {8'h01, 8'h20, 8'h02, 8'h02, 32'd0}, 32'h0000_0E03);
    repeat (3 * 14 * 53750 + 20000) @(posedge clk);
    jr(2, v);
    check(v[15:8] - g0 >= 3, $sformatf("the poll resent by itself (GOOD %0d -> %0d)", g0, v[15:8]));
    answer(n);
    check(n == 10 && ans[2] == 8'h01 && ans[3] == 8'h01 && ans[4] == 8'h80 && ans[5] == 8'h0F && ans[6] == 8'hA5
          && ans[7] == 8'hC3 && ans[8] == 8'h00, $sformatf("the poll's answer byte by byte (%0d bytes: %02x %02x %02x %02x %02x %02x %02x)",
          n, ans[2], ans[3], ans[4], ans[5], ans[6], ans[7], ans[8]));

    // ---- the pads
    check(pads == 0 && joy1 == 16'hFFFF && joy2 == 16'hFFFF, "pads idle before the firmware writes them");
    jw(8, 32'h1234_ABCD); jw(9, 1);
    check(pads == 1 && joy1 == 16'hABCD && joy2 == 16'h1234, "PADS_V and PADS");
    jr(8, v); check(v == 32'h1234_ABCD, "PADS_V reads back");
    ext_r(6, v); check(v[30] == 1 && v[3:0] == 4'd8, "word 6: PADS and IDX");

    // ---- the JVS profile through the bridge (words 0x36 .. 0x3F kept), its load count, and the settings in word 6
    jr(12, v); g0 = v[7:0];
    for (int w = 0; w < 64; w++) bw(32'h5000_0000 + 4 * w, 32'h0101_0101 * w);
    repeat (10) @(posedge clk);
    jr(12, v); check(v[7:0] == 8'(g0 + 1), "PROF_SEQ + 1 after a profile load");
    jw(10, 0); jr(11, v); check(v == 32'h3636_3636, $sformatf("PROF_D word 0x36 (%08x)", v));
    jw(10, 9); jr(11, v); check(v == 32'h3F3F_3F3F, $sformatf("PROF_D word 0x3F (%08x)", v));
    bw(32'h5000_0000, 32'h1234_5678);
    jw(10, 0); jr(11, v); check(v == 32'h3636_3636, "words below 0x36 not kept");
    guns_74 = 2; recoil_74 = 2; repeat (10) @(posedge clk);
    ext_r(6, v); check(v[29:28] == 2'd2 && v[27:26] == 2'd2 && v[25:24] == 2'd1, $sformatf("word 6: GUNS RECOIL VRES (%08x)", v));
    check(gun_on == 2'b11, "both ports guns while PADS");
    jw(13, {14'd0, 1'b1, 1'b0, 8'd77, 8'd200}); jw(14, {14'd0, 1'b0, 1'b1, 8'd99, 8'd11});
    check(gun1_x == 200 && gun1_y == 77 && gun_start == 2'b01 && gun_trig == 2'b10 && gun2_x == 11 && gun2_y == 99,
          "GUN1 / GUN2 registers");
    guns_74 = 1; repeat (10) @(posedge clk);
    check(gun_on == 2'b01, "P1 gun only");

    // ---- the menu's switch off: pins and pads given back
    on_74 = 0; repeat (10) @(posedge clk);
    check(pads == 0 && en == 0 && gun_on == 2'b00, "switch off: PADS cleared, no gun, the pins released");
    jw(9, 1);
    check(pads == 0, "PADS stays 0 while off");

    $display("%s (%0d checks, %0d failed)", fails == 0 ? "ALL PASS" : "SOME FAILED", checks, fails);
    $finish;
end
endmodule
`default_nettype wire
