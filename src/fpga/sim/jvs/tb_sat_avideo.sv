// Bench of glue/sat_avideo.sv's RGBHV line doubler: DCLK at clk_sys / 8 and / 4, a 427-pixel line: two output lines
// an input line, HSYNC half as long, each output line the input line before (R carries the line number), VSYNC.
`timescale 1ns/1ps
module tb_sat_avideo;
reg clk = 0; always #9.3 clk = ~clk;
reg [1:0] mode = 2; integer DIV = 8;
reg dclk = 0; integer dc = 0;
always @(posedge clk) begin dc = (dc + 1) % DIV; dclk <= dc == 0; end
// 427 pixels a line: HS low 30 pixels from 0, visible 60..379; 263 lines, VS low lines 0..2, visible 20..243
integer px = 0, ln = 0;
reg hs_n = 1, vs_n = 1, hbl_n = 0, vbl_n = 0; reg [7:0] r = 0, g = 0, b = 0;
always @(posedge clk) if (dclk) begin
  px = px + 1; if (px == 427) begin px = 0; ln = (ln + 1) % 263; end
  hs_n <= !(px < 30); vs_n <= !(ln < 3); hbl_n <= px >= 60 && px < 380; vbl_n <= ln >= 20 && ln < 244;
  r <= {ln[5:0], 2'b00}; g <= {px[5:0], 2'b00}; b <= {px[8:6], ln[8:6], 2'b00};
end
wire en; wire [7:0] b3, b2; wire [4:0] b1;
sat_avideo dut (.clk(clk), .mode_74(mode), .r(r), .g(g), .b(b), .hs_n(hs_n), .vs_n(vs_n), .hbl_n(hbl_n), .vbl_n(vbl_n), .dclk(dclk),
  .en(en), .bank3(b3), .bank2(b2), .bank1_b(b1));
// out: count HSYNC pulses per input line, their width in clocks, and check visible pixels: R = the line before's number
integer f = 0, hsn = 0, hs_lo = 0, hs_w = 0, in_lines = 0, vis = 0, bad = 0, vsl = 0; reg ohs = 1, ivs = 1;
integer since = 0, per_min = 1 << 30, per_max = 0;   // clocks between two output HSYNC falls: all the same
always @(posedge clk) begin
  ohs <= b3[1];
  since = since + 1;
  if (!b3[1]) hs_lo = hs_lo + 1;
  if (ohs && !b3[1]) begin
    hsn = hsn + 1;
    if (hsn > 4) begin if (since < per_min) per_min = since; if (since > per_max) per_max = since; end
    since = 0;
  end
  if (!ohs && b3[1]) begin hs_w = hs_lo; hs_lo = 0; end
  if (b2[6]) begin vis = vis + 1; if (b3[7:2] !== ((ln - 1) & 63) && b3[7:2] !== (ln & 63) && b3[7:2] !== ((ln - 2) & 63)) bad = bad + 1; end
  if (!b3[0]) vsl = vsl + 1;
end
reg ihs = 1; always @(posedge clk) if (dclk) begin ihs <= hs_n; if (ihs && !hs_n) in_lines = in_lines + 1; end
task automatic run(input integer d, input string name);
  DIV = d; hsn = 0; in_lines = 0; vis = 0; bad = 0; vsl = 0; per_min = 1 << 30; per_max = 0;
  repeat (427 * 263 * d * 2) @(posedge clk);
  $display("%s: input lines %0d, output HSYNCs %0d, HSYNC %0d clocks (input %0d), visible %0d, wrong %0d, VSYNC low %0d clocks",
           name, in_lines, hsn, hs_w, 30 * d, vis, bad, vsl);
  $display("  output line: %0d .. %0d clocks (input line %0d / 2 = %0d)", per_min, per_max, 427 * d, 427 * d / 2);
  if (per_max - per_min > 1) begin f++; $display("FAIL: output lines of different lengths (one in two shifted)"); end
  if (hsn < 2 * in_lines - 2 || hsn > 2 * in_lines + 2) begin f++; $display("FAIL: not two output lines a line"); end
  if (hs_w < 15 * d - d || hs_w > 15 * d + d) begin f++; $display("FAIL: HSYNC width"); end
  if (bad != 0 || vis < 320 * 224 * 2 * 2 * 9 / 10) begin f++; $display("FAIL: pixels"); end
endtask
initial begin
  repeat (427 * 8 * 300) @(posedge clk);
  run(8, "320 (DCLK / 8)");
  run(4, "640 (DCLK / 4)");
  $display("%s", f ? "SOME FAILED" : "ALL PASS"); $finish;
end
endmodule
