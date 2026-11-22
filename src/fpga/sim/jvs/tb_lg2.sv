// Bench: rtl/lightgun2.sv, fed as fw/cdb/smpc_jvs.c converts a gun's position, against two Saturn_MiSTer lightguns
// (lightgun_ref.sv) on a synthetic raster: the sensors must match cycle for cycle once the reference is defined.
//   vlog -sv lightgun_ref.sv ../../rtl/lightgun2.sv tb_lg2.sv; vsim -c tb_lg2 -voptargs=+initreg+0 -do "run -all"
`timescale 1ns/1ps
module tb_lg2;
reg clk = 0; always #5 clk = ~clk;
reg rst = 1, ce = 0, hde = 0, vde = 0; reg [1:0] hres = 0;
reg [7:0] x0 = 100, y0 = 100, x1 = 30, y1 = 200;
integer pc = 0, line = 0, px = 0;
always @(posedge clk) begin ce <= (pc == 3); pc <= (pc + 1) % 4; end
// 427 pixels a line (320 visible), 263 lines (224 visible)
always @(posedge clk) if (ce) begin
  px = px + 1; if (px == (hres[0] ? 455 : 427)) begin px = 0; line = (line + 1) % 263; end
  hde <= px >= 40 && px < 40 + (hres[0] ? 352 : 320); vde <= line >= 16 && line < 16 + 224;
end
wire s0, s1; wire [1:0] s2;
lightgun a (.CLK(clk), .RESET(rst), .MOUSE(25'd0), .MOUSE_XY(1'b0), .JOY_X(x0 ^ 8'h80), .JOY_Y(y0 ^ 8'h80), .JOY(12'd0), .BTN_MODE(1'b0), .RELOAD(1'b0),
  .HDE(hde), .VDE(vde), .CE_PIX(ce), .FIELD(1'b0), .INTERLACE(1'b0), .HRES(hres), .VRES(2'd0), .DCE_R(1'b0), .SIZE(2'd3), .SENSOR_DELAY(8'd2),
  .CROSS_DRAW(), .SENSOR(s0), .BTN_A(), .BTN_B(), .BTN_C(), .BTN_START());
lightgun b (.CLK(clk), .RESET(rst), .MOUSE(25'd0), .MOUSE_XY(1'b0), .JOY_X(x1 ^ 8'h80), .JOY_Y(y1 ^ 8'h80), .JOY(12'd0), .BTN_MODE(1'b0), .RELOAD(1'b0),
  .HDE(hde), .VDE(vde), .CE_PIX(ce), .FIELD(1'b0), .INTERLACE(1'b0), .HRES(hres), .VRES(2'd0), .DCE_R(1'b0), .SIZE(2'd3), .SENSOR_DELAY(8'd2),
  .CROSS_DRAW(), .SENSOR(s1), .BTN_A(), .BTN_B(), .BTN_C(), .BTN_START());
// the firmware's conversion (fw/cdb/smpc_jvs.c), here from the 8-bit positions; the original takes Y as the line + 8
function automatic [9:0] fpx(input [7:0] v, input h); fpx = v + (v >> 2) + (h ? v >> 3 : 0); endfunction
function automatic off(input [7:0] xv, input [8:0] ln, input h); off = fpx(xv, h) < 2 || fpx(xv, h) >= (h ? 352 : 320) - 2 || ln < 2 || ln >= 224 - 2; endfunction
wire [8:0] l0 = y0 < 8 ? 0 : y0 - 8, l1 = y1 < 8 ? 0 : y1 - 8;
lightgun2 c (.CLK(clk), .RESET(rst), .X0(fpx(x0, hres[0])), .Y0(l0), .OFF0(off(x0, l0, hres[0])), .X1(fpx(x1, hres[0])), .Y1(l1), .OFF1(off(x1, l1, hres[0])),
             .HDE(hde), .VDE(vde), .CE_PIX(ce), .SENSOR(s2));
integer xs = 0, diff = 0, hits0 = 0, hits1 = 0, hits2 = 0, frames = 0; reg s0d = 0, s2d = 0;
always @(posedge clk) if (!rst) begin
  if ($isunknown({s0, s1})) xs = xs + 1;
  s0d <= s0; if (s0 && !s0d) hits0 = hits0 + 1;
  s2d <= s2[1]; if (s2[1] && !s2d) hits2 = hits2 + 1;
end
always @(posedge vde) begin frames = frames + 1; if (frames % 3 == 0) begin x0 = x0 + 37; y0 = y0 + 53; x1 = x1 - 29; y1 = y1 + 71; end if (frames == 20) hres = 1; end
initial begin repeat (20) @(posedge clk); rst = 0; wait (frames == 40); $display("frames %0d, sensor rises gun 0 %0d gun 1 (lg2) %0d, mismatches %0d (unknown in the original %0d cycles): %s", frames, hits0, hits2, diff, xs, diff == 0 && hits0 > 10 ? "ALL PASS" : "SOME FAILED"); $finish; end
endmodule
