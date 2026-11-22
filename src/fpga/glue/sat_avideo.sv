// sat_avideo: the Analogizer adapter's analog video out (its video DAC on the cartridge port): RGBS at the Saturn's own
// 15 kHz timing, or RGBHV line-doubled to 31 kHz without scanlines (the other Analogizer cores' "Scandoubler RGBHV 0%").
// No RGsB / YPbPr / Y/C, no scanlines (the FPGA is nearly full).
//   menu "Analogizer video" (pk_settings 0x80000040): 0 off (the pins are not driven), 1 RGBS, 2 RGBHV (31 kHz)
//   pins (as the Analogizer cores): bank 3 = {R[5:0], HSYNC, VSYNC}, bank 2 = {B[0], BLANK_N, G[5:0]},
//   bank 1 [5:0] = {the DAC's clock, B[5:1]} (bank 1 [7:6] are the SNAC's: the JVS's DIR and TX)
//   RGBS: the composite sync ~(HS_N ^ VS_N) on HSYNC, VSYNC high. RGBHV: HSYNC and VSYNC apart, negative.
// The line doubler: each line (its pixels at DCLK, 6 bits a colour and the blanking) goes into one half of a two-line
// buffer while the other half, the line before, is read out twice at twice the pixel rate (a pixel at each DCLK and
// half way between two: the Saturn's DCLK is clk_sys / 8 or / 4). An input line starts at HS_N's fall; an output line
// starts there and again after the input line's pixel count; its HSYNC lasts the input's HSYNC pixel count (half the
// time); VSYNC is the input's, taken at the line starts. Interlaced modes come out as fields, line-doubled (bob).
// The outputs are clk_sys registers; the DAC is clocked by clk_pix_90 (clk_sys / 2 shifted by half a clk_sys period:
// it samples between clk_sys edges).
`default_nettype none
module sat_avideo (
	input  wire        clk,                // clk_sys
	input  wire [ 1:0] mode_74,            // the menu's setting (clk_74a)
	input  wire [ 7:0] r, g, b,
	input  wire        hs_n, vs_n, hbl_n, vbl_n,
	input  wire        dclk,               // the pixel's clock enable
	output wire        en,                 // banks 3, 2 and bank 1 [5:0] are the video's
	output reg  [ 7:0] bank3 = 8'h00,
	output reg  [ 7:0] bank2 = 8'h00,
	output reg  [ 4:0] bank1_b = 5'h00     // bank 1 [4:0] (B[5:1]); [5] is the DAC's clock (saturn_top)
);
	(* altera_attribute = "-name AUTO_SHIFT_REGISTER_RECOGNITION OFF; -name SYNCHRONIZER_IDENTIFICATION FORCED" *)
	reg  [1:0] m0 = 2'd0, m1 = 2'd0, m = 2'd0;
	always @(posedge clk) begin m0 <= mode_74; m1 <= m0; m <= m1; end
	assign en = m != 2'd0;
	wire       dbl = m == 2'd2;

	// ---------------------------------------------------------------- in: the pixel, the line's counts
	reg  [3:0] dcnt = 4'd0, ddiv = 4'd8;   // clocks since DCLK; clocks between two (8 or 4)
	reg  [9:0] hin = 10'd0, len = 10'd0, hsw = 10'd0, hsc = 10'd0;
	reg        hs_d = 1'b1, wbank = 1'b0, vs_l = 1'b1, line_go = 1'b0;
	wire       blank_n = hbl_n & vbl_n;
	wire       hs_fall = dclk && hs_d && !hs_n;
	always @(posedge clk) begin
		line_go <= 1'b0;
		dcnt <= dclk ? 4'd0 : (dcnt == 4'hF ? dcnt : dcnt + 4'd1);
		if (dclk) begin
			ddiv <= dcnt + 4'd1;
			hs_d <= hs_n;
			hin <= (hin == 10'h3FF) ? hin : hin + 10'd1;
			if (!hs_n) hsc <= (hsc == 10'h3FF) ? hsc : hsc + 10'd1;
			if (hs_fall) begin
				hin <= 10'd0; len <= hin + 10'd1; hsw <= hsc; hsc <= 10'd1;   // (len: the line's DCLKs, fall to fall)
				wbank <= ~wbank; vs_l <= vs_n; line_go <= 1'b1;
			end
		end
	end

	// the two-line buffer {blank_n, R, G, B} (6 bits a colour, as the DAC takes them)
	(* ramstyle = "M10K, no_rw_check" *) reg [18:0] lbuf [0:2047];
	always @(posedge clk) if (dclk && dbl) lbuf[{wbank, hin}] <= {blank_n, r[7:2], g[7:2], b[7:2]};

	// ---------------------------------------------------------------- out: twice the pixel rate, each line twice
	wire       oce = dclk || dcnt == {1'b0, ddiv[3:1]} - 4'd1;   // (DCLK and half way to the next)
	reg  [9:0] hout = 10'd0;
	always @(posedge clk) begin
		if (line_go) hout <= 10'd0;
		else if (oce) hout <= (hout + 10'd1 >= len) ? 10'd0 : hout + 10'd1;
	end
	reg [18:0] q;
	always @(posedge clk) q <= lbuf[{~wbank, hout}];
	reg        ohs_n = 1'b1, ohs_n1 = 1'b1;
	always @(posedge clk) begin ohs_n1 <= !(hout < hsw); ohs_n <= ohs_n1; end

	// ---------------------------------------------------------------- the pins
	reg  [5:0] r1, g1, b1;
	reg        cs1, bl1;
	always @(posedge clk) if (dclk) begin
		bl1 <= blank_n;
		r1 <= blank_n ? r[7:2] : 6'd0;
		g1 <= blank_n ? g[7:2] : 6'd0;
		b1 <= blank_n ? b[7:2] : 6'd0;
		cs1 <= ~(hs_n ^ vs_n);
	end
	wire       qb = q[18];
	wire [5:0] ro = dbl ? (qb ? q[17:12] : 6'd0) : r1;
	wire [5:0] go = dbl ? (qb ? q[11:6]  : 6'd0) : g1;
	wire [5:0] bo = dbl ? (qb ? q[5:0]   : 6'd0) : b1;
	wire       hso = dbl ? ohs_n : cs1;
	wire       vso = dbl ? vs_l : 1'b1;
	wire       blo = dbl ? qb : bl1;
	always @(posedge clk) begin
		bank3 <= {ro, hso, vso};
		bank2 <= {bo[0], blo, go};
		bank1_b <= bo[5:1];
	end
endmodule
`default_nettype wire
