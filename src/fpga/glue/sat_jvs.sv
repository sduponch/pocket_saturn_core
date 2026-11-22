// sat_jvs: a JVS I/O board as the Saturn's two pads or two Virtua Guns, through the Analogizer SNAC adapter's JVS
// wiring. The host is an_jvs (glue/an_jvs.sv: framing, escapes, checksums, the poll resent every PERIOD ms by itself);
// the CD block's firmware (fw/cdb/smpc_jvs.c) drives it on the last two words of CDB_HLE's EXT port and writes the
// pads and guns it makes of the answers.
//
// Firmware words (CDB_HLE I/O, EXT_A 6 and 7; words 0-5 are the disc source's, glue/cd_src_b.sv):
//   0x78  W  [3:0] IDX, the register 0x7C reaches
//         R  {ON[31], PADS[30], GUNS[29:28], RECOIL[27:26], VRES[25:24], 20'd0, IDX[3:0]}
//            ON: the menu's "JVS (Analogizer)" (the pins are the JVS's); GUNS: "JVS guns" 0 off, 1 P1, 2 P1 + P2;
//            RECOIL: "JVS recoil" 0 off, 1 single, 2 auto; VRES: the core's (0 224, 1 240, 2 256 lines)
//   0x7C  IDX 0-5: an_jvs's registers ID CTRL STAT XFER PTR DATA (reading DATA moves PTR on, as writing it)
//         IDX 8: PADS_V {port 2[31:16], port 1[15:0]}, the pads as pk_pad's joy1 (active low)
//         IDX 9: [0] PADS, the board answers: the pads (and guns) are the JVS's; cleared while ON is 0
//         IDX 10: W PROF_A [3:0]   IDX 11: R PROF_D, the JVS profile's word 0x36 + PROF_A (file bytes 0xD8 .. 0xFF,
//                 byte 4k in [7:0])   IDX 12: R PROF_SEQ, + 1 at each profile load (its last word written)
//         IDX 13 / 14: W GUN1 / GUN2 {Start[17], Trigger[16], Y[15:8], X[7:0]} (X 0..255 across the screen, Y the line
//                 + 8, as rtl/lightgun.sv takes them)
// The JVS profile is the data slot "JVS Profile" (data.json id 5, bridge 0x50000000, the 256-byte jvs_vendor.bin of
// the other Analogizer JVS cores); only its fixed tail is kept (axes, trigger masks, outputs).
// The bit time follows clk_sys, which the PLL moves with the video mode ({PAL, SMPC_DOTSEL}, saturn_top's pll_mode).
`default_nettype none
module sat_jvs #(parameter int CLK_HZ = 53_748_200) (
	input  wire        clk,                // clk_sys
	input  wire        rst,
	input  wire        on_74,              // the menu's settings (clk_74a)
	input  wire [ 1:0] guns_74,
	input  wire [ 1:0] recoil_74,
	input  wire        clk_74a,            // the JVS profile's data slot
	input  wire        bridge_wr,
	input  wire [31:0] bridge_addr,
	input  wire [31:0] bridge_wr_data,
	input  wire [ 1:0] mode,               // {PAL, SMPC_DOTSEL} (clk_sys)
	input  wire [ 1:0] vres,               // the core's VRES (clk_sys)
	input  wire        ext_we, ext_re,     // CDB_HLE's EXT port
	input  wire [ 2:0] ext_a,
	input  wire [31:0] ext_wd,
	output wire [31:0] ext_rd,             // for EXT_A 6 and 7
	output wire        en,                 // the JVS has the pins: bank 0 an input, bank 1 [7] DIR [6] TX
	output wire        tx, dir,
	input  wire        rx, sense,          // bank 0 [7], [5] (asynchronous)
	output reg         pads = 1'b0,
	output reg  [15:0] joy1 = 16'hFFFF,
	output reg  [15:0] joy2 = 16'hFFFF,
	output wire [ 1:0] gun_on,             // port 1 / 2 a Virtua Gun (rtl/lightgun.sv, rtl/hps2pad.sv)
	output reg  [ 7:0] gun1_x = 8'd128, gun1_y = 8'd128, gun2_x = 8'd128, gun2_y = 8'd128,
	output reg  [ 1:0] gun_trig = 2'b00,
	output reg  [ 1:0] gun_start = 2'b00
);
	reg  [4:0] set_s0 = 5'd0, set_s1 = 5'd0, set_s = 5'd0;
	always @(posedge clk) begin set_s0 <= {recoil_74, guns_74, on_74}; set_s1 <= set_s0; set_s <= set_s1; end
	wire       on = set_s[0];
	wire [1:0] guns = set_s[2:1], recoil = set_s[4:3];
	assign gun_on = {pads && guns == 2'd2, pads && guns != 2'd0};

	// clocks of a bit at 115200: 53748200, 57272800 (SMPC_DOTSEL), 53375000 (PAL), 56875000 (PAL, SMPC_DOTSEL)
	reg [10:0] div;
	always @(posedge clk) case (mode)
		2'b00: div <= 11'd467;
		2'b01: div <= 11'd497;
		2'b10: div <= 11'd463;
		default: div <= 11'd494;
	endcase

	// the JVS profile's tail (words 0x36 .. 0x3F), written by APF (clk_74a); read by the firmware as quasi-static data
	reg [31:0] prof [0:9];
	reg  [7:0] prof_seq = 8'd0;
	integer    pi;
	initial for (pi = 0; pi < 10; pi = pi + 1) prof[pi] = 32'd0;
	wire       prof_wr = bridge_wr && bridge_addr[31:24] == 8'h50 && bridge_addr[23:8] == 16'd0 && bridge_addr[7:2] >= 6'h36;
	wire [5:0] prof_w = bridge_addr[7:2] - 6'h36;   // (0 .. 9)
	always @(posedge clk_74a) if (prof_wr) begin
		prof[prof_w[3:0]] <= bridge_wr_data;
		if (bridge_addr[7:2] == 6'h3F) prof_seq <= prof_seq + 8'd1;
	end

	reg  [3:0] idx = 4'd0, prof_a = 4'd0;
	wire       w7 = ext_we && ext_a == 3'd7, r7 = ext_re && ext_a == 3'd7;
	always @(posedge clk) begin
		if (ext_we && ext_a == 3'd6) idx <= ext_wd[3:0];
		if (w7 && idx == 4'd8) {joy2, joy1} <= ext_wd;
		if (w7 && idx == 4'd9) pads <= ext_wd[0];
		if (w7 && idx == 4'd10) prof_a <= ext_wd[3:0];
		if (w7 && idx == 4'd13) {gun_start[0], gun_trig[0], gun1_y, gun1_x} <= ext_wd[17:0];
		if (w7 && idx == 4'd14) {gun_start[1], gun_trig[1], gun2_y, gun2_x} <= ext_wd[17:0];
		if (rst || !on) pads <= 1'b0;
	end

	wire [31:0] jv_q;
	an_jvs #(.CLK_HZ(CLK_HZ)) host (
		.clk(clk), .rst(rst),
		.sel_wr(w7 && !idx[3]), .sel_rd(r7 && !idx[3]), .a(idx), .wd(ext_wd), .rd(jv_q), .own(on),
		.en(en), .tx(tx), .dir(dir), .rx(rx), .sense(sense), .div(div)
	);

	reg [31:0] r7v;
	always @(*) case (idx)
		4'd8:  r7v = {joy2, joy1};
		4'd9:  r7v = {31'd0, pads};
		4'd11: r7v = prof_a < 4'd10 ? prof[prof_a] : 32'd0;
		4'd12: r7v = {24'd0, prof_seq};
		default: r7v = idx[3] ? 32'd0 : jv_q;
	endcase
	assign ext_rd = ext_a[0] ? r7v : {on, pads, guns, recoil, vres, 20'd0, idx};
endmodule
`default_nettype wire
