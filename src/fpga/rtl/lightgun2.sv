// lightgun2: the sensors of two Virtua Guns, from Saturn_MiSTer's rtl/lightgun.sv (commit 964714a) for the Pocket's
// JVS guns (glue/sat_jvs.sv): one set of raster counters for both guns, and only the sensor. The firmware
// (fw/cdb/smpc_jvs.c) does what lightgun.sv does to the joystick's position: the pixel X, the line Y and OFF (aiming off
// screen: no light, which reloads as on a real gun), so that only the beam's compare is left here. No mouse, no
// crosshair, no buttons (sat_jvs gives them), no auto-reload.
// Per gun as lightgun.sv: the position is taken at each VBLANK's end; SENSOR rises SENSOR_DELAY pixels after the beam
// reaches it (within 8 lines below), unless OFF, and falls 64 pixels later.
module lightgun2 #(parameter [7:0] SENSOR_DELAY = 8'd2)
(
	input            CLK,
	input            RESET,
	input      [9:0] X0, X1,         // the pixel (0..319 / 351)
	input      [8:0] Y0, Y1,         // the line (0 .. lines - 1)
	input            OFF0, OFF1,     // aiming off screen
	input            HDE,            // HBL_N, VBL_N: active-high "data enable"
	input            VDE,
	input            CE_PIX,
	output reg [1:0] SENSOR = 2'b00
);
	// the raster, shared
	reg [16:0] hde_d;
	reg  [9:0] hcnt;
	reg  [8:0] vcnt;
	reg        old_vde;
	always @(posedge CLK) begin
		if (RESET) hde_d <= '0;
		else if (CE_PIX) begin
			hde_d <= {hde_d[15:0], HDE};
			if (~&hcnt) hcnt <= hcnt + 1'd1;
			if (~hde_d[16] & ~HDE) hcnt <= 0;
			if (hde_d[16] & ~hde_d[15]) begin
				if (~VDE) vcnt <= 0;
				else if (~&vcnt) vcnt <= vcnt + 1'd1;
			end
			old_vde <= VDE;
		end
	end
	wire vde_rise = CE_PIX && ~old_vde && VDE;

	genvar g;
	generate for (g = 0; g < 2; g = g + 1) begin : gun
		reg  [9:0] x;
		reg  [8:0] y;
		reg        offscreen = 0, sensor_pend;
		reg  [7:0] sensor_time;
		always @(posedge CLK) begin
			if (vde_rise) begin
				x <= g ? X1 : X0;
				y <= g ? Y1 : Y0;
				offscreen <= g ? OFF1 : OFF0;
			end
			if (CE_PIX) begin
				if (~&sensor_time) sensor_time <= sensor_time + 1'd1;
				if (sensor_pend) begin
					if (sensor_time >= SENSOR_DELAY) begin
						SENSOR[g] <= !offscreen;
						sensor_pend <= 1'b0;
						sensor_time <= 8'd0;
					end
				end
				else if (sensor_time > 64) SENSOR[g] <= 1'b0;
			end
			if (HDE && VDE && (x == hcnt) && (y <= vcnt) && (y > vcnt - 8)) begin
				sensor_pend <= 1'b1;
				sensor_time <= 8'd0;
			end
		end
	end endgenerate
endmodule
