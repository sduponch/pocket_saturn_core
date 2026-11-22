module HPS2PAD (
	input              CLK,              // (the Pocket: for the 3D pad / Arcade Racer handshake below)
	input              RST_N,
	input              SMPC_CE,
	output     [ 6: 0] PDR1I,
	input      [ 6: 0] PDR1O,
	input      [ 6: 0] DDR1,
	output     [ 6: 0] PDR2I,
	input      [ 6: 0] PDR2O,
	input      [ 6: 0] DDR2,
	
	input      [15: 0] JOY1,
	input      [15: 0] JOY2,             // (the Pocket: a JVS board's player 2, glue/sat_jvs.sv)
	input              JOY2_ON,          // port 2 a digital pad (JOY2), else nothing
	// (the Pocket) a Virtua Gun / Stunner on port 1 / 2 instead (rtl/lightgun.sv from a JVS board's gun), as Saturn_MiSTer's
	// HPS2PAD PAD_VIRT_LGUN: PDRxI [6] = the sensor (TH: EXLE latches the VDP2's H / V counters), [5] Start_n,
	// [4] Trigger_n, [3:0] the ID nibble 1100 (the SMPC reads ID 0xA)
	input              LGUN1_ON,
	input              LGUN1_TRIG,
	input              LGUN1_START,
	input              LGUN1_SENSOR,
	input              LGUN2_ON,
	input              LGUN2_TRIG,
	input              LGUN2_START,
	input              LGUN2_SENSOR,
	// (the Pocket) port 1 a 3D Control Pad (ANA1_MODE 1) or an Arcade Racer (2) instead, from a JVS board's axes, as
	// Saturn_MiSTer's HPS2PAD PAD_3D / PAD_WHEEL: the SMPC's TH / TR handshake, a nibble each step on [3:0] with TL [4].
	// JOY1 the buttons (as for the digital pad), X / Y the stick or the wheel and L / R the triggers, 0..255 (80 centred)
	input      [ 1: 0] ANA1_MODE,
	input      [ 7: 0] ANA1_X,
	input      [ 7: 0] ANA1_Y,
	input      [ 7: 0] ANA1_L,
	input      [ 7: 0] ANA1_R
);
	wire        a3d = ANA1_MODE == 2'd1;
	bit  [ 3:0] OUT1;
	bit         TL1;
	bit  [ 4:0] STATE1;
	always @(posedge CLK or negedge RST_N) begin
		if (!RST_N) begin
			STATE1 <= '0; OUT1 <= '0; TL1 <= 1;
		end else if (SMPC_CE && ANA1_MODE != 2'd0) begin
			case (STATE1)
				5'd00,
				5'd01: if (PDR1O[6:5] == 2'b01) begin OUT1 <= 4'h1;                                  TL1 <= 1; STATE1 <= 5'd2; end
				5'd02: if (PDR1O[6:5] == 2'b00) begin OUT1 <= 4'h1;                                  TL1 <= 0; STATE1 <= 5'd3; end
				5'd03: if (PDR1O[6:5] == 2'b01) begin OUT1 <= a3d ? 4'h6 : 4'h3;                     TL1 <= 1; STATE1 <= 5'd4; end
				5'd04: if (PDR1O[6:5] == 2'b00) begin OUT1 <= JOY1[15:12];                           TL1 <= 0; STATE1 <= 5'd5; end
				5'd05: if (PDR1O[6:5] == 2'b01) begin OUT1 <= JOY1[11: 8];                           TL1 <= 1; STATE1 <= 5'd6; end
				5'd06: if (PDR1O[6:5] == 2'b00) begin OUT1 <= a3d ? JOY1[7:4] : {1'b1, JOY1[6:4]};   TL1 <= 0; STATE1 <= 5'd7; end
				5'd07: if (PDR1O[6:5] == 2'b01) begin OUT1 <= a3d ? JOY1[3:0] : 4'b1111;             TL1 <= 1; STATE1 <= 5'd8; end
				5'd08: if (PDR1O[6:5] == 2'b00) begin OUT1 <= ANA1_X[7:4];                           TL1 <= 0; STATE1 <= 5'd9; end
				5'd09: if (PDR1O[6:5] == 2'b01) begin OUT1 <= ANA1_X[3:0];                           TL1 <= 1; STATE1 <= 5'd10; end
				5'd10: if (PDR1O[6:5] == 2'b00 && a3d) begin OUT1 <= ANA1_Y[7:4];                    TL1 <= 0; STATE1 <= 5'd11; end
				5'd11: if (PDR1O[6:5] == 2'b01) begin OUT1 <= ANA1_Y[3:0];                           TL1 <= 1; STATE1 <= 5'd12; end
				5'd12: if (PDR1O[6:5] == 2'b00) begin OUT1 <= ANA1_R[7:4];                          TL1 <= 0; STATE1 <= 5'd13; end
				5'd13: if (PDR1O[6:5] == 2'b01) begin OUT1 <= ANA1_R[3:0];                          TL1 <= 1; STATE1 <= 5'd14; end
				5'd14: if (PDR1O[6:5] == 2'b00) begin OUT1 <= ANA1_L[7:4];                          TL1 <= 0; STATE1 <= 5'd15; end
				5'd15: if (PDR1O[6:5] == 2'b01) begin OUT1 <= ANA1_L[3:0];                          TL1 <= 1; STATE1 <= 5'd16; end
				5'd16: if (PDR1O[6:5] == 2'b00) begin OUT1 <= 4'h0;                                  TL1 <= 0; STATE1 <= 5'd17; end
				5'd17: if (PDR1O[6:5] == 2'b01) begin OUT1 <= 4'h1;                                  TL1 <= 1; STATE1 <= 5'd18; end
			endcase
			if (PDR1O[6:5] == 2'b11) begin OUT1 <= 4'h1; TL1 <= 1; STATE1 <= 5'd0; end
		end
	end


	// Port 1: digital pad. Port 2: a digital pad while JOY2_ON, else nothing connected (inputs read back the pull-ups).
	always_comb begin
		PDR1I = (PDR1O & DDR1) | ~DDR1;
		if (DDR1[6:5] == 2'b10) begin
			case (PDR1O[6])
				1'b0:  PDR1I[3:0] = JOY1[15:12];
				1'b1:  PDR1I[3:0] = {JOY1[3],3'b100};
			endcase
		end else if (DDR1[6:5] == 2'b11) begin
			case (PDR1O[6:5])
				2'b00: PDR1I[3:0] = JOY1[ 7: 4];
				2'b01: PDR1I[3:0] = JOY1[15:12];
				2'b10: PDR1I[3:0] = JOY1[11: 8];
				2'b11: PDR1I[3:0] = {JOY1[3],3'b100};
			endcase
		end
		
		if (LGUN1_ON) PDR1I[6:0] = {!LGUN1_SENSOR, !LGUN1_START, !LGUN1_TRIG, 4'b1100};
		else if (ANA1_MODE != 2'd0) PDR1I[4:0] = {TL1, OUT1};
		PDR2I = (PDR2O & DDR2) | ~DDR2;
		if (LGUN2_ON) PDR2I[6:0] = {!LGUN2_SENSOR, !LGUN2_START, !LGUN2_TRIG, 4'b1100};
		else if (JOY2_ON) begin
			if (DDR2[6:5] == 2'b10) begin
				case (PDR2O[6])
					1'b0:  PDR2I[3:0] = JOY2[15:12];
					1'b1:  PDR2I[3:0] = {JOY2[3],3'b100};
				endcase
			end else if (DDR2[6:5] == 2'b11) begin
				case (PDR2O[6:5])
					2'b00: PDR2I[3:0] = JOY2[ 7: 4];
					2'b01: PDR2I[3:0] = JOY2[15:12];
					2'b10: PDR2I[3:0] = JOY2[11: 8];
					2'b11: PDR2I[3:0] = {JOY2[3],3'b100};
				endcase
			end
		end
	end
	
endmodule
