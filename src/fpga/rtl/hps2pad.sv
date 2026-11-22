module HPS2PAD (
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
	input              LGUN2_SENSOR
);

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
