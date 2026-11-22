// Bench of the JVS path with the real firmware: CDB_HLE (its RISC-V and the firmware image) + SMPC_FW + sat_jvs +
// a behavioural I/O board (jvs_dev.sv), wired as rtl/Saturn/Saturn.sv and pocket_rtl/saturn_top.sv do. The firmware is
// a bench build of fw/cdb (build.sh SMPC_FW=1 CD_SRC=1 DEFS=-DJV_MS_EDGES=400: its JVS milliseconds 10 times shorter).
// Prints the frames the board takes, the answers, and the pads; ALL PASS once both pads carry the board's switches.
//   vlog +define+CD_EXTIO+SMPC_FW+CDB_FEMTORV ... ; vsim +FW=<dir with fw.hex, fw_1.hex>
// NOT RUN YET: Questa rejects the forward references of CDB_HLE.sv, SMPC_FW.sv, picorv32 / femtorv and hps2pad.sv
// (Quartus takes them); it needs a simulator that accepts them, or edited copies.
`timescale 1ns/1ps
`default_nettype none
module tb_sat_fw;
reg          clk = 0, rst_n = 0;
always #9.3 clk = ~clk;                         // ~53.76 MHz (clk_sys, NTSC)

// clock enables: SMPC 4 MHz, TCE 11.2896 MHz, ACE 44.1 kHz (accumulators, as CEGen)
integer      a_smpc = 0, a_tce = 0, a_ace = 0;
reg          smpc_ce = 0, tce = 0, ace = 0;
always @(posedge clk) begin
    a_smpc = a_smpc + 4_000_000;   smpc_ce <= a_smpc >= 53_748_200; if (a_smpc >= 53_748_200) a_smpc = a_smpc - 53_748_200;
    a_tce  = a_tce + 11_289_600;   tce     <= a_tce  >= 53_748_200; if (a_tce  >= 53_748_200) a_tce  = a_tce  - 53_748_200;
    a_ace  = a_ace + 44_100;       ace     <= a_ace  >= 53_748_200; if (a_ace  >= 53_748_200) a_ace  = a_ace  - 53_748_200;
end
// VBLANK (IRQV_N): low 1.4 ms every 16.7 ms
reg          irqv_n = 1;
always begin #15_300_000; irqv_n = 0; #1_400_000; irqv_n = 1; end

wire [4:0]   fw_a;
wire         fw_we;
wire [31:0]  fw_wd, fw_rd;
wire         cdres_n;
wire [6:0]   pa_o, pa_d, pb_o, pb_d, pa_i, pb_i;
wire         pads;
wire [15:0]  joy1, joy2;
HPS2PAD pad (.PDR1I(pa_i), .PDR1O(pa_o), .DDR1(pa_d), .PDR2I(pb_i), .PDR2O(pb_o), .DDR2(pb_d),
             .JOY1(pads ? joy1 : 16'hFFFF), .JOY2(joy2), .JOY2_ON(pads));
SMPC_FW smpc (
    .CLK(clk), .RST_N(rst_n), .EN(1'b1), .CE(smpc_ce), .CE32K(1'b0), .RESET_N(1'b1),
    .A(6'd0), .DBI(8'd0), .DBO(), .CS_N(1'b1), .RW_N(1'b1),
    .D0(), .D1(1'b1), .D2(1'b0), .D3(1'b1), .D4(), .D5(), .D6(), .D7(), .D8(), .D9(), .D10(), .D13_INT0(1'b1),
    .R01_INT2(irqv_n), .PIOA_O(pa_o), .PIOA_I(pa_i), .PIOA_D(pa_d), .PIOB_O(pb_o), .PIOB_I(pb_i), .PIOB_D(pb_d),
    .R5(4'hC), .R60_D(), .R60_O(), .R60_I(1'b0), .R61_D(), .R61_O(), .R61_I(1'b1), .R62_I(1'b0), .R63_I(1'b0),
    .R70_I(1'b0), .R71_I(1'b0), .R72_O(cdres_n), .R73_O(), .EXL_N(), .EXT_RTC(65'd0),
    .FW_A(fw_a), .FW_WE(fw_we), .FW_WD(fw_wd), .FW_RD(fw_rd)
);

wire [2:0]   ext_a;
wire         ext_we, ext_re;
wire [31:0]  ext_wd, ext_rd, jvs_rd;
// the disc source's words 0-5 (glue/cd_src_b.sv) as with no disc: status {slots 6, CD, ...}, table words 0
wire [31:0]  src_rd = ext_a[0] ? 32'd0 : {4'd0, 4'd6, 8'hCD, 16'd0};
assign ext_rd = ext_a[2:1] == 2'b11 ? jvs_rd : src_rd;
CDB_HLE #(.FW_FILE("fw.hex"), .RAM_WORDS(24576), .RAM_BANK0(20480), .FW_FILE1("fw_1.hex")) cdb (
    .CLK(clk), .RST_N(rst_n), .RES_N(cdres_n), .CE_R(1'b0), .CE_F(1'b0), .SHCE_R(1'b0), .TCE(tce), .ACE(ace),
    .AA(14'd0), .ADI(16'd0), .ADO(), .ACS2_N(1'b1), .ARD_N(1'b1), .AWRL_N(1'b1), .AWRU_N(1'b1), .AWAIT_N(), .ARQT_N(),
    .DISC_IN(1'b0), .SEC_REQ(), .SEC_LBA(), .SEC_WR(1'b0), .SEC_DATA(16'd0), .SEC_WAIT(),
    .CD_RAM_A(), .CD_RAM_D(), .CD_RAM_WE(), .CD_RAM_RD(), .CD_RAM_CS(), .CD_RAM_Q(16'd0), .CD_RAM_RDY(1'b1),
    .CD_SL(), .CD_SR(),
    .SMPC_A(fw_a), .SMPC_WE(fw_we), .SMPC_WD(fw_wd), .SMPC_RD(fw_rd),
    .EXT_A(ext_a), .EXT_WE(ext_we), .EXT_RE(ext_re), .EXT_WD(ext_wd), .EXT_RD(ext_rd),
    .FAST(1'b0)
);

wire         en, tx, dir, rx, sense;
sat_jvs #(.CLK_HZ(53_748_200)) jvs (.clk(clk), .rst(!rst_n), .on_74(1'b1), .mode(2'b00),
    .ext_we(ext_we), .ext_re(ext_re), .ext_a(ext_a), .ext_wd(ext_wd), .ext_rd(jvs_rd),
    .en(en), .tx(tx), .dir(dir), .rx(rx), .sense(sense), .pads(pads), .joy1(joy1), .joy2(joy2));

wire         de, dtx;
wire         bus = dir ? tx : de ? dtx : 1'b1;
assign rx = dir ? 1'b1 : bus;
wire [31:0]  d_frames, d_bad, d_ans;
wire [7:0]   d_addr, d_nout;
wire [7:0]   d_out [0:7];
// system 00; P1: Start + Push 1 (80 02 -> bytes 82 00), P2: Push 4 (00 40)
jvs_dev dev (.clk(clk), .rst(!rst_n), .bus(bus), .de(de), .txd(dtx), .sense(sense), .div(16'd467), .delay_us(16'd40),
             .mode(8'd0), .sw(64'h0000_0040_0000_8200), .an(128'd0),
             .n_frames(d_frames), .n_bad(d_bad), .n_ans(d_ans), .addr(d_addr), .last_out(d_out), .n_out(d_nout));

// what the firmware does on the JVS words, as it happens
always @(posedge clk) if (ext_we && ext_a == 3'd7) $display("%0t ms: fw writes reg %0d = %08x", $time / 1_000_000, jvs.idx, ext_wd);
always @(d_frames) if (d_frames) $display("%0t ms: board took frame %0d (addr %0d)", $time / 1_000_000, d_frames, d_addr);
reg          seen_en = 0;
always @(posedge clk) if (en && !seen_en) begin seen_en = 1; $display("%0t ms: JVS pins on", $time / 1_000_000); end
integer      writes = 0;
always @(posedge clk) if (ext_we && ext_a[2:1] == 2'b11) writes = writes + 1;

initial begin
    repeat (50) @(posedge clk); rst_n = 1;
    fork
        begin
            wait (pads && joy1 != 16'hFFFF);
            #20_000_000;
            $display("pads on: joy1 %04x joy2 %04x (frames %0d, answers %0d, bad %0d)", joy1, joy2, d_frames, d_ans, d_bad);
            $display("%s", joy1 == 16'hF3FF && joy2 == 16'hFFBF ? "ALL PASS" : "SOME FAILED: pads");
        end
        begin
            #1_500_000_000;
            $display("timeout at 1.5 s: JVS words written %0d, pins %0d, frames %0d, answers %0d, state %0d",
                     writes, seen_en, d_frames, d_ans, jvs.host.st);
            $display("SOME FAILED: timeout");
        end
    join_any
    $finish;
end
endmodule
`default_nettype wire
