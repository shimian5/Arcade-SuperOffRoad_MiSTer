//============================================================================
//  sor_board_tb.sv — integration testbench for rtl/sor_board.sv
//
//  sim/sdram_tb.sv proved rtl/sdram.sv itself correct (4 tests, 20494 checks,
//  0 errors) against a realistic concurrent-access pattern, yet real hardware
//  showed no change at all after fixing a real bug found by that testbench.
//  That means the remaining bug — if any — is in the INTEGRATION layer:
//  sor_board.sv's actual ioctl-to-SDRAM write path, the wr_pending latch, or
//  the checksum scan trigger logic, none of which the controller-only
//  testbench exercises.
//
//  This instantiates sor_board.sv completely unmodified, with its real
//  sor_master/sor_slave Z80 cores running (providing genuine CPU-driven
//  SDRAM contention once reset falls — more faithful than a synthetic
//  hammering loop), and drives realistic HPS ioctl signals to load a
//  the real Master ROM (extracted from offroad.zip, see the 4 filenames
//  below) — including the same toggle-ioctl_download-between-parts behavior
//  the real MRA's 4-file Master ROM entry produces, which an earlier
//  session found and fixed a real bug in (see the "ioctl_download toggles
//  mid-load" note in sor_board.sv).
//
//  After the load finishes and reset falls, it waits for sor_board's
//  own internal readback scanner (the same one driving the on-hardware
//  debug overlay) to finish and checks its result directly via hierarchical
//  reference — no video/overlay rendering needed.
//============================================================================

`timescale 1ns / 1ps

module sor_board_tb;

localparam CLK_PERIOD = 20.83; // 48 MHz

reg clk_sys = 0;
always #(CLK_PERIOD/2) clk_sys = ~clk_sys;

// clk_sdram: mirrors whichever outclk_1 phase the real PLL ladder build
// (docs/sdram_plan.md Section 3a) is currently using. B6 = 60 deg
// (clk_sdram lags clk_sys by 1/6 period, 3.472ns) -- update alongside
// pll_0002.v's phase_shift1 for each later ladder step (B3=180deg was
// `~clk_sys`, B4=0deg was `clk_sys` directly, B5=90deg was a
// quarter-period lag).
reg clk_sdram = 0;
initial begin
	#(CLK_PERIOD/6); // 60 deg = 1/6-period lag
	forever #(CLK_PERIOD/2) clk_sdram = ~clk_sdram;
end

reg reset          = 1;
reg sdram_init     = 1;
reg ioctl_download = 0;
reg [15:0] ioctl_index = 0;
reg        ioctl_wr = 0;
reg [26:0] ioctl_addr = 0;
reg [7:0]  ioctl_data = 0;
wire       ioctl_wait;

wire [15:0] SDRAM_DQ;
wire [12:0] SDRAM_A;
wire  [1:0] SDRAM_BA;
wire        SDRAM_CLK, SDRAM_CKE, SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE, SDRAM_DQML, SDRAM_DQMH;

wire        ce_pix, HBlank, HSync, VBlank, VSync;
wire [23:0] rgb;

// DL_SETTLE_CYCLES defaults to 12,000,000 clk_sys cycles (250ms) on
// real hardware -- sized for worst-case multi-session download
// settling. That's brutal in sim (250ms of simulated Z80 activity
// before either CPU is even released from reset). Override down to
// a few thousand cycles here; synthesis/real hardware still gets the
// module's real default since only this testbench instance overrides it.
sor_board #(.USE_ALTDDIO(1'b0), .DL_SETTLE_CYCLES(24'd10000)) dut
(
	.clk_sys(clk_sys),
	.clk_sdram(clk_sdram),
	.reset(reset),
	.sdram_init(sdram_init),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_data(ioctl_data),
	.ioctl_wait(ioctl_wait),

	.SDRAM_DQ(SDRAM_DQ),
	.SDRAM_A(SDRAM_A),
	.SDRAM_BA(SDRAM_BA),
	.SDRAM_CLK(SDRAM_CLK),
	.SDRAM_CKE(SDRAM_CKE),
	.SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_nWE(SDRAM_nWE),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),

	.ce_pix(ce_pix),
	.HBlank(HBlank), .HSync(HSync), .VBlank(VBlank), .VSync(VSync),
	.rgb(rgb),

	.p1_btn(4'h0), .p2_btn(4'h0), .p3_btn(4'h0),
	// Hardware-idle values (2026-07-17): SuperOffRoad.sv feeds SIGNED
	// MiSTer analog (joyX_ana) into these ports; with no stick input
	// hardware presents 0x00 here, which sor_board's {~msb, [6:0]}
	// conversion turns into 0x80 at the game-visible ADC ports
	// ($FD-$FF). The old 8'h80 stubs were signed -128 (hard-left) and
	// made the game read 0x00 -- the sim never ran with the values
	// real hardware produces. Keep these matched to hardware idle.
	.p1_wheel(8'h00), .p2_wheel(8'h00), .p3_wheel(8'h00),
	.p1_pedal(8'h00), .p2_pedal(8'h00), .p3_pedal(8'h00),

	.service(1'b0),
	.free_play(1'b0)
);

mt48lc16m16a2 chip
(
	.Dq   (SDRAM_DQ),
	.Addr (SDRAM_A),
	.Ba   (SDRAM_BA),
	.Clk  (SDRAM_CLK),
	.Cke  (SDRAM_CKE),
	.Cs_n (SDRAM_nCS),
	.Ras_n(SDRAM_nRAS),
	.Cas_n(SDRAM_nCAS),
	.We_n (SDRAM_nWE),
	.Dqm  ({SDRAM_DQMH, SDRAM_DQML})
);

//------------------------------------------------------------------
// Full flat ROM image loaded in one ioctl_download session, matching
// the MRA's single index="0" entry.  Place all files in sim/.
//
// Flat layout (offsets match sor_board.sv localparam ADDR_* values):
//   0x000000-0x03FFFF  Master Z80 ROM  (4 x 64KB)   -> SDRAM
//   0x040000-0x0BFFFF  Slave  Z80 ROM  (1x8KB + fill + 5x64KB) -> SDRAM
//   0x0C0000-0x1BFFFF  Sound ROM       (3 interleaved lo/hi pairs) -> SDRAM
//   0x1C0000-0x1D7FFF  GFX tile ROM    (3 x 32KB)    -> BRAM
//   0x1D8000-0x1F7FFF  BG palette PROM (4 x 16KB + fills) -> BRAM
//------------------------------------------------------------------
integer i;
integer fd, rd_count;

// Max single-file size we need to buffer (64 KB is largest single part)
localparam BUF_LEN = 65536;
reg [7:0] file_buf [0:BUF_LEN-1];

// Helper: write N zero bytes starting at flat_addr (for fills / zero regions)
task ioctl_fill_zero(input [26:0] start_addr, input integer len);
	integer j;
	begin
		ioctl_seed_addr(start_addr);
		for (j = 0; j < len; j = j + 1)
			ioctl_write_byte(start_addr + j, 8'h00);
	end
endtask

`ifdef GAMEPLAY_REPRO
//------------------------------------------------------------------
// Fast-forward hack (2026-07-12 session): reaching the real-hardware
// gameplay hang at Master PC $BDAB by actually simulating attract mode
// takes ~230 emulated seconds (~11 billion clk_sys cycles) in real
// MAME -- hours of wall-clock RTL sim, impractical to brute-force.
//
// Instead, force-inject a genuine mid-gameplay snapshot (WRAM +
// full tv80 register state, both CPUs) captured from a real MAME save
// state the user played up to (coin, start, name entry, into actual
// driving). See docs/SESSION_2026-07-12_FINDINGS.md for how this was
// captured and verified (confirmed bank_reg==0 at capture time via a
// byte-exact match of the live $2000-$9FFF window against the flat
// ROM image -- matches our RTL's power-on-reset default, so no
// bank-switch state needs forcing).
//
// tv80's register file (rtl/tv80/rtl/core/tv80_reg.v) has NO reset of
// its own -- RegsH/RegsL are just static memory cells, safe to deposit
// directly at any time with a plain hierarchical assignment. PC, SP,
// ACC, F, Ap, Fp, I, IntE_FF1/2, and IStatus DO get asynchronously
// reset by tv80_core.v's `posedge clk or negedge reset_n` block, so
// those need force/release, timed to land after reset_n has risen but
// before the first real posedge clk_sys resumes normal fetch (i.e.
// called directly after `reset <= 1'b0` with no intervening clock
// edge) -- force then immediately release is equivalent to a clean
// deposit here since nothing else is scheduled to drive them in that
// window.
//------------------------------------------------------------------
task inject_gameplay_snapshot;
	integer fd, rd_count;
	begin
		fd = $fopen("bdab_wram_master.bin", "rb");
		rd_count = $fread(dut.wram_m.mem, fd);
		$fclose(fd);
		if (rd_count != 4096) begin
			$display("ERROR: bdab_wram_master.bin read %0d bytes, expected 4096", rd_count);
			$finish;
		end
		fd = $fopen("bdab_wram_slave.bin", "rb");
		rd_count = $fread(dut.wram_s.mem, fd);
		$fclose(fd);
		if (rd_count != 4096) begin
			$display("ERROR: bdab_wram_slave.bin read %0d bytes, expected 4096", rd_count);
			$finish;
		end

		// Captured live, right at the $BDAB breakpoint itself (zero-input
		// attract-demo run) -- see docs/reference/mame/traces/bdab_dasm.asm
		// and docs/SESSION_2026-07-12_FINDINGS.md. Bank window verified
		// bank_reg==2 (byte-exact against the flat ROM once the
		// cpu_addr[14:0]-masking fix above is applied to both halves of
		// the $2000-$9FFF window) -- forced explicitly below since it
		// won't match sor_master.sv's power-on-reset default of 0.
		//
		// Master: PC=BDAB SP=EFF8 AF=0500 BC=403F DE=0005 HL=01F8
		//         IX=E180 IY=E66A AF2=6024 BC2=FF0B DE2=4427 HL2=2566
		//         I=00 IM=1 IFF1=1 IFF2=1
		force dut.master.master_cpu.i_tv80_core.PC       = 16'hBDAB;
		force dut.master.master_cpu.i_tv80_core.SP       = 16'hEFF8;
		force dut.master.master_cpu.i_tv80_core.ACC      = 8'h05;
		force dut.master.master_cpu.i_tv80_core.F        = 8'h00;
		force dut.master.master_cpu.i_tv80_core.Ap       = 8'h60;
		force dut.master.master_cpu.i_tv80_core.Fp       = 8'h24;
		force dut.master.master_cpu.i_tv80_core.I        = 8'h00;
		force dut.master.master_cpu.i_tv80_core.IntE_FF1 = 1'b1;
		force dut.master.master_cpu.i_tv80_core.IntE_FF2 = 1'b1;
		force dut.master.master_cpu.i_tv80_core.IStatus  = 2'b01;
		dut.master.master_cpu.i_tv80_core.i_reg.RegsH[0] = 8'h40; // B
		dut.master.master_cpu.i_tv80_core.i_reg.RegsL[0] = 8'h3F; // C
		dut.master.master_cpu.i_tv80_core.i_reg.RegsH[1] = 8'h00; // D
		dut.master.master_cpu.i_tv80_core.i_reg.RegsL[1] = 8'h05; // E
		dut.master.master_cpu.i_tv80_core.i_reg.RegsH[2] = 8'h01; // H
		dut.master.master_cpu.i_tv80_core.i_reg.RegsL[2] = 8'hF8; // L
		dut.master.master_cpu.i_tv80_core.i_reg.RegsH[3] = 8'hE1; // IXH
		dut.master.master_cpu.i_tv80_core.i_reg.RegsL[3] = 8'h80; // IXL
		dut.master.master_cpu.i_tv80_core.i_reg.RegsH[4] = 8'hFF; // B'
		dut.master.master_cpu.i_tv80_core.i_reg.RegsL[4] = 8'h0B; // C'
		dut.master.master_cpu.i_tv80_core.i_reg.RegsH[5] = 8'h44; // D'
		dut.master.master_cpu.i_tv80_core.i_reg.RegsL[5] = 8'h27; // E'
		dut.master.master_cpu.i_tv80_core.i_reg.RegsH[6] = 8'h25; // H'
		dut.master.master_cpu.i_tv80_core.i_reg.RegsL[6] = 8'h66; // L'
		dut.master.master_cpu.i_tv80_core.i_reg.RegsH[7] = 8'hE6; // IYH
		dut.master.master_cpu.i_tv80_core.i_reg.RegsL[7] = 8'h6A; // IYL
		force dut.master.bank_reg = 3'd2;

		// Slave: PC=0CDF SP=EFFC AF=DE0A BC=DEE5 DE=1C22 HL=E897
		//        IX=FFFF IY=0405 AF2=F7A2 BC2=E8EC DE2=41AB HL2=EC99
		//        I=00 IM=1 IFF1=1 IFF2=1
		force dut.slave.slave_cpu.i_tv80_core.PC       = 16'h0CDF;
		force dut.slave.slave_cpu.i_tv80_core.SP       = 16'hEFFC;
		force dut.slave.slave_cpu.i_tv80_core.ACC      = 8'hDE;
		force dut.slave.slave_cpu.i_tv80_core.F        = 8'h0A;
		force dut.slave.slave_cpu.i_tv80_core.Ap       = 8'hF7;
		force dut.slave.slave_cpu.i_tv80_core.Fp       = 8'hA2;
		force dut.slave.slave_cpu.i_tv80_core.I        = 8'h00;
		force dut.slave.slave_cpu.i_tv80_core.IntE_FF1 = 1'b1;
		force dut.slave.slave_cpu.i_tv80_core.IntE_FF2 = 1'b1;
		force dut.slave.slave_cpu.i_tv80_core.IStatus  = 2'b01;
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsH[0] = 8'hDE; // B
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsL[0] = 8'hE5; // C
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsH[1] = 8'h1C; // D
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsL[1] = 8'h22; // E
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsH[2] = 8'hE8; // H
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsL[2] = 8'h97; // L
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsH[3] = 8'hFF; // IXH
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsL[3] = 8'hFF; // IXL
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsH[4] = 8'hE8; // B'
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsL[4] = 8'hEC; // C'
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsH[5] = 8'h41; // D'
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsL[5] = 8'hAB; // E'
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsH[6] = 8'hEC; // H'
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsL[6] = 8'h99; // L'
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsH[7] = 8'h04; // IYH
		dut.slave.slave_cpu.i_tv80_core.i_reg.RegsL[7] = 8'h05; // IYL

		release dut.master.master_cpu.i_tv80_core.PC;
		release dut.master.master_cpu.i_tv80_core.SP;
		release dut.master.master_cpu.i_tv80_core.ACC;
		release dut.master.master_cpu.i_tv80_core.F;
		release dut.master.master_cpu.i_tv80_core.Ap;
		release dut.master.master_cpu.i_tv80_core.Fp;
		release dut.master.master_cpu.i_tv80_core.I;
		release dut.master.master_cpu.i_tv80_core.IntE_FF1;
		release dut.master.master_cpu.i_tv80_core.IntE_FF2;
		release dut.master.master_cpu.i_tv80_core.IStatus;
		release dut.master.bank_reg;
		release dut.slave.slave_cpu.i_tv80_core.PC;
		release dut.slave.slave_cpu.i_tv80_core.SP;
		release dut.slave.slave_cpu.i_tv80_core.ACC;
		release dut.slave.slave_cpu.i_tv80_core.F;
		release dut.slave.slave_cpu.i_tv80_core.Ap;
		release dut.slave.slave_cpu.i_tv80_core.Fp;
		release dut.slave.slave_cpu.i_tv80_core.I;
		release dut.slave.slave_cpu.i_tv80_core.IntE_FF1;
		release dut.slave.slave_cpu.i_tv80_core.IntE_FF2;
		release dut.slave.slave_cpu.i_tv80_core.IStatus;

		// /MCONT shadow register (rtl/sor_master.sv:513) is never written
		// by boot code in this fast-forward path (we skip boot entirely),
		// so slave_reset_n (mcont_r[0]) would otherwise stay 0 forever,
		// holding the Slave CPU in permanent reset. Real hardware would
		// already have this set from early boot by the time gameplay is
		// running -- force the two bits that matter for normal operation:
		// bit0=1 (Slave running), bit2=1 (Slave NMI held clear). EEPROM
		// control bits (4-6) left 0; this object-update code path doesn't
		// touch the EEPROM, so low risk leaving them at reset default.
		force dut.master.mcont_r = 8'b0000_0101;
		release dut.master.mcont_r;

		$display("t=%0t === Injected gameplay snapshot: master PC=BDAB slave PC=0CDF ===", $time);
	end
endtask
`endif

// Helper: read a ROM file and stream it starting at flat_addr
task ioctl_load_file(input [8*32-1:0] fname, input [26:0] flat_addr, input integer expected_len);
	integer j;
	begin
		fd = $fopen(fname, "rb");
		if (fd == 0) begin
			$display("ERROR: could not open %0s -- place all offroad.zip ROM files in sim/", fname);
			$finish;
		end
		rd_count = $fread(file_buf, fd);
		$fclose(fd);
		if (rd_count != expected_len) begin
			$display("ERROR: %0s read %0d bytes, expected %0d", fname, rd_count, expected_len);
			$finish;
		end
		ioctl_seed_addr(flat_addr);
		for (j = 0; j < expected_len; j = j + 1)
			ioctl_write_byte(flat_addr + j, file_buf[j]);
	end
endtask

// Helper: read a LOW/HIGH byte-pair of 64KB ROM files and stream them
// interleaved starting at flat_addr, exactly matching
// mra/SuperOffRoad.mra's own <interleave output="16"> map="01"/map="10"
// pairing for the Sound ROM (word k: low file's byte k at flat_addr+2k,
// high file's byte k at flat_addr+2k+1) -- same interleaving
// sim/sor_sound_tb.sv's own load_pair task already reconstructs
// manually for its direct-array load; this is the ioctl-stream
// equivalent, needed because ioctl_load_file above only ever streams
// one already-flat file verbatim.
reg [7:0] pair_lo_buf [0:65535];
reg [7:0] pair_hi_buf [0:65535];
task ioctl_load_pair(input [8*32-1:0] lo_fname, input [8*32-1:0] hi_fname, input [26:0] flat_addr);
	integer lo_fd, hi_fd, lo_count, hi_count, k;
	begin
		lo_fd = $fopen(lo_fname, "rb");
		if (lo_fd == 0) begin
			$display("ERROR: could not open %0s -- place all offroad.zip ROM files in sim/", lo_fname);
			$finish;
		end
		lo_count = $fread(pair_lo_buf, lo_fd);
		$fclose(lo_fd);
		hi_fd = $fopen(hi_fname, "rb");
		if (hi_fd == 0) begin
			$display("ERROR: could not open %0s -- place all offroad.zip ROM files in sim/", hi_fname);
			$finish;
		end
		hi_count = $fread(pair_hi_buf, hi_fd);
		$fclose(hi_fd);
		if (lo_count != 65536 || hi_count != 65536) begin
			$display("ERROR: %0s/%0s read %0d/%0d bytes, expected 65536/65536",
			          lo_fname, hi_fname, lo_count, hi_count);
			$finish;
		end
		ioctl_seed_addr(flat_addr);
		for (k = 0; k < 65536; k = k + 1) begin
			ioctl_write_byte(flat_addr + {k[15:0], 1'b0},       pair_lo_buf[k]);
			ioctl_write_byte(flat_addr + {k[15:0], 1'b0} + 1'b1, pair_hi_buf[k]);
		end
	end
endtask

// Pre-seeds ioctl_addr to a session's starting address, one cycle before
// that session's first ioctl_write_byte call -- required by the
// pre-advance scheme below (see ioctl_write_byte comment).
task ioctl_seed_addr(input [26:0] a);
	begin
		@(posedge clk_sys);
		ioctl_addr <= a;
	end
endtask

// Real hps_io (sys/hps_io.sv) has a documented quirk, reproduced here
// on purpose (see the matching root-cause comment on ioctl_addr_d1 in
// rtl/sor_board.sv): the externally-visible `ioctl_wr` strobe is a
// registered, one-cycle-delayed copy of an internal `wr`, but
// `ioctl_addr` advances in the SAME cycle `wr` is set, not delayed to
// match -- so by the time `ioctl_wr` reads 1 externally, `ioctl_addr`
// has ALREADY advanced to the NEXT byte's address, and `ioctl_data`
// still holds the CURRENT byte's data. sor_board.sv's `ioctl_addr_d1`
// (a plain one-cycle-delayed register of ioctl_addr) exists specifically
// to undo this and recover the true address of ioctl_data.
//
// This task previously drove ioctl_addr/ioctl_data/ioctl_wr all from
// the SAME (correct, unshifted) address in lockstep -- which does NOT
// exercise that quirk, so ioctl_addr_d1 ended up sampling the PREVIOUS
// byte's address instead of the current one, silently corrupting every
// byte after the first in a session by pairing it with the wrong
// address (confirmed via a direct SDRAM-write-FIFO diagnostic: byte
// 0x38's real data 0xC3 was landing under FIFO address label 0x37).
// Pre-advancing ioctl_addr here (to addr+1, the NEXT byte) replicates
// the real quirk so ioctl_addr_d1 recovers the correct address, exactly
// as it does against real hardware. Callers must ioctl_seed_addr() the
// session's start address one cycle before the first call so that first
// byte's ioctl_addr_d1 also resolves correctly.
task ioctl_write_byte(input [26:0] addr, input [7:0] data);
	begin
		@(posedge clk_sys);
		while (ioctl_wait) @(posedge clk_sys);
		ioctl_data <= data;
		ioctl_addr <= addr + 1'b1;
		ioctl_wr   <= 1'b1;
		@(posedge clk_sys);
		ioctl_wr   <= 1'b0;
	end
endtask

//------------------------------------------------------------------
// Master CPU boot trace: watches every I/O write the Master Z80
// makes (port address + data) and flags the two we care about most —
// bank register (0xF0) and /MCONT (0x09, controls slave_reset_n) —
// so we can see directly whether/when boot code reaches them, instead
// of inferring it from the on-hardware debug overlay.
//------------------------------------------------------------------
integer io_wr_count = 0;
always @(posedge clk_sys) begin
	if (dut.master.CE_6M && dut.master.io_wr) begin
		io_wr_count = io_wr_count + 1;
		$display("t=%0t IOWR #%0d addr=0x%02x data=0x%02x pc=0x%04x%s%s",
		          $time, io_wr_count, dut.master.cpu_addr[7:0], dut.master.cpu_dout, dut.dbg_pc,
		          dut.master.io_bank  ? "  <- BANK"  : "",
		          dut.master.io_mcont ? "  <- MCONT" : "");
	end
end

// I/O read trace: mirrors the write trace above but for reads, since a
// polling loop waiting on an input port (EEPROM/status/watchdog bit)
// would be completely invisible to the write-only trace. Only logs
// once reset has fallen, and only when the (port,PC) pair changes from
// the last read -- a long repeating poll loop logs once instead of
// flooding thousands of identical lines, but any new read elsewhere in
// the run (a different poll, or real progress) still shows up.
integer io_rd_count = 0;
reg [7:0]  last_io_rd_addr = 8'hzz;
reg [15:0] last_io_rd_pc   = 16'hzzzz;
always @(posedge clk_sys) begin
	if (!reset && dut.master.CE_6M && dut.master.io_rd &&
	    (dut.master.cpu_addr[7:0] != last_io_rd_addr || dut.dbg_pc != last_io_rd_pc)) begin
		io_rd_count = io_rd_count + 1;
		$display("t=%0t IORD #%0d addr=0x%02x data=0x%02x pc=0x%04x",
		          $time, io_rd_count, dut.master.cpu_addr[7:0], dut.master.cpu_din, dut.dbg_pc);
		last_io_rd_addr = dut.master.cpu_addr[7:0];
		last_io_rd_pc   = dut.dbg_pc;
	end
end

// Slave VRAM write trace: log every VRAM write the Slave makes (addr+data).
// First 32 writes logged individually; thereafter a periodic count so we
// can see the Slave is alive without flooding the transcript.
integer vram_wr_count = 0;
always @(posedge clk_sys) begin
	if (dut.vram_we_cpu) begin
		vram_wr_count = vram_wr_count + 1;
		if (vram_wr_count <= 8 || (vram_wr_count % 20000 == 0))
			$display("t=%0t VRAM_WR #%0d addr=0x%05x data=0x%02x",
			          $time, vram_wr_count, dut.vram_addr_cpu, dut.vram_din_cpu);
	end
end

// Slave PC latch (tapped hierarchically -- kept even though sor_slave.sv
// now also exposes a dbg_pc output of its own (Follow-up 6), since this
// local copy predates that port and several other monitors below already
// reference it; the two are driven by the identical M1-fetch condition
// so they always agree).
reg [15:0] slave_dbg_pc;
always @(posedge clk_sys)
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n) slave_dbg_pc <= dut.slave.cpu_addr;

// Follow-up 6 (docs/SESSION_2026-07-14.md), Task 2 -- organic bank-switch
// monitor: does a real `ld ($C000),a` executed by the tv80 core actually
// latch dut.slave.bank_reg, with no bus forcing involved? Cheap $display
// on every bank_reg value change (time, new value, slave PC) plus a
// saturating count, so the transcript answers "did it ever change" at a
// glance without scrolling.
reg [3:0] bank_reg_prev;
integer   bank_reg_change_count = 0;
initial   bank_reg_prev = 4'd0;
always @(posedge clk_sys) begin
	if (dut.slave.bank_reg !== bank_reg_prev) begin
		if (bank_reg_change_count != 32'h7FFFFFFF) bank_reg_change_count = bank_reg_change_count + 1;
		$display("t=%0t BANK_REG_CHANGE #%0d %0d -> %0d slavePC=0x%04x",
		          $time, bank_reg_change_count, bank_reg_prev, dut.slave.bank_reg, slave_dbg_pc);
		bank_reg_prev = dut.slave.bank_reg;
	end
end

// First-occurrence evidence for the other half of Task 2's question:
// does a subsequent banked-window (0x10000-0x7FFFF via in_banked) ROM
// read actually return nonzero data once bank_reg has organically
// changed? dut.slave.dbg_banked_read_ever is the sticky flag (new
// sor_slave.sv output); log the exact moment/address/data it first
// goes high.
reg banked_read_ever_prev;
initial banked_read_ever_prev = 1'b0;
always @(posedge clk_sys) begin
	if (dut.slave.dbg_banked_read_ever && !banked_read_ever_prev) begin
		$display("t=%0t FIRST_BANKED_ROM_READ bank_reg=%0d rom_addr=0x%05x rom_data=0x%02x slavePC=0x%04x",
		          $time, dut.slave.bank_reg, dut.slave.rom_addr, dut.slave.rom_data, slave_dbg_pc);
	end
	banked_read_ever_prev = dut.slave.dbg_banked_read_ever;
end

// Follow-up 9b -- does the Slave EVER execute the real title-art blit
// routine at all? MAME's fg_bitmap_pc_histogram.lua (C:\MiSTerDev\mame)
// pinpoints the routine that produces the bulk of MAME's nonzero
// fg-bitmap writes to Slave PC 0x07c3-0x0808 (a long unrolled OUTI burst
// in the Slave's FIXED ROM, ~1074 hits per PC address, ~80-85% nonzero) --
// this is disassembled as a straightforward "OUTI x40, then re-latch the
// VRAM dest address and advance the ROM source pointer, repeat" blit
// loop (sim/03-22100-02.u3 disassembly, 0x0700-0x0850ish). Continuous
// (not periodically-sampled) check for whether our RTL's Slave M1-fetch
// PC ever lands in this exact range at all.
reg [15:0] outi_blit_visits = 0;
reg        outi_blit_seen = 1'b0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n &&
	    (dut.slave.cpu_addr >= 16'h0700) && (dut.slave.cpu_addr <= 16'h0850)) begin
		if (outi_blit_visits != 16'hFFFF) outi_blit_visits = outi_blit_visits + 1;
		if (!outi_blit_seen) begin
			outi_blit_seen = 1'b1;
			$display("t=%0t OUTI_BLIT_FIRST_VISIT slavePC=0x%04x", $time, dut.slave.cpu_addr);
		end
	end
end

// Follow-up 9c -- OUTI_BLIT_SUMMARY came back visited=0/m1_visits=0: the
// Slave's dispatcher runs (bank_wr_cnt=0x29=41 write events per
// SLAVE_DBG_FINAL) but never takes the branch leading to the sequential-
// OUTI blit at 0x0700-0x0850. The dispatcher (entry ~0x0482, containing
// the confirmed-executing bank-switch site at 0x05B9) has (at least) two
// other exit branches per the u3 disassembly:
//   0x080A/0x08AD -> falls through to an indexed LD(DE)/OUT($5B/$5D)
//                     blit body at 0x0C80-0x0C9F (a masked/scaled variant,
//                     distinct mechanism from the plain OUTI burst)
//   entry loop itself: 0x0482/0x049F (iy-vectored dispatch), 0x04A5,
//                     0x04C3-ish (per-tick object-walk entry, called from
//                     the INT handler's normal per-tick path)
// Track each candidate region separately so we can see exactly which
// path this ROM's execution actually takes when the dispatcher runs.
reg [15:0] disp_entry_visits = 0;   // 0x0482-0x04C5: the object-walk loop entry/dispatch-vector itself
reg [15:0] disp_body_visits  = 0;   // 0x0541-0x05FF: clipping + the confirmed 0x05B9 bank-switch dispatcher body
reg [15:0] outi_setup_visits = 0;   // 0x05EC-0x06FF: OUTI-path clip/setup (leads into 0x0700 if taken)
reg [15:0] idx_setup_visits  = 0;   // 0x0802-0x089F: indexed-path setup (0x080A/0x08AD branch target)
reg [15:0] idx_blit_visits   = 0;   // 0x0C80-0x0CFF: indexed LD/OUT blit body
initial begin
	disp_entry_visits = 0; disp_body_visits = 0; outi_setup_visits = 0;
	idx_setup_visits = 0; idx_blit_visits = 0;
end
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n) begin
		if (dut.slave.cpu_addr >= 16'h0482 && dut.slave.cpu_addr <= 16'h04C5 && disp_entry_visits != 16'hFFFF)
			disp_entry_visits = disp_entry_visits + 1;
		if (dut.slave.cpu_addr >= 16'h0541 && dut.slave.cpu_addr <= 16'h05FF && disp_body_visits != 16'hFFFF)
			disp_body_visits = disp_body_visits + 1;
		if (dut.slave.cpu_addr >= 16'h05EC && dut.slave.cpu_addr <= 16'h06FF && outi_setup_visits != 16'hFFFF)
			outi_setup_visits = outi_setup_visits + 1;
		if (dut.slave.cpu_addr >= 16'h0802 && dut.slave.cpu_addr <= 16'h089F && idx_setup_visits != 16'hFFFF)
			idx_setup_visits = idx_setup_visits + 1;
		if (dut.slave.cpu_addr >= 16'h0C80 && dut.slave.cpu_addr <= 16'h0CFF && idx_blit_visits != 16'hFFFF)
			idx_blit_visits = idx_blit_visits + 1;
	end
end

// Follow-up 9g -- $EF19 (object-list HEAD pointer, `ld hl,($EF19)` at
// 0x03d4/0x03de) and $EF37 (current-object shadow, `ld ($EF37),hl` at
// 0x04a5) -- MAME's ef19_pointer_trace.lua shows EF19 alternating
// between two list-head pages (e8f8/eaf8, presumably a double-buffered
// object list) roughly every 1-2s. Log on change only, mirroring MAME's
// script, to see whether/how ours moves.
reg [15:0] ef19_prev, ef37_prev;
initial begin ef19_prev = 16'hFFFF; ef37_prev = 16'hFFFF; end
wire [15:0] ef19_cur = {dut.wram_s.mem[16'hF1A], dut.wram_s.mem[16'hF19]};
wire [15:0] ef37_cur = {dut.wram_s.mem[16'hF38], dut.wram_s.mem[16'hF37]};
always @(posedge clk_sys) begin
	if (ef19_cur !== ef19_prev || ef37_cur !== ef37_prev) begin
		$display("t=%0t EF19_EF37_CHANGE EF19(head)=0x%04x EF37(cur)=0x%04x", $time, ef19_cur, ef37_cur);
		ef19_prev <= ef19_cur;
		ef37_prev <= ef37_cur;
	end
end

// Follow-up 9f -- periodic WRAM content dump at the object-table
// addresses MAME's obj_attr_read_trace/wram_e9_eb_dump identified
// (E900-E910, E9F0-EA10, EAF0-EB10), mirroring
// C:\MiSTerDev\mame\wram_e9_eb_dump.lua, to see whether our RTL's WRAM
// content at these addresses matches MAME's or diverges (either in the
// attribute/discriminator bytes themselves, or in whatever "next
// object" chain-pointer field determines list advancement).
time wram_dump_last = 0;
always @(posedge clk_sys) begin
	if ($time - wram_dump_last >= 100_000_000) begin // every 100ms
		wram_dump_last = $time;
		$display("t=%0t WRAM_DUMP E900-E90F: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
		          $time,
		          dut.wram_s.mem[16'h900], dut.wram_s.mem[16'h901], dut.wram_s.mem[16'h902], dut.wram_s.mem[16'h903],
		          dut.wram_s.mem[16'h904], dut.wram_s.mem[16'h905], dut.wram_s.mem[16'h906], dut.wram_s.mem[16'h907],
		          dut.wram_s.mem[16'h908], dut.wram_s.mem[16'h909], dut.wram_s.mem[16'h90a], dut.wram_s.mem[16'h90b],
		          dut.wram_s.mem[16'h90c], dut.wram_s.mem[16'h90d], dut.wram_s.mem[16'h90e], dut.wram_s.mem[16'h90f]);
		$display("t=%0t WRAM_DUMP E9F0-EA0F: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
		          $time,
		          dut.wram_s.mem[16'h9f0], dut.wram_s.mem[16'h9f1], dut.wram_s.mem[16'h9f2], dut.wram_s.mem[16'h9f3],
		          dut.wram_s.mem[16'h9f4], dut.wram_s.mem[16'h9f5], dut.wram_s.mem[16'h9f6], dut.wram_s.mem[16'h9f7],
		          dut.wram_s.mem[16'h9f8], dut.wram_s.mem[16'h9f9], dut.wram_s.mem[16'h9fa], dut.wram_s.mem[16'h9fb],
		          dut.wram_s.mem[16'h9fc], dut.wram_s.mem[16'h9fd], dut.wram_s.mem[16'h9fe], dut.wram_s.mem[16'h9ff]);
		$display("t=%0t WRAM_DUMP EAF0-EB0F: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
		          $time,
		          dut.wram_s.mem[16'haf0], dut.wram_s.mem[16'haf1], dut.wram_s.mem[16'haf2], dut.wram_s.mem[16'haf3],
		          dut.wram_s.mem[16'haf4], dut.wram_s.mem[16'haf5], dut.wram_s.mem[16'haf6], dut.wram_s.mem[16'haf7],
		          dut.wram_s.mem[16'haf8], dut.wram_s.mem[16'haf9], dut.wram_s.mem[16'hafa], dut.wram_s.mem[16'hafb],
		          dut.wram_s.mem[16'hafc], dut.wram_s.mem[16'hafd], dut.wram_s.mem[16'hafe], dut.wram_s.mem[16'haff]);
	end
end

// Follow-up 10d -- direct register capture at PC 0x1220 (the `ld (hl),a`
// that writes the E900 discriminator byte), mirroring
// C:\MiSTerDev\mame\e900_register_trace.lua's direct MAME register read.
// MAME's real values at this exact instruction: A=0x04 B=0x04 C=0x08
// HL=0xe900 DE=0x4000 -- hand-disassembly arithmetic starting from the
// confirmed command-byte reads predicted B=0x00, contradicting this
// measurement, so getting our RTL's own live register values here
// (rather than continuing error-prone manual derivation) directly
// localizes the divergence. tv80_reg.v exposes named register wires
// (B/C/D/E/H/L) on its i_reg instance inside tv80_core.
integer f10d_count = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n && (dut.slave.cpu_addr == 16'h1220)) begin
		f10d_count = f10d_count + 1;
		if (f10d_count <= 20)
			$display("t=%0t F10D_REG_AT_1220 #%0d A=0x%02x B=0x%02x C=0x%02x H=0x%02x L=0x%02x D=0x%02x E=0x%02x",
			          $time, f10d_count,
			          dut.slave.slave_cpu.i_tv80_core.ACC,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.B,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.C,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.H,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.L,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.D,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.E);
	end
end

// Follow-up 10e -- backward register trace across the whole 0x1115-0x1220
// chain, mirroring mame/b_register_backtrace.lua's 8 checkpoints exactly,
// to find the FIRST instruction where our RTL's register state departs
// from MAME's known-correct sequence:
//   1115: A=40 B=02 C=e8 HL=e800 DE=4000
//   11f0: A=04 B=02 C=00 HL=e900 DE=4000
//   11fe: A=04 B=02 C=00
//   1201: A=08 B=04 C=08
//   1203: A=00 B=04 C=08
//   1204: A=04 B=04 C=08
//   1207: A=00 B=04 C=08
//   1220: A=04 B=04 C=08
function automatic int unsigned f10e_idx(input [15:0] pc);
	case (pc)
		16'h1115: f10e_idx = 0; 16'h11f0: f10e_idx = 1; 16'h11fe: f10e_idx = 2;
		16'h1201: f10e_idx = 3; 16'h1203: f10e_idx = 4; 16'h1204: f10e_idx = 5;
		16'h1207: f10e_idx = 6; 16'h1220: f10e_idx = 7;
		default:  f10e_idx = 8;
	endcase
endfunction
integer f10e_hits [0:8];
integer f10e_i;
initial for (f10e_i = 0; f10e_i < 9; f10e_i = f10e_i + 1) f10e_hits[f10e_i] = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n && (f10e_idx(dut.slave.cpu_addr) != 8)) begin
		f10e_hits[f10e_idx(dut.slave.cpu_addr)] = f10e_hits[f10e_idx(dut.slave.cpu_addr)] + 1;
		if (f10e_hits[f10e_idx(dut.slave.cpu_addr)] <= 6)
			$display("t=%0t F10E_REG PC=0x%04x #%0d A=0x%02x B=0x%02x C=0x%02x HL=0x%02x%02x DE=0x%02x%02x",
			          $time, dut.slave.cpu_addr, f10e_hits[f10e_idx(dut.slave.cpu_addr)],
			          dut.slave.slave_cpu.i_tv80_core.ACC,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.B,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.C,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.H, dut.slave.slave_cpu.i_tv80_core.i_reg.L,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.D, dut.slave.slave_cpu.i_tv80_core.i_reg.E);
	end
end

// Follow-up 9e -- what is HL (the object-table pointer) and the actual
// discriminator byte at (HL) each time the dispatcher reaches 0x05ab?
// This is the byte whose bits 1-3 (masked 0x0E, then rrca/dec a/jp z at
// 0x05bc-0x05c2) determine sprite-draw (0x05ec) vs housekeeping
// (0x152e) -- our RTL takes the housekeeping branch 100% of the time
// (Follow-up 9d), so this traces WHAT that byte's value actually is and
// WHERE (HL) points, to see whether the object table itself lacks real
// sprite entries or whether HL addresses the wrong place.
reg armed_05ab;
integer obj_read_count = 0;
initial armed_05ab = 1'b0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n && (dut.slave.cpu_addr == 16'h05ab))
		armed_05ab <= 1'b1;
	else if (dut.slave.CE_6M && dut.slave.mem_access && ~dut.slave.rd_n && dut.slave.m1_n && armed_05ab) begin
		armed_05ab <= 1'b0;
		obj_read_count = obj_read_count + 1;
		if (obj_read_count <= 40)
			$display("t=%0t OBJ_ATTR_READ #%0d HL=0x%04x attr_byte=0x%02x discrim(attr&0x0E)=0x%02x",
			          $time, obj_read_count, dut.slave.cpu_addr, dut.slave.cpu_din, dut.slave.cpu_din & 8'h0E);
	end
end

// Follow-up 10 -- command-queue VRAM-port op trace, both sides of the
// Master->Slave sprite-command mailbox. MAME ground truth (mame/
// slave_cmdread_trace.lua + master_comm_write_trace.lua): each frame the
// Master writes the per-frame status commands via port 0x0B at PC
// ~0x61B0, and from frame 14 an 8-byte sprite command "02 08 40 40 0f
// 00 16 00" at PC 0x624D-0x6261; the Slave's parser polls in a,($4B)
// at PC 0x10CC (returns 0x00 while empty), then consumes the command
// (reads at 0x1107/0x1114/0x1232/0x124B/0x12D9...) and inserts the
// sprite object (discrim 0x04) the dispatcher then draws. Our RTL's
// Master demonstrably writes the same bytes (IOWR pc=0x624b...) but the
// Slave never inserts the object, so this logs the actual VRAM
// addresses+data of every Slave port READ op (vp_pop_s) and of the
// Master port WRITE ops issued from the two command writers, to see
// where the two sides' queue pointers/data diverge.
integer f10_srd_count = 0;
integer f10_mwr_count = 0;
always @(posedge clk_sys) begin
	// Slave-side: every VRAM-port read op the sequencer completes.
	// q0_* still hold the op being popped at this edge; vp_rdata_s was
	// assigned on the same edge that raised vp_pop_s, so it is the
	// returned data. slave_dbg_pc identifies the reader.
	if (dut.vp_pop_s && dut.slave.vport.q0_rd) begin
		f10_srd_count = f10_srd_count + 1;
		if (f10_srd_count <= 600 || dut.vp_rdata_s != 8'h00 || (f10_srd_count % 50 == 0))
			$display("t=%0t F10_SRD #%0d vaddr=0x%04x data=0x%02x spc=0x%04x",
			          $time, f10_srd_count, dut.slave.vport.q0_a, dut.vp_rdata_s, slave_dbg_pc);
	end
	// Master-side: every VRAM-port write op issued while the Master is
	// executing either command writer (0x61A0-0x61C8 status commands,
	// 0x6238-0x6290 sprite command). q0_a is the actual VRAM address.
	if (dut.vp_pop_m && !dut.master.mvport.q0_rd &&
	    ((dut.dbg_pc >= 16'h61A0 && dut.dbg_pc <= 16'h61C8) ||
	     (dut.dbg_pc >= 16'h6238 && dut.dbg_pc <= 16'h6290))) begin
		f10_mwr_count = f10_mwr_count + 1;
		if (f10_mwr_count <= 2500)
			$display("t=%0t F10_MWR #%0d vaddr=0x%04x data=0x%02x mpc=0x%04x",
			          $time, f10_mwr_count, dut.master.mvport.q0_a, dut.master.mvport.q0_d, dut.dbg_pc);
	end
end

// Follow-up 10b -- object-record WRAM traffic. The Follow-up 10 run
// proved the Slave consumes the Master's 8-byte sprite command
// (02 08 40 40 0f 00 16 00 read from VRAM 0xF000 at the same insertion
// PCs as MAME) every frame from t~370ms, yet the dispatcher never
// visits HL=0xE900/0xEB00 (OBJ_ATTR_READ stays 0xE9F8). MAME's writer
// trace (mame/objtable_writer_trace.log) shows insertion writes
// E900=04/E901=80/E902/E903 + link-node writes at E800/E801, and the
// dispatcher's list walk (0x0466: inc l; ld l,(hl); inc l; jr nz)
// terminates on link byte 0xFF -- so reaching E900 requires the head
// node's link byte (0xE8F9) to be relinked from 0xFF to 0x00. This logs
// every Slave WRAM write to the object pages 0xE800-0xEBFF plus the
// head-node bytes 0xE8F8-0xE8FF, and every WRAM read of the candidate
// attr/link bytes, to see whether the insertion's writes land and
// whether the dispatcher ever walks past the head.
integer f10b_wr_count = 0;
integer f10b_rd_count = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && dut.wram_we_s &&
	    (dut.wram_addr_s >= 12'h800 && dut.wram_addr_s <= 12'hBFF) &&
	    ((dut.wram_addr_s[7:0] <= 8'h07) || (dut.wram_addr_s[11:4] == 8'h8F))) begin
		f10b_wr_count = f10b_wr_count + 1;
		if (f10b_wr_count <= 400)
			$display("t=%0t F10B_OBJWR addr=0x%03x data=0x%02x spc=0x%04x",
			          $time, dut.wram_addr_s, dut.wram_din_s, slave_dbg_pc);
	end
end

// Follow-up 10c -- master vport commit-level probe. Follow-up 10 showed
// every master op3 (port 0x0B) command write is followed 3 clk_sys later
// by a second popped op writing 0x00 to 0xFFA1 -- an op the Z80 never
// issued (MAME shows exactly one port write per byte). A q1 entry can
// only be enqueued by op 1/2, so either (a) some commits mis-decode the
// op (cpu_addr sampled wrong), or (b) q1_v is being set/retained
// spuriously. Log every master wr/rd commit with the raw decode inputs,
// plus q1_v at commit time, while the master PC is in either command
// writer, to pin down which.
integer f10c_cnt = 0;
always @(posedge clk_sys) begin
	if ((dut.master.mvport.wr_commit || dut.master.mvport.rd_commit) &&
	    ((dut.dbg_pc >= 16'h61A0 && dut.dbg_pc <= 16'h61C8) ||
	     (dut.dbg_pc >= 16'h6238 && dut.dbg_pc <= 16'h6290))) begin
		f10c_cnt = f10c_cnt + 1;
		if (f10c_cnt <= 300)
			$display("t=%0t F10C_COMMIT wr=%b rd=%b cpu_addr=0x%04x op=%0d addr_q=0x%04x dout=0x%02x q1_v_pre=%b mpc=0x%04x",
			          $time, dut.master.mvport.wr_commit, dut.master.mvport.rd_commit,
			          dut.master.cpu_addr, dut.master.mvport.op, dut.master.mvport.addr_q,
			          dut.master.cpu_dout, dut.master.mvport.q1_v, dut.dbg_pc);
	end
end

// Follow-up 9d -- fine-grained per-PC checkpoint histogram, mirroring
// MAME's dispatch_checkpoint_histogram.lua (C:\MiSTerDev\mame). MAME's
// real trace shows: 26 dispatcher entries reach the first checks
// (0x05ab-0x05c2), 15 divert to a special-case handler at 0x152e, and
// the remaining 11 ALL fall through 0x05c5-0x05ec into the real OUTI
// blit setup -- 0x064b/0x080a/0x08ad never fire in the same window.
// This checks exactly which of these checkpoints our RTL's Slave
// reaches, to localize precisely where execution diverges from MAME.
function automatic int unsigned chk_idx(input [15:0] pc);
	case (pc)
		16'h05ab: chk_idx = 0;  16'h05b9: chk_idx = 1;  16'h05bc: chk_idx = 2;
		16'h05bf: chk_idx = 3;  16'h05c0: chk_idx = 4;  16'h05c2: chk_idx = 5;
		16'h05c5: chk_idx = 6;  16'h05ca: chk_idx = 7;  16'h05cb: chk_idx = 8;
		16'h05d0: chk_idx = 9;  16'h05d5: chk_idx = 10; 16'h05de: chk_idx = 11;
		16'h05df: chk_idx = 12; 16'h05ec: chk_idx = 13; 16'h064b: chk_idx = 14;
		16'h080a: chk_idx = 15; 16'h08ad: chk_idx = 16; 16'h152e: chk_idx = 17;
		default:  chk_idx = 18; // "none" bucket, ignored
	endcase
endfunction
integer chk_hits [0:18];
integer ci;
initial for (ci = 0; ci < 19; ci = ci + 1) chk_hits[ci] = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n) begin
		if (chk_idx(dut.slave.cpu_addr) != 18) chk_hits[chk_idx(dut.slave.cpu_addr)] = chk_hits[chk_idx(dut.slave.cpu_addr)] + 1;
	end
end

// Follow-up 13 -- source-data trace for real VRAM OUT events. Everything
// mechanical (bank-switch arithmetic, VRAM-port encoding/arbitration, the
// write pipeline) is proven correct; the open question per the user's
// synthesis is whether the DATA fed into a real OUT is correct -- either
// because an upstream object-descriptor/table is corrupted, or because a
// CPU-side rom_stall/wait-state timing bug (untested by the earlier
// SLAVEBANKTEST directed single-read cases) lets the CPU sample stale
// data during a TIGHT, repeated sequence of banked-ROM reads like a real
// blit loop. Tracks the most recent completed banked-ROM read
// (addr/data/time), and at every real VRAM OUT, logs cpu_dout alongside
// that most-recent read plus bank_reg/HL/DE/PC, to see whether the
// written byte matches what was just read (expected for a genuine
// "read source, write out" blit) or looks disconnected/stale.
reg [18:0] f13_last_rom_addr;
reg  [7:0] f13_last_rom_data;
time       f13_last_rom_time;
reg        f13_rom_read_seen;
initial begin
	f13_last_rom_addr = 19'h0; f13_last_rom_data = 8'h0; f13_last_rom_time = 0;
	f13_rom_read_seen = 1'b0;
end
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && dut.slave.rom_read_cyc && dut.slave.in_banked && ~dut.slave.rom_stall) begin
		f13_last_rom_addr <= dut.slave.rom_addr;
		f13_last_rom_data <= dut.slave.rom_data;
		f13_last_rom_time <= $time;
		f13_rom_read_seen <= 1'b1;
	end
end
integer f13_out_count = 0, f13_match_count = 0, f13_nonzero_count = 0, f13_nonzero_match_count = 0;
reg f13_slave_outwr_prev2;
initial f13_slave_outwr_prev2 = 1'b0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M) begin
		if (dut.slave.io_wr && dut.slave.io_vram && !f13_slave_outwr_prev2) begin
			f13_out_count = f13_out_count + 1;
			if (dut.slave.cpu_dout == f13_last_rom_data) f13_match_count = f13_match_count + 1;
			if (dut.slave.cpu_dout != 8'h00) begin
				f13_nonzero_count = f13_nonzero_count + 1;
				if (dut.slave.cpu_dout == f13_last_rom_data) f13_nonzero_match_count = f13_nonzero_match_count + 1;
			end
			if (f13_out_count <= 60 || dut.slave.cpu_dout != 8'h00)
				$display("t=%0t F13_VRAM_OUT #%0d cpu_dout=0x%02x pc=0x%04x bank_reg=0x%0x HL=0x%02x%02x DE=0x%02x%02x  last_rom(addr=0x%05x data=0x%02x age=%0dns seen=%b)",
				          $time, f13_out_count, dut.slave.cpu_dout, dut.slave.dbg_pc, dut.slave.bank_reg,
				          dut.slave.slave_cpu.i_tv80_core.i_reg.H, dut.slave.slave_cpu.i_tv80_core.i_reg.L,
				          dut.slave.slave_cpu.i_tv80_core.i_reg.D, dut.slave.slave_cpu.i_tv80_core.i_reg.E,
				          f13_last_rom_addr, f13_last_rom_data, $time - f13_last_rom_time, f13_rom_read_seen);
		end
		f13_slave_outwr_prev2 <= dut.slave.io_wr && dut.slave.io_vram;
	end
end

// Follow-up 12b -- PC=0x10EC / H=$EC checkpoint, the newly-confirmed
// REAL dispatch path (command-type-2 via 0x10CC's `and $07` check,
// C:\MiSTerDev\mame\slave_trace.txt: 22/22 real executions of the
// shared 0x1100-0x1220 routine were reached this way, H always $EC,
// never $E9 -- superseding the retracted E900-centric Follow-up 9/10
// narrative). Checks whether our RTL's Slave ever takes this same path
// at all, and with the same H value.
integer f12b_10ec_hits = 0;
reg [7:0] f12b_h_at_10ec_last;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n && (dut.slave.cpu_addr == 16'h10ec)) begin
		f12b_10ec_hits = f12b_10ec_hits + 1;
		f12b_h_at_10ec_last = dut.slave.slave_cpu.i_tv80_core.i_reg.B; // B holds the command-type byte here (per trace: "10EC: ld a,b")
		if (f12b_10ec_hits <= 20)
			$display("t=%0t F12B_10EC_HIT #%0d B(cmdtype)=0x%02x", $time, f12b_10ec_hits, f12b_h_at_10ec_last);
	end
end
integer f12b_1100_hits = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n && (dut.slave.cpu_addr == 16'h1100)) begin
		f12b_1100_hits = f12b_1100_hits + 1;
		if (f12b_1100_hits <= 20)
			$display("t=%0t F12B_1100_HIT #%0d H=0x%02x", $time, f12b_1100_hits, dut.slave.slave_cpu.i_tv80_core.i_reg.H);
	end
end

// Follow-up 12c -- full VRAM-port pipeline counter: Slave OUT attempts,
// Master OUT attempts, vp_req (new-request events), vp_pop (drained
// events), vram_we_cpu (actual shared-BRAM writes). Counts at each
// pipeline stage over the same run to localize exactly where write
// volume is lost between "the CPU tried to write VRAM" and "the byte
// actually landed in the shared BRAM" -- complements Test 1 as proposed.
integer f12c_slave_out = 0, f12c_master_out = 0;
integer f12c_vpreq_s = 0, f12c_vpreq_m = 0;
integer f12c_vppop_s = 0, f12c_vppop_m = 0;
integer f12c_vram_we = 0;
reg f12c_slave_outwr_prev, f12c_master_outwr_prev;
reg f12c_vpreq_s_prev, f12c_vpreq_m_prev;
initial begin
	f12c_slave_outwr_prev = 1'b0; f12c_master_outwr_prev = 1'b0;
	f12c_vpreq_s_prev = 1'b0; f12c_vpreq_m_prev = 1'b0;
end
always @(posedge clk_sys) begin
	// Raw CPU-side OUT attempts to the VRAM port range, edge-detected
	// (io_wr stays high for 2 CE_6M cycles per Z80 I/O machine cycle;
	// count each real OUT once, same edge-detect pattern used
	// elsewhere in this file for the bank-register write counter).
	if (dut.slave.CE_6M) begin
		if (dut.slave.io_wr && dut.slave.io_vram && !f12c_slave_outwr_prev) f12c_slave_out = f12c_slave_out + 1;
		f12c_slave_outwr_prev <= dut.slave.io_wr && dut.slave.io_vram;
	end
	if (dut.master.CE_6M) begin
		if (dut.master.io_wr && dut.master.io_mvram && !f12c_master_outwr_prev) f12c_master_out = f12c_master_out + 1;
		f12c_master_outwr_prev <= dut.master.io_wr && dut.master.io_mvram;
	end
	// vp_req: count rising edges (a NEW op becoming pending), not every
	// cycle it's held asserted.
	if (dut.vp_req_s && !f12c_vpreq_s_prev) f12c_vpreq_s = f12c_vpreq_s + 1;
	f12c_vpreq_s_prev <= dut.vp_req_s;
	if (dut.vp_req_m && !f12c_vpreq_m_prev) f12c_vpreq_m = f12c_vpreq_m + 1;
	f12c_vpreq_m_prev <= dut.vp_req_m;
	// vp_pop: already a single-cycle pulse per drained op.
	if (dut.vp_pop_s) f12c_vppop_s = f12c_vppop_s + 1;
	if (dut.vp_pop_m) f12c_vppop_m = f12c_vppop_m + 1;
	// vram_we_cpu: single-cycle pulse per actual shared-BRAM write
	// (both the plain SEQ_ADDR path and the transparent SEQ_TWR path).
	if (dut.vram_we_cpu) f12c_vram_we = f12c_vram_we + 1;
end

// Follow-up 14 -- targeted follow-on to Follow-up 12b's confirmed real
// dispatch path (0x10CC's `and $07`==2 check -> 0x10EC, H=$EC, per MAME's
// full slave_trace.txt: 22/22 real executions). Three cheap additions:
// (1) $EF04 state-vector, log-on-change (same technique already proven for
//     $EF19/$EF37 in Follow-up 9g) -- shows whether the per-tick dispatch
//     coroutine's own state cycles normally or gets stuck in one state;
// (2) a denominator counter at 0x10CC itself (every time the command-type
//     byte is tested at all, whether or not it takes the 0x10EC branch) --
//     paired with the existing f12b_10ec_hits numerator to get a hit ratio
//     comparable to MAME's;
// (3) a counter at 0x1591 (the `call $04C3` recursion inside the 0x152e
//     housekeeping handler that advances to the next table entry each
//     tick, per docs/SESSION_2026-07-14.md's disassembly) -- MAME also
//     takes this path on every housekeeping (non-sprite) entry, so a high
//     count alone doesn't distinguish good/bad; what matters is whether it
//     EVER stops firing (matching the "OBJ_ATTR_READ stops after
//     t=552391114ns" symptom) or whether the per-visit table content ever
//     varies away from the single housekeeping entry.
reg [15:0] ef04_prev;
initial ef04_prev = 16'hFFFF;
wire [15:0] ef04_cur = {dut.wram_s.mem[16'hF05], dut.wram_s.mem[16'hF04]};
always @(posedge clk_sys) begin
	if (ef04_cur !== ef04_prev) begin
		$display("t=%0t EF04_CHANGE EF04(state_vec)=0x%04x", $time, ef04_cur);
		ef04_prev <= ef04_cur;
	end
end
integer f14_10cc_hits = 0;
integer f14_1591_hits = 0;
time f14_1591_last_time = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n && (dut.slave.cpu_addr == 16'h10cc))
		f14_10cc_hits = f14_10cc_hits + 1;
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n && (dut.slave.cpu_addr == 16'h1591)) begin
		f14_1591_hits = f14_1591_hits + 1;
		f14_1591_last_time = $time;
	end
end

// Follow-up 17 -- who calls into the 0x6100-0x6300 neighborhood? Rather
// than trust hand-disassembly of the surrounding bytes (which already
// turned out to be a tiny reusable "read VRAM byte" helper at 0x6129,
// not obviously "the" mailbox writer itself -- see the doc), this just
// watches dbg_pc (Master PC, latched at each M1 opcode fetch) for every
// transition from OUTSIDE 0x6100-0x6300 to INSIDE it, and logs the PC
// value the cycle immediately before -- that's the address of whatever
// CALL/JP instruction (or fallthrough) actually entered the range, a
// direct empirical answer with no disassembly assumptions. Bucketed by
// time (5s-equivalent windows scaled to sim time) so we can see whether
// entries into the range themselves stop at t~552ms (upstream gating)
// or continue but land somewhere that doesn't reach 0x6129/0x61EC
// (a different internal branch).
reg [15:0] f17_pc_prev;
initial f17_pc_prev = 16'h0000;
time f17_last_entry_time = 0;
reg [15:0] f17_last_entry_from = 16'h0000;
integer f17_entry_count = 0;
reg [15:0] f17_entry_hist_pc [0:63];
integer f17_entry_hist_time_ms [0:63];
integer f17_hist_idx = 0;
always @(posedge clk_sys) begin
	if (dut.dbg_pc !== f17_pc_prev) begin
		if ((dut.dbg_pc >= 16'h6100 && dut.dbg_pc <= 16'h6300) &&
		    !(f17_pc_prev >= 16'h6100 && f17_pc_prev <= 16'h6300)) begin
			f17_entry_count = f17_entry_count + 1;
			f17_last_entry_time = $time;
			f17_last_entry_from = f17_pc_prev;
			if (f17_hist_idx < 64) begin
				f17_entry_hist_pc[f17_hist_idx] = f17_pc_prev;
				f17_entry_hist_time_ms[f17_hist_idx] = $time / 1_000_000;
				f17_hist_idx = f17_hist_idx + 1;
			end
			if (f17_entry_count <= 300 || (f17_entry_count % 500 == 0))
				$display("t=%0t F17_ENTRY #%0d from_pc=0x%04x to_pc=0x%04x",
				          $time, f17_entry_count, f17_pc_prev, dut.dbg_pc);
		end
		f17_pc_prev <= dut.dbg_pc;
	end
end
time f17_periodic_last = 0;
always @(posedge clk_sys) begin
	if ($time - f17_periodic_last >= 20_000_000) begin // every 20ms
		f17_periodic_last = $time;
		$display("t=%0t F17_PERIODIC entry_count_so_far=%0d", $time, f17_entry_count);
	end
end

// Follow-up 18 -- the Slave-side counterpart to Follow-up 17. The user
// confirmed the Master-side mailbox-writer chase (Follow-up 14-17) was a
// red herring (MAME's own real cold boot also retires it within 36ms) --
// the real, never-retracted lead is docs/SESSION_2026-07-14.md's
// Follow-up 9 finding: our RTL's Slave dispatcher takes the housekeeping
// branch 100% of the time at its discriminator check (0x05bc), never
// once decoding a real sprite entry (HL stuck at 0xe9f8 forever, per the
// existing OBJ_ATTR_READ trace above). Hand-disassembly of the
// candidate chain-walk region (0x03d3-0x04a8) hit a wall there (EX
// AF,AF'/EXX shadow-register tricks, bit-manipulated address math) and
// was marked "diminishing returns" in that doc. Rather than keep
// guessing from static bytes, this dumps the Slave's FULL live register
// state (PC, HL, DE, BC, shadow HL'/DE'/BC', AF) at every M1 fetch
// while PC is in the 0x03d0-0x0500 candidate range, for the first three
// times that range is entered only (kept short and readable on purpose
// -- this is meant to be read by eye, not aggregated) -- ground truth
// from our OWN RTL's actual execution, no disassembly assumptions
// needed, mirroring the technique that worked well for Follow-up 17.
integer f18_window_count = 0;
reg     f18_in_window = 1'b0;
integer f18_addr_hits = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n) begin
		if (dut.slave.cpu_addr >= 16'h03d0 && dut.slave.cpu_addr <= 16'h0500) begin
			if (!f18_in_window) begin
				f18_window_count = f18_window_count + 1;
				f18_in_window <= 1'b1;
			end
		end else begin
			f18_in_window <= 1'b0;
		end
		// Targeted: only the specific previously hand-disassembled
		// chain-walk addresses (0x03d3 head-pointer load, 0x04a5
		// current-position shadow write, 0x04a8 next-link read/advance,
		// 0x0466 the other candidate link-walk site) -- not a broad
		// window, so every printed line is directly relevant.
		if (dut.slave.cpu_addr == 16'h03d3 || dut.slave.cpu_addr == 16'h04a5 ||
		    dut.slave.cpu_addr == 16'h04a8 || dut.slave.cpu_addr == 16'h0466) begin
			f18_addr_hits = f18_addr_hits + 1;
			if (f18_addr_hits <= 200)
				$display("t=%0t F18_REG #%0d pc=0x%04x HL=0x%02x%02x DE=0x%02x%02x BC=0x%02x%02x HL'=0x%02x%02x DE'=0x%02x%02x BC'=0x%02x%02x AF=0x%02x%02x",
				          $time, f18_addr_hits, dut.slave.cpu_addr,
				          dut.slave.slave_cpu.i_tv80_core.i_reg.H, dut.slave.slave_cpu.i_tv80_core.i_reg.L,
				          dut.slave.slave_cpu.i_tv80_core.i_reg.D, dut.slave.slave_cpu.i_tv80_core.i_reg.E,
				          dut.slave.slave_cpu.i_tv80_core.i_reg.B, dut.slave.slave_cpu.i_tv80_core.i_reg.C,
				          dut.slave.slave_cpu.i_tv80_core.i_reg.RegsH[6], dut.slave.slave_cpu.i_tv80_core.i_reg.RegsL[6],
				          dut.slave.slave_cpu.i_tv80_core.i_reg.RegsH[5], dut.slave.slave_cpu.i_tv80_core.i_reg.RegsL[5],
				          dut.slave.slave_cpu.i_tv80_core.i_reg.RegsH[4], dut.slave.slave_cpu.i_tv80_core.i_reg.RegsL[4],
				          dut.slave.slave_cpu.i_tv80_core.ACC, dut.slave.slave_cpu.i_tv80_core.F);
		end
	end
end
// Follow-up 18b -- direct write-tap on the two exact link-byte
// addresses (0xE8F9/0xEAF9, offsets 0x8F9/0xAF9 from WRAM base 0xE000)
// that Follow-up 18 proved are read as 0xFF on every single chain-walk
// attempt for the entire run. If the Slave (the code with private WRAM
// ownership of this table, per rtl/sor_slave.sv's WRAM map) ever writes
// a non-FF value here, this will show it -- and if it never fires at
// all, that's direct, disassembly-independent proof the write is
// simply missing.
integer f18b_wr_count = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && dut.wram_we_s &&
	    (dut.wram_addr_s == 12'h8F9 || dut.wram_addr_s == 12'hAF9)) begin
		f18b_wr_count = f18b_wr_count + 1;
		$display("t=%0t F18B_LINKBYTE_WR #%0d addr=0xE%03x data=0x%02x spc=0x%04x",
		          $time, f18b_wr_count, dut.wram_addr_s, dut.wram_din_s, slave_dbg_pc);
	end
end

// Follow-up 19 -- Fable-review-suggested probe: the link-byte theory
// (Follow-up 18/18b) was ruled out on both sides (E8F9/EAF9 stay 0xFF in
// both RTL and MAME), but a separate, still-unexplained WRAM content
// divergence exists at 0xE900 (docs/SESSION_2026-07-14.md): MAME's real
// content there is `04 80 0c be 9d 9f 98 d0` (attr=0x04, real-sprite
// discriminator); our RTL's is `80 80 00 00` -- a record that would
// fail the dispatcher's earliest reject check regardless of whether the
// chain-walk ever reached it. This traces every write to 0xE900-0xE907
// from EITHER cpu's WRAM path (in case the Master, not just the Slave,
// ever writes there directly -- private-WRAM assumption notwithstanding,
// checking both closes that blind spot cheaply), logging cpu/PC/data,
// to find out who (if anyone) populates this record in our RTL and
// compare against the equivalent MAME trace.
integer f19_wr_count_s = 0, f19_wr_count_m = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && dut.wram_we_s &&
	    (dut.wram_addr_s >= 12'h900 && dut.wram_addr_s <= 12'h907)) begin
		f19_wr_count_s = f19_wr_count_s + 1;
		$display("t=%0t F19_E900WR_SLAVE #%0d addr=0xE%03x data=0x%02x spc=0x%04x",
		          $time, f19_wr_count_s, dut.wram_addr_s, dut.wram_din_s, slave_dbg_pc);
	end
	if (dut.master.CE_6M && dut.wram_we_m &&
	    (dut.wram_addr_m >= 12'h900 && dut.wram_addr_m <= 12'h907)) begin
		f19_wr_count_m = f19_wr_count_m + 1;
		$display("t=%0t F19_E900WR_MASTER #%0d addr=0xE%03x data=0x%02x mpc=0x%04x",
		          $time, f19_wr_count_m, dut.wram_addr_m, dut.wram_din_m, dut.dbg_pc);
	end
end

// Follow-up 20 -- MAME-reference disassembly of the two write sites
// found by Follow-up 19 (0x1120-0x1150 and 0x1210-0x1240 in the real
// ROM) shows: PC=0x1220 is `ld (hl),a` where A was just loaded from B
// (the command-type byte established since Follow-up 14) -- this is
// THE write that lands on E900. PC=0x1229 (`ld (hl),a` after `inc l`,
// A built from `ld a,c; add a,a x4; and $e0`) is the E901 write, and it
// already matches MAME exactly (both write 0x80), so C is not the
// problem -- B is. Separately, PC=0x1136 is an *unconditional* literal
// `ld (hl),$80` (H = C+1 where C was saved as "old H" at 0x112d, L
// unchanged from before) -- not data-dependent at all, so it only
// lands on E900 if, at that exact moment, C=0xE8 and L=0x00 in our RTL --
// confirming or refuting that is the direct test for whether this is
// really an address-aliasing collision or something else. Traces B, C,
// HL live at both PCs.
integer f20_hits_1220 = 0, f20_hits_1136 = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n && (dut.slave.cpu_addr == 16'h1220)) begin
		f20_hits_1220 = f20_hits_1220 + 1;
		if (f20_hits_1220 <= 40)
			$display("t=%0t F20_AT_1220 #%0d B=0x%02x C=0x%02x HL=0x%02x%02x",
			          $time, f20_hits_1220,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.B, dut.slave.slave_cpu.i_tv80_core.i_reg.C,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.H, dut.slave.slave_cpu.i_tv80_core.i_reg.L);
	end
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n && (dut.slave.cpu_addr == 16'h1136)) begin
		f20_hits_1136 = f20_hits_1136 + 1;
		if (f20_hits_1136 <= 40)
			$display("t=%0t F20_AT_1136 #%0d B=0x%02x C=0x%02x HL=0x%02x%02x",
			          $time, f20_hits_1136,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.B, dut.slave.slave_cpu.i_tv80_core.i_reg.C,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.H, dut.slave.slave_cpu.i_tv80_core.i_reg.L);
	end
end

// Follow-up 21 -- Follow-up 20 proved B=0x00 at every single visit to
// 0x1220 (23/23), yet Follow-up 14 already showed B alternates 0x02/0x00
// at 0x10EC (the command-type check) matching the same e900/e8f9
// alternation. So on the "real sprite" pass, B legitimately starts as
// 0x02 at 0x10EC but has become 0x00 by 0x1220 -- it's cleared (or
// mistransformed -- MAME's equivalent becomes 0x04, not just preserved
// 0x02) somewhere in the shared 0x10EC-0x1220 routine. This traces PC/A/B
// at every M1 fetch across that whole range, for the first two full
// traversals only, to find exactly where B changes.
integer f21_step_count = 0;
reg     f21_prev_b;
reg     f21_armed = 1'b0;
initial f21_prev_b = 1'b0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n) begin
		if (dut.slave.cpu_addr == 16'h10ec) f21_armed <= 1'b1;
		else if (dut.slave.cpu_addr == 16'h1220 || dut.slave.cpu_addr > 16'h1230) f21_armed <= 1'b0;
		if (f21_armed && f21_step_count <= 300) begin
			f21_step_count = f21_step_count + 1;
			$display("t=%0t F21_STEP #%0d pc=0x%04x A=0x%02x B=0x%02x C=0x%02x DE=0x%02x%02x HL=0x%02x%02x",
			          $time, f21_step_count, dut.slave.cpu_addr,
			          dut.slave.slave_cpu.i_tv80_core.ACC, dut.slave.slave_cpu.i_tv80_core.i_reg.B,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.C,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.D, dut.slave.slave_cpu.i_tv80_core.i_reg.E,
			          dut.slave.slave_cpu.i_tv80_core.i_reg.H, dut.slave.slave_cpu.i_tv80_core.i_reg.L);
		end
	end
end

// Follow-up 22 -- MAME's live register trace + a direct check of the
// real ROM chip content (via MAME's own :slave region reader) both
// confirm: the byte at flat address 0x50000 (bank_reg=8, cpu_addr=
// 0x4000, i.e. `ld a,(de)` at Slave PC 0x11ec/0x11fa) is genuinely
// 0x04 in the real ROM -- not a MAME-only artifact. If our RTL reads
// something else there, it's a real SDRAM/bank-read correctness bug,
// not a data-population or register-flow issue. Traces every completed
// banked-ROM read (rom_read_cyc && ~rom_stall, the same technique
// Follow-up 13 already used) whose CPU-side address is 0x4000-0x4003
// (a few bytes of margin) while bank_reg==8, logging the actual flat
// rom_addr and the byte returned, to compare directly against the
// confirmed-correct 0x04.
integer f22_hits = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && dut.slave.rom_read_cyc && dut.slave.in_banked && ~dut.slave.rom_stall &&
	    (dut.slave.cpu_addr >= 16'h4000 && dut.slave.cpu_addr <= 16'h4003) && (dut.slave.bank_reg == 4'd8)) begin
		f22_hits = f22_hits + 1;
		if (f22_hits <= 40)
			$display("t=%0t F22_ROMRD #%0d cpu_addr=0x%04x bank_reg=%0d rom_addr=0x%05x rom_data=0x%02x",
			          $time, f22_hits, dut.slave.cpu_addr, dut.slave.bank_reg,
			          dut.slave.rom_addr, dut.slave.rom_data);
	end
end

// Follow-up 15 -- Follow-up 14 established that BOTH the housekeeping
// recursion (0x1591) and the real dispatch (0x10EC) go silent at the same
// t~552ms moment, and that the boot-window command-writer PCs (0x61A0-
// 0x61C8/0x6238-0x6290, Follow-up 10's F10_MWR) stop being OUT'd from at
// t=537749436ns (488 total) -- consistent with the Slave simply running
// dry. But F12C_PIPELINE's *unrestricted* master_out=1333 (all-PC, not
// range-gated) is far larger than F10_MWR's range-gated 488 -- meaning the
// Master keeps writing to the VRAM port range from SOME other PC(s) for
// the whole run, ~845 additional OUTs we haven't looked at yet. Since
// every MAME ground-truth trace so far (F10, checkpoint histograms, etc.)
// was captured during the title/hints BOOT window only -- never an actual
// race -- it's possible MAME's Master uses a *different* routine entirely
// for ongoing per-frame sprite updates once past boot, one we've never
// instrumented, and our RTL's Master might already be calling its
// equivalent (PC_SAMPLE shows m_pc settling into 0x6380-0x638b/0x7625-
// 0x7734 after t=538ms, distinct from the boot command-writer addresses).
// This logs every unrestricted Master VRAM-port OUT (PC, vaddr, data) to
// see exactly which PC(s) issue the post-538ms traffic and whether it
// looks like real per-object command data or something else (e.g. just
// the periodic status-command writer re-sending the same idle state).
integer f15_mout_count = 0;
always @(posedge clk_sys) begin
	if (dut.vp_pop_m && !dut.master.mvport.q0_rd) begin
		f15_mout_count = f15_mout_count + 1;
		if (f15_mout_count <= 200 || (dut.dbg_pc < 16'h61A0 || (dut.dbg_pc > 16'h61C8 && dut.dbg_pc < 16'h6238) || dut.dbg_pc > 16'h6290))
			$display("t=%0t F15_MOUT_ANY #%0d vaddr=0x%04x data=0x%02x mpc=0x%04x bank=0x%0x",
			          $time, f15_mout_count, dut.master.mvport.q0_a, dut.master.mvport.q0_d, dut.dbg_pc, dut.master.bank_reg);
	end
end

// Follow-up 16 -- stale VRAM-port read checker, per the Fable review's
// hypothesis: sor_vram_port.sv's vp_stall drops (releasing the Z80's
// /WAIT) as soon as io_rd_done latches on rd_commit, which can happen
// BEFORE sor_board's sequencer actually pops the queued read op and
// updates rd_data_q with the real vp_rdata -- unlike MAME's vram_port_r,
// which returns the real byte synchronously, same call. If the Z80's IN
// machine cycle ends (io_rd falls) while that op is still unpopped, the
// CPU latches whatever rd_data_q held from an EARLIER, unrelated op --
// a stale read. Tracked per-CPU: a "pending" flag set on rd_commit and
// cleared on the matching vp_pop, checked at every io_rd falling edge.
reg pending_rd_m, pending_rd_s;
reg [15:0] pending_rd_addr_m, pending_rd_addr_s;
reg io_rd_prev_m, io_rd_prev_s;
integer stale_rd_count_m = 0, stale_rd_count_s = 0;
initial begin
	pending_rd_m = 1'b0; pending_rd_s = 1'b0;
	io_rd_prev_m = 1'b0; io_rd_prev_s = 1'b0;
end
always @(posedge clk_sys) begin
	// Master side.
	if (dut.master.mvport.rd_commit) begin
		pending_rd_m      <= 1'b1;
		pending_rd_addr_m <= dut.master.mvport.q0_a;
	end else if (dut.vp_pop_m && dut.master.mvport.q0_rd) begin
		pending_rd_m <= 1'b0;
	end
	if (io_rd_prev_m && !dut.master.io_rd && pending_rd_m) begin
		stale_rd_count_m = stale_rd_count_m + 1;
		if (stale_rd_count_m <= 100)
			$display("t=%0t F16_STALE_RD_MASTER #%0d addr=0x%04x stale_value=0x%02x mpc=0x%04x",
			          $time, stale_rd_count_m, pending_rd_addr_m, dut.master.mvport.rd_data, dut.dbg_pc);
	end
	io_rd_prev_m <= dut.master.io_rd;

	// Slave side.
	if (dut.slave.vport.rd_commit) begin
		pending_rd_s      <= 1'b1;
		pending_rd_addr_s <= dut.slave.vport.q0_a;
	end else if (dut.vp_pop_s && dut.slave.vport.q0_rd) begin
		pending_rd_s <= 1'b0;
	end
	if (io_rd_prev_s && !dut.slave.io_rd && pending_rd_s) begin
		stale_rd_count_s = stale_rd_count_s + 1;
		if (stale_rd_count_s <= 100)
			$display("t=%0t F16_STALE_RD_SLAVE #%0d addr=0x%04x stale_value=0x%02x spc=0x%04x",
			          $time, stale_rd_count_s, pending_rd_addr_s, dut.slave.vport.rd_data, dut.slave.dbg_pc);
	end
	io_rd_prev_s <= dut.slave.io_rd;
end

// Follow-up 9 -- fg-bitmap VRAM write ground-truth comparison. MAME's
// fg_bitmap_early_boot_trace.lua (C:\MiSTerDev\mame) shows real writes to
// the visible fg bitmap (VRAM addr 0x0000-0xEFFF, i.e. NOT the >=0xF000
// mailbox slots the earlier CRAM/bank-switch traces focused on) start at
// frame 0 and taper to zero by frame ~36 (title/hints art fully blitted
// within ~0.6s of boot) -- total 75035 writes, 30261 nonzero, by then.
// This checks whether our RTL's Slave-side VRAM writes show the same
// early-and-then-idle shape, using vram_we_cpu/vram_addr_cpu/vram_din_cpu
// (same signals the existing VRAM_WR monitor above taps, just bucketed
// by address range and nonzero-ness, with a periodic timeline instead of
// only "first 8 + every 20000th").
integer fg_wr_count = 0, fg_wr_nonzero = 0, mbx_wr_count = 0;
integer fg_wr_count_prev = 0, fg_wr_nonzero_prev = 0;
time    fg_last_report = 0;
reg     fg_first_nonzero_seen = 1'b0;
always @(posedge clk_sys) begin
	if (dut.vram_we_cpu) begin
		if (dut.vram_addr_cpu < 17'h0F000) begin
			fg_wr_count = fg_wr_count + 1;
			if (dut.vram_din_cpu != 8'h00) begin
				fg_wr_nonzero = fg_wr_nonzero + 1;
				if (!fg_first_nonzero_seen) begin
					fg_first_nonzero_seen = 1'b1;
					$display("t=%0t FG_BITMAP_FIRST_NONZERO_WRITE addr=0x%04x data=0x%02x",
					          $time, dut.vram_addr_cpu, dut.vram_din_cpu);
				end
			end
		end else begin
			mbx_wr_count = mbx_wr_count + 1;
		end
	end
	// Periodic timeline, every 20ms of sim time, mirroring MAME's
	// per-frame log cadence closely enough to eyeball the shape.
	if ($time - fg_last_report >= 20_000_000) begin
		$display("t=%0t FG_BITMAP_TIMELINE fg_wr_delta=%0d fg_nonzero_delta=%0d  (cum fg_wr=%0d fg_nonzero=%0d mbx_wr=%0d)",
		          $time, fg_wr_count - fg_wr_count_prev, fg_wr_nonzero - fg_wr_nonzero_prev,
		          fg_wr_count, fg_wr_nonzero, mbx_wr_count);
		fg_wr_count_prev    = fg_wr_count;
		fg_wr_nonzero_prev  = fg_wr_nonzero;
		fg_last_report      = $time;
	end
end

`ifdef E973_TRACE
// ============================================================
// E973_TRACE -- logs every Master WRAM write to $E973-$E976 (sim time,
// address, data byte, Master PC, Master bank register), for a direct
// comparison against a MAME ground-truth trace of the same four bytes.
// sim/ instrumentation only -- rtl/ is untouched.
//
// dut.master.wram_we (sor_master.sv) = mem_access & ~wr_n & in_wram is
// a LEVEL signal with no CE_6M gate, so it stays high for several
// clk_sys ticks across one Z80 write cycle -- sampling it directly at
// every posedge clk_sys would print the same logical write several
// times over (this is the "historical double-execution issue" the
// design has seen elsewhere). Edge-detect it here (0->1 on wram_we)
// so each logical write prints exactly once. dut.master.dbg_pc is the
// PC latched at the instruction's opcode fetch (M1), i.e. the PC of
// the instruction doing the write; dut.master.dbg_bank is the live
// bank register at that same moment.
// ============================================================
reg e973_we_d;
always @(posedge clk_sys) begin
	if (reset) begin
		e973_we_d <= 1'b0;
	end else begin
		e973_we_d <= dut.master.wram_we;
		if (dut.master.wram_we && !e973_we_d &&
		    dut.master.wram_addr >= 12'h973 && dut.master.wram_addr <= 12'h976) begin
			$display("E973TRACE t=%0t addr=%04x data=%02x pc=%04x bank=%0d",
			          $time, {4'hE, dut.master.wram_addr}, dut.master.wram_din,
			          dut.master.dbg_pc, dut.master.dbg_bank);
		end
	end
end
`endif // E973_TRACE

`ifdef SCANOUT_DUMP
// ============================================================
// SCANOUT_DUMP -- captures ONE full frame of sor_video's real pixel
// pen stream (cram_addr = {fg_pen[3:0], bg_pen[5:0]}) near the end of
// the run, with the CPUs live and generating genuine VRAM-port
// contention. This is the thing the board TB never otherwise renders:
// vram_dump_final.bin proves the VRAM *contents*; this proves (or
// indicts) the *scan-out fetch path*. Hardware shows title text with
// inserted blank gaps displacing characters; the VRAM render shows
// none -- if the gaps appear in this stream, the defect is the fetch
// pipeline under contention and is traceable in sim.
// Format: 2 bytes per visible pixel (byte0=fg_pen, byte1=bg_pen);
// 0xFF,0xFF sentinel pair at each HBlank rise (end of line). fg_pen
// max 0x0F / bg_pen max 0x3F so the sentinel is unambiguous.
integer scan_fd;
reg scan_active = 0, scan_done_r = 0;
reg scan_vb_d = 0, scan_hb_d = 0;
always @(posedge clk_sys) begin
	scan_vb_d <= dut.video.VBlank;
	scan_hb_d <= dut.video.HBlank;
	if (!scan_done_r && !scan_active &&
	    $time >= 770ms && dut.video.VBlank && !scan_vb_d) begin
		scan_active <= 1;
		scan_fd = $fopen("scanout_frame.bin", "wb");
		$display("=== SCANOUT_DUMP armed, capture starts t=%0t ===", $time);
	end else if (scan_active && dut.video.VBlank && !scan_vb_d) begin
		scan_active <= 0;
		scan_done_r <= 1;
		$fclose(scan_fd);
		$display("=== SCANOUT_DUMP complete t=%0t -> sim/scanout_frame.bin ===", $time);
	end
	if (scan_active) begin
		if (dut.ce_pix && !dut.video.VBlank && !dut.video.HBlank)
			$fwrite(scan_fd, "%c%c",
			        {4'b0, dut.video.cram_addr[9:6]},
			        {2'b0, dut.video.cram_addr[5:0]});
		if (dut.video.HBlank && !scan_hb_d)
			$fwrite(scan_fd, "%c%c", 8'hFF, 8'hFF);
	end
end
`endif // SCANOUT_DUMP

`ifdef FGCK_TRACE
// Prints sor_video's copyright-region fg checksum (the on-hardware
// overlay instrument) at each VBlank rise late in the run -- validates
// that the RTL counters index pixels identically to the SCANOUT_DUMP
// capture (python expected: 9C15 for the title screen). The printed
// value is the authoritative reference to compare against the
// hardware overlay chars 14-17 of status row 2.
reg fgck_vb_tap_d;
always @(posedge clk_sys) begin
	fgck_vb_tap_d <= dut.video.VBlank;
	if (dut.video.VBlank && !fgck_vb_tap_d && $time >= 750ms)
		$display("FGCK t=%0t lat_prev_frame=%04x", $time, dut.video.fgck_acc);
end
`endif // FGCK_TRACE

`ifdef M1RANGE_TRACE
// ============================================================
// M1RANGE_TRACE -- logs every Master M1 opcode fetch with PC in
// $5C00-$5F7F (MAME's boot state machine at $5D87-$5E36, culminating
// in a CALL $5E37 EEPROM serial read at frame 8), to see whether our
// core ever enters that range, how far it gets, and where it exits.
// Also maintains a 16-bucket per-4KB-page (pc[15:12]) M1 histogram
// covering every fetch anywhere, printed periodically and at the end
// of sim -- this proves the tap is alive (and shows overall PC
// distribution) even if the $5C00-$5F7F window is never hit.
// sim/ instrumentation only -- rtl/ is untouched.
//
// dut.master.m1_fetch_now (rtl/sor_master.sv) = mem_access & ~m1_n &
// ~rom_stall is the same fetch strobe the runaway-PC-trap logic uses
// to catch "one event per opcode fetch" (see the comment above
// prev_m1_pc in sor_master.sv) -- the tv80 holds m1_n low across
// several CE_6M ticks per fetch, so this must be edge-detected here
// too, same reasoning as e973_we_d above. dut.master.cpu_addr is the
// live PC at that exact fetch (stable across the whole m1_fetch_now
// pulse); dut.master.dbg_bank is the live bank register.
// ============================================================
reg        m1r_fetch_d;
integer    m1r_page_hist[0:15];
integer    m1r_page_i;
time       m1r_last_report;

initial begin
	for (m1r_page_i = 0; m1r_page_i < 16; m1r_page_i = m1r_page_i + 1)
		m1r_page_hist[m1r_page_i] = 0;
	m1r_last_report = 0;
end

always @(posedge clk_sys) begin
	if (reset) begin
		m1r_fetch_d <= 1'b0;
	end else begin
		m1r_fetch_d <= dut.master.m1_fetch_now;
		if (dut.master.m1_fetch_now && !m1r_fetch_d) begin
			m1r_page_hist[dut.master.cpu_addr[15:12]] = m1r_page_hist[dut.master.cpu_addr[15:12]] + 1;
			if (dut.master.cpu_addr >= 16'h5C00 && dut.master.cpu_addr <= 16'h5F7F) begin
				$display("M1R t=%0t pc=%04x bank=%0d", $time, dut.master.cpu_addr, dut.master.dbg_bank);
			end
		end

		if ($time - m1r_last_report >= 50_000_000) begin
			$display("M1R_HIST t=%0t 0000=%0d 1000=%0d 2000=%0d 3000=%0d 4000=%0d 5000=%0d 6000=%0d 7000=%0d 8000=%0d 9000=%0d A000=%0d B000=%0d C000=%0d D000=%0d E000=%0d F000=%0d",
			          $time,
			          m1r_page_hist[0],  m1r_page_hist[1],  m1r_page_hist[2],  m1r_page_hist[3],
			          m1r_page_hist[4],  m1r_page_hist[5],  m1r_page_hist[6],  m1r_page_hist[7],
			          m1r_page_hist[8],  m1r_page_hist[9],  m1r_page_hist[10], m1r_page_hist[11],
			          m1r_page_hist[12], m1r_page_hist[13], m1r_page_hist[14], m1r_page_hist[15]);
			m1r_last_report = $time;
		end
	end
end

final begin
	$display("M1R_HIST_FINAL t=%0t 0000=%0d 1000=%0d 2000=%0d 3000=%0d 4000=%0d 5000=%0d 6000=%0d 7000=%0d 8000=%0d 9000=%0d A000=%0d B000=%0d C000=%0d D000=%0d E000=%0d F000=%0d",
	          $time,
	          m1r_page_hist[0],  m1r_page_hist[1],  m1r_page_hist[2],  m1r_page_hist[3],
	          m1r_page_hist[4],  m1r_page_hist[5],  m1r_page_hist[6],  m1r_page_hist[7],
	          m1r_page_hist[8],  m1r_page_hist[9],  m1r_page_hist[10], m1r_page_hist[11],
	          m1r_page_hist[12], m1r_page_hist[13], m1r_page_hist[14], m1r_page_hist[15]);
end
`endif // M1RANGE_TRACE

// ============================================================
// MBOX_TRACE -- mailbox (VRAM addr >= 0xF000) elementary-op tap, for
// diffing our RTL's Master<->Slave mailbox traffic against a MAME-side
// trace. sim/ instrumentation only -- rtl/ is untouched.
//
// Sampling point: sor_board.sv's VRAM-port sequencer (the always block
// at ~line 1710, states SEQ_IDLE/SEQ_ADDR/SEQ_POP/SEQ_TRD/SEQ_TPOP/
// SEQ_TWR/SEQ_TWPOP) commits/pops each elementary op exactly once, via
// vp_pop_m/vp_pop_s (registered 1-cycle pulses). This tap fires on
// those same pulses, reading dut.cur_side/cur_rd/cur_addr/cur_data
// (latched by the sequencer when the op was accepted into SEQ_ADDR or
// SEQ_TRD, held stable until it's popped back to SEQ_IDLE) plus, for
// reads, dut.vp_rdata_m/vp_rdata_s -- both of those are written by the
// SAME non-blocking assignment in the SEQ_POP case that raises
// vp_pop_m/vp_pop_s, so they become valid on the identical clock edge
// this tap samples. That gives each elementary op exactly once, in
// true hardware completion order (the sequencer is strictly
// one-op-at-a-time, fixed priority slave-then-master).
//
// Op-number recovery: sor_vram_port's internal queue (q0/q1) only
// carries an address/data pair, not the original Z80 I/O port op
// number (1/2/3/5/6 -- see sor_vram_port.sv header). Ops 1 and 2 each
// push TWO elementary writes (even+odd byte) from one Z80 OUT; ops
// 3/5/6 push exactly one (write or read). To recover the original op
// number for the trace, a small shadow FIFO per side (depth 4, well
// over the max ~2 ever in flight) is pushed whenever that side's
// vport commits a new Z80 I/O cycle (wr_commit/rd_commit, tapped
// hierarchically -- same signals sor_vram_port itself gates its queue
// push on), with `op` pushed twice for 1/2 and once for 3/5/6, then
// popped in the same order the elementary ops complete above. If this
// somehow underflows, op number 0 is used as an explicit "unknown"
// sentinel (printed as op? below) rather than a guess.
// ============================================================
localparam integer MBOX_LOG_CAP = 400;

reg [2:0] mbox_opfifo_m [0:7];
reg [2:0] mbox_opfifo_s [0:7];
integer   mbox_opfifo_m_n;
integer   mbox_opfifo_s_n;

wire         mbox_wrc_m = dut.master.mvport.wr_commit;
wire         mbox_rdc_m = dut.master.mvport.rd_commit;
wire  [2:0]  mbox_op_m  = dut.master.mvport.op;
wire         mbox_wrc_s = dut.slave.vport.wr_commit;
wire         mbox_rdc_s = dut.slave.vport.rd_commit;
wire  [2:0]  mbox_op_s  = dut.slave.vport.op;

integer mbox_total_rd_m, mbox_total_wr_m, mbox_total_rd_s, mbox_total_wr_s;

// Per-address (0xF000-0xFFFF -> index 0-4095) stats.
integer mbox_rd_cnt_m      [0:4095];
integer mbox_wr_cnt_m      [0:4095];
integer mbox_rd_cnt_s      [0:4095];
integer mbox_wr_cnt_s      [0:4095];
reg     mbox_addr_touched  [0:4095];
integer mbox_distinct_n    [0:4095];
reg [7:0] mbox_distinct_val[0:4095][0:11];

// Chronological log of the first MBOX_LOG_CAP ops.
integer mbox_log_n;
reg           mbox_log_side [0:MBOX_LOG_CAP-1]; // 0=M 1=S
reg           mbox_log_rd   [0:MBOX_LOG_CAP-1]; // 0=W 1=R
reg [2:0]     mbox_log_op   [0:MBOX_LOG_CAP-1]; // 0 = unknown (op?)
reg [15:0]    mbox_log_addr [0:MBOX_LOG_CAP-1];
reg [7:0]     mbox_log_data [0:MBOX_LOG_CAP-1];

integer mbox_init_i;
initial begin
	mbox_opfifo_m_n  = 0;
	mbox_opfifo_s_n  = 0;
	mbox_total_rd_m  = 0;
	mbox_total_wr_m  = 0;
	mbox_total_rd_s  = 0;
	mbox_total_wr_s  = 0;
	mbox_log_n       = 0;
	for (mbox_init_i = 0; mbox_init_i < 4096; mbox_init_i = mbox_init_i + 1) begin
		mbox_rd_cnt_m[mbox_init_i]     = 0;
		mbox_wr_cnt_m[mbox_init_i]     = 0;
		mbox_rd_cnt_s[mbox_init_i]     = 0;
		mbox_wr_cnt_s[mbox_init_i]     = 0;
		mbox_addr_touched[mbox_init_i] = 1'b0;
		mbox_distinct_n[mbox_init_i]   = 0;
	end
end

always @(posedge clk_sys) begin
	if (!reset) begin
		// -- push: a new Z80 I/O cycle just committed on the master side --
		if (mbox_wrc_m) begin
			if (mbox_op_m == 3'd1 || mbox_op_m == 3'd2) begin
				mbox_opfifo_m[mbox_opfifo_m_n]     = mbox_op_m;
				mbox_opfifo_m[mbox_opfifo_m_n + 1] = mbox_op_m;
				mbox_opfifo_m_n = mbox_opfifo_m_n + 2;
			end else if (mbox_op_m == 3'd3 || mbox_op_m == 3'd5 || mbox_op_m == 3'd6) begin
				mbox_opfifo_m[mbox_opfifo_m_n] = mbox_op_m;
				mbox_opfifo_m_n = mbox_opfifo_m_n + 1;
			end
		end else if (mbox_rdc_m) begin
			if (mbox_op_m == 3'd3 || mbox_op_m == 3'd5 || mbox_op_m == 3'd6) begin
				mbox_opfifo_m[mbox_opfifo_m_n] = mbox_op_m;
				mbox_opfifo_m_n = mbox_opfifo_m_n + 1;
			end
		end
		// -- push: slave side, same logic --
		if (mbox_wrc_s) begin
			if (mbox_op_s == 3'd1 || mbox_op_s == 3'd2) begin
				mbox_opfifo_s[mbox_opfifo_s_n]     = mbox_op_s;
				mbox_opfifo_s[mbox_opfifo_s_n + 1] = mbox_op_s;
				mbox_opfifo_s_n = mbox_opfifo_s_n + 2;
			end else if (mbox_op_s == 3'd3 || mbox_op_s == 3'd5 || mbox_op_s == 3'd6) begin
				mbox_opfifo_s[mbox_opfifo_s_n] = mbox_op_s;
				mbox_opfifo_s_n = mbox_opfifo_s_n + 1;
			end
		end else if (mbox_rdc_s) begin
			if (mbox_op_s == 3'd3 || mbox_op_s == 3'd5 || mbox_op_s == 3'd6) begin
				mbox_opfifo_s[mbox_opfifo_s_n] = mbox_op_s;
				mbox_opfifo_s_n = mbox_opfifo_s_n + 1;
			end
		end

		// -- pop: master elementary op completed --
		if (dut.vp_pop_m) begin : mbox_pop_m_blk
			reg [2:0]  opn;
			reg [15:0] a;
			reg [7:0]  d;
			integer    idx, k, j;
			reg        found;
			if (mbox_opfifo_m_n > 0) begin
				opn = mbox_opfifo_m[0];
				for (k = 0; k < 7; k = k + 1) mbox_opfifo_m[k] = mbox_opfifo_m[k+1];
				mbox_opfifo_m_n = mbox_opfifo_m_n - 1;
			end else opn = 3'd0;
			a = dut.cur_addr;
			d = dut.cur_rd ? dut.vp_rdata_m : dut.cur_data;
			if (a >= 16'hF000) begin
				idx = a - 16'hF000;
				if (dut.cur_rd) begin
					mbox_total_rd_m = mbox_total_rd_m + 1;
					mbox_rd_cnt_m[idx] = mbox_rd_cnt_m[idx] + 1;
				end else begin
					mbox_total_wr_m = mbox_total_wr_m + 1;
					mbox_wr_cnt_m[idx] = mbox_wr_cnt_m[idx] + 1;
				end
				mbox_addr_touched[idx] = 1'b1;
				if (mbox_distinct_n[idx] < 12) begin
					found = 1'b0;
					for (j = 0; j < mbox_distinct_n[idx]; j = j + 1)
						if (mbox_distinct_val[idx][j] == d) found = 1'b1;
					if (!found) begin
						mbox_distinct_val[idx][mbox_distinct_n[idx]] = d;
						mbox_distinct_n[idx] = mbox_distinct_n[idx] + 1;
					end
				end
				if (mbox_log_n < MBOX_LOG_CAP) begin
					mbox_log_side[mbox_log_n] = 1'b0;
					mbox_log_rd[mbox_log_n]   = dut.cur_rd;
					mbox_log_op[mbox_log_n]   = opn;
					mbox_log_addr[mbox_log_n] = a;
					mbox_log_data[mbox_log_n] = d;
					mbox_log_n = mbox_log_n + 1;
				end
			end
		end

		// -- pop: slave elementary op completed --
		if (dut.vp_pop_s) begin : mbox_pop_s_blk
			reg [2:0]  opn;
			reg [15:0] a;
			reg [7:0]  d;
			integer    idx, k, j;
			reg        found;
			if (mbox_opfifo_s_n > 0) begin
				opn = mbox_opfifo_s[0];
				for (k = 0; k < 7; k = k + 1) mbox_opfifo_s[k] = mbox_opfifo_s[k+1];
				mbox_opfifo_s_n = mbox_opfifo_s_n - 1;
			end else opn = 3'd0;
			a = dut.cur_addr;
			d = dut.cur_rd ? dut.vp_rdata_s : dut.cur_data;
			if (a >= 16'hF000) begin
				idx = a - 16'hF000;
				if (dut.cur_rd) begin
					mbox_total_rd_s = mbox_total_rd_s + 1;
					mbox_rd_cnt_s[idx] = mbox_rd_cnt_s[idx] + 1;
				end else begin
					mbox_total_wr_s = mbox_total_wr_s + 1;
					mbox_wr_cnt_s[idx] = mbox_wr_cnt_s[idx] + 1;
				end
				mbox_addr_touched[idx] = 1'b1;
				if (mbox_distinct_n[idx] < 12) begin
					found = 1'b0;
					for (j = 0; j < mbox_distinct_n[idx]; j = j + 1)
						if (mbox_distinct_val[idx][j] == d) found = 1'b1;
					if (!found) begin
						mbox_distinct_val[idx][mbox_distinct_n[idx]] = d;
						mbox_distinct_n[idx] = mbox_distinct_n[idx] + 1;
					end
				end
				if (mbox_log_n < MBOX_LOG_CAP) begin
					mbox_log_side[mbox_log_n] = 1'b1;
					mbox_log_rd[mbox_log_n]   = dut.cur_rd;
					mbox_log_op[mbox_log_n]   = opn;
					mbox_log_addr[mbox_log_n] = a;
					mbox_log_data[mbox_log_n] = d;
					mbox_log_n = mbox_log_n + 1;
				end
			end
		end
	end
end

// Follow-up 8 (docs/SESSION_2026-07-14.md) -- cram_we / mcont_r[1] gating
// fix evidence. The fix added `& mcont_r[1]` to sor_master.sv's cram_we
// (matching MAME's m_palette_view select(0)/disable() in
// leland_master_output_w case 0x09), on the hypothesis that ungated
// stray writes to 0xF000-0xF3FF were corrupting the fg-indexed Color RAM
// entries sor_video.sv's cram_addr={fg_pen,bg_pen} reads sprites/portrait/
// flag color from. A hardware reflash with the fix made NO visible
// difference (still no Ironman/flag/cars), so this traces the ACTUAL
// organic write attempts (not just the mcont_r write events already
// logged by IOWR) to find out whether the gate ever had any real effect
// in this run at all, and whether the fg-indexed half of Color RAM
// (address>=64, i.e. cram_addr[9:6]!=0) ever gets written by anything.
wire cram_attempt = dut.master.mem_access & ~dut.master.wr_n & dut.master.in_cram;
integer cram_attempt_count = 0;
integer cram_blocked_count = 0; // attempted while mcont_r[1]=0 -- these are what the fix now drops
integer cram_landed_count  = 0; // attempted while mcont_r[1]=1 -- these still land, same as before the fix
integer cram_fg_landed_count = 0; // landed AND address>=64 (the fg-relevant half)
reg cram_attempt_prev;
initial cram_attempt_prev = 1'b0;
always @(posedge clk_sys) begin
	if (cram_attempt && !cram_attempt_prev) begin
		cram_attempt_count = cram_attempt_count + 1;
		if (dut.master.mcont_r[1]) begin
			cram_landed_count = cram_landed_count + 1;
			if (dut.master.cpu_addr[9:6] != 4'd0) cram_fg_landed_count = cram_fg_landed_count + 1;
		end else begin
			cram_blocked_count = cram_blocked_count + 1;
		end
		if (cram_attempt_count <= 20 || (cram_attempt_count % 500 == 0))
			$display("t=%0t CRAM_WR #%0d addr=0x%04x(cramidx=%0d) data=0x%02x mcont_bit1=%0d pc=0x%04x %s",
			          $time, cram_attempt_count, dut.master.cpu_addr, dut.master.cpu_addr[9:0],
			          dut.master.cpu_dout, dut.master.mcont_r[1], dut.master.dbg_pc_r,
			          dut.master.mcont_r[1] ? "LANDED" : "BLOCKED");
	end
	cram_attempt_prev = cram_attempt;
end

// Raw M1-fetch (PC + opcode byte) trace for both CPUs, gated to start
// once the post-boot idle loop should be underway (t > 90ms) and
// bounded to a few hundred fetches -- I/O, VRAM, WRAM, and interrupt
// tracing all came up empty for what the idle loops at Master
//0x1283-0x128a / Slave ~0x0039-0x006a are actually doing, so the next
// step is reconstructing the real instructions by hand from raw bytes.
// Widened to 450ms-650ms: the Master's HL-countdown loop at 0x1283-
// 0x128a can take up to ~415ms worst-case (16-bit counter at 6 MHz),
// so a trace ending at t=90.3ms only ever shows one pass through it.
// This window straddles where the loop should exit (if it's a single
// bounded wait) so we can see whether the fall-through code does
// something new, or re-enters the same loop from an outer retry
// (explaining a real-hardware stall far longer than one 16-bit delay).
integer opcode_trace_count_m = 0, opcode_trace_count_s = 0;

// Ground-truth-diffable PC-only trace, unrated and running from reset
// fall. Reconstructing "settled" (addr,byte) pairs from the external
// bus (cpu_addr/rom_data/m1_n/mreq_n) proved fragile -- multiple
// attempts at picking the right sample cycle produced plausible-looking
// but wrong data at jump targets (e.g. spurious "0038: 0d" /
// "ffff: 45" entries that turned out to be bus-timing artifacts of the
// trace capture itself, not real CPU behavior). Tapping the tv80 core's
// own internal PC register (tv80_core.v:102) directly sidesteps all of
// that: log every clk_sys cycle where PC actually changes value. This
// gives the exact control-flow address sequence with zero ambiguity,
// diffable directly against the address column of
// docs/reference/mame/traces/mastertrace.log / slavetrace.log.
// Gated on the RISING edge of "in an M1 opcode-fetch cycle" (not just
// any PC change) so this only logs one address per REAL instruction
// boundary -- matching the granularity of MAME's own debugger `trace`
// output (docs/reference/mame/traces/*.log), which likewise only shows
// one line per executed instruction, not one per byte fetched. An
// earlier version logged on every PC register change, which also
// captures operand-byte fetches for multi-byte instructions (PC
// increments during those too) -- fine for the earlier targeted
// investigations here, but made a direct structural diff against the
// real traces (sim/difftrace.py) meaningless without this fix.
integer pc_trace_file_m, pc_trace_file_s;
integer pc_trace_count_m = 0, pc_trace_count_s = 0;
reg m1_prev_m, m1_prev_s;
initial begin
	pc_trace_file_m = $fopen("master_pc_trace.log", "w");
	pc_trace_file_s = $fopen("slave_pc_trace.log", "w");
	m1_prev_m = 1'b0;
	m1_prev_s = 1'b0;
end
always @(posedge clk_sys) begin
	if (dut.master.CE_6M) begin
		if (~dut.master.mreq_n && ~dut.master.m1_n && !m1_prev_m && pc_trace_count_m < 2_000_000) begin
			pc_trace_count_m = pc_trace_count_m + 1;
			$fdisplay(pc_trace_file_m, "%04x", dut.master.master_cpu.i_tv80_core.PC);
		end
		m1_prev_m = (~dut.master.mreq_n && ~dut.master.m1_n);
	end
	if (dut.slave.CE_6M) begin
		if (~dut.slave.mreq_n && ~dut.slave.m1_n && !m1_prev_s && pc_trace_count_s < 2_000_000) begin
			pc_trace_count_s = pc_trace_count_s + 1;
			$fdisplay(pc_trace_file_s, "%04x", dut.slave.slave_cpu.i_tv80_core.PC);
		end
		m1_prev_s = (~dut.slave.mreq_n && ~dut.slave.m1_n);
	end
end

// TEMP diagnostic: real MAME visits 0x376C (the boot-time WRAM-clear
// entry point) exactly ONCE in its entire 778K-instruction trace. User
// reports real hardware cycling repeatedly between ~0x61Fx and 0x377x
// after running much longer than our sim has covered so far -- this
// logs every time PC re-enters 0x376C after the first, plus whatever
// PC immediately preceded it, to tell a genuine CPU reset (would show
// pc_prev==0xFFFF-ish/garbage or a direct 0x0000 entry) apart from a
// deliberate JP back into this code from elsewhere.
integer revisit_376c_count = 0;
reg [15:0] pc_prev2_m;
always @(posedge clk_sys) begin
	if (dut.master.master_cpu.i_tv80_core.PC !== pc_prev2_m) begin
		if (dut.master.master_cpu.i_tv80_core.PC == 16'h376C && revisit_376c_count < 200) begin
			revisit_376c_count = revisit_376c_count + 1;
			$display("t=%0t REVISIT_376C #%0d prev_pc=0x%04x", $time, revisit_376c_count, pc_prev2_m);
		end
		pc_prev2_m = dut.master.master_cpu.i_tv80_core.PC;
	end
end
always @(posedge clk_sys) begin
	if ($time > 450_000_000 && dut.master.CE_6M && ~dut.master.mreq_n && ~dut.master.m1_n && opcode_trace_count_m < 400) begin
		opcode_trace_count_m = opcode_trace_count_m + 1;
		$display("t=%0t OPCODE_M #%0d pc=0x%04x byte=0x%02x", $time, opcode_trace_count_m, dut.master.cpu_addr, dut.master.rom_data);
	end
	if ($time > 450_000_000 && dut.slave.CE_6M && ~dut.slave.mreq_n && ~dut.slave.m1_n && opcode_trace_count_s < 400) begin
		opcode_trace_count_s = opcode_trace_count_s + 1;
		$display("t=%0t OPCODE_S #%0d pc=0x%04x byte=0x%02x", $time, opcode_trace_count_s, dut.slave.cpu_addr, dut.slave.rom_data);
	end
end

// TEMP diagnostic: confirmed root cause of the periodic Master restart
// -- rom_data at cpu_addr=0x2036 settles to 0xD0 instead of the real
// (file-verified-correct) 0xC3, specifically under heavy concurrent
// Master+Slave+video SDRAM traffic (never reproduced during quiet
// early-boot reads). The reset port itself was proven clean (no
// glitches), so this is a genuine SDRAM arbiter/controller race.
// Log the FULL arbiter state every cycle any read channel is active,
// windowed tightly around the known corruption timestamp (~810.26ms)
// to catch it without flooding the log across the whole run.
always @(posedge clk_sys) begin
	if ($time > 810_200_000 && $time < 810_300_000 &&
	    (dut.sdram_rd0_req || dut.sdram_rd1_req || dut.sdram_rd2_req || dut.sdram_wr_req || dut.in_flight)) begin
		$display("t=%0t ARB in_flight=%b issued_ch=%0d done_seen=%b sd_ready=%b sd_req_done=%b sd_rd=%b sd_we_word=%b sd_addr=0x%06x sd_dout=0x%02x rd0_req=%b rd0_ack=%b rd0_addr=0x%06x rd1_req=%b rd1_ack=%b rd1_addr=0x%06x wr_req=%b wr_ack=%b",
		          $time, dut.in_flight, dut.issued_ch, dut.done_seen, dut.sd_ready, dut.sd_req_done,
		          dut.sd_rd, dut.sd_we_word, dut.sd_addr, dut.sd_dout,
		          dut.sdram_rd0_req, dut.sdram_rd0_ack, dut.sdram_rd0_addr,
		          dut.sdram_rd1_req, dut.sdram_rd1_ack, dut.sdram_rd1_addr,
		          dut.sdram_wr_req, dut.sdram_wr_ack);
	end
end

// Shared WRAM (0xE000-0xEFFF) read/write trace for both CPUs -- the two
// CPUs went idle in lockstep right after the Slave's first VRAM-write
// burst, with the Master's interrupts masked (intack_cnt stuck at 0),
// suggesting a busy-poll handshake through shared WRAM rather than the
// VRAM mailbox or an interrupt. Dedup reads like the IORD trace so a
// tight poll loop logs once instead of flooding.
integer wram_wr_count_m = 0, wram_wr_count_s = 0;
always @(posedge clk_sys) begin
	if (dut.master.CE_6M && dut.wram_we_m) begin
		wram_wr_count_m = wram_wr_count_m + 1;
		if (wram_wr_count_m <= 16 || (wram_wr_count_m % 20000 == 0))
			$display("t=%0t WRAM_WR_M #%0d addr=0x%03x data=0x%02x pc=0x%04x",
			          $time, wram_wr_count_m, dut.wram_addr_m, dut.wram_din_m, dut.dbg_pc);
	end
	if (dut.slave.CE_6M && dut.wram_we_s) begin
		wram_wr_count_s = wram_wr_count_s + 1;
		if (wram_wr_count_s <= 16 || (wram_wr_count_s % 20000 == 0))
			$display("t=%0t WRAM_WR_S #%0d addr=0x%03x data=0x%02x pc=0x%04x",
			          $time, wram_wr_count_s, dut.wram_addr_s, dut.wram_din_s, slave_dbg_pc);
	end
end

integer wram_rd_count_m = 0, wram_rd_count_s = 0;
reg [11:0] last_wram_rd_addr_m = 12'hzzz, last_wram_rd_addr_s = 12'hzzz;
reg [15:0] last_wram_rd_pc_m   = 16'hzzzz, last_wram_rd_pc_s   = 16'hzzzz;
always @(posedge clk_sys) begin
	if (!reset && dut.master.CE_6M && dut.master.mem_access && ~dut.master.rd_n && dut.master.in_wram &&
	    (dut.master.cpu_addr[11:0] != last_wram_rd_addr_m || dut.dbg_pc != last_wram_rd_pc_m)) begin
		wram_rd_count_m = wram_rd_count_m + 1;
		$display("t=%0t WRAM_RD_M #%0d addr=0x%03x data=0x%02x pc=0x%04x",
		          $time, wram_rd_count_m, dut.master.cpu_addr[11:0], dut.wram_dout_m, dut.dbg_pc);
		last_wram_rd_addr_m = dut.master.cpu_addr[11:0];
		last_wram_rd_pc_m   = dut.dbg_pc;
	end
	if (!reset && dut.slave.CE_6M && dut.slave.mem_access && ~dut.slave.rd_n && dut.slave.in_wram &&
	    (dut.slave.cpu_addr[11:0] != last_wram_rd_addr_s || slave_dbg_pc != last_wram_rd_pc_s)) begin
		wram_rd_count_s = wram_rd_count_s + 1;
		$display("t=%0t WRAM_RD_S #%0d addr=0x%03x data=0x%02x pc=0x%04x",
		          $time, wram_rd_count_s, dut.slave.cpu_addr[11:0], dut.wram_dout_s, slave_dbg_pc);
		last_wram_rd_addr_s = dut.slave.cpu_addr[11:0];
		last_wram_rd_pc_s   = slave_dbg_pc;
	end
end

// Slave I/O write trace: mirrors the Master trace above. We had zero
// visibility into the Slave CPU until this was added -- it could be
// stuck in its own loop (never reaching a VRAM write) with no way to
// tell from the Master-only trace. Now that the VRAM-mirror fix has
// the Slave actively pixel-plotting, this fires continuously (one
// entry per pixel write); rate-limit like VRAM_WR so a real drawing
// loop doesn't flood the transcript.
integer slave_io_wr_count = 0;
always @(posedge clk_sys) begin
	if (dut.slave.CE_6M && dut.slave.io_wr) begin
		slave_io_wr_count = slave_io_wr_count + 1;
		if (slave_io_wr_count <= 8 || (slave_io_wr_count % 20000 == 0))
			$display("t=%0t SLV_IOWR #%0d addr=0x%02x data=0x%02x pc=0x%04x",
			          $time, slave_io_wr_count, dut.slave.cpu_addr[7:0], dut.slave.cpu_dout, slave_dbg_pc);
	end
end

integer slave_io_rd_count = 0;
reg [7:0]  slave_last_io_rd_addr = 8'hzz;
reg [15:0] slave_last_io_rd_pc   = 16'hzzzz;
always @(posedge clk_sys) begin
	if (!reset && dut.slave.CE_6M && dut.slave.io_rd &&
	    (dut.slave.cpu_addr[7:0] != slave_last_io_rd_addr || slave_dbg_pc != slave_last_io_rd_pc)) begin
		slave_io_rd_count = slave_io_rd_count + 1;
		$display("t=%0t SLV_IORD #%0d addr=0x%02x data=0x%02x pc=0x%04x",
		          $time, slave_io_rd_count, dut.slave.cpu_addr[7:0], dut.slave.cpu_din, slave_dbg_pc);
		slave_last_io_rd_addr = dut.slave.cpu_addr[7:0];
		slave_last_io_rd_pc   = slave_dbg_pc;
	end
end

// Periodic PC sample so a hung/looping boot shows up as a narrow
// repeating PC range instead of silence. Now covers both CPUs.
// dbg_irq_cnt (Master raster/VA10 interrupt firing count), an intack
// (M1&IORQ) cycle count, and the live periodic_int_n level are included
// to distinguish "CPU stopped re-arming interrupts" (intack_count frozen,
// periodic_int_n stuck low) from a genuine per-frame VBlank wait.
`ifdef GAMEPLAY_REPRO
// How long to let the injected snapshot run before giving up and
// reporting a stall. $BDAB is hit many times per second in real
// gameplay (part of the per-frame object-update loop), so a real
// repro should hit it within the first handful of ms; 200ms @ 48MHz
// is generous headroom. bdab_hit_count/master_pc_stall_count let the
// final report distinguish "never even got moving" from "ran fine but
// never reached BDAB" from "reached BDAB, then froze there".
localparam integer bdab_repro_run_cycles = 9_600_000; // 200ms @ 48MHz
integer bdab_hit_count = 0;
integer master_pc_stall_count = 0;
reg [15:0] last_dbg_pc_seen = 16'hzzzz;
integer same_pc_streak = 0;
always @(posedge clk_sys) begin
	if (dut.master.CE_6M && ~dut.master.mreq_n && ~dut.master.m1_n) begin
		if (dut.master.cpu_addr == 16'hBDAB) begin
			bdab_hit_count = bdab_hit_count + 1;
			if (bdab_hit_count <= 20 || (bdab_hit_count % 5000 == 0))
				$display("t=%0t BDAB_HIT #%0d", $time, bdab_hit_count);
		end
		if (dut.master.cpu_addr == last_dbg_pc_seen) begin
			same_pc_streak = same_pc_streak + 1;
			if (same_pc_streak == 480_000) begin // ~10ms stuck on one PC
				master_pc_stall_count = master_pc_stall_count + 1;
				$display("t=%0t MASTER_PC_STALL pc=0x%04x (same M1-fetch PC for ~10ms)", $time, dut.master.cpu_addr);
			end
		end else begin
			same_pc_streak = 0;
			last_dbg_pc_seen = dut.master.cpu_addr;
		end
	end
end
`endif

integer pc_sample_count = 0;
integer gin3_rd_count = 0;
integer mcont_wr_count = 0;
integer intack_count = 0;
always @(posedge clk_sys) begin
	if (dut.master.CE_6M && dut.master.io_rd && dut.master.io_gin3) gin3_rd_count = gin3_rd_count + 1;
	if (dut.master.CE_6M && dut.master.io_wr && dut.master.io_mcont) mcont_wr_count = mcont_wr_count + 1;
	if (dut.master.CE_6M && ~dut.master.iorq_n && ~dut.master.m1_n) intack_count = intack_count + 1;
	if (!reset && (pc_sample_count % 200000 == 0))
		$display("t=%0t PC_SAMPLE m_pc=0x%04x s_pc=0x%04x slave_reset_n=%b vram_wr_count=%0d irq_cnt=%0d intack_cnt=%0d periodic_int_n=%b gin3_rd=%0d mcont_wr=%0d vblank=%b",
		          $time, dut.dbg_pc, slave_dbg_pc, dut.slave_reset_n, vram_wr_count,
		          dut.dbg_irq_cnt, intack_count, dut.master.periodic_int_n, gin3_rd_count, mcont_wr_count, dut.VBlank);
	if (!reset) pc_sample_count = pc_sample_count + 1;
end

//------------------------------------------------------------------
// NMI wiring verification probe (2026-07-14 fix): rtl/sor_slave.sv's
// tv80 core previously had nmi_n hardwired 1'b1 (dead wire), so the
// Slave's NMI handler ($0066: increments WRAM $EF06) could never run
// and $EF06 could never move off its reset value of 0x00. Watch $EF06
// (WRAM offset 0xF06, i.e. 0xE000-based shared WRAM, dut.wram_s.mem[])
// and slave_halt_n across the whole run: PASS = $EF06 goes nonzero at
// least once (NMI ticks are arriving) with no HALT-stuck condition
// (slave_halt_n recovers after any halt rather than freezing low).
//------------------------------------------------------------------
reg  [7:0] ef06_last          = 8'h00;
integer    ef06_nonzero_count = 0;
integer    ef06_toggle_count  = 0;
reg        ef06_ever_nonzero  = 1'b0;
reg        halt_seen          = 1'b0;
reg        halt_stuck_flag    = 1'b0;
integer    halt_low_streak    = 0;
always @(posedge clk_sys) begin
	if (!reset) begin
		if (dut.wram_s.mem[12'hF06] != ef06_last) begin
			ef06_toggle_count = ef06_toggle_count + 1;
			if (ef06_toggle_count <= 20)
				$display("t=%0t EF06_CHANGE old=%02x new=%02x", $time, ef06_last, dut.wram_s.mem[12'hF06]);
			ef06_last = dut.wram_s.mem[12'hF06];
		end
		if (dut.wram_s.mem[12'hF06] != 8'h00) begin
			ef06_ever_nonzero  = 1'b1;
			ef06_nonzero_count = ef06_nonzero_count + 1;
		end

		if (!dut.slave_halt_n) begin
			halt_seen       = 1'b1;
			halt_low_streak = halt_low_streak + 1;
			if (halt_low_streak == 480_000) begin // ~10ms continuously halted
				halt_stuck_flag = 1'b1;
				$display("t=%0t SLAVE_HALT_STUCK: slave_halt_n low for ~10ms continuously", $time);
			end
		end else begin
			halt_low_streak = 0;
		end
	end
end

final begin
	$display("=== NMI probe summary: EF06 ever_nonzero=%b nonzero_samples=%0d toggle_count=%0d halt_seen=%b halt_stuck=%b ===",
	          ef06_ever_nonzero, ef06_nonzero_count, ef06_toggle_count, halt_seen, halt_stuck_flag);
end

//------------------------------------------------------------------
// DIRECTED NMI STIMULUS (2026-07-14, TEST-ONLY -- not RTL behavior):
// the passive probe above watches for the Master's own firmware to
// clear /MCONT bit2 organically, but this TB's ~900ms boot/idle
// window never does so (confirmed by decoding every observed /MCONT
// write in this run: bit2=1 -- NMI held CLEAR -- on all of them, e.g.
// data=0x1d/0x15/0x0d/... all have bit 2 set). So the passive probe's
// EF06-stays-zero result is an UNREACHABLE-CRITERION outcome, not
// evidence the fix is broken. To actually exercise the new nmi_n
// wiring, force/release the Master's own `mcont_r[2]` shadow bit --
// the exact storage element `assign slave_nmi_n = mcont_r[2]` reads
// from in rtl/sor_master.sv -- to inject one realistic ASSERT-then-
// CLEAR NMI edge once boot has settled, then confirm $EF06 (the
// Slave's NMI-handler counter, rtl disassembly: $0066 increments it)
// moves as a direct, immediate result. Real hardware's actual NMI
// cadence comes from the Master's periodic ISR elsewhere in its ROM,
// not from this block -- this only proves the wire is alive end to
// end, not that the Master exercises it during this window.
//------------------------------------------------------------------
initial begin
	reg [7:0] ef06_before, ef06_after;
	wait (dut.slave_reset_n);
	repeat (200_000) @(posedge clk_sys); // let boot settle into a steady idle/poll state
	ef06_before = dut.wram_s.mem[12'hF06];
	$display("t=%0t === DIRECTED NMI TEST: EF06 before=%02x -- forcing mcont_r[2]=0 (assert NMI) ===",
	          $time, ef06_before);
	force dut.master.mcont_r[2] = 1'b0;
	repeat (100) @(posedge clk_sys); // hold low across several CE_6M cycles for tv80's edge detect
	release dut.master.mcont_r[2];
	$display("t=%0t === DIRECTED NMI TEST: released mcont_r[2] (resumes normal /MCONT writes) ===", $time);
	repeat (10_000) @(posedge clk_sys); // give the Slave time to reach $0066 and increment EF06
	ef06_after = dut.wram_s.mem[12'hF06];
	$display("t=%0t === DIRECTED NMI TEST RESULT: EF06 before=%02x after=%02x moved=%b (probe totals: ever_nonzero=%b toggle_count=%0d) ===",
	          $time, ef06_before, ef06_after, (ef06_after != ef06_before),
	          ef06_ever_nonzero, ef06_toggle_count);
end

//------------------------------------------------------------------
// VRAM PORT OP-BY-OP + OTIR-BURST DIRECTED TEST (2026-07-14, TEST-ONLY,
// not RTL behavior). Follow-up 4 (docs/SESSION_2026-07-14.md) named the
// Master's OTIR VRAM burst-write routines (0x612c-0x6151 family, real
// port 0x0B = op3/inc/non-trans) and the Slave-only transparent-write
// ops (SEQ_TRD/TPOP/TWR/TWPOP in rtl/sor_board.sv) as the next suspect
// for the hardware garbage-spray symptom, and noted neither had been
// exercised by any sim run so far -- only small mailbox clear/poll
// traffic had been traced up to that point.
//
// This drives rtl/sor_vram_port.sv's Slave instance (dut.slave.vport,
// TRANS_EN=1) directly at its I/O boundary -- forcing
// cpu_addr/cpu_dout/io_wr/io_rd/io_vram_sel exactly as a real OUT/IN
// ($xx) bus cycle would present them to the module, honoring vp_stall
// the same way a real Z80's /WAIT input would (never proceeding to the
// next op until the current one actually commits) -- to get
// deterministic, exhaustive coverage of every op (1,2,3,5,6; transparent
// and non-transparent; read and write; the two "unknown op" default
// cases) plus:
//   - a burst paced at the real "otir" instruction's timing (21
//     T-states/CE_6M ticks between OUTs, matching a real OTIR loop
//     executing port 0x0B), and
//   - a synthetic tight-pacing stress burst (2 CE_6M ticks between
//     OUTs -- far faster than any real Z80 instruction sequence could
//     issue them) to specifically hammer the 2-deep op queue's
//     vp_stall/backpressure contract harder than real hardware ever
//     would.
// Both Z80 cores are frozen (reset_n forced low) for the duration so no
// organic boot traffic can interleave with the directed stimulus.
// PASS = every byte lands at exactly the address this testbench's own
// reference model (mirroring MAME leland_v.cpp's vram_port_r/w
// addressing/merge math, already audited byte-for-byte equivalent to
// rtl/sor_vram_port.sv in an earlier session) predicts, with no
// drop/duplicate, under both pacings.
//
// Gated behind +define+VPTEST: the test freezes and later un-freezes
// both Z80 cores (restarting them from PC=0), which would perturb the
// standard boot-watchdog regression if it ran unconditionally.
//------------------------------------------------------------------
`ifdef VPTEST
integer vp_errors = 0;
integer vp_checks = 0;

task vp_write(input [4:0] op5, input [7:0] data, input integer gap_cycles);
	begin
		force dut.slave.vport.cpu_addr    = {11'd0, op5};
		force dut.slave.vport.cpu_dout    = data;
		force dut.slave.vport.io_vram_sel = 1'b1;
		force dut.slave.vport.io_wr       = 1'b1;
		wait (dut.slave.vport.io_wr_done);
		force dut.slave.vport.io_wr       = 1'b0;
		wait (!dut.slave.vport.io_wr_done);
		release dut.slave.vport.io_wr;
		release dut.slave.vport.io_vram_sel;
		release dut.slave.vport.cpu_dout;
		release dut.slave.vport.cpu_addr;
		repeat (gap_cycles) @(posedge clk_sys); // let the sequencer drain + model OUT-to-OUT pacing
	end
endtask

task vp_read(input [4:0] op5, output [7:0] data_out);
	begin
		force dut.slave.vport.cpu_addr    = {11'd0, op5};
		force dut.slave.vport.io_vram_sel = 1'b1;
		force dut.slave.vport.io_rd       = 1'b1;
		wait (dut.slave.vport.io_rd_done);
		force dut.slave.vport.io_rd       = 1'b0;
		wait (!dut.slave.vport.io_rd_done);
		release dut.slave.vport.io_rd;
		release dut.slave.vport.io_vram_sel;
		release dut.slave.vport.cpu_addr;
		repeat (10) @(posedge clk_sys); // let the sequencer's SEQ_ADDR/SEQ_POP drain
		data_out = dut.slave.vport.rd_data;
	end
endtask

task automatic vp_check(input [23*8-1:0] name, input [7:0] got, input [7:0] exp);
	begin
		vp_checks = vp_checks + 1;
		if (got !== exp) begin
			vp_errors = vp_errors + 1;
			$display("t=%0t VPTEST FAIL %0s: got=%02x expected=%02x", $time, name, got, exp);
		end
	end
endtask

// Reference model for op3's address math -- mirrors MAME leland_v.cpp
// exactly (`addr += inc & (addr << 1); addr ^= 1;`) and
// rtl/sor_vram_port.sv's addr_op3_next: increment (+2) lands only when
// inc is requested AND the address is currently odd, then bit0 toggles.
function automatic [15:0] vp_ref_op3_next(input [15:0] a, input inc);
	vp_ref_op3_next = (a + ((inc && a[0]) ? 16'd2 : 16'd0)) ^ 16'd1;
endfunction

initial begin
	reg [7:0] rd;
	reg [15:0] a3;
	integer i;
	reg [15:0] burst_addr [0:63];
	reg  [7:0] burst_data [0:63];
	integer burst_errors;

	wait (dut.slave_reset_n);
	repeat (50_000) @(posedge clk_sys); // let boot settle, same margin as the NMI directed test

	$display("t=%0t === VRAM PORT DIRECTED TEST: freezing both cores, driving dut.slave.vport directly ===", $time);
	force dut.master.master_cpu.reset_n = 1'b0;
	force dut.slave.slave_cpu.reset_n   = 1'b0;
	repeat (20) @(posedge clk_sys);

	// --- op6 write (no inc, no trans): primes latch0, verify direct write ---
	force dut.slave.vport.addr_q = 16'h2000;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	vp_write(5'b00_110, 8'hA5, 10); // trans=0 inc=0 op=6
	vp_check("op6_write",      dut.vram.mem[17'h2000], 8'hA5);
	vp_check("op6_latch0",     dut.slave.vport.latch0, 8'hA5);
	vp_check("op6_addr_noinc", dut.slave.vport.addr_q[7:0], 8'h00); // unchanged low byte

	// --- op5 write (inc=1, no trans): primes latch1, verify write + addr+=2 ---
	force dut.slave.vport.addr_q = 16'h2000;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	vp_write(5'b01_101, 8'h5A, 10); // trans=0 inc=1 op=5
	vp_check("op5_write",   dut.vram.mem[17'h2001], 8'h5A);
	vp_check("op5_latch1",  dut.slave.vport.latch1, 8'h5A);
	vp_check("op5_addr_inc",dut.slave.vport.addr_q[7:0], 8'h02); // +2 unconditional

	// --- op1 write: even byte <- latch0 (set above =0xA5), odd byte <- data ---
	force dut.slave.vport.addr_q = 16'h3000;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	vp_write(5'b00_001, 8'h22, 10); // trans=0 inc=0 op=1
	vp_check("op1_even_latch0", dut.vram.mem[17'h3000], 8'hA5);
	vp_check("op1_odd_data",    dut.vram.mem[17'h3001], 8'h22);

	// --- op2 write: even byte <- data, odd byte <- latch1 (set above =0x5A) ---
	force dut.slave.vport.addr_q = 16'h3010;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	vp_write(5'b00_010, 8'h66, 10); // trans=0 inc=0 op=2
	vp_check("op2_even_data",   dut.vram.mem[17'h3010], 8'h66);
	vp_check("op2_odd_latch1",  dut.vram.mem[17'h3011], 8'h5A);

	// --- op3 write, alternating, starting EVEN with inc=1: addr toggles to
	// odd with no increment (inc only lands when addr is ODD), then next
	// write lands even again WITH the +2 (matches vp_ref_op3_next) ---
	force dut.slave.vport.addr_q = 16'h4000; // even
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	a3 = 16'h4000;
	vp_write(5'b01_011, 8'h11, 10); // trans=0 inc=1 op=3
	vp_check("op3_w0", dut.vram.mem[a3], 8'h11);
	a3 = vp_ref_op3_next(a3, 1'b1); // -> 0x4001, no +2 (was even)
	vp_check("op3_addr1_lo", dut.slave.vport.addr_q[7:0], a3[7:0]);
	vp_write(5'b01_011, 8'h12, 10);
	vp_check("op3_w1", dut.vram.mem[a3], 8'h12);
	a3 = vp_ref_op3_next(a3, 1'b1); // was odd -> +2 lands, -> 0x4002
	vp_check("op3_addr2_lo", dut.slave.vport.addr_q[7:0], a3[7:0]);
	vp_write(5'b01_011, 8'h13, 10);
	vp_check("op3_w2", dut.vram.mem[a3], 8'h13);

	// --- op3/op5/op6 reads: verify addressing mirrors the write side and
	// returned data matches what was just written above ---
	force dut.slave.vport.addr_q = 16'h4000;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	vp_read(5'b01_011, rd); vp_check("op3_read0", rd, 8'h11); // trans bit ignored on reads
	vp_read(5'b01_011, rd); vp_check("op3_read1", rd, 8'h12);
	force dut.slave.vport.addr_q = 16'h3000;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	vp_read(5'b00_110, rd); vp_check("op6_read_lo", rd, 8'hA5); // addr&~1
	vp_read(5'b00_101, rd); vp_check("op5_read_hi", rd, 8'h22); // addr|1, inc=0 so still 0x3000/1

	// --- unknown write op (op=0): no side effect, VRAM/addr unchanged ---
	force dut.slave.vport.addr_q = 16'h5000;
	dut.vram.mem[17'h5000] = 8'h99;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	vp_write(5'b00_000, 8'hFF, 10);
	vp_check("unknown_write_no_sideeffect", dut.vram.mem[17'h5000], 8'h99);
	vp_check("unknown_write_addr_unchanged", dut.slave.vport.addr_q[7:0], 8'h00);

	// --- unknown read op (op=4): returns 0 ---
	force dut.slave.vport.addr_q = 16'h5000;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	vp_read(5'b00_100, rd);
	vp_check("unknown_read_returns_zero", rd, 8'h00);

	// --- transparent op6 write (Slave-only nibble merge): preserve-low,
	// preserve-high, preserve-both, overwrite-both ---
	dut.vram.mem[17'h6000] = 8'hC3; // 1100_0011: hi=C lo=3
	force dut.slave.vport.addr_q = 16'h6000;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	vp_write(5'b10_110, 8'hF0, 10); // trans=1 inc=0 op=6; lo nibble 0 -> preserve old lo (3)
	vp_check("trans6_preserve_lo", dut.vram.mem[17'h6000], 8'hF3);
	vp_write(5'b10_110, 8'h0A, 10); // hi nibble 0 -> preserve current hi (F)
	vp_check("trans6_preserve_hi", dut.vram.mem[17'h6000], 8'hFA);
	vp_write(5'b10_110, 8'h00, 10); // both nibbles 0 -> fully preserved
	vp_check("trans6_preserve_both", dut.vram.mem[17'h6000], 8'hFA);
	vp_write(5'b10_110, 8'hFF, 10); // both nonzero -> full overwrite
	vp_check("trans6_overwrite_both", dut.vram.mem[17'h6000], 8'hFF);

	// --- transparent op3 write: same merge semantics, plus addr still
	// toggles/increments exactly like the non-transparent case ---
	dut.vram.mem[17'h6100] = 8'h5C; // hi=5 lo=C
	force dut.slave.vport.addr_q = 16'h6100;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	a3 = 16'h6100;
	vp_write(5'b11_011, 8'hA0, 10); // trans=1 inc=1 op=3; lo=0 -> preserve old lo (C)
	vp_check("trans3_merge", dut.vram.mem[a3], 8'hAC);
	a3 = vp_ref_op3_next(a3, 1'b1);
	vp_check("trans3_addr_toggle", dut.slave.vport.addr_q[7:0], a3[7:0]);

	// --- OTIR-paced burst write via the real burst port (0x0B = trans=0
	// inc=1 op=3, exactly what "otir"/port 0x0B in the 0x612c-0x6151
	// Master ROM family uses): 48 bytes, 21-CE_6M-tick (168 clk_sys
	// cycle) gap between OUTs, matching the real "otir" instruction's
	// per-iteration timing at 6 MHz. PASS = every byte lands exactly
	// where the reference model predicts, no drop/duplicate. ---
	force dut.slave.vport.addr_q = 16'h7000;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	a3 = 16'h7000;
	burst_errors = 0;
	for (i = 0; i < 48; i = i + 1) begin
		burst_addr[i] = a3;
		burst_data[i] = 8'hA0 + i[7:0];
		vp_write(5'b01_011, burst_data[i], 168); // realistic OTIR pacing
		a3 = vp_ref_op3_next(a3, 1'b1);
	end
	for (i = 0; i < 48; i = i + 1) begin
		if (dut.vram.mem[burst_addr[i]] !== burst_data[i]) begin
			burst_errors = burst_errors + 1;
			$display("t=%0t VPTEST FAIL otir_paced_burst[%0d] addr=%04x got=%02x expected=%02x",
			          $time, i, burst_addr[i], dut.vram.mem[burst_addr[i]], burst_data[i]);
		end
	end
	vp_checks = vp_checks + 1;
	if (burst_errors) vp_errors = vp_errors + 1;
	$display("t=%0t VPTEST otir_paced_burst: %0d/48 bytes correct, final addr_q=%04x (expected %04x)",
	          $time, 48 - burst_errors, dut.slave.vport.addr_q, a3);

	// --- Tight-pacing stress burst: same 0x0B port, same 48-byte pattern,
	// but only 2 CE_6M ticks (16 clk_sys cycles) between OUTs -- far
	// faster than any real "otir" loop, deliberately hammering the
	// 2-deep queue's vp_stall backpressure. PASS here (no drop/duplicate
	// even under this unrealistic pacing) is the strongest evidence the
	// queue's "hold, never drop" contract (sor_vram_port.sv's 2026-07-12
	// header note) actually holds. ---
	force dut.slave.vport.addr_q = 16'h7100;
	@(posedge clk_sys);
	release dut.slave.vport.addr_q;
	a3 = 16'h7100;
	burst_errors = 0;
	for (i = 0; i < 48; i = i + 1) begin
		burst_addr[i] = a3;
		burst_data[i] = 8'h50 + i[7:0];
		vp_write(5'b01_011, burst_data[i], 16); // tight/stress pacing
		a3 = vp_ref_op3_next(a3, 1'b1);
	end
	for (i = 0; i < 48; i = i + 1) begin
		if (dut.vram.mem[burst_addr[i]] !== burst_data[i]) begin
			burst_errors = burst_errors + 1;
			$display("t=%0t VPTEST FAIL tight_stress_burst[%0d] addr=%04x got=%02x expected=%02x",
			          $time, i, burst_addr[i], dut.vram.mem[burst_addr[i]], burst_data[i]);
		end
	end
	vp_checks = vp_checks + 1;
	if (burst_errors) vp_errors = vp_errors + 1;
	$display("t=%0t VPTEST tight_stress_burst: %0d/48 bytes correct, final addr_q=%04x (expected %04x)",
	          $time, 48 - burst_errors, dut.slave.vport.addr_q, a3);

	$display("t=%0t === VRAM PORT DIRECTED TEST DONE: %0d checks, %0d errors -- %0s ===",
	          $time, vp_checks, vp_errors, (vp_errors == 0) ? "PASS" : "FAIL");

	release dut.master.master_cpu.reset_n;
	release dut.slave.slave_cpu.reset_n;
	$finish;
end
`endif // VPTEST

//------------------------------------------------------------------
// SLAVE ROM BANK-MAP DIRECTED TEST (2026-07-15, TEST-ONLY, not RTL
// behavior). Follow-up 5 (docs/SESSION_2026-07-14.md) found rtl/sor_slave.sv
// had been implementing MAME leland.cpp's slave_small_map_program (bank
// register at memory 0xF803, 48 KB banked window at 0x2000-0xDFFF), but
// offroad's machine config (`lelandi`) actually installs
// slave_large_map_program: bank register at 0xC000 (exact address,
// memory WRITE), 32 KB banked window at 0x4000-0xBFFF, and bank_base
// arithmetic 0x10000+0x8000*(data&0xF) (MAME leland_m.cpp
// slave_large_banksw_w) instead of the old ad-hoc bank0->0x30000 LUT.
// Since the ROM's own bank-switch sites (`ld ($C000),a`, nine of them
// per docs/reference/mame/traces/slavedump.asm) never touched F803, the
// bank register in the old RTL never changed and every banked read used
// the wrong base -- explaining the missing Ironman portrait/checkered
// flag, boxy car sprites, and race-end garbage spray.
//
// This drives dut.slave's internal Z80-bus signals (cpu_addr/cpu_dout/
// mreq_n/rfsh_n/rd_n/wr_n) directly with `force`, exactly like the
// VPTEST harness above does for dut.slave.vport -- both Z80 cores are
// frozen (reset_n forced low) first so no organic boot traffic can
// interleave with the directed stimulus. It checks:
//   (a) a memory write of N to 0xC000 updates bank_reg to N;
//   (b) a subsequent read at address A in 0x4000-0xBFFF computes
//       rom_addr = 0x10000 + 0x8000*N + (A-0x4000) (checked immediately,
//       purely combinational -- no SDRAM wait needed), AND once the
//       real SDRAM path (sor_board's sdram_rd1_* arbitration) services
//       the request, rom_data matches the real byte sitting in the ROM
//       chip file on disk at the corresponding offset -- not just the
//       RTL's own arithmetic re-checking itself.
//
// Gated behind +define+SLAVEBANKTEST for the same reason VPTEST is:
// freezing/unfreezing both cores would perturb the standard boot-
// watchdog regression if it ran unconditionally.
//------------------------------------------------------------------
`ifdef SLAVEBANKTEST
integer sb_errors = 0;
integer sb_checks = 0;

task automatic sb_check(input [23*8-1:0] name, input [18:0] got, input [18:0] exp);
	begin
		sb_checks = sb_checks + 1;
		if (got !== exp) begin
			sb_errors = sb_errors + 1;
			$display("t=%0t SLAVEBANKTEST FAIL %0s: got=%05x expected=%05x", $time, name, got, exp);
		end
	end
endtask

// Forces a memory write of `n` to the slave's 0xC000 bank register,
// held across a full CE_6M period (8 clk_sys cycles) so the
// `always @(posedge clk_sys) if (CE_6M && ...)` in sor_slave.sv is
// guaranteed to sample it at least once, then releases the forces.
task sb_write_bank(input [3:0] n);
	begin
		force dut.slave.cpu_addr = 16'hC000;
		force dut.slave.cpu_dout = {4'b0, n};
		force dut.slave.mreq_n   = 1'b0;
		force dut.slave.rfsh_n   = 1'b1;
		force dut.slave.wr_n     = 1'b0;
		repeat (12) @(posedge clk_sys); // > one full 8-cycle CE_6M period
		release dut.slave.wr_n;
		release dut.slave.rfsh_n;
		release dut.slave.mreq_n;
		release dut.slave.cpu_dout;
		release dut.slave.cpu_addr;
		repeat (4) @(posedge clk_sys);
	end
endtask

// Forces a memory read at `addr`, samples the combinational rom_addr
// immediately, then waits for the real SDRAM path to service the
// request (slave_rom_stall falling) before sampling rom_data.
task sb_read_banked(input [15:0] addr, output [18:0] rom_addr_out, output [7:0] rom_data_out);
	begin
		force dut.slave.cpu_addr = addr;
		force dut.slave.mreq_n   = 1'b0;
		force dut.slave.rfsh_n   = 1'b1;
		force dut.slave.rd_n     = 1'b0;
		force dut.slave.wr_n     = 1'b1;
		@(posedge clk_sys);
		rom_addr_out = dut.slave.rom_addr; // combinational; no wait needed
		while (dut.slave_rom_stall) @(posedge clk_sys);
		@(posedge clk_sys); // let slave_rom_data_r register settle
		rom_data_out = dut.slave.rom_data;
		release dut.slave.wr_n;
		release dut.slave.rd_n;
		release dut.slave.rfsh_n;
		release dut.slave.mreq_n;
		release dut.slave.cpu_addr;
		repeat (4) @(posedge clk_sys);
	end
endtask

initial begin
	reg [18:0] ra;
	reg [7:0]  rd_byte;

	wait (dut.slave_reset_n);
	repeat (50_000) @(posedge clk_sys); // let boot settle, same margin as VPTEST

	$display("t=%0t === SLAVE BANK-MAP DIRECTED TEST: freezing both cores, driving dut.slave bus directly ===", $time);
	force dut.master.master_cpu.reset_n = 1'b0;
	force dut.slave.slave_cpu.reset_n   = 1'b0;
	repeat (20) @(posedge clk_sys);

	// --- (a) bank_reg write check: write 6, confirm bank_reg captured it ---
	sb_write_bank(4'd6);
	sb_check("bankreg_write_6", {15'd0, dut.slave.bank_reg}, 19'd6);

	// --- (b) bank 6 (base 0x40000, u5t low half): A=0x4010 -> rom offset
	// 0x40000+0x10=0x40010 -> flat slave-region offset 0x40010, which is
	// u5t file offset 0x10. Real byte confirmed via PowerShell file read
	// during test authoring: 03-22109-02.u5t[0x10] = 0x1F. ---
	sb_read_banked(16'h4010, ra, rd_byte);
	sb_check("bank6_rom_addr", ra, 19'h40010);
	sb_check("bank6_rom_data", {11'd0, rd_byte}, 19'h0001F);

	// --- bank 4 (base 0x30000, u4t low half): A=0x5234 -> offset
	// 0x30000+0x1234=0x31234, i.e. u4t file offset 0x1234 = 0x15. ---
	sb_write_bank(4'd4);
	sb_check("bankreg_write_4", {15'd0, dut.slave.bank_reg}, 19'd4);
	sb_read_banked(16'h5234, ra, rd_byte);
	sb_check("bank4_rom_addr", ra, 19'h31234);
	sb_check("bank4_rom_data", {11'd0, rd_byte}, 19'h00015);

	// --- bank 13 (base 0x78000, u8t high half): A=0xBFFF (last byte of
	// the banked window) -> offset 0x78000+0x7FFF=0x7FFFF, i.e. u8t file
	// offset 0xFFFF (last byte of u8t) = 0x66. Also exercises rom_addr's
	// top bit (19 bits wide; 0x7FFFF fits). ---
	sb_write_bank(4'd13);
	sb_check("bankreg_write_13", {15'd0, dut.slave.bank_reg}, 19'd13);
	sb_read_banked(16'hBFFF, ra, rd_byte);
	sb_check("bank13_rom_addr", ra, 19'h7FFFF);
	sb_check("bank13_rom_data", {11'd0, rd_byte}, 19'h00066);

	// --- bank 0 (base 0x10000, zero-fill gap): A=0x5000 -> offset
	// 0x10000+0x1000=0x11000, inside the zero-fill region -- expect 0x00,
	// faithful to MAME (the real slave ROM region is unpopulated there
	// too). Also doubles as the machine_reset()/bank_reg==0 check. ---
	sb_write_bank(4'd0);
	sb_check("bankreg_write_0", {15'd0, dut.slave.bank_reg}, 19'd0);
	sb_read_banked(16'h5000, ra, rd_byte);
	sb_check("bank0_rom_addr", ra, 19'h11000);
	sb_check("bank0_rom_data", {11'd0, rd_byte}, 19'h00000);

	// --- out-of-range fallback: bank 15 -> MAME's
	// "bankaddress >= region length" fallback to 0x10000 (same as bank 0). ---
	sb_write_bank(4'd15);
	sb_check("bankreg_write_15", {15'd0, dut.slave.bank_reg}, 19'd15);
	sb_read_banked(16'h5000, ra, rd_byte);
	sb_check("bank15_rom_addr_fallback", ra, 19'h11000);
	sb_check("bank15_rom_data_fallback", {11'd0, rd_byte}, 19'h00000);

	$display("t=%0t === SLAVE BANK-MAP DIRECTED TEST DONE: %0d checks, %0d errors -- %0s ===",
	          $time, sb_checks, sb_errors, (sb_errors == 0) ? "PASS" : "FAIL");

	release dut.master.master_cpu.reset_n;
	release dut.slave.slave_cpu.reset_n;
	$finish;
end
`endif // SLAVEBANKTEST

initial begin
	sdram_init = 1;
	reset      = 1;
	repeat (10) @(posedge clk_sys);
	sdram_init = 0;
	repeat (5) @(posedge clk_sys);

	$display("=== Loading full flat ROM image (one ioctl session, offroad.zip) ===");
	ioctl_index    = 16'h00;
	ioctl_download = 1'b1;

	// ── Master Z80 ROM: 0x000000-0x03FFFF (4 x 64 KB) ──────────────
	$display("t=%0t  Master ROM...", $time);
	ioctl_load_file("03-22121-04.u58t", 27'h000000, 65536);
	ioctl_load_file("03-22122-03.u59t", 27'h010000, 65536);
	ioctl_load_file("03-22120-01.u57t", 27'h020000, 65536);
	ioctl_load_file("03-22119-02.u56t", 27'h030000, 65536);

	// ── Slave Z80 ROM: 0x040000-0x0BFFFF ────────────────────────────
	$display("t=%0t  Slave ROM...", $time);
	ioctl_load_file("03-22100-02.u3",   27'h040000, 8192);
	ioctl_fill_zero(                    27'h042000, 184320);  // 0x2E000 fill
	ioctl_load_file("03-22108-02.u4t",  27'h070000, 65536);
	ioctl_load_file("03-22109-02.u5t",  27'h080000, 65536);
	ioctl_load_file("03-22110-02.u6t",  27'h090000, 65536);
	ioctl_load_file("03-22111-01.u7t",  27'h0A0000, 65536);
	ioctl_load_file("03-22112-01.u8t",  27'h0B0000, 65536);

	// ── Sound ROM: 0x0C0000-0x1BFFFF (80186's own 0x00000-0xFFFFF
	// address space, offset by ADDR_SOUND_LO=0x0C0000 -- matches
	// sdram_rd3_addr's own base in rtl/sor_board.sv) ────────────────
	// Real bytes now actually reach SDRAM: rtl/sor_board.sv's
	// ioctl_wr_rom gate used to stop at ADDR_SOUND_LO (pre-WP10, when
	// this region was still an unemulated dropped fill) -- widened this
	// session to ADDR_GFX_LO, see that file's own comment. Same 3
	// interleaved lo/hi pairs, same base offsets within the 80186's own
	// space, as mra/SuperOffRoad.mra and sim/sor_sound_tb.sv's own
	// load_pair.
	$display("t=%0t  Sound ROM...", $time);
	ioctl_load_pair("03-22113-03.u13t", "03-22116-03.u25t", 27'h0C0000 + 27'h040000);
	ioctl_load_pair("03-22114-03.u14t", "03-22117-03.u26t", 27'h0C0000 + 27'h060000);
	ioctl_load_pair("03-22115-03.u15t", "03-22118-03.u27t", 27'h0C0000 + 27'h0E0000);

	// ── GFX tile ROM: 0x1C0000-0x1D7FFF (3 x 32 KB) ────────────────
	$display("t=%0t  GFX tile ROM...", $time);
	ioctl_load_file("03-22105-02.u93",  27'h1C0000, 32768);
	ioctl_load_file("03-22106-02.u94",  27'h1C8000, 32768);
	ioctl_load_file("03-22107-02.u95",  27'h1D0000, 32768);

	// ── BG palette PROM: 0x1D8000-0x1F7FFF (4 x 16KB + fills) ──────
	$display("t=%0t  Palette PROM...", $time);
	ioctl_fill_zero(                    27'h1D8000, 16384);   // u70 empty
	ioctl_load_file("03-22104-01.u92",  27'h1DC000, 16384);
	ioctl_load_file("03-22102-01.u69",  27'h1E0000, 16384);
	ioctl_fill_zero(                    27'h1E4000, 32768);   // u91+u68 empty
	ioctl_load_file("03-22103-02.u90",  27'h1EC000, 16384);
	ioctl_load_file("03-22101-02.u67",  27'h1F0000, 16384);
	ioctl_fill_zero(                    27'h1F4000, 16384);   // u89 empty

	ioctl_download = 1'b0;

	// GFX/PROM BRAM real-ioctl-path loading check (2026-07-12 session):
	// verifies gfx_rom/prom_rom content immediately after the SAME real
	// ioctl_wr/ioctl_addr_d1 path every other test uses -- unlike
	// sim/sor_video_tb.sv, which loads ROM files directly via $fread,
	// bypassing ioctl entirely. Expected values computed directly from
	// the real ROM chip files in sim/ (first byte of each):
	//   gfx_rom[0]      (u93) expect 00      prom_rom[0x04000] (u92) expect 00
	//   gfx_rom[0x8000] (u94) expect ff      prom_rom[0x08000] (u69) expect 69
	//   gfx_rom[0x10000](u95) expect ff      prom_rom[0x14000] (u90) expect 00
	//                                        prom_rom[0x18000] (u67) expect 60
	$display("=== GFX/PROM BRAM real-ioctl-load check ===");
	$display("gfx_rom[0]=%02x (u93, expect 00)  gfx_rom[0x8000]=%02x (u94, expect ff)  gfx_rom[0x10000]=%02x (u95, expect ff)",
	          dut.gfx_rom[17'h0], dut.gfx_rom[17'h8000], dut.gfx_rom[17'h10000]);
	$display("prom_rom[0x4000]=%02x (u92, expect 00)  prom_rom[0x8000]=%02x (u69, expect 69)  prom_rom[0x14000]=%02x (u90, expect 00)  prom_rom[0x18000]=%02x (u67, expect 60)",
	          dut.prom_rom[17'h4000], dut.prom_rom[17'h8000], dut.prom_rom[17'h14000], dut.prom_rom[17'h18000]);

	// Download-stream telemetry check (2026-07-13 session): the exact
	// counters the hardware status row now displays, verified here
	// against a known-good full-image load so a hardware readout can be
	// compared against sim ground truth field by field.
	repeat (4) @(posedge clk_sys); // let the session-end fall-edge latches update
	$display("=== Download-stream telemetry check ===");
	$display("max_ioctl_addr[26:12]=%03x (expect 1f7)  gfx_wr_hits=%05x (expect 18000)  prom_wr_hits=%05x (expect 20000)",
	          dut.dbg_max_ioctl_addr, dut.gfx_wr_hits, dut.prom_wr_hits);
	$display("dl_sessions=%02x  dl_index_last=%02x (expect 00)  dl_end_addr[26:12]=%03x (expect 1f8)",
	          dut.dbg_dl_sessions, dut.dbg_dl_index_last, dut.dbg_dl_end_addr);
	if (dut.dbg_max_ioctl_addr != 15'h01F7 || dut.gfx_wr_hits != 20'h18000 || dut.prom_wr_hits != 20'h20000)
		$display("*** TELEMETRY MISMATCH -- compare against expected values above ***");

	$display("=== Load complete at t=%0t, dropping reset ===", $time);
	reset = 1'b0;

	// prom_rom self-test (2026-07-13): runs shortly after loading_done
	// (DL_SETTLE_CYCLES + a few states). Forced read of prom_rom[0x8000]
	// expect 69; sentinel write+readback expect pass.
	fork begin
		wait (dut.prom_st_done);
		repeat (2) @(posedge clk_sys);
		$display("=== prom self-test: pass=%b (expect 1)  fp_forced=%02x (expect 69) ===",
		          dut.prom_st_pass, dut.prom_fp_forced);
	end join_none

`ifdef GAMEPLAY_REPRO
	// sor_master's REAL reset input is `reset | ~sdram_ready | ~dl_settled`
	// (rtl/sor_board.sv:1628), not the testbench's raw `reset` register --
	// dl_settled only goes true DL_SETTLE_CYCLES after ioctl_download
	// falls. Injecting right after `reset <= 1'b0` (as a first attempt
	// here did) lands inside that still-asserted window: the CPU's own
	// async-reset always block (tv80_core.v:378) fires again a few
	// hundred cycles later and wipes PC/SP/AF/etc straight back to
	// power-on defaults, silently discarding the injection (confirmed:
	// PC was observed back at the real cold-boot address $001F, then
	// $376C -- the genuine boot-time WRAM-clear entry -- moments after
	// injecting). Wait for the real combined reset to fall first.
	wait (dut.master.reset == 1'b0);
	// (slave's own reset also gates on slave_reset_n -- forced below as
	// part of the mcont_r injection, so no separate wait needed here)
	inject_gameplay_snapshot;
	$display("=== GAMEPLAY_REPRO: running post-injection, watching for PC=0xBDAB and for a stall ===");
	repeat (bdab_repro_run_cycles) @(posedge clk_sys);
	$display("t=%0t === GAMEPLAY_REPRO run complete: bdab_hit_count=%0d master_pc_stall_count=%0d last_master_pc=0x%04x last_slave_pc=0x%04x ===",
	          $time, bdab_hit_count, master_pc_stall_count, dut.dbg_pc, slave_dbg_pc);
	$fclose(pc_trace_file_m);
	$fclose(pc_trace_file_s);
	$finish;
`else
	$display("=== Running Master CPU boot; watching for /MCONT (port 0x09) write to release slave_reset_n ===");
	fork
		begin
			wait (dut.slave_reset_n);
			$display("=== slave_reset_n released at t=%0t -- boot handshake reached ===", $time);
		end
		begin
			// Boot code has multiple legitimate bounded hardware-presence
			// timeout loops (confirmed via IORD trace): a ~57ms LDIR RAM
			// clear, and a ~247ms poll on port 0x06 (unmapped on real
			// hardware too -- expected to time out and fall through via
			// its own error handler, not something to stub). Rather than
			// keep bumping this one loop at a time, give it 2s of
			// headroom to clear this whole class of "unmapped hardware,
			// designed to time out" loops in one shot.
			#2_000_000_000; // 2s boot-watchdog
			if (!dut.slave_reset_n)
				$display("=== WARNING: 2s elapsed and slave_reset_n still LOW -- Master boot code never wrote /MCONT bit0 ===");
		end
	join_any

	$display("=== Waiting for sor_board's internal readback scan to finish ===");
	wait (dut.rd_scan_done);
	repeat (5) @(posedge clk_sys);

	// Live fetch-mismatch counter sim check (docs/sdram_plan.md Section 2,
	// B1 pre-build gate): confirm live_poll_cnt visibly spins (liveness)
	// and live_mm_cnt stays 0 (the ideal-clock sim model has no timing
	// marginality, so mismatches here would indicate an FSM/addressing
	// bug, not the hardware phenomenon under investigation).
	begin
		reg [3:0] live_poll_start;
		live_poll_start = dut.live_poll_cnt;
		repeat (3) @(posedge dut.live_tick);
		repeat (5) @(posedge clk_sys);
		$display("live_mm_cnt=0x%02x live_poll_cnt=0x%01x (start=0x%01x)",
		         dut.live_mm_cnt, dut.live_poll_cnt, live_poll_start);
		if (dut.live_poll_cnt == live_poll_start)
			$display("=== LIVE_MM FAIL: live_poll_cnt did not spin ===");
		else if (dut.live_mm_cnt != 8'h00)
			$display("=== LIVE_MM FAIL: mismatch count nonzero in ideal-clock sim ===");
		else
			$display("=== LIVE_MM PASS ===");
	end

	$display("");
	$display("wr_chk    = 0x%02x", dut.wr_chk);
	$display("rd_chk    = 0x%02x", dut.rd_chk);
	$display("wr_chk_q0 = 0x%02x  rd_chk_q0 = 0x%02x  match=%b", dut.wr_chk_q0, dut.rd_chk_q0, dut.dbg_chk_match_q0);
	$display("wr_chk_q1 = 0x%02x  rd_chk_q1 = 0x%02x  match=%b", dut.wr_chk_q1, dut.rd_chk_q1, dut.dbg_chk_match_q1);
	$display("wr_chk_q2 = 0x%02x  rd_chk_q2 = 0x%02x  match=%b", dut.wr_chk_q2, dut.rd_chk_q2, dut.dbg_chk_match_q2);
	$display("wr_chk_q3 = 0x%02x  rd_chk_q3 = 0x%02x  match=%b", dut.wr_chk_q3, dut.rd_chk_q3, dut.dbg_chk_match_q3);
	$display("overall match (dbg_chk_match) = %b", dut.dbg_chk_match);
	$display("byte-test: done=%b pass=%b readback=0x%02x", dut.bt_done, dut.bt_pass, dut.bt_readback);
	$display("");

	if (dut.dbg_chk_match && dut.bt_pass) $display("=== PASS ===");
	else                                   $display("=== FAIL ===");

	$fclose(pc_trace_file_m);
	$fclose(pc_trace_file_s);
`endif
	$fclose(board_pcm_fd);
	stitch_board_wav();
	$finish;
end

//------------------------------------------------------------------
// Sound-board instrumentation (WP10 board-level audio check,
// docs/WP10_PROGRESS.md "Two ways to get a real audio capture" -- this
// is option 2): tracks real command traffic from the REAL Master Z80
// ROM code into the sound board (via its actual 0xF2/0xF4 port writes,
// sor_master.sv's io_cmd/io_snd_hi decode -- no synthetic bench-driven
// guessing, unlike sim/sor_sound_tb.sv's own attempt, which never
// learned the real command protocol and so never produced audible
// content) and the resulting DAC activity. Mirrors sor_sound_tb.sv's
// own counters/WAV-capture convention.
//------------------------------------------------------------------
longint unsigned board_cmd_wr_count = 0;
longint unsigned board_dac_write_count = 0, board_dac9_write_count = 0;
always @(posedge clk_sys) begin
	if (dut.sound_cmd_wr_lo || dut.sound_cmd_wr_hi)
		board_cmd_wr_count <= board_cmd_wr_count + 1;
	if (dut.sound.board.dac9_wr)
		board_dac9_write_count <= board_dac9_write_count + 1;
	if (dut.sound.board.dac_wr[0] || dut.sound.board.dac_wr[1] || dut.sound.board.dac_wr[2] ||
	    dut.sound.board.dac_wr[3] || dut.sound.board.dac_wr[4] || dut.sound.board.dac_wr[5])
		board_dac_write_count <= board_dac_write_count + 1;
end

// WAV capture of dut.audio_out -- same real-time-accurate decimation
// convention as sor_sound_tb.sv (48,000,000/44,100 ~= 1088 clk_sys
// cycles/sample).
localparam integer BOARD_SAMPLE_PERIOD_CLKS = 1088;
localparam integer BOARD_WAV_SAMPLE_RATE_HZ = 44100;
integer board_pcm_fd;
integer board_sample_div;
longint unsigned board_wav_sample_count = 0;
initial begin
	board_pcm_fd = $fopen("sor_board_tb_audio.pcm", "wb");
	board_sample_div = 0;
end
always @(posedge clk_sys) begin
	if (!reset) begin
		board_sample_div <= board_sample_div + 1;
		if (board_sample_div >= BOARD_SAMPLE_PERIOD_CLKS) begin
			board_sample_div <= 0;
			$fwrite(board_pcm_fd, "%c%c", dut.audio_out[7:0], dut.audio_out[15:8]);
			board_wav_sample_count <= board_wav_sample_count + 1;
		end
	end
end

task automatic board_wav_u16(input integer fd_, input integer v);
	begin $fwrite(fd_, "%c%c", v[7:0], v[15:8]); end
endtask
task automatic board_wav_u32(input integer fd_, input integer v);
	begin $fwrite(fd_, "%c%c%c%c", v[7:0], v[15:8], v[23:16], v[31:24]); end
endtask
task automatic stitch_board_wav;
	integer wav_fd, rd_fd, c;
	integer data_bytes, byte_rate, riff_bytes;
	begin
		data_bytes = board_wav_sample_count * 2;
		byte_rate  = BOARD_WAV_SAMPLE_RATE_HZ * 2;
		riff_bytes = 36 + data_bytes;
		wav_fd = $fopen("sor_board_tb_audio.wav", "wb");
		$fwrite(wav_fd, "RIFF"); board_wav_u32(wav_fd, riff_bytes);
		$fwrite(wav_fd, "WAVE"); $fwrite(wav_fd, "fmt ");
		board_wav_u32(wav_fd, 16);
		board_wav_u16(wav_fd, 1); board_wav_u16(wav_fd, 1);
		board_wav_u32(wav_fd, BOARD_WAV_SAMPLE_RATE_HZ);
		board_wav_u32(wav_fd, byte_rate);
		board_wav_u16(wav_fd, 2); board_wav_u16(wav_fd, 16);
		$fwrite(wav_fd, "data"); board_wav_u32(wav_fd, data_bytes);
		rd_fd = $fopen("sor_board_tb_audio.pcm", "rb");
		c = $fgetc(rd_fd);
		while (c != -1) begin
			$fwrite(wav_fd, "%c", c[7:0]);
			c = $fgetc(rd_fd);
		end
		$fclose(rd_fd);
		$fclose(wav_fd);
		$display("=== SOUND_BOARD_AUDIO wav written: sor_board_tb_audio.wav (%0d samples @ %0d Hz) cmd_wr_count=%0d dac_write_count=%0d dac9_write_count=%0d ===",
		          board_wav_sample_count, BOARD_WAV_SAMPLE_RATE_HZ, board_cmd_wr_count, board_dac_write_count, board_dac9_write_count);
	end
endtask

// Safety timeout
initial begin
	// TEMP: shortened from 2.5s for fast iteration. Now that the
	// ioctl_addr_d1/write-FIFO alignment bug is fixed, the Master runs
	// real, MAME-matching code instead of stalling immediately, so this
	// needs more headroom than the 200ms used to chase that bug -- but
	// still far short of the full 2.5s/37-minute run. Restore to
	// 2_500_000_000 for real PASS/FAIL boot-watchdog runs.
	// 3s total, split to avoid a 32-bit-signed overflow on the literal
	// (>2.147B ns) -- need enough headroom to reach the repeating
	// 0x61Fx/0x377x cycle the user reported on real hardware after 20+
	// real seconds; 800ms only got as far as the raster-sync wait loop
	// (0x60F4-0x6126).
	// 2026-07-15 (Follow-up 12): cut back to 0.8s for faster iteration on
	// the current fg-bitmap/VRAM-content investigation. The 1.15s figure
	// below was tuned for a DIFFERENT, since-resolved question (whether
	// the late /MCONT reset pulse at ~869.8ms was a genuine deadlock --
	// Follow-up 4 killed that hypothesis, confirmed sim-timeout artifact,
	// not a real hang) and isn't needed for this investigation. Real
	// hardware's frame rate is 65.95Hz (15.16ms/frame, sor_video.sv
	// header); MAME's real title/hints art finishes by frame ~36
	// (~546ms after boot) per Follow-up 9/12's fg_bitmap_early_boot_trace
	// -- our sim's boot handshake completes at t~83ms, so 83+546=~629ms
	// is the real "graphics should be loaded" milestone. 0.8s leaves
	// ~170ms of margin past that. Restore to 1_150_000_000 (or higher)
	// if the late-reset-pulse scenario needs re-checking.
	// Follow-up 20: back to 0.8s -- Follow-up 19's 2s run already showed
	// the E900/E901/0x1136 write pattern repeating identically every
	// ~30ms starting at t=371ms, so 0.8s captures several repeats without
	// the extra wait.
	#800_000_000;
	$display("=== TIMEOUT: simulation did not finish in time ===");
	// Follow-up 6: final values of the new slave-side status-row debug
	// taps -- the exact values the on-hardware status row (sor_video
	// chars 10-28) would show at this moment, so the sim transcript
	// corroborates what a hardware photo should read.
	$display("=== SLAVE_DBG_FINAL s_pc=0x%04x bank_reg=%0x bank_wr_cnt=0x%02x bank_max=%0x vram_wr_cnt=0x%04x banked_read_ever=%0d ===",
	          dut.slave.dbg_pc, dut.slave.dbg_bank_reg, dut.slave.dbg_bank_wr_cnt,
	          dut.slave.dbg_bank_max, dut.slave.dbg_vram_wr_cnt, dut.slave.dbg_banked_read_ever);
	// RUNAWAY_TRAP (2026-07-16): the Master's runaway-PC trap (sor_master.sv).
	// This run reaches the title screen and the Master is healthy there, so the
	// EXPECTED result is fired=0 -- a nonzero fired here would mean the 16-NOP
	// sled threshold false-triggers on legitimate code, which would make the
	// trap useless on hardware. jump_from/jump_to are live (unfrozen) when
	// fired=0 and simply show the last control-flow transfer.
	$display("=== RUNAWAY_TRAP fired=%0d jump_from=0x%04x jump_to=0x%04x (expect fired=0 on a healthy run) ===",
	          dut.master.dbg_sled_trapped, dut.master.dbg_jump_from, dut.master.dbg_jump_to);
	// Follow-up 8: cram_we/mcont_r[1] gate evidence summary + a sample of
	// the actual fg-indexed (address>=64) Color RAM content at run's end.
	$display("=== CRAM_WR_SUMMARY attempts=%0d landed=%0d blocked=%0d fg_landed(idx>=64)=%0d ===",
	          cram_attempt_count, cram_landed_count, cram_blocked_count, cram_fg_landed_count);
	$display("=== FG_BITMAP_SUMMARY fg_wr=%0d fg_nonzero=%0d mbx_wr=%0d (MAME reference by frame 36: fg_wr=75035 fg_nonzero=30261, starting frame 0) ===",
	          fg_wr_count, fg_wr_nonzero, mbx_wr_count);
// Follow-up 11 -- final-state VRAM content dump + fg-range nonzero
	// count, for direct byte-level comparison against a live MAME
	// video_ram dump (C:\MiSTerDev\mame\video_ram_dump_*.bin, captured via
	// the interactive debugger's `save` command at the title/hints
	// screen -- 47029-47046 nonzero bytes out of 61440 in the same
	// 0x0000-0xEFFF range, ~76% fill). Writes a raw binary dump of the
	// first 0x10000 bytes of dut.vram (matching the MAME dump's 64KB
	// size) to sim/vram_dump_final.bin.
	begin : vram_final_dump
		integer vram_fd, vi;
		integer vram_nonzero_count;
		vram_nonzero_count = 0;
		for (vi = 0; vi < 17'hF000; vi = vi + 1)
			if (dut.vram.mem[vi] != 8'h00) vram_nonzero_count = vram_nonzero_count + 1;
		$display("=== VRAM_FINAL_STATE fg_nonzero=%0d (of 61440, 0x0000-0xEFFF) (MAME reference: ~47000/61440, ~76%% fill) ===",
		          vram_nonzero_count);
		vram_fd = $fopen("vram_dump_final.bin", "wb");
		if (vram_fd) begin
			for (vi = 0; vi < 17'h10000; vi = vi + 1)
				$fwrite(vram_fd, "%c", dut.vram.mem[vi]);
			$fclose(vram_fd);
			$display("=== VRAM_FINAL_STATE dumped to sim/vram_dump_final.bin (0x10000 bytes) ===");
		end
	end
	$display("=== F12B_SUMMARY 10ec_hits=%0d 1100_hits=%0d (MAME: 22/22 real executions reach here with H=$EC) ===",
	          f12b_10ec_hits, f12b_1100_hits);
	$display("=== F14_SUMMARY 10cc_hits=%0d 10ec_hits=%0d (ratio=%0d%%) 1591_hits=%0d last_1591_at_t=%0t ===",
	          f14_10cc_hits, f12b_10ec_hits, (f14_10cc_hits > 0) ? (f12b_10ec_hits*100/f14_10cc_hits) : 0,
	          f14_1591_hits, f14_1591_last_time);
	$display("=== F15_SUMMARY mout_any=%0d (vs F10_MWR range-gated count printed above) ===", f15_mout_count);
	$display("=== F16_SUMMARY stale_rd_master=%0d stale_rd_slave=%0d ===", stale_rd_count_m, stale_rd_count_s);
	$display("=== F17_SUMMARY entry_count=%0d (into 0x6100-0x6300 from outside) last_entry_t=%0t last_entry_from=0x%04x ===",
	          f17_entry_count, f17_last_entry_time, f17_last_entry_from);
	$display("=== F18_SUMMARY window_count=%0d (visits to 0x03d0-0x0500) addr_hits=%0d (0x03d3/0x04a5/0x04a8/0x0466) ===",
	          f18_window_count, f18_addr_hits);
	$display("=== F18B_SUMMARY linkbyte_wr_count=%0d (writes to 0xE8F9/0xEAF9) ===", f18b_wr_count);
	$display("=== F19_SUMMARY e900wr_slave=%0d e900wr_master=%0d ===", f19_wr_count_s, f19_wr_count_m);
	$display("=== F19_CONTENT_FINAL E900-E907(slave wram)=%02x %02x %02x %02x %02x %02x %02x %02x ===",
	          dut.wram_s.mem[16'h900], dut.wram_s.mem[16'h901], dut.wram_s.mem[16'h902], dut.wram_s.mem[16'h903],
	          dut.wram_s.mem[16'h904], dut.wram_s.mem[16'h905], dut.wram_s.mem[16'h906], dut.wram_s.mem[16'h907]);
	$display("=== F20_SUMMARY hits_1220=%0d hits_1136=%0d ===", f20_hits_1220, f20_hits_1136);
	$display("=== F21_SUMMARY step_count=%0d ===", f21_step_count);
	$display("=== F22_SUMMARY rom_read_hits=%0d ===", f22_hits);
	begin : f17_hist_dump
		integer f17_i;
		for (f17_i = 0; f17_i < f17_hist_idx; f17_i = f17_i + 1)
			$display("F17_ENTRY_HIST[%0d] t~%0dms from_pc=0x%04x", f17_i, f17_entry_hist_time_ms[f17_i], f17_entry_hist_pc[f17_i]);
	end
	$display("=== F12C_PIPELINE slave_out=%0d master_out=%0d vpreq_s=%0d vpreq_m=%0d vppop_s=%0d vppop_m=%0d vram_we=%0d ===",
	          f12c_slave_out, f12c_master_out, f12c_vpreq_s, f12c_vpreq_m, f12c_vppop_s, f12c_vppop_m, f12c_vram_we);
	$display("=== F13_SUMMARY out=%0d match=%0d(%0d%%) nonzero=%0d(%0d%%) nonzero_and_match=%0d ===",
	          f13_out_count, f13_match_count, (f13_out_count > 0) ? (f13_match_count*100/f13_out_count) : 0,
	          f13_nonzero_count, (f13_out_count > 0) ? (f13_nonzero_count*100/f13_out_count) : 0,
	          f13_nonzero_match_count);
	$display("=== OUTI_BLIT_SUMMARY visited=%0d m1_visits=%0d (MAME: this PC range does the bulk real-art blit, ~1074 hits/addr) ===",
	          outi_blit_seen, outi_blit_visits);
	$display("=== DISPATCH_PATH_SUMMARY disp_entry(0x0482-04C5)=%0d disp_body(0x0541-05FF)=%0d outi_setup(0x05EC-06FF)=%0d idx_setup(0x0802-089F)=%0d idx_blit(0x0C80-0CFF)=%0d ===",
	          disp_entry_visits, disp_body_visits, outi_setup_visits, idx_setup_visits, idx_blit_visits);
	$display("=== CHECKPOINT_HISTOGRAM 05ab=%0d 05b9=%0d 05bc=%0d 05bf=%0d 05c0=%0d 05c2=%0d 05c5=%0d 05ca=%0d 05cb=%0d 05d0=%0d 05d5=%0d 05de=%0d 05df=%0d 05ec=%0d 064b=%0d 080a=%0d 08ad=%0d 152e=%0d ===",
	          chk_hits[0], chk_hits[1], chk_hits[2], chk_hits[3], chk_hits[4], chk_hits[5], chk_hits[6],
	          chk_hits[7], chk_hits[8], chk_hits[9], chk_hits[10], chk_hits[11], chk_hits[12], chk_hits[13],
	          chk_hits[14], chk_hits[15], chk_hits[16], chk_hits[17]);
	$display("=== CRAM_CONTENT_SAMPLE idx0=0x%02x idx1=0x%02x idx63=0x%02x idx64=0x%02x idx65=0x%02x idx127=0x%02x idx191=0x%02x idx255=0x%02x idx511=0x%02x idx1023=0x%02x ===",
	          dut.cram.mem[0], dut.cram.mem[1], dut.cram.mem[63], dut.cram.mem[64], dut.cram.mem[65],
	          dut.cram.mem[127], dut.cram.mem[191], dut.cram.mem[255], dut.cram.mem[511], dut.cram.mem[1023]);

	// MBOX_TRACE final report -- see the tap comment above (~line 1287)
	// for exactly what's captured and why. Written to
	// sim/rtl_mailbox.txt: header counts, a per-address SUMMARY, then a
	// CHRONOLOGICAL log of the first MBOX_LOG_CAP ops in the exact
	// "<M|S> <R|W> op<n> addr=XXXX data=XX" format requested for diffing
	// against a MAME-side mailbox trace.
	begin : mbox_report
		integer mbox_fd, mi, mj, mbox_distinct_addr_count;
		mbox_distinct_addr_count = 0;
		for (mi = 0; mi < 4096; mi = mi + 1)
			if (mbox_addr_touched[mi]) mbox_distinct_addr_count = mbox_distinct_addr_count + 1;
		mbox_fd = $fopen("rtl_mailbox.txt", "w");
		if (mbox_fd) begin
			$fdisplay(mbox_fd, "MBOX_TRACE header: master_rd=%0d master_wr=%0d slave_rd=%0d slave_wr=%0d",
			          mbox_total_rd_m, mbox_total_wr_m, mbox_total_rd_s, mbox_total_wr_s);
			$fdisplay(mbox_fd, "");
			$fdisplay(mbox_fd, "SUMMARY (distinct mailbox addresses touched):");
			for (mi = 0; mi < 4096; mi = mi + 1) begin
				if (mbox_addr_touched[mi]) begin
					$fwrite(mbox_fd, "  addr=%04x  M_rd=%0d M_wr=%0d S_rd=%0d S_wr=%0d  distinct_data=[",
					        16'hF000 + mi[15:0], mbox_rd_cnt_m[mi], mbox_wr_cnt_m[mi],
					        mbox_rd_cnt_s[mi], mbox_wr_cnt_s[mi]);
					for (mj = 0; mj < mbox_distinct_n[mi]; mj = mj + 1) begin
						if (mj > 0) $fwrite(mbox_fd, ",");
						$fwrite(mbox_fd, "%02x", mbox_distinct_val[mi][mj]);
					end
					if (mbox_distinct_n[mi] >= 12) $fwrite(mbox_fd, ",...(capped)");
					$fdisplay(mbox_fd, "]");
				end
			end
			$fdisplay(mbox_fd, "");
			$fdisplay(mbox_fd, "CHRONOLOGICAL (first %0d mailbox ops):", mbox_log_n);
			for (mi = 0; mi < mbox_log_n; mi = mi + 1) begin
				$fdisplay(mbox_fd, "%s %s %s addr=%04x data=%02x",
				          mbox_log_side[mi] ? "S" : "M",
				          mbox_log_rd[mi]   ? "R" : "W",
				          (mbox_log_op[mi] == 3'd0) ? "op?" :
				          (mbox_log_op[mi] == 3'd1) ? "op1" :
				          (mbox_log_op[mi] == 3'd2) ? "op2" :
				          (mbox_log_op[mi] == 3'd3) ? "op3" :
				          (mbox_log_op[mi] == 3'd5) ? "op5" :
				          (mbox_log_op[mi] == 3'd6) ? "op6" : "op?",
				          mbox_log_addr[mi], mbox_log_data[mi]);
			end
			$fclose(mbox_fd);
			$display("=== MBOX_TRACE written to sim/rtl_mailbox.txt: master_rd=%0d master_wr=%0d slave_rd=%0d slave_wr=%0d, %0d distinct addrs, %0d chronological ops logged ===",
			          mbox_total_rd_m, mbox_total_wr_m, mbox_total_rd_s, mbox_total_wr_s,
			          mbox_distinct_addr_count, mbox_log_n);
		end else begin
			$display("=== MBOX_TRACE ERROR: could not open sim/rtl_mailbox.txt for write ===");
		end
	end

	$fclose(pc_trace_file_m);
	$fclose(pc_trace_file_s);
	$fclose(board_pcm_fd);
	stitch_board_wav();
	$finish;
end

// TEMP diagnostic: every Master read/write of WRAM address 0x039
// (the E039 MCONT-shadow byte the periodic ISR masks with $F7 and
// echoes to port 0xC9) -- pinpoints exactly which instruction/PC wrote
// bit0=0 there, since the ISR's own masked write-back can't clear
// bit0 itself ($F7 = ~bit3 only).
integer e039_count = 0;
always @(posedge clk_sys) begin
	if (!reset && dut.master.CE_6M && dut.master.mem_access && dut.master.in_wram &&
	    (dut.master.cpu_addr[11:0] == 12'h039) && e039_count < 500) begin
		if (~dut.master.rd_n) begin
			e039_count = e039_count + 1;
			$display("t=%0t E039_RD data=0x%02x pc=0x%04x", $time, dut.wram_dout_m, dut.dbg_pc);
		end else if (~dut.master.wr_n) begin
			e039_count = e039_count + 1;
			$display("t=%0t E039_WR data=0x%02x pc=0x%04x", $time, dut.wram_din_m, dut.dbg_pc);
		end
	end
end

endmodule
