//============================================================================
//  Super Off Road — Board Top Level
//
//  Connects:
//    - Master Z80 (Chunk 2)
//    - Slave  Z80 (Chunk 3)
//    - Video system (sor_video)
//    - SDRAM controller (program ROMs for both Z80s)
//    - On-chip BRAM (VRAM 128 KB, WRAM 4 KB, CRAM 1 KB)
//    - Inter-CPU latch ports
//    - Sound CPU (Chunk 5 — real s80x86-based 80186 board, WP10)
//
//  SDRAM address layout (byte addresses):
//    0x000000–0x03FFFF  Master Z80 program ROM  (256 KB, ioctl_index 0x00)
//    0x040000–0x0BFFFF  Slave  Z80 program ROM  (512 KB flat, ioctl_index 0x01)
//============================================================================

module sor_board #(
	// Passed straight through to the internal sdram controller. See
	// rtl/sdram.sv's own USE_ALTDDIO parameter comment — default 1 is
	// required for real hardware; simulation testbenches override to 0
	// to avoid needing Altera's vendor simulation libraries.
	parameter bit USE_ALTDDIO = 1'b1,

	// HARDWARE TIMING-CLOSURE EXPERIMENT (temporary): the on-screen
	// diagnostic overlay's byte-test + full-ROM readback scanner issue
	// real extra SDRAM read/write transactions purely for diagnostics,
	// on top of the real ROM load and CPU traffic. Every RTL fix this
	// session has verified correctly in a zero-delay behavioral
	// simulator and produced no change on real hardware -- the
	// signature of a timing-closure problem invisible to sim, not a
	// logic bug. SuperOffRoad.sdc false-paths the entire SDRAM
	// interface (including SDRAM_DQ), so Quartus never checks real
	// setup/hold there either. Set to 0 to stop the diagnostic
	// scanner from ever issuing a request at all (rd_phase stays
	// parked in RD_IDLE permanently), removing that extra SDRAM bus
	// load/switching activity, as a test of whether reduced load
	// changes real hardware boot behavior. Set back to 1 to resume
	// normal diagnostics once this experiment is answered.
	// Diagnostic job (the byte-test/full-scan investigation) was done
	// and this was disabled, but docs/sdram_plan.md B1 (Section 2) re-
	// enables it: the new live fetch-mismatch counter (RD_LIVE_IDLE/
	// REQ/WAIT below) is chained after rd_scan_done, which only fires
	// through this same byte-test+scan chain. Flip back to 0 once B1's
	// diagnostic build cycle is over and the overlay should stop
	// obstructing gameplay again.
	parameter bit ENABLE_SDRAM_DIAG_TRAFFIC = 1'b1,

	// Granular A/B switch: the two diagnostic reads that run in the
	// MIDDLE of the ROM download (the "immediate" read of address 0
	// right after its own write ack, and the "early" read at the
	// master->slave index transition). These are the only diagnostic
	// operations interleaved with the live write stream -- the
	// byte-test and the full scan both run strictly after loading
	// finishes and cannot perturb it. Sim shows the immediate read
	// returning the correct 0xF3 while real hardware returns 0x00 for
	// the identical sequence, and the earlier "all diagnostics off,
	// still broken" exoneration is invalid (it ran on top of the
	// broken-MRA load, which guaranteed failure regardless). 0 = keep
	// the overlay and post-load diagnostics but inject nothing into
	// the download window.
	parameter bit ENABLE_MIDLOAD_DIAG_READS = 1'b0,

	// How long ioctl_download must stay LOW before the load is
	// considered genuinely finished (250ms at 48MHz by default; sim
	// overrides smaller). Restored after the single-session MRA
	// restructuring turned out NOT to guarantee a single download
	// session in practice -- hardware measured dbg_dl_sessions==02 for
	// one <rom index="0"> tag (some ARM-side chunking behavior on the
	// ~2MB transfer, not visible from this repo), with address 0/1
	// written twice (accept0/1_cnt==02) confirming ioctl_addr really
	// did reset mid-load. Rather than chase the exact session count,
	// this makes "loading done" independent of it: wait for
	// ioctl_download to go solidly quiet, however many times it
	// toggled getting there.
	parameter int unsigned DL_SETTLE_CYCLES = 12_000_000
)
(
	input         clk_sys,   // 48 MHz
	input         clk_sdram, // phase-shifted 48 MHz PLL output, drives SDRAM_CLK (docs/sdram_plan.md Section 3a)
	input         reset,      // CPU/game reset (includes ioctl_download)
	input         sdram_init, // SDRAM hardware reset only (~pll_locked | RESET)

	// ROM loading from HPS
	input         ioctl_download,
	input  [15:0] ioctl_index,
	input         ioctl_wr,
	input  [26:0] ioctl_addr,
	input   [7:0] ioctl_data,
	output        ioctl_wait, // stall HPS while SDRAM write is in progress

	// SDRAM chip pins (pass-through to top level)
	inout  [15:0] SDRAM_DQ,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output        SDRAM_nCS,
	output        SDRAM_nRAS,
	output        SDRAM_nCAS,
	output        SDRAM_nWE,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,

	// Video
	output        ce_pix,
	output        HBlank,
	output        HSync,
	output        VBlank,
	output        VSync,
	output [23:0] rgb,

	// Player controls (digital)
	input   [3:0] p1_btn, p2_btn, p3_btn,
	// Wheel: free-running mod-256 "virtual dial" position (analog stick +
	// d-pad + spinner already combined upstream by steering_input.sv) --
	// sor_master samples this on each real CPU read and reproduces MAME's
	// dial_compute_value() direction+magnitude encoding from it (see that
	// module's p1_wheel port comment; the wheel is a free-spinning rotary
	// encoder on real hardware, not an absolute position).
	input   [7:0] p1_wheel, p2_wheel, p3_wheel,
	// Pedal: raw 8-bit value (0=released, 255=full), matches MAME's
	// IPT_PEDAL AN0/AN1/AN2 exactly -- read directly, no encoding.
	input   [7:0] p1_pedal, p2_pedal, p3_pedal,

	// OSD options
	input         service,
	input         free_play,

	// Audio (WP10: leland_dac_mixer's mono output, via sor_sound)
	output signed [15:0] audio_out
);

//------------------------------------------------------------------
// Clock enables
//   clk_sys = 48 MHz
//   CE_6M  — Z80 master and slave (6 MHz, every 8 cycles)
//   CE_8M  — 80186 sound CPU     (8 MHz, every 6 cycles)
//   ce_pix — pixel clock (~7.16 MHz, phase-accumulator)
//------------------------------------------------------------------
reg [2:0] ce_z80_cnt = 3'd0; // sim-only initializer -- Quartus/hardware default to 0
                              // anyway, but Icarus/ModelSim leave X forever without
                              // it, which permanently starves CE_6M and freezes both
                              // Z80 cores in simulation
reg [2:0] ce_186_cnt = 3'd0; // same reasoning

wire CE_6M = (ce_z80_cnt == 3'd0);
wire CE_8M = (ce_186_cnt == 3'd0); // WP10: consumed by sor_sound's i186_periph instance

always @(posedge clk_sys) begin
	ce_z80_cnt <= (ce_z80_cnt == 3'd7) ? 3'd0 : ce_z80_cnt + 1'd1;
	ce_186_cnt <= (ce_186_cnt == 3'd5) ? 3'd0 : ce_186_cnt + 1'd1;
end

// Pixel clock: 7.159090 MHz from 48 MHz
reg [15:0] pix_acc = 16'd0; // sim-only initializer -- same reasoning as
                            // ce_z80_cnt/ce_186_cnt above: Quartus/hardware
                            // default to 0 anyway, but Icarus/ModelSim leave
                            // X forever without it. An uninitialized pix_acc
                            // makes every "pix_acc+7159 >= 48000" comparison
                            // X, which an `if` treats as false, so ce_pix_r
                            // never asserts and hc/vc (and everything gated
                            // on ce_pix, including the background tile-fetch
                            // FSM) never run at all in simulation.
reg        ce_pix_r;
always @(posedge clk_sys) begin
	if (pix_acc + 16'd7159 >= 16'd48000) begin
		pix_acc  <= pix_acc + 16'd7159 - 16'd48000;
		ce_pix_r <= 1;
	end else begin
		pix_acc  <= pix_acc + 16'd7159;
		ce_pix_r <= 0;
	end
end
assign ce_pix = ce_pix_r;

//------------------------------------------------------------------
// SDRAM controller — single-port sdram_simple + external priority
// arbiter (wr > rd0 > rd1 > rd2).
//
// Replaces the old internal 4-channel design. This session's
// exhaustive debugging effort proved the only thing that EVER worked
// reliably, across every hardware test, was a fully isolated, non-
// arbitrated transaction (the byte-test) -- strong circumstantial
// evidence the bug lived in the multi-channel sharing/arbitration
// logic itself (shared a/data/bank/last_a registers, cache-hit
// heuristics, casex-driven command decode across 4 channels), not the
// raw SDRAM protocol/timing. sdram_simple is a known-good, externally
// reviewed single-port controller with a properly JEDEC-sequenced init
// (PRECHARGE-ALL -> 2x AUTO_REFRESH -> LOAD_MODE, which our old
// design's init omitted -- see its own file header) and internally
// owned periodic refresh, so there's no board-level refresh-pulse
// generator or refresh-vs-write priority ordering left to get wrong.
//------------------------------------------------------------------
reg         sdram_ready; // sticky "init genuinely completed" -- see always block below

// Write port (ioctl) -- ALWAYS a full 16-bit word write now (both
// bytes known up front: sdram_wr_data = even/low byte,
// sdram_wr_data_hi = odd/high byte), never a single masked byte. See
// the paired-write comment on the drain logic below for why: real
// hardware testing proved DQM has no effect on which byte lane
// actually gets stored on this board, so any write that depended on
// DQM masking one lane was silently corrupting the other lane's
// previous content. sdram_wr_addr is always the EVEN (word-aligned)
// address of the pair.
wire        sdram_wr_req;
reg         sdram_wr_ack;
wire [24:0] sdram_wr_addr;
wire  [7:0] sdram_wr_data;
wire  [7:0] sdram_wr_data_hi;

// Read port 0 (master Z80)
wire        sdram_rd0_req;
reg         sdram_rd0_ack;
wire [24:0] sdram_rd0_addr;
reg   [7:0] sdram_rd0_data;

// Read port 1 (slave Z80)
wire        sdram_rd1_req;
reg         sdram_rd1_ack;
wire [24:0] sdram_rd1_addr;
reg   [7:0] sdram_rd1_data;

// Read port 3 (WP10: sound-CPU ROM, via rtl/sor_sound.sv's byte-wide
// rom_req/rom_addr/rom_data/rom_stall port -- built the same way as
// rd0/rd1 below, mirroring the already hardware-validated pattern
// rather than inventing a new one for this most hardware-fragile part
// of the codebase). Priority placed between rd1 (slave) and rd2
// (diagnostic) -- diagnostic keeps its already-lowest priority.
wire        sdram_rd3_req;
reg         sdram_rd3_ack;
wire [24:0] sdram_rd3_addr;
reg   [7:0] sdram_rd3_data;

// Read port 2 (diagnostic readback scanner)
reg         sdram_rd2_req;
reg         sdram_rd2_ack;
reg  [24:0] sdram_rd2_addr;
reg   [7:0] sdram_rd2_data;

// SDRAM chip-side signals (16-bit DQ, matches the physical SDRAM_DQ pin)
wire        sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n;
wire  [1:0] sd_ba;
wire [12:0] sd_a;
wire  [1:0] sd_dqm;
wire [15:0] sd_dq_out;
wire        sd_dq_oe;
wire [15:0] sd_dq_in = SDRAM_DQ;
assign SDRAM_DQ = sd_dq_oe ? sd_dq_out : 16'bz;

assign SDRAM_CKE  = sd_cke;
assign SDRAM_nCS  = sd_cs_n;
assign SDRAM_nRAS = sd_ras_n;
assign SDRAM_nCAS = sd_cas_n;
assign SDRAM_nWE  = sd_we_n;
assign SDRAM_BA   = sd_ba;
assign SDRAM_A    = sd_a;
assign {SDRAM_DQMH, SDRAM_DQML} = sd_dqm;

// Single-port module's own client interface
// (* keep *) attributes on this whole cluster: for the SignalTap
// hardware-debug capture -- see SIGNALTAP_SETUP.md -- so these survive
// synthesis with their exact RTL names instead of being optimized
// away or renamed, regardless of how Quartus flattens hierarchy.
(* keep *) wire        sd_ready;
(* keep *) wire        sd_req_done;
(* keep *) wire  [7:0] sd_dout;

localparam CH_WR = 3'd0, CH_RD0 = 3'd1, CH_RD1 = 3'd2, CH_RD3 = 3'd3, CH_RD2 = 3'd4;

// Fixed priority: wr > rd0 > rd1 > rd3(sound) > rd2(diagnostic), gated
// by !in_flight. rd3 (WP10) inserted between rd1 and rd2 -- diagnostic
// keeps its already-lowest priority.
//
// THE DUPLICATE-TRANSACTION RACE this gate closes (root cause of the
// hardware read-back corruption): sdram_simple is LEVEL-sensitive --
// its S_IDLE accepts whenever (rd||we) is high. Our per-channel req
// lines are level-held until the client sees its ack, and the ack is
// REGISTERED here (client sees it one cycle after req_done). So on
// the very cycle a transaction completes (ready re-asserts, req_done
// fires), the winning client's req line is unavoidably STILL HIGH --
// and without this gate the controller immediately accepted the same
// request a second time. That duplicate completed several cycles
// later and fired a second req_done, delivering a spurious ack (with
// the OLD transaction's data) after the client had already moved on
// to its next request -- so every read channel ran exactly one
// transaction behind (scan_b0 showed the byte-test's 0xA5, the CPU's
// RD0 trace showed each address paired with the previous byte's
// data, and the read checksum was a shifted sum). Writes were
// immune in effect only because a duplicated write rewrites the
// same byte -- harmless -- which is why every write-side probe
// looked correct while all read paths corrupted. Edge-triggered
// controllers (NES/C64-family: rd & ~old_rd) structurally can't do
// this; a level-sensitive one must be fenced externally, which is
// what in_flight does: no request is presented from the moment one
// is accepted until the granted client has dropped its req line
// (i.e. its ack has been fully consumed). All clients in this design
// deassert req for >=1 cycle after ack, so release is guaranteed.
(* keep *) reg        in_flight;   // transaction accepted, client hasn't dropped req yet
(* keep *) reg        done_seen;   // its req_done has fired (completion latched)
(* keep *) reg  [2:0] issued_ch;

wire sel_wr  = !in_flight && sdram_wr_req;
wire sel_rd0 = !in_flight && !sel_wr && sdram_rd0_req;
wire sel_rd1 = !in_flight && !sel_wr && !sel_rd0 && sdram_rd1_req;
wire sel_rd3 = !in_flight && !sel_wr && !sel_rd0 && !sel_rd1 && sdram_rd3_req;
wire sel_rd2 = !in_flight && !sel_wr && !sel_rd0 && !sel_rd1 && !sel_rd3 && sdram_rd2_req;

(* keep *) wire [24:0] sd_addr = sel_wr  ? sdram_wr_addr  :
                       sel_rd0 ? sdram_rd0_addr :
                       sel_rd1 ? sdram_rd1_addr :
                       sel_rd3 ? sdram_rd3_addr :
                                 sdram_rd2_addr;
(* keep *) wire  [7:0] sd_din    = sdram_wr_data;
(* keep *) wire  [7:0] sd_din_hi = sdram_wr_data_hi;
// Every write is now a full-word write (see sdram_wr_data_hi comment
// above) -- the single-byte-masked `we` path in sdram_simple is kept
// only as a legacy fallback for a future caller that might need it,
// unused here.
(* keep *) wire        sd_we      = 1'b0;
(* keep *) wire        sd_we_word = sel_wr;
(* keep *) wire        sd_rd      = sel_rd0 | sel_rd1 | sel_rd3 | sel_rd2;

// Level of the granted channel's own req line -- in_flight releases
// only once this drops, guaranteeing the controller never re-sees a
// request whose ack is still propagating back to its client.
wire issued_req_level = (issued_ch == CH_WR)  ? sdram_wr_req  :
                        (issued_ch == CH_RD0) ? sdram_rd0_req :
                        (issued_ch == CH_RD1) ? sdram_rd1_req :
                        (issued_ch == CH_RD3) ? sdram_rd3_req :
                                                sdram_rd2_req;

always @(posedge clk_sys) begin
	sdram_wr_ack  <= 1'b0;
	sdram_rd0_ack <= 1'b0;
	sdram_rd1_ack <= 1'b0;
	sdram_rd3_ack <= 1'b0;
	sdram_rd2_ack <= 1'b0;

	if (sdram_init) begin
		sdram_ready <= 1'b0;
		in_flight   <= 1'b0;
		done_seen   <= 1'b0;
	end else begin
		// Sticky "init genuinely completed" -- sd_ready itself toggles
		// low/high per-transaction after init, but CPU reset gating,
		// ioctl_wait, and the on-screen status tile all need "has init
		// completed at least once", not "is the port idle right now".
		if (sd_ready) sdram_ready <= 1'b1;

		// Accept: latch the granted channel on the exact cycle the
		// module is idle AND a request is presented (the cycle it
		// samples addr/rd/we). sel_* are gated by !in_flight, so an
		// accept can only happen when no transaction is outstanding --
		// issued_ch can never be overwritten mid-flight by a higher-
		// priority channel raising its req.
		if (sd_ready && (sd_rd || sd_we_word)) begin
			issued_ch <= sel_wr ? CH_WR : sel_rd0 ? CH_RD0 : sel_rd1 ? CH_RD1 : sel_rd3 ? CH_RD3 : CH_RD2;
			in_flight <= 1'b1;
			done_seen <= 1'b0;
		end

		// Completion -- sd_req_done unambiguously distinguishes a real
		// client transaction finishing from an internal refresh cycle
		// finishing (both otherwise look identical from the outside:
		// ready drops, then re-asserts). With the in_flight gate above
		// it can now fire at most once per accepted transaction.
		if (sd_req_done) begin
			done_seen <= 1'b1;
			case (issued_ch)
				CH_WR:  sdram_wr_ack  <= 1'b1;
				CH_RD0: begin sdram_rd0_data <= sd_dout; sdram_rd0_ack <= 1'b1; end
				CH_RD1: begin sdram_rd1_data <= sd_dout; sdram_rd1_ack <= 1'b1; end
				CH_RD3: begin sdram_rd3_data <= sd_dout; sdram_rd3_ack <= 1'b1; end
				CH_RD2: begin sdram_rd2_data <= sd_dout; sdram_rd2_ack <= 1'b1; end
			endcase
		end

		// Release: only after completion has fired AND the granted
		// client has dropped its req (proof it consumed the ack). Only
		// then may the priority mux present the next request.
		if (in_flight && (done_seen || sd_req_done) && !issued_req_level) begin
			in_flight <= 1'b0;
			done_seen <= 1'b0;
		end
	end
end

sdram_simple #(
	.CLK_MHZ(48),
	// CAS_LAT=3: matches Darius's real, deployed, hardware-proven value.
	// (The reference file's own default of 2 -- and briefly, widening
	// TRCD_NS -- were both tried and ruled out as fixes for the 'zz'
	// read-back symptom seen in sim; the real cause was an off-by-one
	// in the module's own read-capture scheduling, fixed directly in
	// rtl/sdram.sv rather than by tuning timing parameters here. CAS_LAT
	// itself is left at this conservative, widely-supported value
	// rather than tuned further against any one simulator or physical
	// module.)
	.CAS_LAT(3)
	// READ_CAP_EXTRA left at module default (0) for B5 (docs/sdram_plan.md
	// Section 3a/4 Step 3): 90-deg SDRAM_CLK phase (pll_0002.v
	// phase_shift1) with capture back at the original T4 point --
	// Section 1.2 predicts +3.5/+14 ns here without needing the capture
	// stretch B2-B4 used. (B2-B4 used READ_CAP_EXTRA=1; see git history /
	// docs/SDRAM_TIMING_INVESTIGATION.md if reviving that combination.)
) sdram_ctrl
(
	.sd_cke   (sd_cke),
	.sd_cs_n  (sd_cs_n),
	.sd_ras_n (sd_ras_n),
	.sd_cas_n (sd_cas_n),
	.sd_we_n  (sd_we_n),
	.sd_ba    (sd_ba),
	.sd_a     (sd_a),
	.sd_dqm   (sd_dqm),
	.sd_dq_out(sd_dq_out),
	.sd_dq_oe (sd_dq_oe),
	.sd_dq_in (sd_dq_in),

	.clk     (clk_sys),
	.rst_n   (~sdram_init),
	.addr    (sd_addr),
	.din     (sd_din),
	.dout    (sd_dout),
	.rd      (sd_rd),
	.we      (sd_we),
	.din_hi  (sd_din_hi),
	.we_word (sd_we_word),
	.ready   (sd_ready),
	.req_done(sd_req_done)
);

//------------------------------------------------------------------
// SDRAM clock forwarding -- the single-port module leaves this to the
// top level (same as it leaves the DQ tristate to the top level, done
// above). Real hardware needs the registered DDR output for correct
// physical timing; simulation doesn't need/support the Altera-specific
// primitive. Moved here from the old sdram.sv, same USE_ALTDDIO switch.
//
// outclk_1 scheme (docs/sdram_plan.md Section 3a, replaces the old
// inverted-clk_sys/180-deg-via-DDIO trick): ALL phase now comes from the
// PLL's second output (clk_sdram); the DDIO stays in the IOE purely for
// low-skew forwarding, non-inverting (datain_h=1, datain_l=0).
//------------------------------------------------------------------
generate
	if (USE_ALTDDIO) begin : g_ddr_clk
		altddio_out #(
			.extend_oe_disable    ("OFF"),
			.intended_device_family("Cyclone V"),
			.invert_output        ("OFF"),
			.lpm_hint             ("UNUSED"),
			.lpm_type             ("altddio_out"),
			.oe_reg               ("UNREGISTERED"),
			.power_up_high        ("OFF"),
			.width                (1)
		) sdramclk_ddr (
			.datain_h  (1'b1),
			.datain_l  (1'b0),
			.outclock  (clk_sdram),
			.dataout   (SDRAM_CLK),
			.aclr      (1'b0),
			.aset      (1'b0),
			.oe        (1'b1),
			.outclocken(1'b1),
			.sclr      (1'b0),
			.sset      (1'b0)
		);
	end else begin : g_sim_clk
		assign SDRAM_CLK = clk_sdram;
	end
endgenerate

//------------------------------------------------------------------
// ioctl → SDRAM write
//
//   ioctl_index 0x00: master ROM, flat 256 KB, SDRAM 0x000000+
//   ioctl_index 0x01: slave  ROM, flat 512 KB, SDRAM 0x040000+
//
// ioctl_wait stays high from wr_req until wr_ack, holding off the
// HPS downloader.  We also stall during SDRAM init (sdram_ready=0).
//------------------------------------------------------------------
//------------------------------------------------------------------
// ROM-region routing by ioctl_addr RANGE, single download session.
//
// ROOT CAUSE of the SDRAM-era total boot failure (found via the
// accept-time probes: byte 0 accepted exactly FIVE times per load):
// the MRA used to declare FIVE separate <rom index="N"> regions
// (master/slave/sound/gfx/proms), each its own download session.
// That is non-standard MRA structure -- convention (per the MiSTer
// wiki) is a single index="0" session per core, with non-zero
// indices reserved for metadata, not region splitting. The
// multi-session structure caused two distinct failures in turn:
// first, hex index attributes ("0x00".."0x04") decimal-parsed to 0,
// so ALL five sessions arrived as ioctl_index==0 and each region
// overwrote the master ROM's SDRAM address in sequence; after fixing
// that (decimal indexes), the underlying multi-session structure
// itself remained a problem, because ioctl_download (and this core's
// `reset`, which includes it) drops between EVERY session -- so
// treating the first such edge as "loading finished" launched the
// CPUs and diagnostics into a race against the remaining downloads
// (confirmed by rd_chk changing between otherwise-identical runs).
//
// Fixed at the root: the MRA is now ONE <rom index="0"> session
// covering a single flat address space (master+slave ROM contiguous
// and identical to their SDRAM addresses, sound sent as a fill and
// dropped, gfx/palette PROM after it for the on-chip BRAMs). One
// session means ioctl_download only ever drops once, at the true end
// of loading -- restoring the simple, standard "reset falling edge
// means done" assumption that every other single-session MiSTer core
// relies on. The core now decodes ioctl_addr ranges instead of
// ioctl_index for everything.
//------------------------------------------------------------------
// 2026-07-12 session diagnostic: sound region removed from the MRA
// entirely (previously a 1MB fill/dropped placeholder at this offset;
// see mra/SuperOffRoad.mra) to test whether ioctl_download misbehaves
// around a huge dead span immediately preceding gfx_rom's first real
// bytes -- gfx_wr_count read exactly 0 on hardware every time despite
// prom_rom (the next region over) loading correctly, and two prior
// MRA-only variations (all-fill, then partial-real-data) both showed
// no change. This removes the gap variable completely instead of
// partially: GFX now starts immediately where the sound region used
// to (ADDR_SOUND_LO's old value), and PROM/PROM_HI shift down by the
// same 0x100000 the sound region used to occupy. ADDR_SOUND_LO itself
// is unchanged -- it's still the correct SDRAM/BRAM boundary, just now
// immediately adjacent to GFX with nothing in between.
// 2026-07-12 session, reverted: the sound-region-removal experiment
// (GFX/PROM addresses shifted down 0x100000, sound region dropped
// from the MRA entirely) showed zero change in gfx_rom's write-hit
// counter, so it didn't confirm the theory it was testing -- and the
// very next build (with this revert not yet applied) showed prom_rom
// had ALSO gone completely dead (prom_fp_u69 read 00, expected 69),
// something never seen before that removal. Reverting back to the
// original layout (sound gap restored in the MRA, addresses back to
// their original values) to isolate whether that specific experiment
// caused the prom_rom regression, or whether the bug predates it.
localparam [26:0] ADDR_SOUND_LO = 27'h0C0000; // 1MB unemulated sound ROM, dropped
localparam [26:0] ADDR_GFX_LO   = 27'h1C0000; // 96KB gfx tile ROM -> BRAM
localparam [26:0] ADDR_PROM_LO  = 27'h1D8000; // 128KB palette PROM -> BRAM

// ioctl_addr, one clk_sys cycle delayed. Declared here (ahead of its
// first use just below) rather than down near the write-path logic
// it was originally added for -- tools that require strict
// declare-before-use (e.g. vlog) rejected the forward reference.
reg [26:0] ioctl_addr_d1;
always @(posedge clk_sys) ioctl_addr_d1 <= ioctl_addr;
localparam [26:0] ADDR_PROM_HI  = 27'h1F8000; // end of flat ROM image

// ioctl_index==0 qualifier is MANDATORY alongside the address-range
// check: the ARM sends more than ROM over ioctl. In particular the
// MRA's <switches> DIP settings arrive as their own download session
// (ioctl_index 254) starting at ioctl_addr 0 -- and are RE-SENT
// whenever the user touches the OSD DIP menu. An address-only gate
// (as briefly used after the addr-range routing rewrite) let that
// payload overwrite master ROM bytes 0..N after the ROM had loaded:
// measured on hardware as dbg_dl_sessions==02, accept0/1_cnt==02
// (each low address written twice -- ROM byte first, captured as
// F3/ED, then the DIP byte), and the readback scan showing 00s at
// exactly the first addresses. Every established arcade core gates
// its ROM write path on ioctl_index for precisely this reason.
// Upper bound is ADDR_GFX_LO, not ADDR_SOUND_LO: this admits the
// sound-ROM range (0x0C0000-0x1BFFFF) through the SAME gate/FIFO/drain
// pipeline as Master/Slave, not a separate one. That's a deliberate
// choice, not an oversight -- sound ROM shares this pipeline's exact
// destination (SDRAM, flat-address==SDRAM-address, same even/odd
// pairing) with Master/Slave, unlike GFX/PROM (which land in BRAM via
// their own independent gate at ADDR_GFX_LO/ADDR_PROM_LO below) --
// giving sound ROM a second FIFO here would just be duplicated risk in
// the single most hardware-fragility-proven block in this file, for a
// pipeline that already handles this shape of data correctly.
// ADDR_SOUND_LO itself was the ORIGINAL upper bound, from when the
// sound region was a dropped 1MB zero-fill placeholder (pre-WP10) --
// leaving this gate there after the MRA started streaming real sound
// ROM bytes silently dropped every one of them before they ever
// reached SDRAM (found and fixed this session; see
// docs/WP10_PROGRESS.md). See also wfifo's own width below, which
// needed widening from 20 to 21 address bits for the same reason (20
// bits only covered the old Master+Slave-only 0x000000-0x0BFFFF
// range).
wire ioctl_wr_rom = ioctl_wr && ioctl_download && (ioctl_index[7:0] == 8'h00) &&
                    (ioctl_addr_d1 < ADDR_GFX_LO);

// DIP switches (ioctl_index 254): capture instead of discarding --
// the game will eventually need them on an input port; for now this
// also documents the index-254 stream's existence explicitly.
reg [63:0] dips;
always @(posedge clk_sys) begin
	if (ioctl_wr && ioctl_download && (ioctl_index[7:0] == 8'hFE) && (ioctl_addr_d1 < 27'd8))
		dips[ioctl_addr_d1[2:0]*8 +: 8] <= ioctl_data;
end

// Sanity-check counter, kept as an overlay row: total ioctl_download
// falling edges seen. Measured at 02 for the single <rom index="0">
// MRA on real hardware -- the ARM does not guarantee one download
// session per <rom> tag, so "loading done" below is settle-timer
// based (session-count-independent), not keyed off this value.
reg       ioctl_download_r;
reg [7:0] dbg_dl_sessions;
always @(posedge clk_sys) begin
	ioctl_download_r <= ioctl_download;
	if (sdram_init) begin
		dbg_dl_sessions <= 8'd0;
	end else if (ioctl_download_r && !ioctl_download) begin
		if (dbg_dl_sessions != 8'hFF) dbg_dl_sessions <= dbg_dl_sessions + 8'd1;
	end
end

// "Downloads settled": at least one ioctl_download has been seen AND
// it has stayed low for DL_SETTLE_CYCLES since. This -- not a reset
// falling edge, and not a specific expected session count -- is the
// true "all ROM loading finished" condition. Gates the CPUs, the
// byte-test, and the readback scan.
reg        dl_seen;
reg [23:0] dl_settle_cnt;
wire       dl_settled = dl_seen && !ioctl_download &&
                        (dl_settle_cnt >= DL_SETTLE_CYCLES[23:0]);
always @(posedge clk_sys) begin
	if (sdram_init) begin
		dl_seen       <= 1'b0;
		dl_settle_cnt <= 24'd0;
	end else if (ioctl_download) begin
		dl_seen       <= 1'b1;
		dl_settle_cnt <= 24'd0;
	end else if (dl_settle_cnt != 24'hFFFFFF) begin
		dl_settle_cnt <= dl_settle_cnt + 24'd1;
	end
end

// THE root cause, found by tracing sys/hps_io.sv directly: ioctl_wr
// (the externally-visible strobe) is a REGISTERED, one-cycle-delayed
// copy of an internal `wr` signal ("ioctl_wr <= wr;"), but ioctl_addr
// is incremented in the SAME cycle `wr` is set ("ioctl_addr <=
// ioctl_addr + 1"), not delayed to match. So by the time ioctl_wr
// finally reads 1 externally, ioctl_addr has ALREADY advanced to the
// NEXT byte's address -- ioctl_data is unaffected (it isn't
// overwritten until the following real transfer), so every byte's
// data ends up paired with the address of the byte AFTER it. This is
// a universal characteristic of hps_io affecting every core that uses
// it; the standard, correct compensation is a locally registered,
// one-cycle-delayed copy of ioctl_addr, sampled instead of the live
// signal. Confirmed via the accept-time debug probe: watching for
// ioctl_addr==0 caught the correct byte-0 data (0xF3) at all -- only
// possible because the loader's own `skip_add` mechanism happens to
// suppress the address increment for a transfer's very first byte,
// masking the bug there specifically while every subsequent byte in
// the same transfer is silently shifted one address early.
// Isolated single-byte write/readback self-test (see below): steals the
// wr channel for exactly one transaction before ioctl loading logic
// takes over. Safe — this only fires once, before the HPS ever starts
// streaming ROM data (gated on sdram_ready, well before ioctl_download).
wire        bt_write_active;
wire [24:0] bt_wr_addr;
wire  [7:0] bt_wr_data;    // low/even byte
wire  [7:0] bt_wr_data_hi; // high/odd byte -- paired word write, same as the real ROM path
reg         bt_wr_done;   // write half complete; declared here (used below) — defined in the byte-test block further down

// Latched copies of ioctl_addr/ioctl_data/ioctl_index, captured the
// same cycle a byte is accepted (ioctl_wr_rom && !wr_pending). The
// actual SDRAM write is a multi-cycle transaction (accepted by the
// controller's arbiter, then STATE_START..STATE_READY before wr_ack
// finally pulses); driving sdram_wr_addr/sdram_wr_data combinationally
// from the LIVE ioctl_addr/ioctl_data (as before) relied on HPS holding
// those signals perfectly stable for that entire window. Hardware
// evidence: the write-side ground-truth debug row showed addresses 0-3
// of the Master ROM never written at all (still their power-up-zero
// default) despite the combined write checksum genuinely accumulating
// real, nonzero values from elsewhere in the same load -- i.e. the
// accept-byte gating fires correctly in general, but whatever value
// ioctl_addr/ioctl_data held at the moment of accept for the very
// first few bytes didn't survive to reach the controller. Latching
// removes any dependence on HPS-side timing assumptions entirely.
reg [26:0] latched_ioctl_addr;      // always the EVEN (word) address of the pair
reg  [7:0] latched_ioctl_data;      // low/even byte
reg  [7:0] latched_ioctl_data_hi;   // high/odd byte -- see paired-write comment below
reg        latched_ioctl_slave; // 1 = addr >= 0x040000, i.e. NOT Master ROM (Slave OR,
                                 // since this session, Sound ROM too -- name kept for
                                 // history, but every consumer below only ever tests
                                 // !latched_ioctl_slave meaning "is this Master ROM", so
                                 // Sound ROM correctly falling under the 1-case is fine)

// Master (0x000000-0x03FFFF) and Slave (0x040000-0x0BFFFF) ROM are
// contiguous in the flat MRA address space AND numerically identical
// to their SDRAM addresses -- no per-region offset math needed at all,
// unlike the old ioctl_index-keyed version. Straight pass-through.
assign sdram_wr_addr    = bt_write_active ? bt_wr_addr    : latched_ioctl_addr[24:0];
assign sdram_wr_data    = bt_write_active ? bt_wr_data    : latched_ioctl_data;
assign sdram_wr_data_hi = bt_write_active ? bt_wr_data_hi : latched_ioctl_data_hi;

// Accept-time ground truth, upstream of everything else: does HPS ever
// even present address 0 (ioctl_index==0) with ioctl_wr asserted at
// all? Taps ioctl_addr_d1/ioctl_data/ioctl_index at the exact instant
// a byte is accepted -- before the write-port mux, before the SDRAM
// controller ever sees it. (Originally written against the live,
// undelayed ioctl_addr, back when the one-cycle skew above was still
// undiscovered -- this was in fact the very probe that exposed it:
// it caught byte 0's correct data purely because `skip_add` masks the
// bug for a transfer's first byte specifically.) The isolated
// byte-test bypasses this entire path (it drives bt_wr_addr/bt_wr_data
// directly, never touching ioctl_addr/data), so it can't tell us
// anything about whether this path itself ever fires for address 0.
// dbg_accept0_cnt saturates at 0xFF; dbg_accept0_data latches only on
// the FIRST occurrence.
reg  [7:0] dbg_accept0_cnt;
reg  [7:0] dbg_accept0_data;
reg        dbg_accept0_seen;
// Direct A/B check of the derived skew theory: the RAW (undelayed)
// ioctl_addr's low byte, captured at the exact same first-occurrence
// event as dbg_accept0_data. If ioctl_addr_d1==0 at that moment and
// the skew theory is right, raw ioctl_addr should read 1 here (already
// advanced); if it also reads 0, the derived one-cycle skew doesn't
// actually exist as described and the theory needs to be discarded.
// RESULT: read back 0x00 -- skew theory disproven, ioctl_addr_d1 kept
// (harmless, proven equivalent to live ioctl_addr in practice) but no
// longer load-bearing for any fix.
reg  [7:0] dbg_accept0_raw_addr;

// Same accept-time probe, applied to address 1: is address 0's
// behavior (accepted repeatedly, ack-time capture never fires)
// specific to address 0, or does address 1 show the same pattern?
// If addresses 0 AND 1 both get accepted multiple times with correct
// data, but ack-time (dbg_wr_b2/b3, watching addresses 2/3) never
// fires at all, that localizes the anomaly to "the first couple of
// bytes of a transfer" rather than "address 0 specifically".
reg  [7:0] dbg_accept1_cnt;
reg  [7:0] dbg_accept1_data;
reg        dbg_accept1_seen;

// Re-test the skew theory where it actually matters: the root-cause
// comment above claims the transfer's very first byte is specifically
// EXEMPT from the skew (via hps_io's own `skip_add` mechanism), which
// means the earlier A/B disproof -- done at address 0 -- tested the
// one byte the theory itself predicts would show no skew. It proved
// nothing about byte 2+. Re-run the same raw-vs-delayed comparison at
// address 2: if ioctl_addr_d1==2 here but the RAW (undelayed)
// ioctl_addr reads 3 (already advanced), the skew is real for this
// byte and ioctl_addr_d1 is load-bearing; if raw also reads 2, the
// skew genuinely doesn't exist here either.
reg  [7:0] dbg_accept2_data;
reg  [7:0] dbg_accept2_raw_addr;
reg        dbg_accept2_seen;

// Most decisive remaining test: read address 0 back IMMEDIATELY after
// its own write acknowledges -- essentially zero elapsed time, unlike
// dbg_early_b0 which still waits for the rest of the ~256KB Master ROM
// to finish loading first (still potentially tens of ms later).
// accept0/accept1 already proved correct data (0xF3/0xED) reaches the
// write-acceptance stage; if THIS also reads back wrong, the fault is
// in the actual SDRAM write or in reading it back moments later --
// not decay, not anything ioctl/HPS-side. Set here (write-ack side);
// consumed by the rd_phase state machine below (sole driver of the
// rd2 channel), which is idle (parked in RD_IDLE) throughout the real
// Master ROM load in this design, so stealing it briefly here doesn't
// conflict with the byte-test/early-checkpoint/main-scan users, which
// all run later.
reg       immediate_verify_want;
reg       immediate_verify_done;
reg [7:0] dbg_immediate_b0;

//------------------------------------------------------------------
// NON-LOSSY ioctl capture: skid FIFO between hps_io and the SDRAM
// write channel.
//
// WHY (the BRAM->SDRAM regression, root cause): reading the real
// sys/hps_io.sv shows ioctl_wait is NEVER consumed by hps_io's own
// download state machine -- it is only exported to the ARM CPU
// (HPS_BUS[37]), which polls it IN SOFTWARE and pauses the stream
// with software reaction latency. Until the ARM notices, io_strobes
// keep coming (potentially only a few clk_sys cycles apart in a
// burst), each one advancing ioctl_addr and pulsing ioctl_wr
// unconditionally. The old accept logic here
// (`ioctl_wr_rom && !wr_pending`) silently DISCARDED any strobe that
// arrived during the ~10-cycle SDRAM write transaction -- the byte
// was simply never written, and hps_io had already moved past it.
// The original BRAM loader (`rom[addr] <= data` every cycle,
// unconditionally) was structurally incapable of dropping a strobe,
// which is exactly why the core worked on BRAM and broke the moment
// ROM storage moved to SDRAM. No simulation showed it because the
// testbench respected ioctl_wait immediately, like the real ARM
// does not.
//
// FIX: capture EVERY strobe in a single cycle, unconditionally
// (restoring BRAM's acceptance semantics), into a 32-deep FIFO
// drained at SDRAM pace. ioctl_wait asserts at half-full, so the
// ARM's slow software reaction has 16 spare slots of cushion --
// orders of magnitude more headroom than the burst spacing needs.
//------------------------------------------------------------------
localparam WFIFO_AW = 5;                    // 32 entries
// {addr[20:0], data[7:0]}: 21 address bits (widened this session from
// the original 20 -- see docs/WP10_PROGRESS.md) to cover the full
// 0x000000-0x1BFFFF Master+Slave+Sound flat range now that
// ioctl_wr_rom's own gate above admits Sound ROM through this same
// pipeline (0x1BFFFF needs 21 bits; the old 20-bit field only covered
// up to 0x0FFFFF, silently wrapping/aliasing anything at or above
// 0x100000 back onto Master/Slave's own addresses -- exactly the kind
// of corruption a naive "just widen the gate" fix would have caused
// without also widening this). The master/slave/sound split is
// derived from the address at drain time, no separate per-region flag
// needed now that addressing is flat.
reg [28:0] wfifo [0:(1<<WFIFO_AW)-1];
reg [WFIFO_AW:0] wfifo_wptr, wfifo_rptr;    // extra bit for full/empty
wire [WFIFO_AW:0] wfifo_level = wfifo_wptr - wfifo_rptr;
wire [WFIFO_AW-1:0] wfifo_rptr_p1 = wfifo_rptr[WFIFO_AW-1:0] + 1'b1; // odd entry of a pair
wire wfifo_empty = (wfifo_level == 0);
wire wfifo_full  = wfifo_level[WFIFO_AW];

// Sticky overflow flag: should be impossible with the half-full wait
// threshold; if it ever sets, bytes were lost and the load is known
// bad (kept for future debug visibility).
reg wfifo_overflow;

// Enqueue: every ROM strobe, one cycle, no busy check. Uses
// ioctl_addr_d1 -- hardware probes proved raw and delayed addr are
// identical at strobe time, so d1 is safe and unchanged from the
// proven-correct capture timing.
always @(posedge clk_sys) begin
	if (sdram_init) begin
		wfifo_wptr     <= '0;
		wfifo_overflow <= 1'b0;
	end else if (ioctl_wr_rom) begin
		if (!wfifo_full) begin
			wfifo[wfifo_wptr[WFIFO_AW-1:0]] <= {ioctl_addr_d1[20:0], ioctl_data};
			wfifo_wptr <= wfifo_wptr + 1'd1;
		end else begin
			wfifo_overflow <= 1'b1;
		end
	end
end

// Accept-time ground-truth probes, now keyed off the enqueue event
// (same sampling instant as before -- the strobe cycle itself).
always @(posedge clk_sys) begin
	if (sdram_init) begin
		dbg_accept0_cnt      <= 8'h00;
		dbg_accept0_data     <= 8'h00;
		dbg_accept0_seen     <= 1'b0;
		dbg_accept0_raw_addr <= 8'h00;
		dbg_accept1_cnt      <= 8'h00;
		dbg_accept1_data     <= 8'h00;
		dbg_accept1_seen     <= 1'b0;
		dbg_accept2_data     <= 8'h00;
		dbg_accept2_raw_addr <= 8'h00;
		dbg_accept2_seen     <= 1'b0;
	end else if (ioctl_wr_rom && (ioctl_addr_d1 < 27'h040000)) begin
		if (ioctl_addr_d1 == 27'd0) begin
			if (dbg_accept0_cnt != 8'hFF) dbg_accept0_cnt <= dbg_accept0_cnt + 8'd1;
			if (!dbg_accept0_seen) begin
				dbg_accept0_seen     <= 1'b1;
				dbg_accept0_data     <= ioctl_data;
				dbg_accept0_raw_addr <= ioctl_addr[7:0];
			end
		end
		if (ioctl_addr_d1 == 27'd1) begin
			if (dbg_accept1_cnt != 8'hFF) dbg_accept1_cnt <= dbg_accept1_cnt + 8'd1;
			if (!dbg_accept1_seen) begin
				dbg_accept1_seen <= 1'b1;
				dbg_accept1_data <= ioctl_data;
			end
		end
		if ((ioctl_addr_d1 == 27'd2) && !dbg_accept2_seen) begin
			dbg_accept2_seen     <= 1'b1;
			dbg_accept2_data     <= ioctl_data;
			dbg_accept2_raw_addr <= ioctl_addr[7:0];
		end
	end
end

// Dequeue/drain: pop a PAIR of bytes (even address then its odd
// neighbor -- guaranteed adjacent and in that order, since the Master
// and Slave ROM regions both start at address 0 and are both an even
// number of bytes long, so the byte stream never produces an odd
// leftover) into the latched_* staging registers, and issue ONE
// 16-bit word write covering both, whenever the write channel is
// free. wr_pending keeps its exact old meaning -- one SDRAM write
// transaction in progress.
//
// WHY PAIRED, NOT PER-BYTE: real hardware testing (a paired A5/5A
// byte-test, and the original ROM readback corruption pattern itself
// -- ED ED 31 31 instead of F3 ED 56 31, i.e. every word's second
// byte silently overwriting the first) proved DQM has no effect on
// which byte lane actually gets stored on this board, regardless of
// what the RTL drives it to (confirmed independently by forcing
// sd_dqm_nxt to a constant with no change in symptom). A design that
// writes one byte at a time and relies on DQM to mask the untouched
// lane is not viable here. Writing full, already-paired words with
// DQM held at 2'b00 (see sdram.sv's we_word) sidesteps DQM achieving
// the same thing every simple word-oriented SDRAM controller does
// when loading byte-stream ROM data: buffer a pair, write the word.
reg wr_pending;
always @(posedge clk_sys) begin
	if (sdram_init) begin
		wr_pending            <= 1'b0;
		wfifo_rptr            <= '0;
		immediate_verify_want <= 1'b0;
	end
	else if (sdram_wr_ack) begin
		wr_pending <= 1'b0;
		if (ENABLE_SDRAM_DIAG_TRAFFIC && ENABLE_MIDLOAD_DIAG_READS && !latched_ioctl_slave && (latched_ioctl_addr == 27'd0) && !immediate_verify_done)
			immediate_verify_want <= 1'b1;
	end
	else if (!wr_pending && (wfifo_level >= 2)) begin
		wr_pending            <= 1'b1;
		// Even entry (low byte) -- its address is the word address.
		// [28:8] = the 21-bit address field (widened this session, see
		// wfifo's own declaration comment above).
		latched_ioctl_addr    <= {6'b0, wfifo[wfifo_rptr[WFIFO_AW-1:0]][28:8]};
		latched_ioctl_slave   <= (wfifo[wfifo_rptr[WFIFO_AW-1:0]][28:8] >= 21'h40000);
		latched_ioctl_data    <= wfifo[wfifo_rptr[WFIFO_AW-1:0]][7:0];
		// Odd entry (high byte) -- guaranteed to be rptr+1.
		latched_ioctl_data_hi <= wfifo[wfifo_rptr_p1][7:0];
		wfifo_rptr            <= wfifo_rptr + 2'd2;
	end
end

assign sdram_wr_req = bt_write_active | wr_pending;
// The isolated byte-test's write now runs strictly after the entire
// ROM load finishes (triggered on `reset` falling — see bt_wstate
// above), so it can no longer overlap with real ioctl_wr traffic on
// the shared wr channel by construction (the FIFO is drained long
// before reset falls).
//
// ioctl_wait: assert while SDRAM init is pending, and once the FIFO
// reaches half full. NOT per-byte -- the ARM reacts to this in
// software with real latency, so it must be an early warning with
// slack behind it (8 free slots), not a hard stop. Per-byte wait
// (the old `wr_pending | ~sdram_ready`) throttled the stream to the
// software polling rate at best and lost bytes at worst.
assign ioctl_wait   = (wfifo_level >= (1<<(WFIFO_AW-1))) | ~sdram_ready;

// ioctl_wait activity diagnostic (2026-07-12 session, following up on
// the prom_rom-also-dead finding): ioctl_wait is a GLOBAL stall signal
// -- it doesn't distinguish "this byte is headed for SDRAM" from
// "this byte is headed for gfx_rom/prom_rom BRAM". It only gets
// asserted from SDRAM/wfifo backpressure (which can only happen during
// the Master/Slave ROM portion of the stream, addresses before
// ADDR_SOUND_LO), but once asserted it pauses the ENTIRE HPS byte
// stream, including bytes destined for gfx_rom/prom_rom much later.
// The ARM only reacts to this in software with real latency (see the
// wfifo comment above), so a resume-after-stall firmware quirk
// happening early could plausibly corrupt something that only
// manifests once the stream reaches gfx/prom far downstream.
//
// First pass (combined ioctl_wait) came back W=1, exactly 1 rising
// edge -- genuinely ambiguous, since ioctl_wait is OR'd from two
// different causes (wfifo_level half-full, and ~sdram_ready). A single
// assertion is exactly what a brief ~sdram_ready during SDRAM's own
// power-on init sequence would produce -- normal on every boot,
// unrelated to this theory -- and much less consistent with genuine
// wfifo backpressure, which would more plausibly fire more than once
// across a real Master+Slave ROM load if it fired at all. Split into
// two independent sticky bits + counts to attribute the single edge
// to its actual cause.
reg       ever_wfifo_stall, ever_sdram_notready;
reg [7:0] wfifo_stall_rise_count, sdram_notready_rise_count;
reg       wfifo_stall_prev, sdram_notready_prev;
wire      wfifo_stall_now = (wfifo_level >= (1<<(WFIFO_AW-1)));
always @(posedge clk_sys) begin
	if (sdram_init) begin
		ever_wfifo_stall          <= 1'b0;
		ever_sdram_notready       <= 1'b0;
		wfifo_stall_rise_count    <= 8'd0;
		sdram_notready_rise_count <= 8'd0;
		wfifo_stall_prev          <= 1'b0;
		sdram_notready_prev       <= 1'b0;
	end else begin
		wfifo_stall_prev    <= wfifo_stall_now;
		sdram_notready_prev <= ~sdram_ready;
		if (wfifo_stall_now)  ever_wfifo_stall    <= 1'b1;
		if (~sdram_ready)     ever_sdram_notready <= 1'b1;
		if (wfifo_stall_now && !wfifo_stall_prev && (wfifo_stall_rise_count != 8'hFF))
			wfifo_stall_rise_count <= wfifo_stall_rise_count + 8'd1;
		if (~sdram_ready && !sdram_notready_prev && (sdram_notready_rise_count != 8'hFF))
			sdram_notready_rise_count <= sdram_notready_rise_count + 8'd1;
	end
end

//------------------------------------------------------------------
// Diagnostic: isolated single-byte SDRAM write/readback self-test
//
// Completely independent of ROM loading, ROM content, and the CPUs —
// the simplest possible proof that the raw SDRAM datapath can store
// and retrieve one byte correctly. Runs once, immediately after SDRAM
// init completes (before any ioctl download), at a fixed address far
// outside every real ROM region (Master 0x000000-0x03FFFF, Slave
// 0x040000-0x0BFFFF, and -- since this session -- Sound
// 0x0C0000-0x1BFFFF too, now that it's real live SDRAM content and no
// longer a dropped placeholder) so it can never collide with real ROM
// data.
//
// If this fails, the bug is in the raw SDRAM controller/datapath
// itself (FSM timing, DQM, address decode) independent of anything
// ROM-loading or CPU-arbitration related. If it passes, the bug is
// specific to sustained/concurrent access patterns (arbitration under
// load, ROM-sized address ranges, or interaction with the ioctl
// loader) and this test has ruled out the simplest explanation.
//
// BT_ADDR moved this session from 0x100000 to 0x200000: the old value
// fell inside the Sound ROM's own range (0x0C0000-0x1BFFFF) once that
// became real, live content instead of a dropped fill -- would have
// let the byte-test's A5/5A pattern corrupt real sound-CPU ROM bytes
// at its own address (mid-load) and, on real hardware, wherever the
// byte-test's own write landed relative to the sound ROM's actual
// content after load. 0x200000 sits past even GFX/PROM's flat
// addresses (0x1C0000-0x1F7FFF, BRAM-routed, never touches SDRAM) --
// clear of every real consumer of this SDRAM, not just the ROM ranges.
//------------------------------------------------------------------
localparam [24:0] BT_ADDR     = 25'h200000; // 2MB in — far outside every real ROM region (even)
localparam [24:0] BT_ADDR2    = 25'h200001; // adjacent odd address, same 16-bit word
localparam [7:0]  BT_PATTERN  = 8'hA5;
localparam [7:0]  BT_PATTERN2 = 8'h5A;      // distinct value -- a lane swap is visible either way

// PAIRED byte-test: write BT_PATTERN to the even address and
// BT_PATTERN2 to the ODD address immediately after (same 16-bit SDRAM
// word), then read both back. This fingerprints the even/odd
// corruption seen on hardware (scan reads ED ED 31 31 -- odd bytes
// correct, even bytes clobbered by their odd neighbor -- exactly what
// a stuck DQM/byte-lane-select would produce). The original single-
// address test used ONE address for both write and read, so a
// symmetric lane fault (write always hits lane X, read always samples
// lane X) cancels out and reports P regardless -- it could never have
// caught this. Four readback outcomes fingerprint four different
// faults:
//   A5 5A -- lanes fine, this theory is dead, look elsewhere
//   5A 5A -- both ops pinned to one lane (matches the ED ED / 31 31
//            pattern seen on the real scan)
//   5A xx -- write address bit 0 (byte_sel) stuck at 0
//   xx 5A -- write address bit 0 stuck at 1
//
// Write-side only here (single driver of the wr channel mux). The
// read-back half runs inside the rd2-scan always block below, since
// sdram_rd2_req/addr must have exactly one driver — it runs as a
// phase that precedes the big 256KB scan, gated on bt_wr_done.
//
// ONE paired word write now (not two sequential byte writes): matches
// the real ROM path exactly (see the paired-write comment on the
// drain logic above) -- both bytes known up front, single 16-bit
// write, DQM held at 2'b00. An earlier version of this test did two
// separate byte writes with a gap state between them; that's now
// simply how the real ROM writes work too, so there's no reason for
// the byte-test to do anything different.
localparam BT_IDLE=0, BT_WR_WAIT=1, BT_WR_DONE=2;
reg [1:0] bt_wstate;
reg       bt_pass;       // BOTH bytes read back correctly
reg       bt_done;       // write+both reads complete
reg [7:0] bt_readback;   // even address (BT_ADDR)
reg [7:0] bt_readback2;  // odd address  (BT_ADDR2)
reg       bt_reset_prev;

assign bt_write_active = (bt_wstate == BT_WR_WAIT);
assign bt_wr_addr    = BT_ADDR;
assign bt_wr_data    = BT_PATTERN;
assign bt_wr_data_hi = BT_PATTERN2;

always @(posedge clk_sys) begin
	bt_reset_prev <= reset;
	if (sdram_init) begin
		bt_wstate  <= BT_IDLE;
		bt_wr_done <= 1'b0;
	end else begin
		case (bt_wstate)
			// Triggered on `reset` falling (all ROM loading fully done),
			// not on sdram_ready rising (well before ioctl_download even
			// starts). Previously this self-test's write shared the wr
			// channel with the very start of the real HPS ROM stream --
			// the intended safeguard (ioctl_wait includes ~bt_wr_done, so
			// HPS should not send anything until this completes) assumes
			// HPS reacts to ioctl_wait with no buffering/burst-ahead.
			// Hardware evidence said otherwise: the write-side ground-
			// truth debug row showed Master ROM addresses 0-3 never
			// written at all, even after fixing a related but different
			// race (latching ioctl_addr/data at accept-time). Running
			// this strictly after the entire load finishes removes any
			// temporal overlap with the real ROM stream by construction,
			// rather than depending on exact HPS-side wait-signal timing.
			// dl_settled (session-count-independent), not a reset edge:
			// see the DL_SETTLE_CYCLES parameter comment -- hardware
			// showed the single <rom> tag still isn't one download
			// session in practice.
			BT_IDLE:    if (ENABLE_SDRAM_DIAG_TRAFFIC && dl_settled) bt_wstate <= BT_WR_WAIT;
			BT_WR_WAIT: if (sdram_wr_ack) begin bt_wr_done <= 1'b1; bt_wstate <= BT_WR_DONE; end
			default: ; // BT_WR_DONE: stay
		endcase
	end
end

//------------------------------------------------------------------
// Diagnostic: SDRAM write/readback integrity check for Master ROM
//
// Accumulates an 8-bit running sum of every byte actually accepted
// (wr_ack'd) by the SDRAM controller while loading the Master ROM
// (ioctl_index 0x00, 256 KB). Once loading finishes, a dedicated
// scanner (using the new rd2 diagnostic read port, so it never
// contends with the live CPUs) walks the same 256 KB region back out
// of SDRAM and accumulates an identical running sum. If the two sums
// match, the SDRAM write→read pipeline is provably lossless for this
// data; a mismatch proves data corruption in that pipeline itself
// (as opposed to e.g. a wrong assumption about ROM content).
//------------------------------------------------------------------
// NOTE: ioctl_download can toggle between individual <part> files and
// between ROM indices (the Master ROM alone is 4 separate parts under
// index 0x00) — it is NOT a single continuous session that only drops
// once at the very end. Resetting/triggering on its edges mid-load was
// wiping the write checksum and starting the readback scan on a still-
// incomplete ROM. Use the top-level `reset` instead (includes
// ioctl_download and only deasserts once ALL parts of ALL ROMs have
// finished loading) as the "loading is fully done" signal.
// Split by address parity (even/odd byte) to test whether the SDRAM
// controller's DQM low/high-byte write-select logic — the most custom,
// bug-prone part of this design — is the source of any corruption.
reg  [7:0] wr_chk, wr_chk_even, wr_chk_odd;
// Per-64KB-quarter checksums — matches the Master ROM's 4 physical
// files (u58t/u59t/u57t/u56t) so a mismatch localizes to one file.
reg  [7:0] wr_chk_q0, wr_chk_q1, wr_chk_q2, wr_chk_q3;
reg  [7:0] dbg_wr_b0, dbg_wr_b1, dbg_wr_b2, dbg_wr_b3; // actual bytes handed to SDRAM write port at addr 0-3
always @(posedge clk_sys) begin
	if (sdram_init) begin
		wr_chk      <= 8'h00;
		wr_chk_even <= 8'h00;
		wr_chk_odd  <= 8'h00;
		wr_chk_q0   <= 8'h00;
		wr_chk_q1   <= 8'h00;
		wr_chk_q2   <= 8'h00;
		wr_chk_q3   <= 8'h00;
		dbg_wr_b0   <= 8'h00;
		dbg_wr_b1   <= 8'h00;
		dbg_wr_b2   <= 8'h00;
		dbg_wr_b3   <= 8'h00;
	end else if (sdram_wr_ack && !latched_ioctl_slave && !bt_write_active) begin
		// !bt_write_active excludes the isolated byte-test's write
		// (originally at fixed address 0x100000, far outside the Master
		// ROM range; moved to 0x200000 this session -- see BT_ADDR's own
		// comment -- same "quarter-0" address-bit overlap this comment
		// describes still applies at the new address, so this exclusion
		// is still load-bearing, not just historical) from this
		// accumulator. Without it, that write was being double-counted
		// here — ioctl_index defaults to 0 before any real load even
		// starts, and the byte-test address shares the same
		// sdram_wr_addr[17:16]=00 bits as legitimate quarter-0 ROM
		// addresses, so it silently landed in wr_chk/wr_chk_q0 while
		// the readback scanner (bounded to 0x00000-0x3FFFF) never
		// revisits it to match. That guaranteed a permanent false
		// mismatch on the checksum diagnostic, unrelated to any real
		// SDRAM data corruption — confirmed via sim/sor_board_tb.sv,
		// which traced wr_chk_q0 accumulating a contribution from the
		// byte-test's address that rd_chk_q0 (correctly) never sees.
		// Every ack now covers a WHOLE 16-bit word (both bytes at once
		// -- see the paired-write comment above), so accumulate both
		// sdram_wr_data (even/low) and sdram_wr_data_hi (odd/high)
		// per ack, not one byte per ack as before.
		wr_chk      <= wr_chk + sdram_wr_data + sdram_wr_data_hi;
		wr_chk_even <= wr_chk_even + sdram_wr_data;
		wr_chk_odd  <= wr_chk_odd  + sdram_wr_data_hi;
		// Ground-truth spot check on the WRITE side, mirroring
		// dbg_scan_b0..b3 on the read side: latch the actual byte value
		// the core hands to the SDRAM write port at addresses 0-3.
		// Separates "the write path never got/forwarded the real ROM
		// byte" from "the write landed fine but the read-back scan (or
		// SDRAM itself) returns something different later" -- the
		// checksum alone can't distinguish those.
		if (sdram_wr_addr == 25'd0) begin
			dbg_wr_b0 <= sdram_wr_data;
			dbg_wr_b1 <= sdram_wr_data_hi;
		end else if (sdram_wr_addr == 25'd2) begin
			dbg_wr_b2 <= sdram_wr_data;
			dbg_wr_b3 <= sdram_wr_data_hi;
		end
		case (sdram_wr_addr[17:16])
			2'd0: wr_chk_q0 <= wr_chk_q0 + sdram_wr_data + sdram_wr_data_hi;
			2'd1: wr_chk_q1 <= wr_chk_q1 + sdram_wr_data + sdram_wr_data_hi;
			2'd2: wr_chk_q2 <= wr_chk_q2 + sdram_wr_data + sdram_wr_data_hi;
			2'd3: wr_chk_q3 <= wr_chk_q3 + sdram_wr_data + sdram_wr_data_hi;
		endcase
	end
end

reg        rd_scan_active;
reg        rd_scan_done;
reg [17:0] rd_scan_addr;
reg  [7:0] rd_chk, rd_chk_even, rd_chk_odd;
reg  [7:0] rd_chk_q0, rd_chk_q1, rd_chk_q2, rd_chk_q3;
reg        rd2_req_pend;
reg  [7:0] dbg_scan_b0, dbg_scan_b1, dbg_scan_b2, dbg_scan_b3; // first 4 bytes read back from SDRAM addr 0-3

// Saturating count of real rd2-channel completions (sdram_rd2_ack
// pulses). Purely diagnostic: tells us, on the next hardware run,
// whether dbg_scan_b0 -- which came back matching the byte-test's own
// BT_PATTERN instead of the real ROM byte -- reflects a genuinely
// fresh SDRAM transaction for address 0, or something upstream failed
// to issue a real new transaction and it's a stale reuse.
reg  [7:0] dbg_rd2_ack_cnt;
always @(posedge clk_sys) begin
	if (sdram_init) dbg_rd2_ack_cnt <= 8'h00;
	else if (sdram_rd2_ack && dbg_rd2_ack_cnt != 8'hFF)
		dbg_rd2_ack_cnt <= dbg_rd2_ack_cnt + 8'd1;
end

// Decisive time-based test: read address 0 back immediately after the
// Master ROM finishes loading (ioctl_index transitions 0->1, meaning
// the Slave ROM's ~512KB is just about to start -- correct data has
// had almost no time to sit in SDRAM yet), rather than only at the
// very end after everything (both ROMs, ~768KB, likely ~100+ ms) has
// loaded. If this reads correctly but the final dbg_scan_b0 (end of
// everything) doesn't, that's direct proof of decay over the load's
// duration rather than a one-off logic/addressing bug -- accept0/
// accept1 already proved the correct bytes (0xF3/0xED) reach the
// write-acceptance stage, so the remaining open question is whether
// SDRAM retains them over time.
reg  [7:0] dbg_early_b0;
reg [15:0] ioctl_index_prev;

// Sticky level version of "loading is done" (see dl_settled below) --
// removes any ordering dependency on catching a single-cycle pulse
// from a specific state.
reg loading_done;

// Sole driver of sdram_rd2_req/sdram_rd2_addr: the byte-test readback
// (BT_RD_*) runs first as soon as the write half completes, then the
// big 256KB scan follows once the byte test is done.
localparam RD_IDLE=0, RD_BT_REQ=1, RD_BT_WAIT=2, RD_EARLY_WAIT=5, RD_EARLY_ACK=6, RD_SCAN=3, RD_DONE=4, RD_IMMEDIATE_WAIT=7, RD_BT_WAIT2=8, RD_BT_REQ2=9,
           RD_LIVE_IDLE=10, RD_LIVE_REQ=11, RD_LIVE_WAIT=12;
reg [3:0] rd_phase; // widened 3->4 bits for RD_BT_WAIT2/RD_BT_REQ2 (paired byte-test read)

// Live fetch-mismatch counter (docs/sdram_plan.md Section 2): once the
// boot-time rd_scan finishes, keep polling a fixed Master-ROM address
// (alternating both DQ byte lanes) at a slow, negligible-bandwidth
// cadence for the rest of the session, converting "erratic behavior"
// during gameplay into one number. rd2 stays lowest arbiter priority
// (sel_rd2 above), so this has zero impact on CPU fetches.
reg [15:0] live_timer = 16'd0;     // free-running; wraps every 65536 clk_sys cycles (~1.37ms @ 48MHz). Explicit
                                    // initial value (Quartus power-up state), not a reset -- without it sim's
                                    // X-initialized reg never resolves (X+1=X forever); real hardware doesn't
                                    // have this problem (Quartus registers power up to their declared value).
reg        live_lane;              // 0 = addr ...0100 (even), 1 = ...0101 (odd) -- covers both DQ byte lanes
reg        live_ref_lo_set, live_ref_hi_set;
reg  [7:0] live_ref_lo, live_ref_hi; // latched from the FIRST poll of each lane after rd_scan_done
reg  [7:0] live_mm_cnt;            // saturating mismatch count (sticky)
reg  [3:0] live_poll_cnt;          // free-running liveness counter -- must visibly spin on screen
wire       live_tick = (live_timer == 16'hFFFF);
localparam LIVE_ADDR_BASE = 25'h000100;

always @(posedge clk_sys) begin
	live_timer <= live_timer + 16'd1; // free-running, decoupled from rd_phase/reset
end

always @(posedge clk_sys) begin
	ioctl_index_prev <= ioctl_index;
	// dl_settled, not a reset edge -- see DL_SETTLE_CYCLES comment.
	if (dl_settled) loading_done <= 1'b1;
	if (sdram_init) begin
		loading_done   <= 1'b0;
		rd_phase       <= RD_IDLE;
		rd_scan_active <= 1'b0;
		rd_scan_done   <= 1'b0;
		rd_scan_addr   <= 18'd0;
		rd_chk         <= 8'h00;
		rd_chk_even    <= 8'h00;
		rd_chk_odd     <= 8'h00;
		rd_chk_q0      <= 8'h00;
		rd_chk_q1      <= 8'h00;
		rd_chk_q2      <= 8'h00;
		rd_chk_q3      <= 8'h00;
		rd2_req_pend   <= 1'b0;
		sdram_rd2_req  <= 1'b0;
		bt_pass        <= 1'b0;
		bt_done        <= 1'b0;
		dbg_early_b0   <= 8'h00;
		dbg_scan_b0    <= 8'h00;
		dbg_scan_b1    <= 8'h00;
		dbg_scan_b2    <= 8'h00;
		dbg_scan_b3    <= 8'h00;
		immediate_verify_done <= 1'b0;
		dbg_immediate_b0      <= 8'h00;
		live_lane        <= 1'b0;
		live_ref_lo_set  <= 1'b0;
		live_ref_hi_set  <= 1'b0;
		live_ref_lo      <= 8'h00;
		live_ref_hi      <= 8'h00;
		live_mm_cnt      <= 8'h00;
		live_poll_cnt    <= 4'h0;
	end else begin
		case (rd_phase)
			// Highest priority within RD_IDLE: the immediate read-after-
			// write verification (see immediate_verify_want above) --
			// checked ahead of the normal bt_wr_done path since it needs
			// to fire early, during the real Master ROM load, well
			// before bt_wr_done ever becomes true in this design.
			RD_IDLE: if (immediate_verify_want && !immediate_verify_done) begin
				sdram_rd2_addr <= 25'd0;
				sdram_rd2_req  <= 1'b1;
				rd_phase       <= RD_IMMEDIATE_WAIT;
			end else if (bt_wr_done) begin
				sdram_rd2_addr <= BT_ADDR;
				sdram_rd2_req  <= 1'b1;
				rd_phase       <= RD_BT_WAIT;
			end

			RD_IMMEDIATE_WAIT: if (sdram_rd2_ack) begin
				dbg_immediate_b0      <= sdram_rd2_data;
				immediate_verify_done <= 1'b1;
				sdram_rd2_req         <= 1'b0;
				rd_phase              <= RD_IDLE;
			end

			// Drop req for a cycle between the two reads: the arbiter's
			// in_flight fence only releases once the client's own req
			// line drops (see the duplicate-transaction race fix) --
			// changing the address while req stays asserted would get
			// the second read silently ignored.
			RD_BT_WAIT: if (sdram_rd2_ack) begin
				bt_readback   <= sdram_rd2_data;
				sdram_rd2_req <= 1'b0;
				rd_phase      <= RD_BT_REQ2;
			end

			RD_BT_REQ2: begin
				sdram_rd2_addr <= BT_ADDR2;
				sdram_rd2_req  <= 1'b1;
				rd_phase       <= RD_BT_WAIT2;
			end

			RD_BT_WAIT2: if (sdram_rd2_ack) begin
				bt_readback2  <= sdram_rd2_data;
				bt_pass       <= (bt_readback == BT_PATTERN) && (sdram_rd2_data == BT_PATTERN2);
				bt_done       <= 1'b1;
				sdram_rd2_req <= 1'b0;
				rd_phase      <= RD_EARLY_WAIT;
			end

			// Wait here for ioctl_index to transition 0->1 (Master ROM
			// stream finished, Slave ROM's about to start) to steal the
			// rd2 channel for one quick read of address 0. REGRESSION
			// FIX: this stage must never be allowed to permanently block
			// the main scan below it -- confirmed on real hardware that
			// the assumed clean, immediately-adjacent 0->1 transition
			// doesn't reliably happen (ROM loading order across all 5
			// ioctl_index values in the .mra isn't guaranteed to produce
			// one), which silently stalled the entire scan pipeline
			// (rd_scan_done never went true, previously-working
			// diagnostics went dark). Escape hatch: if reset falls (all
			// loading genuinely finished) before the edge was ever seen,
			// give up on the early read and proceed to the real scan
			// anyway, leaving dbg_early_b0 at its reset default.
			RD_EARLY_WAIT: begin
				// Gated by ENABLE_MIDLOAD_DIAG_READS: this is the second
				// of the two mid-download rd2 injections (see parameter
				// comment). When disabled, park here until loading ends
				// and proceed straight to the scan, leaving dbg_early_b0
				// at its reset default.
				if (ENABLE_MIDLOAD_DIAG_READS && (ioctl_index_prev == 16'h00) && (ioctl_index == 16'h01)) begin
					sdram_rd2_addr <= 25'd0;
					sdram_rd2_req  <= 1'b1;
					rd_phase       <= RD_EARLY_ACK;
				end else if (loading_done) begin
					rd_phase <= RD_SCAN;
				end
			end

			RD_EARLY_ACK: if (sdram_rd2_ack) begin
				dbg_early_b0  <= sdram_rd2_data;
				sdram_rd2_req <= 1'b0;
				rd_phase      <= RD_SCAN;
			end else if (loading_done) begin
				// Same escape hatch in case the ack itself never arrives.
				sdram_rd2_req <= 1'b0;
				rd_phase      <= RD_SCAN;
			end

			RD_SCAN: begin
				// Kick off the big scan once ALL ROM loading has finished
				// (loading_done, sticky -- not ioctl_download's edges,
				// which toggle mid-load between parts/indices). Must also
				// gate on !rd_scan_active: unlike the one-shot edge this
				// replaced, loading_done stays high for the rest of time,
				// so without this the scan would re-trigger and reset
				// itself back to address 0 every single cycle for as
				// long as rd_scan_done stays false (i.e. throughout the
				// entire scan), never able to make progress.
				if (loading_done && !rd_scan_active && !rd_scan_done) begin
					rd_scan_active <= 1'b1;
					rd_scan_addr   <= 18'd0;
					rd_chk         <= 8'h00;
					rd_chk_even    <= 8'h00;
					rd_chk_odd     <= 8'h00;
					rd_chk_q0      <= 8'h00;
					rd_chk_q1      <= 8'h00;
					rd_chk_q2      <= 8'h00;
					rd_chk_q3      <= 8'h00;
					rd2_req_pend   <= 1'b0;
					sdram_rd2_req  <= 1'b0;
				end

				if (rd_scan_active) begin
					if (!rd2_req_pend) begin
						sdram_rd2_addr <= {7'b0, rd_scan_addr};
						sdram_rd2_req  <= 1'b1;
						rd2_req_pend   <= 1'b1;
					end else if (sdram_rd2_ack) begin
						rd_chk <= rd_chk + sdram_rd2_data;
						if (sdram_rd2_addr[0]) rd_chk_odd  <= rd_chk_odd  + sdram_rd2_data;
						else                   rd_chk_even <= rd_chk_even + sdram_rd2_data;
						case (sdram_rd2_addr[17:16])
							2'd0: rd_chk_q0 <= rd_chk_q0 + sdram_rd2_data;
							2'd1: rd_chk_q1 <= rd_chk_q1 + sdram_rd2_data;
							2'd2: rd_chk_q2 <= rd_chk_q2 + sdram_rd2_data;
							2'd3: rd_chk_q3 <= rd_chk_q3 + sdram_rd2_data;
						endcase
						// Ground-truth spot check: latch the actual first 4
						// bytes read back from SDRAM at addresses 0-3, so
						// they can be compared directly against a known ROM
						// hex dump (e.g. "F3 ED 56 31...") rather than only
						// having an opaque checksum pass/fail.
						case (rd_scan_addr)
							18'd0: dbg_scan_b0 <= sdram_rd2_data;
							18'd1: dbg_scan_b1 <= sdram_rd2_data;
							18'd2: dbg_scan_b2 <= sdram_rd2_data;
							18'd3: dbg_scan_b3 <= sdram_rd2_data;
						endcase
						sdram_rd2_req <= 1'b0;
						rd2_req_pend  <= 1'b0;
						if (rd_scan_addr == 18'h3FFFF) begin
							rd_scan_active <= 1'b0;
							rd_scan_done   <= 1'b1;
							rd_phase       <= RD_LIVE_IDLE;
						end else begin
							rd_scan_addr <= rd_scan_addr + 18'd1;
						end
					end
				end
			end

			// Live fetch-mismatch counter (docs/sdram_plan.md Section 2):
			// runs forever once the boot-time scan is done. live_tick
			// gates the cadence (~1.37ms @ 48MHz); rd2 is lowest arbiter
			// priority so this never delays a real CPU fetch.
			RD_LIVE_IDLE: if (live_tick) rd_phase <= RD_LIVE_REQ;

			RD_LIVE_REQ: begin
				sdram_rd2_addr <= LIVE_ADDR_BASE | {24'd0, live_lane};
				sdram_rd2_req  <= 1'b1;
				rd_phase       <= RD_LIVE_WAIT;
			end

			RD_LIVE_WAIT: if (sdram_rd2_ack) begin
				sdram_rd2_req <= 1'b0;
				live_poll_cnt <= live_poll_cnt + 4'd1;
				if (!live_lane) begin
					if (!live_ref_lo_set) begin
						live_ref_lo     <= sdram_rd2_data;
						live_ref_lo_set <= 1'b1;
					end else if ((sdram_rd2_data != live_ref_lo) && (live_mm_cnt != 8'hFF)) begin
						live_mm_cnt <= live_mm_cnt + 8'd1;
					end
				end else begin
					if (!live_ref_hi_set) begin
						live_ref_hi     <= sdram_rd2_data;
						live_ref_hi_set <= 1'b1;
					end else if ((sdram_rd2_data != live_ref_hi) && (live_mm_cnt != 8'hFF)) begin
						live_mm_cnt <= live_mm_cnt + 8'd1;
					end
				end
				live_lane <= ~live_lane;
				rd_phase  <= RD_LIVE_IDLE;
			end

			default: ; // RD_DONE: unused, RD_SCAN stays active/idle forever
		endcase
	end
end

wire dbg_chk_match_q0 = rd_scan_done && (wr_chk_q0 == rd_chk_q0);
wire dbg_chk_match_q1 = rd_scan_done && (wr_chk_q1 == rd_chk_q1);
wire dbg_chk_match_q2 = rd_scan_done && (wr_chk_q2 == rd_chk_q2);
wire dbg_chk_match_q3 = rd_scan_done && (wr_chk_q3 == rd_chk_q3);

wire dbg_chk_match_even = rd_scan_done && (wr_chk_even == rd_chk_even);
wire dbg_chk_match_odd  = rd_scan_done && (wr_chk_odd  == rd_chk_odd);

wire dbg_chk_match = rd_scan_done && (wr_chk == rd_chk);

//------------------------------------------------------------------
// Graphics / palette ROMs (still on BRAM — small, fit M10K budget)
//   0x1C0000-0x1D7FFF: GFX tile ROM    (96 KB)
//   0x1D8000-0x1F7FFF: palette PROM  (128 KB sparse)
//------------------------------------------------------------------
// NOTE: these upper bounds must be sized wide enough to hold the full
// value -- 16'h17FFF and 16'h1FFFF are 16-BIT-sized literals, which can
// only represent 0-0xFFFF; the literal value gets silently truncated
// to its low 16 bits (0x17FFF -> 0x7FFF, 0x1FFFF -> 0xFFFF), shrinking
// gfx_rom from 98304 to 32768 entries and prom_rom from 131072 to
// 65536. That exactly matches a real symptom: GFX bitplane 0 (address
// range 0x0000-0x7FFF) stayed inside the truncated bound and read
// real data; bitplanes 1 and 2 (0x8000-0x17FFF) were out of bounds
// and always read back X in sim -- and Quartus would synthesize BRAMs
// sized to the same truncated depth, so this very likely also explains
// the solid-color/missing-tile-graphics symptom seen on real hardware.
// 2026-07-12 session (docs/planning.md Step 2): gfx_rom's actual
// content only needs 98304 entries (0x17FFF), but that's not a power
// of 2 -- unlike prom_rom's clean 131072 (0x1FFFF), which needs no
// special handling. Confirmed via the fitted design's RAM Summary
// report that this is a REAL structural difference, not just a
// cosmetic one: Quartus's altsyncram inference for gfx_rom required
// extra auto-generated address-decode/mux logic (decode_pma + mux_9hb)
// that prom_rom's implementation doesn't have at all, to route between
// 96 separate physical M10K block instances for the odd-sized array
// (prom_rom uses a simpler structure across 128 blocks). This is
// exactly the kind of Quartus-synthesized glue logic that plain RTL
// behavioral simulation can never exercise -- it doesn't exist until
// the fitter builds it -- matching the observed pattern precisely:
// the isolated sim testbench (sim/sor_video_tb.sv) always "worked",
// while real hardware never once returned real gfx_rom content
// (confirmed via a live fingerprint diagnostic reading gfx_rom[0x8003]
// as 0x00 where the ROM file's real content is 0x FF, and a sticky bit
// confirming zero gfx_rom reads ever returned nonzero, on any screen).
// Padding the depth to a clean power of 2 (matching prom_rom's own
// declaration exactly) gives Quartus the same simple case it already
// handles correctly for prom_rom, eliminating the need for that extra
// decode/mux logic entirely. Wastes 32KB of M10K (this project has
// budget: 364/553 M10K blocks used per the last fit report, 66%) in
// exchange for using the same, already-proven-correct BRAM structure
// for both arrays. The extra address range (0x18000-0x1FFFF) is never
// written by ioctl loading and never read by sor_video's tile-fetch
// address computation, so this is purely a depth-padding change with
// no other functional effect.
// ramstyle="no_rw_check" REVERTED (2026-07-12 session): it was added
// in the exact same commit that restored the prom_rom fingerprint
// check and first showed prom_rom dead -- meaning no build ever ran
// with the check present and this attribute absent, so it couldn't be
// ruled in or out. The backpressure/ioctl_wait theory tested since has
// been ruled out, so this is the one remaining untested change from
// tonight. Reverting to isolate it cleanly: if prom_rom comes back
// correct with this gone, the attribute is the cause; if it's still
// dead, the bug predates even this and the reference-core research
// this session's ramstyle change was based on doesn't explain it.
reg [7:0] gfx_rom  [0:17'h1FFFF];
reg [7:0] prom_rom [0:17'h1FFFF];

// Uses ioctl_addr_d1 (one-cycle-delayed, declared above near
// ioctl_wr_rom) rather than the live ioctl_addr -- same hps_io timing
// skew as the SDRAM ROM writes: ioctl_addr has already advanced to the
// next byte's address by the time ioctl_wr reads 1 externally. Routed
// by address range now (single flat download session), not ioctl_index.
// gfx_rom write+readback self-test (2026-07-12 session): every check
// this session so far has only ever tested READS of ioctl-loaded ROM
// content -- gfx_rom's WRITE path (distinct physical BRAM port from
// prom_rom's, despite the shared always block below) has never been
// independently verified on real hardware, decoupled from the ioctl
// download session's own timing/concurrent-traffic characteristics.
// This writes a known sentinel to SELFTEST_ADDR (17'h1FFFF -- the top
// of gfx_rom's padded address space, address bits [16:15]=11, a plane
// value sor_video's fetch FSM can never legitimately produce since
// fetch_tile_code's plane field only ever takes 00/01/10, so this
// address is guaranteed both (a) never ioctl-loaded with real content
// and (b) never organically requested by the video-side read port,
// eliminating any collision risk) shortly after loading_done, then
// reads it back through the SAME existing 2 physical ports (write
// port A gets one more mux'd write-enable source below; read port B
// briefly has its address overridden -- see the gfx_addr_vid mux
// below). No third accessor is added on either port, so this can't
// repeat the earlier Quartus BRAM-inference hang.
localparam [16:0] GFX_SELFTEST_ADDR    = 17'h1FFFF;
localparam [7:0]  GFX_SELFTEST_PATTERN = 8'hA5;
localparam [2:0]  ST_IDLE=0, ST_WAIT=1, ST_WRITE=2, ST_SETTLE=3, ST_READ_ADDR=4, ST_READ_WAIT=5, ST_READ_CAPTURE=6, ST_DONE=7;
reg [2:0]  selftest_state;
reg [7:0]  selftest_wait_cnt;
reg        selftest_wr;
reg        selftest_addr_override;
reg [7:0]  gfx_selftest_readback;
reg        gfx_selftest_done, gfx_selftest_pass;
reg [16:0] override_addr_r; // drives gfx_addr_vid while selftest_addr_override is set

// prom_rom self-test (2026-07-13 session, mirrors the gfx pattern
// above): the opportunistic prom fingerprint is scroll-DEPENDENT --
// the video fetch only presents address 0x8000 when the game's
// scroll/gfxbank map a visible tile row there, and on low-scroll
// screens every fetched address sits in the PROM's empty u70 region,
// which reads 00 from a perfectly loaded prom_rom. So prom_fp=00 /
// ever_prom_nonzero=0 cannot distinguish "prom_rom dead" from
// "prom_rom fine but the scroll registers never move". This forces
// the question, scroll-independently, right after loading_done:
//   1. read prom_rom[0x8000] via a brief address override on the
//      EXISTING video read port (expect 0x69, u69's first byte) --
//      proves ioctl content + read path;
//   2. write 0xA5 to 0x1FFFF (u89 empty-socket fill byte, loaded as
//      00, tile row 0xFF -- never fetched by any real screen) and
//      read it back -- proves the BRAM ports themselves.
// Same two-physical-ports discipline as the gfx self-test: one more
// mux source on each existing port, no third accessor.
localparam [16:0] PROM_ST_SENTINEL_ADDR = 17'h1FFFF;
localparam [7:0]  PROM_ST_PATTERN       = 8'hA5;
localparam [3:0]  PST_IDLE=0, PST_WAIT=1, PST_RD69_ADDR=2, PST_RD69_WAIT=3, PST_RD69_CAP=4,
                  PST_WRITE=5, PST_SETTLE=6, PST_RD_ADDR=7, PST_RD_WAIT=8, PST_RD_CAP=9, PST_DONE=10;
reg [3:0]  prom_st_state;
reg [7:0]  prom_st_wait_cnt;
reg        prom_st_wr;
reg        prom_st_override;
reg [16:0] prom_override_addr;
reg [7:0]  prom_fp_forced; // prom_rom[0x8000] via forced read, expect 0x69
reg        prom_st_done, prom_st_pass;

// Second read port for sor_video's background-tilemap tile fetch
// (time-multiplexed across the 3 gfx_rom bitplanes by sor_video).
// gfx_addr_vid is muxed, not a straight wire, so the self-test above
// can briefly borrow this same physical read port -- still exactly one
// read port, one address bus, into gfx_rom. Declared here (ahead of the
// self-test always block that reads gfx_data_vid) to avoid a forward-
// reference compile error.
wire [16:0] gfx_addr_from_video, prom_addr_from_video;
wire [16:0] gfx_addr_vid  = selftest_addr_override ? override_addr_r : gfx_addr_from_video;
wire [16:0] prom_addr_vid = prom_st_override ? prom_override_addr : prom_addr_from_video;
reg  [7:0]  gfx_data_vid, prom_data_vid;
always @(posedge clk_sys) begin
	gfx_data_vid  <= gfx_rom[gfx_addr_vid];
	prom_data_vid <= prom_rom[prom_addr_vid];
end

// gfx_rom write+readback self-test: writes a known sentinel to
// SELFTEST_ADDR (17'h1FFFF -- top of gfx_rom's padded address space, a
// plane value sor_video's fetch FSM can never legitimately produce, so
// guaranteed collision-free) shortly after loading_done, then reads it
// back through the SAME 2 physical ports (write port A gets one more
// mux'd write-enable source below; read port B briefly has its address
// overridden). Passed on hardware (D P A5) -- proves gfx_rom's BRAM
// interface itself works, decoupled from the ioctl download session.
// The follow-up checksum-scan/write-hit-counter/range-transit
// diagnostics that chased *why* real ioctl-loaded content still never
// lands have all been removed after ruling out several theories
// (giant sound-ROM fill, gap position) without resolving it -- see
// docs/SESSION_2026-07-12_SUNDAY_EVENING.md. Freed up for the
// ramstyle="no_rw_check" experiment and the restored prom_rom
// fingerprint check below.
always @(posedge clk_sys) begin
	selftest_wr            <= 1'b0;
	selftest_addr_override <= 1'b0;
	if (sdram_init) begin
		selftest_state    <= ST_IDLE;
		gfx_selftest_done <= 1'b0;
	end else case (selftest_state)
		ST_IDLE: if (loading_done) begin
			selftest_wait_cnt <= 8'd8;
			selftest_state    <= ST_WAIT;
		end
		// A few settle cycles after loading_done so this never races the
		// final real ioctl write landing.
		ST_WAIT: begin
			if (selftest_wait_cnt == 8'd0) selftest_state <= ST_WRITE;
			else selftest_wait_cnt <= selftest_wait_cnt - 8'd1;
		end
		ST_WRITE: begin
			selftest_wr    <= 1'b1;
			selftest_state <= ST_SETTLE;
		end
		ST_SETTLE: selftest_state <= ST_READ_ADDR; // let the write land
		ST_READ_ADDR: begin
			override_addr_r         <= GFX_SELFTEST_ADDR;
			selftest_addr_override <= 1'b1;
			selftest_state          <= ST_READ_WAIT;
		end
		ST_READ_WAIT: begin
			override_addr_r         <= GFX_SELFTEST_ADDR;
			selftest_addr_override <= 1'b1; // hold address stable one more cycle for the BRAM's registered output
			selftest_state          <= ST_READ_CAPTURE;
		end
		ST_READ_CAPTURE: begin
			gfx_selftest_readback <= gfx_data_vid;
			gfx_selftest_pass     <= (gfx_data_vid == GFX_SELFTEST_PATTERN);
			gfx_selftest_done     <= 1'b1;
			selftest_state         <= ST_DONE;
		end
		ST_DONE: ; // stays here forever
	endcase
end

// prom_rom self-test FSM (see the PROM_ST_* comment above the reg
// declarations): forced read of prom_rom[0x8000] (expect 0x69,
// scroll-independent), then a write+readback sentinel at 0x1FFFF.
//   prom_rom[0x8000] should read 0x69 (03-22102-01.u69's first byte)
localparam [16:0] PROM_FP_ADDR = 17'h8000;
always @(posedge clk_sys) begin
	prom_st_wr       <= 1'b0;
	prom_st_override <= 1'b0;
	if (sdram_init) begin
		prom_st_state <= PST_IDLE;
		prom_st_done  <= 1'b0;
	end else case (prom_st_state)
		PST_IDLE: if (loading_done) begin
			prom_st_wait_cnt <= 8'd8;
			prom_st_state    <= PST_WAIT;
		end
		PST_WAIT: begin
			if (prom_st_wait_cnt == 8'd0) prom_st_state <= PST_RD69_ADDR;
			else prom_st_wait_cnt <= prom_st_wait_cnt - 8'd1;
		end
		// Forced read #1: ioctl-loaded content at 0x8000 (expect 69)
		PST_RD69_ADDR: begin
			prom_override_addr <= PROM_FP_ADDR;
			prom_st_override   <= 1'b1;
			prom_st_state      <= PST_RD69_WAIT;
		end
		PST_RD69_WAIT: begin
			prom_override_addr <= PROM_FP_ADDR;
			prom_st_override   <= 1'b1; // hold for the registered output
			prom_st_state      <= PST_RD69_CAP;
		end
		PST_RD69_CAP: begin
			prom_fp_forced <= prom_data_vid;
			prom_st_state  <= PST_WRITE;
		end
		// Write+readback sentinel: proves the ports independent of ioctl
		PST_WRITE: begin
			prom_st_wr    <= 1'b1;
			prom_st_state <= PST_SETTLE;
		end
		PST_SETTLE: prom_st_state <= PST_RD_ADDR;
		PST_RD_ADDR: begin
			prom_override_addr <= PROM_ST_SENTINEL_ADDR;
			prom_st_override   <= 1'b1;
			prom_st_state      <= PST_RD_WAIT;
		end
		PST_RD_WAIT: begin
			prom_override_addr <= PROM_ST_SENTINEL_ADDR;
			prom_st_override   <= 1'b1;
			prom_st_state      <= PST_RD_CAP;
		end
		PST_RD_CAP: begin
			prom_st_pass  <= (prom_data_vid == PROM_ST_PATTERN);
			prom_st_done  <= 1'b1;
			prom_st_state <= PST_DONE;
		end
		PST_DONE: ; // stays here forever
	endcase
end

// Download-stream telemetry: closes the 0x0C0000-0x1F8000 diagnostic
// blind spot. SDRAM-bound bytes end at ADDR_SOUND_LO (0x0C0000) and
// every existing download check (SDRAM checksums, accept0/1/2 latches)
// watches only that low region -- nothing has ever observed whether
// the stream actually REACHES the gfx/prom range (0x1C0000+) on a
// given boot. The write-hit counter that previously read 0x00000 is
// plain fabric logic upstream of any BRAM placement, so if it is
// honest the "dead BRAM" mystery is a dead STREAM: the ARM/HPS side
// never delivering (or the core never accepting) bytes in that range
// on bad boots. These make that directly visible, per boot:
//
//  - dbg_max_ioctl_addr: running max of ioctl_addr_d1[26:12] over all
//    accepted index-0 write strobes (4KB resolution). Expected 0x1F7
//    (flat image ends at 0x1F8000). Anything less = stream truncated.
//  - gfx/prom_wr_hits: 20-bit counts of the EXACT write conditions
//    used by the BRAM write block above. Displayed as [19:8], i.e.
//    256-byte units: expected 0x180 (96KB gfx) and 0x200 (128KB prom).
//  - dbg_dl_index_last: ioctl_index latched at each download-session
//    START (rising edge), so a session arriving with an unexpected
//    index is visible. 0xFF = no session seen yet.
//  - dbg_dl_end_addr: ioctl_addr_d1[26:12] latched at each download
//    END (falling edge). If the last session is the ROM (index 0)
//    expect 0x1F8; if the DIP session (index 254) lands last expect
//    0x000 with dbg_dl_index_last=0xFE.
reg [14:0] dbg_max_ioctl_addr;
reg [19:0] gfx_wr_hits, prom_wr_hits;
reg [7:0]  dbg_dl_index_last;
reg [14:0] dbg_dl_end_addr;
always @(posedge clk_sys) begin
	if (sdram_init) begin
		dbg_max_ioctl_addr <= 15'd0;
		gfx_wr_hits        <= 20'd0;
		prom_wr_hits       <= 20'd0;
		dbg_dl_index_last  <= 8'hFF;
		dbg_dl_end_addr    <= 15'd0;
	end else begin
		if (ioctl_wr && ioctl_download && (ioctl_index[7:0] == 8'h00)) begin
			if (ioctl_addr_d1[26:12] > dbg_max_ioctl_addr)
				dbg_max_ioctl_addr <= ioctl_addr_d1[26:12];
			if ((ioctl_addr_d1 >= ADDR_GFX_LO) && (ioctl_addr_d1 < ADDR_PROM_LO) &&
			    (gfx_wr_hits != 20'hFFFFF))
				gfx_wr_hits <= gfx_wr_hits + 20'd1;
			if ((ioctl_addr_d1 >= ADDR_PROM_LO) && (ioctl_addr_d1 < ADDR_PROM_HI) &&
			    (prom_wr_hits != 20'hFFFFF))
				prom_wr_hits <= prom_wr_hits + 20'd1;
		end
		if (ioctl_download && !ioctl_download_r) dbg_dl_index_last <= ioctl_index[7:0];
		if (!ioctl_download && ioctl_download_r) dbg_dl_end_addr   <= ioctl_addr_d1[26:12];
	end
end

always @(posedge clk_sys) begin
	// Same ioctl_index==0 qualifier as the SDRAM ROM write path: other
	// index values (e.g. 254 = DIP switches) share the ioctl bus and
	// must never be treated as ROM content.
	if (ioctl_wr && ioctl_download && (ioctl_index[7:0] == 8'h00)) begin
		if ((ioctl_addr_d1 >= ADDR_GFX_LO) && (ioctl_addr_d1 < ADDR_PROM_LO))
			gfx_rom[ioctl_addr_d1 - ADDR_GFX_LO] <= ioctl_data;
		else if ((ioctl_addr_d1 >= ADDR_PROM_LO) && (ioctl_addr_d1 < ADDR_PROM_HI))
			prom_rom[ioctl_addr_d1 - ADDR_PROM_LO] <= ioctl_data;
	end else if (selftest_wr) begin
		gfx_rom[GFX_SELFTEST_ADDR] <= GFX_SELFTEST_PATTERN;
	end else if (prom_st_wr) begin
		// MUST stay inside this else-if chain, exactly like selftest_wr
		// above. An earlier revision put this write in a separate
		// trailing `if` -- functionally identical (the pulses can never
		// coincide: both FSMs gate on loading_done, and prom's write
		// state is 3+ cycles later than gfx's), but Quartus can't prove
		// the two write statements exclusive, infers a SECOND write
		// port on prom_rom, and with the read port that's a third
		// accessor: BRAM inference fails and quartus_map melts down
		// trying to build 128KB from logic cells (10+GB, stuck --
		// the same failure signature as the original third-accessor
		// incident). One write statement per array per chain, always.
		// (prom_st_wr never coincides with selftest_wr either -- see
		// timing note above -- so priority order here is moot.)
		prom_rom[PROM_ST_SENTINEL_ADDR] <= PROM_ST_PATTERN;
	end
end

//------------------------------------------------------------------
// Video RAM — 128 KB dual-port (Master+Slave VRAM I/O ports write/read
// through the sequencer below, video reads through port B)
//------------------------------------------------------------------
reg  [16:0] vram_addr_cpu;
reg   [7:0] vram_din_cpu;
reg         vram_we_cpu;
wire  [7:0] vram_dout_cpu;
wire [16:0] vram_addr_vid;
wire  [7:0] vram_dout_vid;

sor_dpram #(.ADDR_WIDTH(17), .DATA_WIDTH(8)) vram
(
	.clk(clk_sys),
	.addr_a(vram_addr_cpu), .din_a(vram_din_cpu), .we_a(vram_we_cpu), .dout_a(vram_dout_cpu),
	.addr_b(vram_addr_vid), .din_b(8'd0),          .we_b(1'b0),        .dout_b(vram_dout_vid)
);

//------------------------------------------------------------------
// VRAM I/O port sequencer — arbitrates the Master's and Slave's
// sor_vram_port elementary op streams (vp_req/vp_rd/vp_trans/vp_addr/
// vp_data) against the single CPU-side VRAM BRAM port A. Fixed
// priority: Slave first (it is the primary VRAM/blit user), then
// Master (used far less often -- boot handshake mailbox only).
//
// Transparent writes (vp_trans, Slave-only in MAME) implement the
// exact leland_v.cpp vram_port_w merge:
//   if (!(data & 0xf0)) data |= old & 0xf0;
//   if (!(data & 0x0f)) data |= old & 0x0f;
// by inserting a read phase before the write phase.
//------------------------------------------------------------------
wire        vp_req_m, vp_rd_m, vp_trans_m;
wire [15:0] vp_addr_m;
wire  [7:0] vp_data_m;
reg         vp_pop_m;
reg   [7:0] vp_rdata_m;

wire        vp_req_s, vp_rd_s, vp_trans_s;
wire [15:0] vp_addr_s;
wire  [7:0] vp_data_s;
reg         vp_pop_s;
reg   [7:0] vp_rdata_s;

localparam SEQ_IDLE = 3'd0, SEQ_ADDR = 3'd1, SEQ_POP = 3'd2,
           SEQ_TRD   = 3'd3, SEQ_TPOP = 3'd4, SEQ_TWR  = 3'd5, SEQ_TWPOP = 3'd6;
reg [2:0] seq_state;
reg       cur_side;   // 0 = master, 1 = slave
reg       cur_rd;
reg [15:0] cur_addr;
reg  [7:0] cur_data;

always @(posedge clk_sys) begin
	vp_pop_m <= 1'b0;
	vp_pop_s <= 1'b0;
	if (reset) begin
		seq_state   <= SEQ_IDLE;
		vram_we_cpu <= 1'b0;
	end else begin
		case (seq_state)
			// The !vp_pop_* guards are load-bearing. vp_pop_m/vp_pop_s are
			// REGISTERED 1-cycle pulses, and sor_vram_port clears its q0_v on
			// the same edge that consumes the pulse -- so during the cycle a
			// pop is asserted, seq_state is already back here while vp_req_*
			// still reads the PRE-pop value. Without these guards the head op
			// is re-latched and executed a second time, every single time.
			// Confirmed empirically in a board-TB mailbox trace (2026-07-16):
			// every op appeared exactly twice, same addr/data, 200/200
			// alternating. It went unnoticed because the duplicate is
			// idempotent (same byte, same address), but it doubled the Slave's
			// occupancy of this sequencer -- squeezing the Master, which is
			// held in /WAIT via mvport_stall whenever it can't be serviced --
			// and left a window where q0_v clears one cycle before the
			// duplicate's own pop, so a newly committed op could be popped
			// without being executed.
			SEQ_IDLE: begin
				vram_we_cpu <= 1'b0;
				if (vp_req_s && !vp_pop_s) begin
					cur_side <= 1'b1;
					cur_rd   <= vp_rd_s;
					cur_addr <= vp_addr_s;
					cur_data <= vp_data_s;
					if (vp_trans_s && !vp_rd_s) begin
						vram_addr_cpu <= {1'b0, vp_addr_s};
						vram_we_cpu   <= 1'b0;
						seq_state     <= SEQ_TRD;
					end else begin
						vram_addr_cpu <= {1'b0, vp_addr_s};
						vram_din_cpu  <= vp_data_s;
						vram_we_cpu   <= ~vp_rd_s;
						seq_state     <= SEQ_ADDR;
					end
				end else if (vp_req_m && !vp_pop_m) begin
					cur_side <= 1'b0;
					cur_rd   <= vp_rd_m;
					cur_addr <= vp_addr_m;
					cur_data <= vp_data_m;
					vram_addr_cpu <= {1'b0, vp_addr_m};
					vram_din_cpu  <= vp_data_m;
					vram_we_cpu   <= ~vp_rd_m;
					seq_state     <= SEQ_ADDR;
				end
			end

			// Plain read/write: address held for one cycle, BRAM's
			// registered output/write completes on the next edge.
			SEQ_ADDR: begin
				vram_we_cpu <= 1'b0;
				seq_state   <= SEQ_POP;
			end
			SEQ_POP: begin
				if (cur_rd) begin
					if (cur_side) vp_rdata_s <= vram_dout_cpu;
					else          vp_rdata_m <= vram_dout_cpu;
				end
				if (cur_side) vp_pop_s <= 1'b1;
				else          vp_pop_m <= 1'b1;
				seq_state <= SEQ_IDLE;
			end

			// Transparent write: read old byte first, merge zero
			// nibbles from the old VRAM byte, then write the result.
			SEQ_TRD: seq_state <= SEQ_TPOP;
			SEQ_TPOP: begin
				vram_addr_cpu <= {1'b0, cur_addr};
				vram_din_cpu  <= { (cur_data[7:4] == 4'h0) ? vram_dout_cpu[7:4] : cur_data[7:4],
				                    (cur_data[3:0] == 4'h0) ? vram_dout_cpu[3:0] : cur_data[3:0] };
				vram_we_cpu   <= 1'b1;
				seq_state     <= SEQ_TWR;
			end
			SEQ_TWR: begin
				vram_we_cpu <= 1'b0;
				seq_state   <= SEQ_TWPOP;
			end
			SEQ_TWPOP: begin
				if (cur_side) vp_pop_s <= 1'b1;
				else          vp_pop_m <= 1'b1;
				seq_state <= SEQ_IDLE;
			end

			default: seq_state <= SEQ_IDLE;
		endcase
	end
end

//------------------------------------------------------------------
// Work RAM — 4 KB, PRIVATE per CPU (Master and Slave each get their
// own, never shared). Validated against MAME leland.cpp:
// master_common_map_program has `map(0xe000, 0xefff).ram().share
// (m_mainram)`, but slave_large_map_program has `map(0xe000, 0xefff)
// .ram()` with NO .share() -- the Slave's 0xE000-0xEFFF is a
// completely separate RAM device from the Master's. The two CPUs only
// ever communicate through shared VRAM (the mailbox at e.g. 0xEF06 in
// VRAM address space) and the SLAVEHALT/GIN1 poll, never through WRAM.
// A single shared dual-port WRAM instance here (as this used to be)
// let the two CPUs silently alias and corrupt each other's private
// variables at the same offset -- found via a genuine "value changed
// with no write logged" WRAM read/write trace on Master address
// 0xE039 (the MCONT-shadow byte), which flipped bit0 (Slave reset)
// off with no Master instruction ever writing it.
//------------------------------------------------------------------
wire [11:0] wram_addr_m, wram_addr_s;
wire  [7:0] wram_din_m,  wram_din_s;
wire        wram_we_m,   wram_we_s;
wire  [7:0] wram_dout_m, wram_dout_s;

// Port B is otherwise unused -- repurposed as a read-only debug-scan
// channel for the freeze investigation: once the stall detector (below)
// latches, spend a few cycles reading Master WRAM $E712/$E715/$E716 --
// the suspected list-scan loop's flag byte ($E712) and 16-bit list
// pointer ($E715-E716, the value under direct suspicion: a bad/stale
// pointer here would explain an unbounded scan). Skips the $E713-E714
// rate-limiter counter (lower diagnostic value, row-2 space is tight).
// Independent of the CPU's own port-A traffic, so this can't perturb
// the hang itself.
reg  [11:0] wram_dbg_addr_tbl [0:2];
initial begin
	wram_dbg_addr_tbl[0] = 12'h712;
	wram_dbg_addr_tbl[1] = 12'h715;
	wram_dbg_addr_tbl[2] = 12'h716;
end
reg   [1:0] wram_dump_idx;
wire [11:0] wram_dbg_addr_b = wram_dbg_addr_tbl[wram_dump_idx];
wire  [7:0] wram_dbg_dout_b;

sor_dpram #(.ADDR_WIDTH(12), .DATA_WIDTH(8)) wram_m
(
	.clk(clk_sys),
	.addr_a(wram_addr_m), .din_a(wram_din_m), .we_a(wram_we_m), .dout_a(wram_dout_m),
	.addr_b(wram_dbg_addr_b), .din_b(8'd0), .we_b(1'b0), .dout_b(wram_dbg_dout_b)
);

sor_dpram #(.ADDR_WIDTH(12), .DATA_WIDTH(8)) wram_s
(
	.clk(clk_sys),
	.addr_a(wram_addr_s), .din_a(wram_din_s), .we_a(wram_we_s), .dout_a(wram_dout_s),
	.addr_b(12'd0), .din_b(8'd0), .we_b(1'b0), .dout_b()
);

//------------------------------------------------------------------
// Battery-backed RAM — 16 KB, Master-private (0xA000-0xDFFF, selected
// only when bank_reg==1; see sor_master.sv header comment). Volatile
// here (no cross-session persistence) -- real hardware boots from a
// genuinely blank battery RAM on a brand-new PCB too, and the game's
// own recovery path (write the magic checksum + defaults, matching
// masterdump.asm's $5AA5/$A55B signature check at 0xA000/0xDFFE) is
// what's expected to populate it on first boot.
//------------------------------------------------------------------
wire [13:0] battram_addr_m;
wire  [7:0] battram_din_m;
wire        battram_we_m;
wire  [7:0] battram_dout_m;

sor_dpram #(.ADDR_WIDTH(14), .DATA_WIDTH(8)) battram_m
(
	.clk(clk_sys),
	.addr_a(battram_addr_m), .din_a(battram_din_m), .we_a(battram_we_m), .dout_a(battram_dout_m),
	.addr_b(14'd0), .din_b(8'd0), .we_b(1'b0), .dout_b()
);

//------------------------------------------------------------------
// EEPROM (93C46, 64 x 16-bit) -- real hardware wires DI/CLK/CS via
// /MCONT bits 4/5/6 and DO via GIN3 bit0 (see sor_master.sv). This was
// previously hardwired to a constant 1 (GIN3 DO stub), which reads
// back as an all-ones EEPROM to any real checksum/signature check.
//------------------------------------------------------------------
wire eeprom_di, eeprom_clk, eeprom_cs, eeprom_do;

sor_eeprom_93c46 eeprom
(
	.clk_sys(clk_sys),
	.reset(reset),
	.cs(eeprom_cs),
	.clk_in(eeprom_clk),
	.di(eeprom_di),
	.do_out(eeprom_do)
);

//------------------------------------------------------------------
// Color RAM — 1 KB dual-port (Master writes, video reads)
//------------------------------------------------------------------
wire  [9:0] cram_addr_cpu;
wire  [7:0] cram_din_cpu;
wire        cram_we_cpu;
wire  [9:0] cram_addr_vid;
wire  [7:0] cram_dout_vid;

sor_dpram #(.ADDR_WIDTH(10), .DATA_WIDTH(8)) cram
(
	.clk(clk_sys),
	.addr_a(cram_addr_cpu), .din_a(cram_din_cpu), .we_a(cram_we_cpu), .dout_a(),
	.addr_b(cram_addr_vid), .din_b(8'd0),          .we_b(1'b0),        .dout_b(cram_dout_vid)
);

//------------------------------------------------------------------
// Inter-CPU signals
//------------------------------------------------------------------
wire        slave_reset_n;
wire        slave_nmi_n;
wire        slave_int_req;
wire  [7:0] raster_line;
wire        slave_halt_n;

//------------------------------------------------------------------
// Master Z80 — SDRAM ROM stall logic
//
// master_rom_req (level): high while master CPU is in a ROM read cycle
// master_ack_hold:        set when SDRAM ack arrives; cleared when CPU
//                         advances (rom_req drops)
// sdram_rd0_req:          one-shot to SDRAM (high only before ack_hold set)
// master_rom_stall:       stalls master Z80 via rom_stall port
//------------------------------------------------------------------
wire        master_rom_req;          // from sor_master
wire [17:0] master_rom_addr_w;       // from sor_master (flat 256 KB offset)
reg  [7:0]  master_rom_data_r;       // latched SDRAM byte

reg  master_ack_hold;
always @(posedge clk_sys) begin
	if (!master_rom_req)
		master_ack_hold <= 1'b0;
	else if (sdram_rd0_ack)
		master_ack_hold <= 1'b1;
end

// Explicit sdram_ready gate on the request itself (not just relying on
// the controller's own internal mode==MODE_NORMAL check inside its access
// manager) -- removes any dependence on that internal FSM behavior for
// correctness here, and guarantees this channel never asserts a request
// during the init window under any timing corner case.
assign sdram_rd0_req  = master_rom_req & ~master_ack_hold & sdram_ready;
assign sdram_rd0_addr = {7'b0, master_rom_addr_w};   // 7+18=25 bits, base=0x000000
// Explicit ~sdram_ready term here too (matching e.g. gnz's rom_stall
// convention), even though it's currently redundant with the request
// gate above (no ack can arrive pre-ready, so stall already stays
// high) -- removes any dependence on that chain of reasoning holding,
// belt-and-suspenders against future changes to the ack/request logic.
wire   master_rom_stall = master_rom_req & (~master_ack_hold | ~sdram_ready);

always @(posedge clk_sys)
	if (sdram_rd0_ack) master_rom_data_r <= sdram_rd0_data;

//------------------------------------------------------------------
// Master Z80
//------------------------------------------------------------------
wire [15:0] vid_addr_m;
wire        vid_addr_wr_m;
wire [15:0] scroll_x_m, scroll_y_m;
wire  [7:0] gfxbank_m;

//------------------------------------------------------------------
// Debug signals from Master CPU (must precede the sor_master
// instantiation below, which connects them as output ports — some
// SystemVerilog tools require declare-before-use even for port
// connections, which Quartus tolerated).
//------------------------------------------------------------------
wire [3:0] dbg_bank;        // Master ROM bank register (from sor_master)
wire       dbg_cpu_active;  // Master CPU memory-read activity pulse
wire [15:0] dbg_pc;         // Master CPU PC latched at each opcode fetch
// Runaway-PC trap: frozen last control-flow transfer before the Master ran
// off into dead ROM (see sor_master.sv's trap block).
wire [15:0] dbg_jump_from, dbg_jump_to;
wire  [2:0] dbg_jump_bank;  // bank_reg latched at the jump_from discontinuity
wire        dbg_sled_trapped;
wire [31:0] dbg_snd_cmd_hist; // freeze investigation: 2-deep sound-command {hi,lo} history
// 2026-07-18, "no sound during gameplay on real hardware" investigation
// (docs/WP10_PROGRESS.md): pure pass-through taps from sor_sound's own
// new debug ports, feeding sor_video's status overlay.
wire        dbg_dac_activity, dbg_dac9_activity;
wire        dbg_snd_int0_pin;
wire  [7:0] dbg_snd_intc_request, dbg_snd_intc_in_service;
wire  [6:0] dbg_snd_intc_ext0_ctrl, dbg_snd_intc_ext1_ctrl;
wire        dbg_snd_intc_poll_pending;
wire        dbg_snd_audiocpu_reset_n, dbg_snd_core_bus_activity, dbg_snd_rom_fetch_activity;
wire [15:0] dbg_snd_core_ip;
wire        dbg_mvport_stall; // freeze investigation: live level of Master's VRAM-port /WAIT stall
wire [7:0]  dbg_rom_byte = master_rom_data_r; // last byte fetched from SDRAM (Master ROM)
wire [7:0]  dbg_io_addr;    // last I/O port address accessed by Master
wire        dbg_io_rd;      // 1=read, 0=write
wire        dbg_pc_left_fixed; // sticky: Master PC ever fetched from banked ROM
wire  [7:0] dbg_irq_cnt;    // saturating count of periodic_int_n (raster IRQ) firings
wire        dbg_read_gin0, dbg_read_gin1, dbg_mcont_wr; // sticky I/O indicators
wire        dbg_pc_isr; // sticky: Master PC ever entered 0x0038-0x0066

// Latched status bits for the on-screen overlay (left 4 blocks):
//   bit 0 (red)  : sdram_ready — SDRAM initialised and ready
//   bit 1 (green): slave_reset_n released by Master (/MCONT bit 0)
//   bit 2 (blue) : vp_req_m ever fired — Master issued at least one VRAM port op
//   bit 3 (white): vram_we ever fired — Slave has written at least one VRAM byte
reg dbg_ever_cmd;
reg dbg_ever_vram;
reg dbg_ever_cram;
reg dbg_ever_scroll;
wire dbg_scroll_wr;
always @(posedge clk_sys) begin
    if (reset) begin
        dbg_ever_cmd    <= 1'b0;
        dbg_ever_vram   <= 1'b0;
        dbg_ever_cram   <= 1'b0;
        dbg_ever_scroll <= 1'b0;
    end else begin
        if (vp_req_m)    dbg_ever_cmd    <= 1'b1;
        if (vram_we_cpu) dbg_ever_vram   <= 1'b1;
        if (cram_we_cpu) dbg_ever_cram   <= 1'b1;
        if (dbg_scroll_wr) dbg_ever_scroll <= 1'b1;
    end
end
wire [4:0] dbg_status = {dbg_pc_left_fixed, dbg_ever_vram, dbg_ever_cmd, slave_reset_n, sdram_ready};

//------------------------------------------------------------------
// Phase-locked CPU/video release (2026-07-16, docs/SESSION_2026-07-16.md
// section 8-ZERO: "our RTL is non-deterministic, the real board is not").
//
// Root cause of the run-to-run divergence: the old gate released the
// Master/Slave CPUs on `~sdram_ready | ~dl_settled` while sor_video's
// hc/vc counters ran on bare `reset` -- i.e. video free-ran for the
// entire (variable-length, HPS-scheduling-dependent) ROM download, so
// CPU cycle 0 always landed at an arbitrary, boot-dependent raster line.
// Every seed the game has (the VA10 raster IRQ, the Slave's $F802 raster
// read, the VBLANK poll) is downstream of that phase. On real hardware,
// power-on reset releases the CPUs and the raster counters together, so
// every boot starts at a fixed phase -- exactly what we weren't doing.
//
// UPDATE 2026-07-16 (later same session) -- ground truth from a MAME Lua
// probe (reset_phase_cold.lua), cold boot and soft reset IDENTICAL: MAME
// releases the Master Z80 at raster position vpos=240, hpos=0 -- the
// FIRST LINE OF VBLANK, not the top of active display (vc=0/hc=0). A
// hardware test of the original single-stage "release everything at
// vc==0/hc==0" scheme (both regs tied to the same edge) empirically
// produced a badly broken boot -- consistent with this being the wrong
// phase, not merely "a" phase.
//
// Fix, now TWO deterministic stages instead of one:
//
//   Stage 1 -- `video_release`: set on the first clk_sys edge where
//   sdram_ready && dl_settled && ce_z80_cnt==0 (this is exactly what
//   `cpu_release` used to gate on directly). sor_video's reset is tied
//   to `reset | ~video_release`, so hc/vc are held at 0 the entire time
//   video_release is low (they cannot free-run during the ROM download)
//   and then start counting from a fixed origin the instant it fires.
//
//   Stage 2 -- `cpu_release`: held low until video_release has run the
//   counters up to the MAME-measured release point, vc==240 (the first
//   line of VBLANK), at hc==0. Implemented via sor_video's own VBlank
//   output rather than wiring hc/vc out of the module: VBlank is
//   registered as `(vc >= V_ACTIVE)` on every ce_pix tick (sor_video.sv
//   ~line 285) with NO reset branch of its own, so while video_release
//   is low (hc/vc pinned at 0) it continuously re-evaluates to 0 every
//   ce_pix tick and is guaranteed settled/stable before release. The
//   moment vc rolls over from 239 to 240 is definitionally the same
//   clk_sys edge hc rolls over to 0 (both updates are in the same
//   `if (hc == H_TOTAL-1)` branch, sor_video.sv ~line 271), so VBlank's
//   rising edge is an exact, single-cycle marker for "vc==240 && hc==0"
//   with no ambiguity about which of the many hc==0 cycles-per-line-toggle
//   it is -- it can only ever fire once per frame. That rising edge is
//   latched as `cpu_release_pending`; the same ce_z80_cnt==0 phase-lock
//   used for video_release then fires `cpu_release` on the next divider
//   tick 0 after the pending flag sets (VBlank itself toggles only on
//   ce_pix, asynchronous to the ce_z80_cnt divider, so this is the
//   deterministic hand-off between the two clock domains -- a few
//   clk_sys cycles of slop between the VBlank edge and the CPU's actual
//   first tick, never more than one ce_z80_cnt period, i.e. a small,
//   fixed, one-time offset from MAME's exact hpos=0 rather than any
//   frame-level ambiguity). Master and Slave resets are UNCHANGED --
//   still gated on `~cpu_release` exactly as before; only what feeds
//   cpu_release itself has moved.
//
// The remaining condition, `ce_z80_cnt == 0`, phase-locks the 6 MHz
// Z80 clock-enable divider (see CE_6M above) to the release edge, so the
// CPUs' very first tick after release is always the divider's tick 0
// rather than an arbitrary phase 0-7 of it.
//
// NOTE -- refresh alignment NOT done: sdram.sv's `refresh_cnt` (see its
// own file, ~line 143) is a free-running counter entirely internal to
// that module, reset only by its `rst_n` (tied to ~sdram_init, i.e. the
// hardware-level PLL/RESET reset, not this game reset). It exposes no
// port to restart or align it, and per this task's constraints the SDRAM
// controller's refresh logic itself is not to be modified. So SDRAM
// refresh-vs-fetch collision timing remains a possible residual entropy
// source relative to CPU cycle 0, even after this fix.
//------------------------------------------------------------------
reg video_release;
always @(posedge clk_sys) begin
	if (reset || sdram_init)
		video_release <= 1'b0;
	else if (!video_release && sdram_ready && dl_settled && (ce_z80_cnt == 3'd0))
		video_release <= 1'b1;
end

reg cpu_release;
reg cpu_release_pending; // latched VBlank rising edge (vc==240 && hc==0), awaiting ce_z80_cnt==0
reg vblank_prev;
always @(posedge clk_sys) begin
	if (reset || sdram_init) begin
		cpu_release         <= 1'b0;
		cpu_release_pending <= 1'b0;
		vblank_prev         <= 1'b0;
	end else begin
		vblank_prev <= VBlank;
		if (video_release && !cpu_release) begin
			if (VBlank && !vblank_prev)
				cpu_release_pending <= 1'b1;
			if ((cpu_release_pending || (VBlank && !vblank_prev)) && (ce_z80_cnt == 3'd0))
				cpu_release <= 1'b1;
		end
	end
end

// WP10: Master -> sound-board control/command latch wires (see
// sor_master.sv's own port comments for the real-hardware rationale --
// port 0xF0 doubles as both the graphics bank switch and the 80186's
// control register on real hardware, docs/WP10_PROGRESS.md).
wire  [7:0] sound_ctrl_data;
wire        sound_ctrl_wr;
wire [15:0] sound_cmd_wr_data;
wire        sound_cmd_wr_lo, sound_cmd_wr_hi;
wire  [7:0] sound_response_data; // 2026-07-18: real 80186 response latch, sor_sound -> sor_master

// Held in reset until SDRAM init completes, in addition to (not instead
// of) the implicit wait-state stall the SDRAM controller's own access
// manager already provides pre-ready. Removes any dependence on that
// internal FSM behavior for correctness -- the CPU core itself never
// even attempts to fetch until the SDRAM subsystem reports ready.
sor_master master
(
	.clk_sys(clk_sys),
	.reset(reset | ~cpu_release),
	.CE_6M(CE_6M),

	.rom_addr(master_rom_addr_w),
	.rom_data(master_rom_data_r),

	.wram_addr(wram_addr_m),
	.wram_din(wram_din_m),
	.wram_we(wram_we_m),
	.wram_dout(wram_dout_m),

	.battram_addr(battram_addr_m),
	.battram_din(battram_din_m),
	.battram_we(battram_we_m),
	.battram_dout(battram_dout_m),

	.cram_addr(cram_addr_cpu),
	.cram_din(cram_din_cpu),
	.cram_we(cram_we_cpu),

	.vp_req(vp_req_m),
	.vp_rd(vp_rd_m),
	.vp_trans(vp_trans_m),
	.vp_addr(vp_addr_m),
	.vp_data(vp_data_m),
	.vp_pop(vp_pop_m),
	.vp_rdata(vp_rdata_m),

	.slave_reset_n(slave_reset_n),
	.slave_nmi_n(slave_nmi_n),
	.slave_int_req(slave_int_req),

	.slave_halt_n(slave_halt_n),
	.vblank(VBlank),
	.raster_line(raster_line),

	.eeprom_di(eeprom_di),
	.eeprom_clk(eeprom_clk),
	.eeprom_cs(eeprom_cs),
	.eeprom_do(eeprom_do),

	.vid_addr(vid_addr_m),
	.vid_addr_wr(vid_addr_wr_m),

	.scroll_x(scroll_x_m),
	.scroll_y(scroll_y_m),
	.gfxbank(gfxbank_m),

	.p1_pedal(p1_pedal),
	.p2_pedal(p2_pedal),
	.p3_pedal(p3_pedal),

	.p1_wheel(p1_wheel),
	.p2_wheel(p2_wheel),
	.p3_wheel(p3_wheel),

	.p1_btn(p1_btn),
	.p2_btn(p2_btn),
	.p3_btn(p3_btn),
	.service(service),
	.free_play(free_play),

	.dbg_bank(dbg_bank),
	.dbg_cpu_active(dbg_cpu_active),
	.dbg_pc(dbg_pc),
	.dbg_io_addr(dbg_io_addr),
	.dbg_io_rd(dbg_io_rd),
	.dbg_pc_left_fixed(dbg_pc_left_fixed),
	.dbg_irq_cnt(dbg_irq_cnt),
	.dbg_read_gin0_o(dbg_read_gin0),
	.dbg_read_gin1_o(dbg_read_gin1),
	.dbg_mcont_wr_o(dbg_mcont_wr),
	.dbg_scroll_wr_o(dbg_scroll_wr),
	.dbg_pc_isr_o(dbg_pc_isr),

	.dbg_jump_bank(dbg_jump_bank),
	.dbg_jump_from(dbg_jump_from),
	.dbg_jump_to(dbg_jump_to),
	.dbg_sled_trapped(dbg_sled_trapped),

	.rom_req  (master_rom_req),
	.rom_stall(master_rom_stall),

	.dbg_snd_cmd_hist(dbg_snd_cmd_hist),
	.dbg_mvport_stall(dbg_mvport_stall),

	.sound_ctrl_data(sound_ctrl_data),
	.sound_ctrl_wr(sound_ctrl_wr),
	.cmd_wr_data(sound_cmd_wr_data),
	.cmd_wr_lo(sound_cmd_wr_lo),
	.cmd_wr_hi(sound_cmd_wr_hi),
	.response_data(sound_response_data)
);

//------------------------------------------------------------------
// Slave Z80 — SDRAM ROM stall logic (mirror of master scheme)
//------------------------------------------------------------------
wire        slave_rom_req;           // from sor_slave
wire [18:0] slave_rom_addr_w;        // from sor_slave (flat 512 KB offset)
reg  [7:0]  slave_rom_data_r;        // latched SDRAM byte

// Slave debug taps (Follow-up 6) -- routed straight into sor_video's
// status row; see sor_slave.sv's port comment for what each one is.
wire [15:0] dbg_s_pc;
wire  [3:0] dbg_s_bank_reg;
wire  [7:0] dbg_s_bank_wr_cnt;
wire  [3:0] dbg_s_bank_max;
wire [15:0] dbg_s_vram_wr_cnt;
wire        dbg_s_banked_read_ever;

reg  slave_ack_hold;
always @(posedge clk_sys) begin
	if (!slave_rom_req)
		slave_ack_hold <= 1'b0;
	else if (sdram_rd1_ack)
		slave_ack_hold <= 1'b1;
end

// Explicit sdram_ready gate — see matching comment on sdram_rd0_req above.
assign sdram_rd1_req  = slave_rom_req & ~slave_ack_hold & sdram_ready;
assign sdram_rd1_addr = 25'h040000 + {6'b0, slave_rom_addr_w};  // 6+19=25, base=0x040000
wire   slave_rom_stall = slave_rom_req & (~slave_ack_hold | ~sdram_ready);

always @(posedge clk_sys)
	if (sdram_rd1_ack) slave_rom_data_r <= sdram_rd1_data;

//------------------------------------------------------------------
// Slave Z80
//------------------------------------------------------------------
// Slave CPU is held in reset by the board-level reset OR by the Master's
// /MCONT bit0=0 (slave_reset_n=0). The Master explicitly releases the Slave
// by writing /MCONT bit0=1 after it has finished its own initialization.
// Without this gate the Slave runs from power-on independent of the Master,
// which causes it to execute before any inter-CPU handshake is established.

sor_slave slave
(
	.clk_sys(clk_sys),
	.reset(reset | ~cpu_release | ~slave_reset_n),
	.CE_6M(CE_6M),

	.rom_addr(slave_rom_addr_w),
	.rom_data(slave_rom_data_r),

	.vp_req(vp_req_s),
	.vp_rd(vp_rd_s),
	.vp_trans(vp_trans_s),
	.vp_addr(vp_addr_s),
	.vp_data(vp_data_s),
	.vp_pop(vp_pop_s),
	.vp_rdata(vp_rdata_s),

	.wram_addr(wram_addr_s),
	.wram_din(wram_din_s),
	.wram_we(wram_we_s),
	.wram_dout(wram_dout_s),

	.slave_int_req(slave_int_req),
	.nmi_n(slave_nmi_n),
	.slave_halt_n(slave_halt_n),

	.raster_line(raster_line),

	.rom_req  (slave_rom_req),
	.rom_stall(slave_rom_stall),

	.dbg_pc              (dbg_s_pc),
	.dbg_bank_reg        (dbg_s_bank_reg),
	.dbg_bank_wr_cnt     (dbg_s_bank_wr_cnt),
	.dbg_bank_max        (dbg_s_bank_max),
	.dbg_vram_wr_cnt     (dbg_s_vram_wr_cnt),
	.dbg_banked_read_ever(dbg_s_banked_read_ever)
);

//------------------------------------------------------------------
// Sound board (WP10 of docs/planning_80186_sound.md): real s80x86
// Core + i186_periph (WP2-4, WP8 DMA) + leland_sound_board (WP6) +
// leland_dac_mixer (WP7), bundled in rtl/sor_sound.sv.
//------------------------------------------------------------------
wire        sound_rom_req;
wire [19:0] sound_rom_addr_w;
reg   [7:0] sound_rom_data_r;

reg  sound_ack_hold;
always @(posedge clk_sys) begin
	if (!sound_rom_req)
		sound_ack_hold <= 1'b0;
	else if (sdram_rd3_ack)
		sound_ack_hold <= 1'b1;
end

// Explicit sdram_ready gate -- same convention as rd0/rd1 above.
assign sdram_rd3_req  = sound_rom_req & ~sound_ack_hold & sdram_ready;
assign sdram_rd3_addr = 25'h0C0000 + {5'b0, sound_rom_addr_w}; // 5+20=25, base=0x0C0000 (MRA layout)
wire   sound_rom_stall = sound_rom_req & (~sound_ack_hold | ~sdram_ready);

always @(posedge clk_sys)
	if (sdram_rd3_ack) sound_rom_data_r <= sdram_rd3_data;

sor_sound sound(
	.clk_sys(clk_sys),
	.reset(reset | ~cpu_release),
	.ce_8m(CE_8M),

	.sound_ctrl_data(sound_ctrl_data),
	.sound_ctrl_wr(sound_ctrl_wr),
	.cmd_wr_data(sound_cmd_wr_data),
	.cmd_wr_lo(sound_cmd_wr_lo),
	.cmd_wr_hi(sound_cmd_wr_hi),
	.response_data(sound_response_data), // 2026-07-18: wired to sor_master, see docs/WP10_PROGRESS.md

	.rom_req(sound_rom_req),
	.rom_addr(sound_rom_addr_w),
	.rom_data(sound_rom_data_r),
	.rom_stall(sound_rom_stall),

	.audio_out(audio_out),

	.dbg_dac_activity(dbg_dac_activity),
	.dbg_dac9_activity(dbg_dac9_activity),
	.dbg_int0_pin(dbg_snd_int0_pin),
	.dbg_intc_request(dbg_snd_intc_request),
	.dbg_intc_in_service(dbg_snd_intc_in_service),
	.dbg_intc_ext0_ctrl(dbg_snd_intc_ext0_ctrl),
	.dbg_intc_ext1_ctrl(dbg_snd_intc_ext1_ctrl),
	.dbg_intc_poll_pending(dbg_snd_intc_poll_pending),
	.dbg_audiocpu_reset_n(dbg_snd_audiocpu_reset_n),
	.dbg_core_bus_activity(dbg_snd_core_bus_activity),
	.dbg_rom_fetch_activity(dbg_snd_rom_fetch_activity),
	.dbg_core_ip(dbg_snd_core_ip)
);

// Stall detector (2026-07-17, freeze investigation): the row-1 live PC
// field flickers unreadably during the race-freeze bug because the
// Master keeps fetching in a non-terminating loop rather than halting
// (see docs/SDRAM_TIMING_INVESTIGATION.md's "PINNED" section and the
// handed-off investigation for full context). Freezes dbg_pc + the
// VRAM-port sequencer's state ONCE, the first time dbg_s_vram_wr_cnt
// (proxy for "the game is still rendering") holds flat for
// STALL_THRESHOLD cycles -- long enough to clear the ~3.1s legitimate
// post-race/scoring transition a live MAME trace found (near-zero VRAM
// writes but not a hang), short enough to catch the real hang (which
// runs indefinitely until reboot). Latches once (stall_latched gates
// further writes) so the displayed values are a stable snapshot, not
// another flicker.
localparam STALL_THRESHOLD = 30'd384_000_000; // ~8s @ 48MHz
reg [29:0] stall_timer;
reg [15:0] stall_vram_ref;
reg        stall_latched;
reg [15:0] dbg_stall_pc;
reg        dbg_stall_mvport;
reg        dbg_stall_side;
reg  [2:0] dbg_stall_seq;
reg  [2:0] dbg_stall_bank; // live bank_reg (dbg_bank[2:0]) at the frozen instant -- resolves
                            // which of the 7 banked ROM images dbg_stall_pc belongs to

reg         wram_dump_active;
reg         wram_dump_phase; // 0 = addr_b just changed (dout_b stale, wait) 1 = dout_b valid, capture
reg   [7:0] dbg_wram_e712, dbg_wram_e715, dbg_wram_e716;

always @(posedge clk_sys) begin
	if (reset) begin
		stall_timer      <= 30'd0;
		stall_vram_ref   <= 16'd0;
		stall_latched    <= 1'b0;
		dbg_stall_pc     <= 16'd0;
		dbg_stall_mvport <= 1'b0;
		dbg_stall_side   <= 1'b0;
		dbg_stall_seq    <= 3'd0;
		dbg_stall_bank   <= 3'd0;
		wram_dump_idx    <= 2'd0;
		wram_dump_active <= 1'b0;
		wram_dump_phase  <= 1'b0;
		dbg_wram_e712    <= 8'h00;
		dbg_wram_e715    <= 8'h00;
		dbg_wram_e716    <= 8'h00;
	end else begin
		if (dbg_s_vram_wr_cnt != stall_vram_ref) begin
			stall_vram_ref <= dbg_s_vram_wr_cnt;
			stall_timer    <= 30'd0;
		end else if (stall_timer != STALL_THRESHOLD) begin
			stall_timer <= stall_timer + 30'd1;
		end
		if (!stall_latched && stall_timer == STALL_THRESHOLD) begin
			stall_latched    <= 1'b1;
			dbg_stall_pc     <= dbg_pc;
			dbg_stall_mvport <= dbg_mvport_stall;
			dbg_stall_side   <= cur_side;
			dbg_stall_seq    <= seq_state;
			dbg_stall_bank   <= dbg_bank[2:0];
			wram_dump_active <= 1'b1;
			wram_dump_idx    <= 2'd0;
			wram_dump_phase  <= 1'b0;
		end else if (wram_dump_active) begin
			if (!wram_dump_phase) begin
				// addr_b (wram_dbg_addr_tbl[wram_dump_idx]) just became
				// stable this cycle -- dout_b is still the PREVIOUS
				// address's data. Wait one more cycle before capturing.
				wram_dump_phase <= 1'b1;
			end else begin
				case (wram_dump_idx)
					2'd0: dbg_wram_e712 <= wram_dbg_dout_b;
					2'd1: dbg_wram_e715 <= wram_dbg_dout_b;
					2'd2: dbg_wram_e716 <= wram_dbg_dout_b;
				endcase
				if (wram_dump_idx == 2'd2) begin
					wram_dump_active <= 1'b0;
				end else begin
					wram_dump_idx   <= wram_dump_idx + 2'd1;
					wram_dump_phase <= 1'b0;
				end
			end
		end
	end
end

//------------------------------------------------------------------
// Video system
//------------------------------------------------------------------
sor_video video
(
	.clk_sys(clk_sys),
	.reset(reset | ~video_release), // phase-locked video release -- see video_release/cpu_release above
	.ce_pix(ce_pix),

	.HBlank(HBlank),
	.HSync(HSync),
	.VBlank(VBlank),
	.VSync(VSync),

	.vram_addr(vram_addr_vid),
	.vram_data(vram_dout_vid),

	.cram_addr(cram_addr_vid),
	.cram_data(cram_dout_vid),

	.rgb(rgb),

	.scroll_x(scroll_x_m),
	.scroll_y(scroll_y_m),
	.gfxbank(gfxbank_m),

	.gfx_addr(gfx_addr_from_video),
	.gfx_data(gfx_data_vid),
	.prom_addr(prom_addr_from_video),
	.prom_data(prom_data_vid),

	.raster_line(raster_line),

	.dbg_bank(dbg_status),
	.dbg_cpu_active(dbg_cpu_active),
	.dbg_pc(dbg_pc),
	.dbg_irq_cnt(dbg_irq_cnt),
	.dbg_read_gin0(dbg_read_gin0),
	.dbg_read_gin1(dbg_read_gin1),
	.dbg_mcont_wr(dbg_mcont_wr),
	.dbg_ever_cram(dbg_ever_cram),
	.dbg_ever_scroll(dbg_ever_scroll),
	.dbg_ever_vram(dbg_ever_vram),
	.dbg_pc_isr(dbg_pc_isr),
	.dbg_rom_byte(dbg_rom_byte),
	.dbg_io_addr(dbg_io_addr),
	.dbg_io_rd(dbg_io_rd),

	.dbg_wr_chk(wr_chk),
	.dbg_rd_chk(rd_chk),
	.dbg_chk_done(rd_scan_done),
	.dbg_chk_match(dbg_chk_match),

	.dbg_wr_chk_even(wr_chk_even),
	.dbg_rd_chk_even(rd_chk_even),
	.dbg_chk_match_even(dbg_chk_match_even),
	.dbg_wr_chk_odd(wr_chk_odd),
	.dbg_rd_chk_odd(rd_chk_odd),
	.dbg_chk_match_odd(dbg_chk_match_odd),

	.dbg_chk_match_q({dbg_chk_match_q3, dbg_chk_match_q2, dbg_chk_match_q1, dbg_chk_match_q0}),
	.dbg_chk_done_q(rd_scan_done),

	.dbg_bt_done(bt_done),
	.dbg_bt_pass(bt_pass),
	.dbg_bt_readback(bt_readback),
	.dbg_bt_readback2(bt_readback2),

	.dbg_scan_b0(dbg_scan_b0),
	.dbg_scan_b1(dbg_scan_b1),
	.dbg_scan_b2(dbg_scan_b2),
	.dbg_scan_b3(dbg_scan_b3),

	// Repurposed row: slot0=accept0 hit count, slot1=accept0 data
	// (0xF3 expected). Skew theory A/B check answered (0x00 -- theory
	// disproven) and retired; slots 2/3 now mirror the same probe
	// applied to address 1 instead: slot2=accept1 hit count,
	// slot3=accept1 data (0xED expected).
	.dbg_wr_b0(dbg_accept0_cnt),
	.dbg_wr_b1(dbg_accept0_data),
	.dbg_wr_b2(dbg_accept1_cnt),
	.dbg_wr_b3(dbg_accept1_data),

	.dbg_early_b0(dbg_early_b0),
	.dbg_immediate_b0(dbg_immediate_b0),
	.dbg_rd2_ack_cnt(dbg_rd2_ack_cnt),
	.dbg_accept2_raw_addr(dbg_accept2_raw_addr),
	.dbg_accept2_data(dbg_accept2_data),
	.dbg_dl_sessions(dbg_dl_sessions),

	.dbg_s_pc(dbg_s_pc),
	.dbg_s_bank_reg(dbg_s_bank_reg),
	.dbg_s_bank_wr_cnt(dbg_s_bank_wr_cnt),
	.dbg_s_bank_max(dbg_s_bank_max),
	.dbg_s_vram_wr_cnt(dbg_s_vram_wr_cnt),
	.dbg_s_banked_read_ever(dbg_s_banked_read_ever),

	.dbg_snd_cmd_hist(dbg_snd_cmd_hist),
	.dbg_dac_activity(dbg_dac_activity),
	.dbg_dac9_activity(dbg_dac9_activity),
	.dbg_snd_int0_pin(dbg_snd_int0_pin),
	.dbg_snd_intc_request(dbg_snd_intc_request),
	.dbg_snd_intc_in_service(dbg_snd_intc_in_service),
	.dbg_snd_intc_ext0_ctrl(dbg_snd_intc_ext0_ctrl),
	.dbg_snd_intc_ext1_ctrl(dbg_snd_intc_ext1_ctrl),
	.dbg_snd_intc_poll_pending(dbg_snd_intc_poll_pending),
	.dbg_snd_audiocpu_reset_n(dbg_snd_audiocpu_reset_n),
	.dbg_snd_core_bus_activity(dbg_snd_core_bus_activity),
	.dbg_snd_rom_fetch_activity(dbg_snd_rom_fetch_activity),
	.dbg_snd_core_ip(dbg_snd_core_ip),
	.dbg_jump_from(dbg_jump_from),
	.dbg_jump_to(dbg_jump_to),
	.dbg_sled_trapped(dbg_sled_trapped),

	// Master bank register at the trap-latched jump (2026-07-16,
	// docs/SESSION_2026-07-16.md section 8d): resolves which of the 7
	// banked-ROM images a banked jump_from/jump_to PC actually belongs
	// to -- without it a banked PC is ambiguous across 7 different ROM
	// addresses. This is sor_master's jump_bank_r, latched together with
	// jump_from at each discontinuity, NOT the live bank_reg (which may
	// have changed by the time the frozen row is photographed; the live
	// value is already shown on status row 1).
	.dbg_m_bank_reg(dbg_jump_bank),
	.dbg_live_mm({live_mm_cnt, live_poll_cnt}),
	.dbg_stall_pc(dbg_stall_pc),
	.dbg_stall_flags({dbg_stall_mvport, dbg_stall_side, dbg_stall_seq}),
	.dbg_stall_bank(dbg_stall_bank),
	.dbg_wram_dump({dbg_wram_e712, dbg_wram_e715, dbg_wram_e716})
);

endmodule
