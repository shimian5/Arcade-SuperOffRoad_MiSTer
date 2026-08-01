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
//  SDRAM address layout (byte addresses, post-16-byte-header, see
//  rtl/leland_board_pkg.sv for the canonical values):
//    0x000000  Master Z80 program ROM (256 KB used of 1 MB reserved)
//    0x100000  Slave  Z80 program ROM (~576 KB used of 2 MB reserved)
//    0x300000  80186 sound ROM        (1 MB used, sparse)
//    0x400000  bg_gfx tile ROM        (96 KB used of 2 MB reserved) -> BRAM
//    0x600000  bg_prom palette PROM   (128 KB used of 256 KB reserved) -> BRAM
//============================================================================

import leland_board_pkg::*;

module sor_board #(
	// Passed straight through to the internal sdram controller. See
	// rtl/sdram.sv's own USE_ALTDDIO parameter comment — default 1 is
	// required for real hardware; simulation testbenches override to 0
	// to avoid needing Altera's vendor simulation libraries.
	parameter bit USE_ALTDDIO = 1'b1,

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
	input         clk_sdram, // phase-shifted 48 MHz PLL output, drives SDRAM_CLK
	                          // (docs/sdram_plan.md Section 3a; WP-L3's dedicated
	                          // 96MHz clock/CDC bridge was reverted 2026-07-22 --
	                          // see rtl/pll/pll_0002.v's outclk_1 comment)
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

	// WP-L3: 4-player digital joystick, used only when the active game's
	// input_scheme is JOY4_DIGITAL (pigout) -- unconsumed for WHEELS3_
	// PEDALS3 games. Bit layout: [0]=right [1]=left [2]=down [3]=up
	// [4]=btn1 [5]=btn2 [6]=start [7]=coin (standard MiSTer joystick
	// vector convention).
	input   [7:0] p1_joy, p2_joy, p3_joy, p4_joy,

	// OSD options
	input         service,
	input         free_play,

	// Debug overlay toggle (2026-07-25, Pig Out slow-motion investigation
	// instrumentation): OSD-controlled on-screen hex counters, see
	// sor_video.sv's overlay render block. Purely display; touches no
	// game logic.
	input         show_overlay,

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

// Read port 2 (WP-L2: sor_video's background-tilemap gfx/prom fetch --
// reuses the rd2 channel slot/naming freed by WP-L0's removal of the
// old diagnostic byte-test scanner; unrelated new use). Highest read
// priority (second only to writes) per docs/planning_leland_multiboard.md
// section 5 -- the video fetch FSM has a tight, timing-critical
// arm-to-commit deadline (LEAD=8 pixel-clock ticks, see sor_video.sv),
// while master/slave/sound CPU reads are comparatively forgiving via
// their own stall mechanisms.
// sdram_rd2_req/addr feed the arbiter and are muxed (below, near the
// gfx-repack FSM) between sor_video's own request (sdram_rd2_req_v/addr_v)
// and the one-shot boot-time repack FSM's -- the two are mutually
// exclusive in time (repack always finishes, gated via repack_done into
// video_release, before sor_video's reset ever releases and it can start
// requesting) so there's no real arbitration needed between them, just a
// simple mux.
wire        sdram_rd2_req;
wire [24:0] sdram_rd2_addr;
wire        sdram_rd2_req_v;
wire [24:0] sdram_rd2_addr_v;
reg         sdram_rd2_ack;
reg   [7:0] sdram_rd2_data;
reg  [15:0] sdram_rd2_data16; // wider-reads path, see sor_video.sv's FP_GFXROW_REQ/WAIT
reg  [15:0] sdram_rd2_data16_hi; // WP-M8: second burst word (BURST_LEN=2) of the same GFXROW read
wire        rd2_fetch_busy; // whole-tile-burst-in-progress, see sor_video.sv's fetch_busy comment
wire  [3:0] rd2_rbuf_count; // ring-buffer occupancy, see sor_video.sv's rbuf_count_out comment

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
(* keep *) wire        sd_req_hit;
(* keep *) wire  [7:0] sd_dout;
(* keep *) wire [15:0] sd_dout16; // full captured word, see sdram.sv's dout16
// WP-M8 (2026-07-24): burst-safe views of the above -- see sdram_banked.sv's
// dout_b0/burst_words port comments. sd_dout_b0/sd_burst_words are what the
// arbiter chokepoint below actually latches into each channel's data
// register now (not sd_dout/sd_dout16), so raising BURST_LEN later can't
// silently corrupt a client that never migrated off the legacy path.
(* keep *) wire  [7:0] sd_dout_b0;
(* keep *) wire [7:0][15:0] sd_burst_words;

localparam CH_WR = 3'd0, CH_RD2 = 3'd1, CH_RD0 = 3'd2, CH_RD1 = 3'd3, CH_RD3 = 3'd4;

// Priority: wr always on top; among the reads, rd2(video) is normally at
// the BOTTOM (below rd0/rd1/rd3) but AGES to the TOP of the read group
// (above rd0/rd1/rd3, never above wr) once its pending request has
// waited too long -- see the rd2_age_cnt/rd2_boost block further down
// (right before the sel_wr/sel_rd* wires) for the full aging scheme and
// its threshold arithmetic. Gated by !in_flight throughout.
//
// WP-L2 POST-HARDWARE-BRINGUP REVISION (2026-07-20): the plan doc's
// original order (wr > rd2 > rd0 > rd1 > rd3, video given top read
// priority for its LEAD=8 deadline) measured out on real hardware as a
// severe master/slave slowdown and sound dropouts, even though sim's
// pixel-diff regression stayed clean (sim only checks gfx correctness,
// never CPU timing headroom). Root cause, computed from this file's own
// fetch FSM (sor_video.sv fetch_ph): each background tile fetch is 4
// sequential SDRAM req/ack round trips (prom + 3 gfx planes), held
// nearly back-to-back (only a single idle cycle between each) -- ~20-25
// clk_sys cycles of near-continuous sdram_rd2_req assertion per tile.
// The fetch arm (col_in_tile==0) fires every 8 hc-ticks across the
// ENTIRE raster, blanking included (H_TOTAL=424 x V_TOTAL=256 = 13568
// tile-fetches/frame, vs only 320/8 x 240 = 9600 actually needed for the
// visible 320x240 area -- roughly 1.4x more fetches than necessary, but
// that overshoot is NOT the dominant term). At ~727,700 clk_sys cycles/
// frame (48MHz / 65.95Hz refresh) and 13568 fetches x ~22 cycles busy
// each, rd2 is asserted roughly 298,000/727,700 = ~41% OF EVERY FRAME'S
// CYCLES. Because rd2 sat above rd0/rd1/rd3, any CPU/sound request
// landing in that ~41% window had to wait out the remainder of rd2's
// current 4-transaction burst (worst case ~20-25 cycles) before being
// granted -- compare to master/slave Z80s only getting one memory-cycle
// opportunity per CE_6M tick (every 8 clk_sys cycles) and sound's 80186
// one per CE_8M tick (every 6): a ~20-25 cycle stall on ~41% of accesses
// is a multi-times inflation of effective memory latency, consistent
// with the observed slowdown/audio dropouts. Pre-WP-L2, rd0 (master) sat
// at the top of the read priority chain with zero read-side contention
// at all.
//
// Fix: put video (rd2) at the BOTTOM of the read priority chain instead
// of the top. sor_video.sv's own LEAD=8 arm-to-commit window (~54
// clk_sys cycles) already budgets ~2x margin over the ~20-25 cycles a
// fetch needs even WITHOUT any priority advantage, specifically so it
// could tolerate exactly this kind of reordering; master/slave/sound
// have no equivalent slack in this design and must not be made to wait
// behind the highest-duty-cycle channel on the bus. rd3 (sound) placed
// above rd2 too, for the same reason its ROM reads were also starving.
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

//------------------------------------------------------------------
// rd2 (video) AGING / PRIORITY-BOOST (2026-07-20, post-attempt-2 hardware
// bringup): neither static ordering worked. rd2-on-top (original plan doc
// order) starved master/slave/sound -- see the big comment above this
// block. rd2-on-bottom (the fix applied there) went the other way: on
// real hardware it produced visible horizontal tearing/near-loss-of-sync,
// because under genuine CPU/sound bus load rd2's own request can now sit
// pending long enough to blow its LEAD=8 arm-to-commit deadline (~54
// clk_sys cycles, see sor_video.sv) -- fixed-bottom priority provides NO
// bound on worst-case wait when rd0/rd1/rd3 are busy enough of the time.
//
// Fix: age rd2's pending request. It starts each pending request at the
// BOTTOM of the read chain (preserving attempt-2's normal-case win for
// CPU/sound), and only jumps to the TOP (attempt-1's position, matching
// wr > rd2 > rd0 > rd1 > rd3) once it has been waiting long enough that
// missing the deadline becomes a real risk. This bounds worst-case rd2
// latency (aging forces top-priority within a fixed number of cycles)
// while keeping average-case CPU/sound interference low (a boost only
// fires when video is actually at risk, not on every fetch).
//
// Threshold arithmetic: LEAD=8 gives a full tile-fetch (arm to commit)
// ~54 clk_sys cycles (per sor_video.sv's own LEAD=8 comment). A tile
// fetch is 4 sequential req/ack round trips (prom + 3 gfx planes); the
// prior attempt's own measurement put the whole burst at ~20-25 cycles
// when it runs back-to-back with no contention, i.e. ~5-6 clk_sys cycles
// per sub-request on average (sdram_simple's CAS_LAT=3 read latency plus
// the FSM's one idle cycle between phases). We age the *whole pending
// tile-fetch sequence* with a single counter (simplest correct choice --
// per-sub-request separate counters would only matter if we wanted a
// tighter bound, which isn't needed here): it starts counting from the
// first cycle sdram_rd2_req is asserted-but-ungranted after a fetch_ph
// phase change, and is cleared on every rd2 grant (each sub-request ack)
// so it always reflects "how long has the CURRENT sub-request been
// waiting", not cumulative wait across the whole tile.
//
// Chosen threshold: 24 clk_sys cycles. Reasoning:
//   - 24 is comfortably above the ~5-6 cycles a sub-request needs when
//     granted immediately (so under light/no contention the boost never
//     fires at all -- zero steady-state interference with CPU/sound,
//     identical behavior to attempt 2 in the common case).
//   - 24 leaves 54 - 24 = 30 cycles of remaining budget after the boost
//     fires. Once boosted, rd2 sits at the TOP of the read chain, so the
//     very next arbitration slot after the in-flight transaction (if any)
//     drains is rd2's -- worst case one already-in-flight transaction
//     (bounded by sdram_simple's own fixed CAS_LAT=3 protocol, a handful
//     of cycles) plus this sub-request's own ~5-6 cycles, then any
//     REMAINING sub-requests in the tile (up to 3 more at ~5-6 cycles
//     each = ~18) still comfortably fit inside 30. Even the pessimistic
//     case -- boost fires on the FIRST sub-request of the tile, at cycle
//     24, and all 4 sub-requests had to each independently re-age and
//     re-boost -- keeps every individual wait capped at 24, i.e. total
//     worst case bounded near 4*24=96 only if boosting somehow failed to
//     win priority each time, which it cannot: once boosted, rd2 is
//     top-priority and wins the very next free slot, so in practice the
//     worst realistic case is one 24-cycle wait (first sub-request boosts)
//     followed by the remaining sub-requests each granted promptly because
//     rd2 stays boosted (aged) as long as ANY sub-request of the current
//     tile is still pending -- see rd2_boost latch behavior below.
//   - 24 was chosen over a lower value (e.g. 16) to keep meaningful
//     average-case margin before boosting -- boosting too eagerly would
//     approach attempt 1's always-high-priority behavior and reintroduce
//     its CPU/sound interference. 24 was chosen over a higher value
//     (e.g. 36-40) because that would leave too little slack (< 20
//     cycles) to guarantee the remaining sub-requests and any single
//     already-in-flight transaction complete before the ~54-cycle wall.
//------------------------------------------------------------------
// ROUND-ROBIN READ ARBITRATION (2026-07-22, replaces the fixed-priority-
// with-single-aging-exception scheme above): real hardware testing
// after the wider-reads/tile-cache follow-up showed rd3 (the 80186
// sound CPU's own ROM-fetch channel) starving hardest of all four read
// channels under real gameplay load -- confirmed via a dedicated
// per-channel latency diagnostic overlay (rd0/rd1/rd3_txn_max, see
// their declarations below): rd3 went RED (>=80 cycles) the moment the
// race started while rd0/rd1/rd2 stayed green/yellow. Root cause: rd3
// sat at the ABSOLUTE BOTTOM of a fixed priority chain with zero
// protection at all, while rd2 alone had a bespoke aging/boost escape
// hatch (the RD2_AGE_THRESH scheme this replaces). Bolting a THIRD
// bespoke aging counter onto rd3 (after rd2's) would be whack-a-mole --
// generalizing to genuine round-robin fairness across all 4 read
// channels fixes this class of problem for good, and is exactly what
// Darius_MiSTer's tile_rom_arbiter.sv does (referenced during the
// original sanity-check investigation but not adopted until now). wr
// keeps absolute top priority throughout, unchanged -- ioctl-load
// correctness depends on writes never being delayed, and writes aren't
// part of the starvation problem (nothing else waits behind rd3-class
// starvation there).
//
// rr_last holds the channel granted last time; the next grant scans
// starting from rr_last+1 and wraps, so whichever channel just went
// checks last next time -- no channel can be skipped for more than 3
// consecutive read grants no matter how busy the others are, a hard
// worst-case bound fixed-priority-with-exceptions can't offer.
localparam RR_RD0 = 2'd0, RR_RD1 = 2'd1, RR_RD2 = 2'd2, RR_RD3 = 2'd3;
reg [1:0] rr_last;

wire [3:0] rr_req = {sdram_rd3_req, sdram_rd2_req, sdram_rd1_req, sdram_rd0_req}; // bit N = channel N's req

function automatic [1:0] rr_pick(input [3:0] req, input [1:0] last);
	reg [1:0] c1, c2, c3;
	begin
		c1 = last + 2'd1;
		c2 = last + 2'd2;
		c3 = last + 2'd3;
		if      (req[c1]) rr_pick = c1;
		else if (req[c2]) rr_pick = c2;
		else if (req[c3]) rr_pick = c3;
		else if (req[last]) rr_pick = last;
		else rr_pick = last; // no request pending at all -- don't-care, sel_* below all read 0
	end
endfunction

// URGENCY ESCALATION -- rd2 ONLY (2026-07-22, narrowed same-day after a
// second hardware round): the first attempt at this made ALL FOUR
// channels urgency-escalating and made things WORSE overall (all four
// latency boxes went from green to yellow, audio noticeably worse) --
// symmetric urgency across every channel means frequent preemptions,
// and each preemption pushes whichever channels it delayed closer to
// THEIR OWN threshold too, cascading into more total contention than
// plain round-robin had, not less. The actual evidence only ever showed
// ONE channel with a real problem: rd2 at boot (stale-commit box red,
// recovering to green once the initial burst settled). Plain round-
// robin alone already fixed rd3 (the original starvation target) with
// no reported issue -- rd0/rd1/rd3 never needed an escape hatch, only
// rd2 did, so only rd2 gets one. This is back to "round-robin baseline
// + one bounded anti-starvation exception," same shape as the original
// rd2-only aging scheme this whole arbiter section replaced, just
// riding on round-robin (fair to rd0/rd1/rd3) instead of fixed priority
// (which is what actually starved rd3 in the first place) as the
// non-urgent baseline.
//
// rd2_wait_cnt uses rd2_fetch_busy (whole-tile-burst-in-progress), not
// a plain req&&!ack pending check, for the same reason the original
// rd2-only aging fix needed it: sor_video.sv's fetch FSM drops its req
// line for exactly one cycle between each tile's 3-4 sub-requests, so a
// plain pending check would silently reset urgency at every sub-request
// boundary instead of tracking the whole burst -- the bug this
// session's earlier fix (see the BUGFIX comment history in this file)
// already found and fixed once for the old scheme; carried forward
// here rather than reintroduced.
//
// ROOT-CAUSE FIX (2026-07-22, same-day third round): rd2-only urgency
// as first written (wait-time alone) still moved rd0/rd1/rd3 from green
// to yellow simultaneously -- not the cascading-preemption failure mode
// symmetric urgency had, but a milder version of the same class of
// problem. Root cause, confirmed by reading sor_video.sv's fetch FSM
// directly: the producer re-arms a new fetch every single clk_sys cycle
// it has ANY room in its 8-deep ring buffer (`fetch_ph==FP_IDLE &&
// rbuf_has_room`), so rd2_fetch_busy -- and therefore rd2_wait_cnt -- is
// true almost continuously regardless of how full the buffer actually
// is. That means rd2_urgent was tripping on ordinary, harmless refill
// bursts just as often as on genuine near-underrun risk, stealing far
// more round-robin slots from rd0/rd1/rd3 than actually necessary.
//
// Fix: gate urgency on ring-buffer occupancy (rbuf_count, exposed from
// sor_video.sv as rbuf_count_out/rd2_rbuf_count here) IN ADDITION TO
// the existing wait-time check, not wait-time alone. rd2 now only
// preempts round-robin when it has ALSO waited past URGENT_THRESH AND
// the buffer has fallen to RD2_LOW_WATER or fewer of its 8 slots --
// i.e. only when it's both been waiting a while AND is genuinely close
// to running dry, not merely "busy doing its normal thing." A full
// buffer rides plain round-robin like everyone else, handing those
// slots back to rd0/rd1/rd3. Boot-time behavior is preserved (buffer
// starts fully empty, so urgency still fires essentially immediately,
// same as before) and rd3 is untouched (still plain round-robin
// throughout, so its earlier fix isn't at risk).
localparam URGENT_THRESH   = 8'd32;
// Tightened from 2 to 1 (2026-07-22, same-day fourth round): with
// LOW_WATER=2, real sustained gameplay load (not just boot) still
// dropped rd0/rd1/rd3 to yellow during the actual race, and a visible
// gameplay symptom (a HUD/score update lagging ~1-2s behind a nitro
// pickup) pointed at real CPU-instruction-fetch throughput loss, not a
// stuck write (the HUD write itself is direct dual-port BRAM, never
// SDRAM-arbitrated -- the lag is consistent with rd0/rd1 fetch latency
// slowing the CPUs' own execution, not a delayed write). rd2's own
// latency box has been solidly green with real margin at LOW_WATER=2,
// suggesting escalation was still firing earlier than strictly
// necessary -- letting the buffer run one slot closer to empty before
// contesting for the bus should further cut how often rd2 takes a slot
// from rd0/rd1/rd3, without giving up real underrun protection (buffer
// depth is 8; even LOW_WATER=1 still escalates with room to spare
// before an actual empty-buffer stall).
localparam RD2_LOW_WATER   = 4'd1; // buffer depth is 8 (RBUF_N in sor_video.sv); escalate only at <=1 of 8

reg [7:0] rd2_wait_cnt;
always @(posedge clk_sys) begin
	if (sdram_init) rd2_wait_cnt <= 8'd0;
	else rd2_wait_cnt <= rd2_fetch_busy ? ((rd2_wait_cnt == 8'hFF) ? rd2_wait_cnt : rd2_wait_cnt + 8'd1) : 8'd0;
end

wire       rd2_urgent  = sdram_rd2_req && (rd2_wait_cnt >= URGENT_THRESH) && (rd2_rbuf_count <= RD2_LOW_WATER);
wire [1:0] rr_grant_ch = rd2_urgent ? RR_RD2 : rr_pick(rr_req, rr_last);

wire sel_wr  = !in_flight && sdram_wr_req;
wire sel_rd0 = !in_flight && !sel_wr && sdram_rd0_req && (rr_grant_ch == RR_RD0);
wire sel_rd1 = !in_flight && !sel_wr && sdram_rd1_req && (rr_grant_ch == RR_RD1);
wire sel_rd2 = !in_flight && !sel_wr && sdram_rd2_req && (rr_grant_ch == RR_RD2);
wire sel_rd3 = !in_flight && !sel_wr && sdram_rd3_req && (rr_grant_ch == RR_RD3);

// WP-M (open-row multichannel): each client is pinned to one SDRAM bank
// (see leland_board_pkg::sdram_addr_to_bank / sdram_bank_base for the map).
// sd_addr_rel is the region-relative byte offset within the client's bank;
// sd_bank is the 2-bit physical bank index. Together they replace the old
// 25-bit absolute sd_addr that encoded bank in bits [11:10] of the address.
//
// Per-client bank assignment (fixed, matching planning_sdram_multichannel.md §2):
//   rd0 (master Z80)  → bank 0   base ADDR_MASTER_BASE (0x000000)
//   rd1 (slave  Z80)  → bank 1   base ADDR_SLAVE_BASE  (0x100000)
//   rd3 (80186 sound) → bank 2   base ADDR_SOUND_BASE  (0x300000)
//   rd2 (video gfx)   → bank 3   base ADDR_GFX_BASE    (0x400000)
//   wr  (ioctl+repack) → decoded from sdram_wr_addr via package functions
wire [1:0]  wr_bank    = sdram_addr_to_bank({2'b0, sdram_wr_addr});
wire [22:0] wr_rel     = sdram_wr_addr[22:0] - sdram_bank_base({2'b0, sdram_wr_addr});

(* keep *) wire  [1:0] sd_bank     = sel_wr  ? wr_bank :
                                      sel_rd0 ? 2'd0 :
                                      sel_rd1 ? 2'd1 :
                                      sel_rd3 ? 2'd2 :
                                                2'd3;   // sel_rd2
(* keep *) wire [22:0] sd_addr_rel = sel_wr  ? wr_rel :
                       sel_rd0 ? sdram_rd0_addr[22:0] :                           // MASTER_BASE=0
                       sel_rd1 ? (sdram_rd1_addr[22:0] - ADDR_SLAVE_BASE[22:0]) :
                       sel_rd2 ? (sdram_rd2_addr[22:0] - ADDR_GFX_BASE[22:0])   :
                                 (sdram_rd3_addr[22:0] - ADDR_SOUND_BASE[22:0]);
(* keep *) wire  [7:0] sd_din    = sdram_wr_data;
(* keep *) wire  [7:0] sd_din_hi = sdram_wr_data_hi;
// Every write is now a full-word write (see sdram_wr_data_hi comment
// above) -- the single-byte-masked `we` path in sdram_simple is kept
// only as a legacy fallback for a future caller that might need it,
// unused here.
(* keep *) wire        sd_we      = 1'b0;
(* keep *) wire        sd_we_word = sel_wr;
(* keep *) wire        sd_rd      = sel_rd2 | sel_rd0 | sel_rd1 | sel_rd3;

// Level of the granted channel's own req line -- in_flight releases
// only once this drops, guaranteeing the controller never re-sees a
// request whose ack is still propagating back to its client.
wire issued_req_level = (issued_ch == CH_WR)  ? sdram_wr_req  :
                        (issued_ch == CH_RD2) ? sdram_rd2_req :
                        (issued_ch == CH_RD0) ? sdram_rd0_req :
                        (issued_ch == CH_RD1) ? sdram_rd1_req :
                                                sdram_rd3_req;

always @(posedge clk_sys) begin
	sdram_wr_ack  <= 1'b0;
	sdram_rd2_ack <= 1'b0;
	sdram_rd0_ack <= 1'b0;
	sdram_rd1_ack <= 1'b0;
	sdram_rd3_ack <= 1'b0;

	if (sdram_init) begin
		sdram_ready <= 1'b0;
		in_flight   <= 1'b0;
		done_seen   <= 1'b0;
		rr_last     <= RR_RD0;
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
			issued_ch <= sel_wr ? CH_WR : sel_rd2 ? CH_RD2 : sel_rd0 ? CH_RD0 : sel_rd1 ? CH_RD1 : CH_RD3;
			in_flight <= 1'b1;
			done_seen <= 1'b0;
			// Round-robin pointer: only reads rotate it (a write grant
			// doesn't consume a "turn" among the read channels, so the
			// next read arbitration still starts from wherever it left
			// off before the write interrupted).
			if (!sel_wr) rr_last <= rr_grant_ch;
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
	CH_RD2: begin sdram_rd2_data <= sd_dout_b0; sdram_rd2_data16 <= sd_burst_words[0]; sdram_rd2_data16_hi <= sd_burst_words[1]; sdram_rd2_ack <= 1'b1; end
				CH_RD0: begin sdram_rd0_data <= sd_dout_b0; sdram_rd0_ack <= 1'b1; end
				CH_RD1: begin sdram_rd1_data <= sd_dout_b0; sdram_rd1_ack <= 1'b1; end
				CH_RD3: begin sdram_rd3_data <= sd_dout_b0; sdram_rd3_ack <= 1'b1; end
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

// WP-L3's dedicated 96MHz clock domain + async CDC bridge (sdram_cdc_bridge)
// was reverted 2026-07-22 (net-negative for SDRAM bus bandwidth -- see
// rtl/pll/pll_0002.v's outclk_1 comment) -- sdram_simple runs directly on
// clk_sys again, exactly as before WP-L3.
//
// CAS_LAT=3: matches Darius's real, deployed, hardware-proven value.
// (The reference file's own default of 2 -- and briefly, widening
// TRCD_NS -- were both tried and ruled out as fixes for the 'zz'
// read-back symptom seen in sim; the real cause was an off-by-one
// in the module's own read-capture scheduling, fixed directly in
// rtl/sdram.sv rather than by tuning timing parameters here. CAS_LAT
// itself is left at this conservative, widely-supported value
// rather than tuned further against any one simulator or physical
// module.)
// WP-M: sdram_banked replaces sdram_simple. Open-row/multi-bank controller;
// bank_sel pins each client to its own physical bank so cross-client
// interleaving no longer thrashes open rows. Physical layer (init, read
// capture, DQM/word-write, registered outputs) preserved verbatim.
// Rollback: revert to sdram_simple instantiation and remove sd_bank/sd_addr_rel.
// WP-M8 (2026-07-24): BURST_LEN raised from its WP-M6 default of 1 to 2 --
// this is a SHARED instance, so every read on every channel (rd0/rd1/rd3/
// rd2's own PROM byte fetch) now captures 2 words per transaction, not
// just rd2's new GFXROW fetch. This is safe for all of them because of
// the WP-M8 step-1 dout_b0/burst_words[0] migration (see that port's
// comment in rtl/sdram_banked.sv): every single-word caller already reads
// via sd_dout_b0/sd_burst_words[0], which stays correct regardless of
// BURST_LEN. The extra word capture costs those callers a small amount of
// unused latency (READ_WAIT_LOAD grows by BURST_LEN-1 cycles) but no
// correctness risk. Only rd2's GFXROW fetch (rtl/sor_video.sv's
// FP_GFXROW_REQ/WAIT) actually uses the second word (sd_burst_words[1]).
sdram_banked #(
	.CLK_MHZ(48),
	.CAS_LAT(3),
	.BURST_LEN(2)
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

	.clk      (clk_sys),
	.rst_n    (~sdram_init),
	.addr     (sd_addr_rel),
	.bank_sel (sd_bank),
	.din      (sd_din),
	.dout     (sd_dout),
	.dout16   (sd_dout16),
	.rd       (sd_rd),
	.we       (1'b0),
	.din_hi   (sd_din_hi),
	.we_word  (sd_we_word),
	.ready    (sd_ready),
	.req_done (sd_req_done),
	.req_hit  (sd_req_hit),
	.burst_words(sd_burst_words),
	.dout_b0    (sd_dout_b0)
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
//
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
// Single index="0" session (post-16-byte-header, see leland_board_pkg):
//   master ROM: ADDR_MASTER_BASE+, 256 KB used
//   slave  ROM: ADDR_SLAVE_BASE+,  ~576 KB used
//   sound  ROM: ADDR_SOUND_BASE+,  1 MB used (sparse)
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
// Region routing now goes through rtl/leland_board_pkg.sv's canonical
// layout constants (WP-L1) instead of hand-kept local ADDR_* offsets.
// The upper bound admitting Sound ROM through the same gate/FIFO/drain
// pipeline as Master/Slave is ADDR_GFX_BASE, same rationale as before
// this rewrite: sound ROM shares this pipeline's exact destination
// (SDRAM, flat-address==SDRAM-address post-header-subtract, same
// even/odd pairing) with Master/Slave, unlike GFX/PROM (which land in
// BRAM via their own independent gate below).
//------------------------------------------------------------------

// ioctl_addr, one clk_sys cycle delayed. Declared here (ahead of its
// first use just below) rather than down near the write-path logic
// it was originally added for -- tools that require strict
// declare-before-use (e.g. vlog) rejected the forward reference.
reg [26:0] ioctl_addr_d1;
always @(posedge clk_sys) ioctl_addr_d1 <= ioctl_addr;

//------------------------------------------------------------------
// 16-byte MRA header parse (plan section 3): the first HDR_LEN bytes
// of the ioctl_index==0 stream are the header, not ROM content.
// sdram_addr is only meaningful once ioctl_addr_d1 has advanced past
// the header.
//------------------------------------------------------------------
wire ioctl_wr_hdr = ioctl_wr && ioctl_download && (ioctl_index[7:0] == 8'h00) &&
                    (ioctl_addr_d1 < HDR_LEN[26:0]);

reg [7:0] hdr_board_class_raw;
reg [7:0] hdr_game_id;
reg [7:0] hdr_input_scheme_raw;
reg [7:0] hdr_flags;

always @(posedge clk_sys) begin
	if (ioctl_wr_hdr) begin
		case (ioctl_addr_d1[3:0])
			HDR_OFF_BOARD_CLASS[3:0]:  hdr_board_class_raw  <= ioctl_data;
			HDR_OFF_GAME_ID[3:0]:      hdr_game_id           <= ioctl_data;
			HDR_OFF_INPUT_SCHEME[3:0]: hdr_input_scheme_raw  <= ioctl_data;
			HDR_OFF_FLAGS[3:0]:        hdr_flags             <= ioctl_data;
			default: ; // magic/version/reserved: not consumed by the loader
		endcase
	end
end

// board_id drives a real runtime mux from day one (plan section 9,
// "SNK lesson"): board_class selects the write-gate's upper bound via
// a genuine case statement, even though every populated game table row
// currently resolves to the same GEN3_LELANDI value. Later WPs (L3+)
// add board classes/games whose branches genuinely diverge here.
board_class_e board_class_r;
assign board_class_r = board_class_e'(hdr_board_class_raw);

// WP-L3: game_id (header byte 3) indexes the package's per-game config
// table (leland_board_pkg::game_cfg) for I/O port bases / input scheme /
// flags, rather than deriving them only from the raw header bytes. The
// header's own board_class/input_scheme/flags bytes are kept as sanity-
// check values (plan section 3: "loader can sanity-check the MRA against
// the RBF's table"); the table is authoritative and drives real muxes.
leland_board_pkg::game_cfg_t game_cfg_r;
assign game_cfg_r = leland_board_pkg::game_cfg(hdr_game_id);

wire [7:0] io_base_r    = game_cfg_r.io_base;
wire [7:0] mvram_base_r = game_cfg_r.mvram_base;
wire       dual_io_window_r = game_cfg_r.flags[leland_board_pkg::FLAG_DUAL_IO_WINDOW];
wire       in4_port_en_r    = game_cfg_r.flags[leland_board_pkg::FLAG_IN4_PORT];
leland_board_pkg::input_scheme_e input_scheme_r;
assign input_scheme_r = game_cfg_r.input_scheme;

// WP-L2: gfx/prom tile ROMs moved from BRAM to SDRAM (see the rd2
// arbiter channel above and sor_video.sv's fetch FSM) -- they are now
// "just more SDRAM content" routed through the same wfifo->SDRAM-write
// pipeline as master/slave/sound, so the write-gate's upper bound moves
// out from ADDR_GFX_BASE to ADDR_PROM_REAL_HI (real populated content
// only, not the full PROM_MAX reservation -- matches the old BRAM
// write-gate's own real-content bound, same rationale, see
// ADDR_GFX_REAL_HI/ADDR_PROM_REAL_HI below).
localparam [26:0] ADDR_GFX_REAL_HI  = ADDR_GFX_BASE  + 27'h018000;
localparam [26:0] ADDR_PROM_REAL_HI = ADDR_PROM_BASE + 27'h020000;
// WP-L3: the MRA now carries the per-game EEPROM default image (128
// bytes = 64 x 16-bit words) at ADDR_EEPROM_BASE, consumed by the boot
// FSM below. Write-gate upper bound extends out to cover it -- real
// content only, same "real, not full reservation" convention as
// ADDR_GFX_REAL_HI/ADDR_PROM_REAL_HI above.
localparam [26:0] ADDR_EEPROM_REAL_HI = ADDR_EEPROM_BASE + 27'h000080;

logic [26:0] wr_gate_hi;
always @(*) begin
	case (board_class_r)
		GEN3_LELANDI: wr_gate_hi = ADDR_EEPROM_REAL_HI;
		default:      wr_gate_hi = ADDR_EEPROM_REAL_HI;
	endcase
end

// sdram_addr = ioctl_addr - HDR_LEN, valid once past the header.
wire [26:0] sdram_addr = ioctl_addr_d1 - HDR_LEN[26:0];

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
wire ioctl_wr_rom = ioctl_wr && ioctl_download && (ioctl_index[7:0] == 8'h00) &&
                    (ioctl_addr_d1 >= HDR_LEN[26:0]) && (sdram_addr < wr_gate_hi);

// DIP switches (ioctl_index 254): capture instead of discarding --
// the game will eventually need them on an input port; for now this
// also documents the index-254 stream's existence explicitly.
reg [63:0] dips;
always @(posedge clk_sys) begin
	if (ioctl_wr && ioctl_download && (ioctl_index[7:0] == 8'hFE) && (ioctl_addr_d1 < 27'd8))
		dips[ioctl_addr_d1[2:0]*8 +: 8] <= ioctl_data;
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
reg        latched_ioctl_slave; // 1 = sdram_addr >= ADDR_SLAVE_BASE, i.e. NOT Master ROM
                                 // (Slave OR Sound ROM -- name kept for history; not
                                 // consumed downstream, kept for debug visibility only)

// Master (0x000000+) and Slave/Sound (ADDR_SLAVE_BASE+) ROM share this
// pipeline in the flat post-header address space AND are numerically identical
// to their SDRAM addresses -- no per-region offset math needed at all,
// unlike the old ioctl_index-keyed version. Straight pass-through.
// Renamed to _ioctl (final sdram_wr_* muxed against the gfx-repack FSM
// further below, after wr_pending/dl_settled are in scope).
wire [24:0] sdram_wr_addr_ioctl    = latched_ioctl_addr[24:0];
wire  [7:0] sdram_wr_data_ioctl    = latched_ioctl_data;
wire  [7:0] sdram_wr_data_hi_ioctl = latched_ioctl_data_hi;

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
// {addr[22:0], data[7:0]}: 23 address bits (widened this WP from 22 --
// WP-L2 moves the write-gate upper bound out to ADDR_PROM_REAL_HI=
// 0x620000, which needs 23 bits: 0x61FFFF (highest real address below
// the gate) is 0x61FFFF < 0x800000 (2^23) but > 0x3FFFFF (2^22), so the
// old 22-bit field (max representable 0x3FFFFF, WP-L1's bound) would
// silently wrap/alias anything at or above 0x400000 -- exactly the gfx/
// prom range this WP newly routes through this FIFO. This is the same
// class of bug that once caused real missing-playfield-graphics
// corruption on hardware (see WP-L1's own widening comment history);
// checked carefully here for the same reason. Field holds sdram_addr
// (post-header-subtract), not ioctl_addr_d1. The master/slave/sound/
// gfx/prom region split is derived from the address at drain time, no
// separate per-region flag needed now that addressing is flat.
reg [30:0] wfifo [0:(1<<WFIFO_AW)-1];
reg [WFIFO_AW:0] wfifo_wptr, wfifo_rptr;    // extra bit for full/empty
wire [WFIFO_AW:0] wfifo_level = wfifo_wptr - wfifo_rptr;
wire [WFIFO_AW-1:0] wfifo_rptr_p1 = wfifo_rptr[WFIFO_AW-1:0] + 1'b1; // odd entry of a pair
wire wfifo_empty = (wfifo_level == 0);
wire wfifo_full  = wfifo_level[WFIFO_AW];

// Sticky overflow flag: should be impossible with the half-full wait
// threshold; if it ever sets, bytes were lost and the load is known
// bad (kept for future debug visibility).
reg wfifo_overflow;

// Enqueue: every ROM strobe, one cycle, no busy check. Uses sdram_addr
// (ioctl_addr_d1 - HDR_LEN) -- hardware probes proved raw and delayed
// addr are identical at strobe time, so the d1-derived value is safe
// and unchanged from the proven-correct capture timing.
always @(posedge clk_sys) begin
	if (sdram_init) begin
		wfifo_wptr     <= '0;
		wfifo_overflow <= 1'b0;
	end else if (ioctl_wr_rom) begin
		if (!wfifo_full) begin
			wfifo[wfifo_wptr[WFIFO_AW-1:0]] <= {sdram_addr[22:0], ioctl_data};
			wfifo_wptr <= wfifo_wptr + 1'd1;
		end else begin
			wfifo_overflow <= 1'b1;
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
	end
	else if (sdram_wr_ack) begin
		wr_pending <= 1'b0;
	end
	else if (!wr_pending && (wfifo_level >= 2)) begin
		wr_pending            <= 1'b1;
		// Even entry (low byte) -- its address is the word address.
		// [30:8] = the 23-bit address field (widened this WP, see
		// wfifo's own declaration comment above). This is sdram_addr,
		// i.e. already past the 16-byte header subtract.
		latched_ioctl_addr    <= {4'b0, wfifo[wfifo_rptr[WFIFO_AW-1:0]][30:8]};
		latched_ioctl_slave   <= (wfifo[wfifo_rptr[WFIFO_AW-1:0]][30:8] >= ADDR_SLAVE_BASE[22:0]);
		latched_ioctl_data    <= wfifo[wfifo_rptr[WFIFO_AW-1:0]][7:0];
		// Odd entry (high byte) -- guaranteed to be rptr+1.
		latched_ioctl_data_hi <= wfifo[wfifo_rptr_p1][7:0];
		wfifo_rptr            <= wfifo_rptr + 2'd2;
	end
end

// ioctl_wait: assert while SDRAM init is pending, and once the FIFO
// reaches half full. NOT per-byte -- the ARM reacts to this in
// software with real latency, so it must be an early warning with
// slack behind it (8 free slots), not a hard stop. Per-byte wait
// (the old `wr_pending | ~sdram_ready`) throttled the stream to the
// software polling rate at best and lost bytes at worst.
assign ioctl_wait   = (wfifo_level >= (1<<(WFIFO_AW-1))) | ~sdram_ready;

//------------------------------------------------------------------
// Wider-reads bandwidth optimization (2026-07-22, post-hardware-bringup
// sanity check -- see docs/planning_video_sdram_prefetch.md): builds a
// repacked COPY of bg_gfx planes 0+1, interleaved into 16-bit words
// (word i = {plane1[i], plane0[i]}), at ADDR_GFXW_BASE. The real
// ADDR_GFX_BASE content stays byte-for-byte exactly as loaded (still
// matching the MRA/MAME ROM_LOAD layout) -- this is a derived cache
// built AFTER loading, not a relayout of the load itself. Lets
// sor_video.sv's fetch FSM read both bitplane bytes needed per tile-row
// in one 16-bit SDRAM transaction instead of two 8-bit ones, directly
// cutting rd2 bus demand rather than fighting over arbiter priority
// (which the same investigation found to be zero-sum against CPU/sound
// -- see the rd2_fetch_busy/aging-boost comment above).
//
// One-shot boot-time FSM. Runs after dl_settled (all real ROM loading
// finished and settled) and before video_release, borrowing the rd2 and
// wr arbiter channels: rd2 is guaranteed idle in this window because
// sor_video stays held in reset until video_release, which THIS FSM
// gates (repack_done, see video_release below) so it cannot start
// requesting rd2 until repack has already finished with it; wr is
// guaranteed idle because wr_pending is driven off the ioctl write-back
// FIFO, which drains continuously at SDRAM pace throughout the download
// and is empty well before dl_settled's extra DL_SETTLE_CYCLES margin
// elapses (checked explicitly below anyway, belt-and-suspenders). No
// real per-cycle arbitration needed between repack and the ioctl
// loader/sor_video -- just a mux, since the two are mutually exclusive
// in time by construction.
localparam [16:0] REPACK_LEN = 17'h8000; // one plane's worth of bytes

// WP-M8 (2026-07-24): extended with RP_RD2_REQ/WAIT (fetch plane2) and
// RP_WR2_REQ/WAIT (write the ADDR_GFXROW_BASE combined 4-byte entry) --
// purely additive alongside the original RD0/RD1/WR sequence, which still
// builds ADDR_GFXW_BASE exactly as before. See leland_board_pkg.sv's
// ADDR_GFXROW_BASE comment for the entry layout.
typedef enum logic [3:0] {
	RP_IDLE, RP_RD0_REQ, RP_RD0_WAIT, RP_RD1_REQ, RP_RD1_WAIT,
	RP_RD2_REQ, RP_RD2_WAIT,
	RP_WR_REQ, RP_WR_WAIT, RP_WR2_REQ, RP_WR2_WAIT,
	RP_WR3_REQ, RP_WR3_WAIT, RP_DONE
} repack_state_e;

repack_state_e repack_st;
reg [16:0] repack_idx;
reg  [7:0] repack_b0;
reg  [7:0] repack_b2; // plane2 byte, latched at RP_RD2_WAIT for the GFXROW word1 write
reg        repack_done;

reg        repack_rd_req_r;
reg [24:0] repack_rd_addr_r;
reg        repack_wr_req_r;
reg [24:0] repack_wr_addr_r;
reg  [7:0] repack_wr_data_r, repack_wr_data_hi_r;

wire repack_active = (repack_st != RP_IDLE) && (repack_st != RP_DONE);

always @(posedge clk_sys) begin
	if (sdram_init) begin
		repack_st       <= RP_IDLE;
		repack_idx      <= 17'd0;
		repack_done     <= 1'b0;
		repack_rd_req_r <= 1'b0;
		repack_wr_req_r <= 1'b0;
	end else begin
		case (repack_st)
			RP_IDLE: if (dl_settled && !wr_pending) repack_st <= RP_RD0_REQ;

			// plane0[idx] -- ADDR_GFX_BASE + idx (u93, the first 32KB third)
			RP_RD0_REQ: begin
				repack_rd_addr_r <= ADDR_GFX_BASE[24:0] + {8'b0, repack_idx};
				repack_rd_req_r  <= 1'b1;
				repack_st        <= RP_RD0_WAIT;
			end
			RP_RD0_WAIT: if (sdram_rd2_ack) begin
				repack_b0       <= sdram_rd2_data;
				repack_rd_req_r <= 1'b0;
				repack_st       <= RP_RD1_REQ;
			end

			// plane1[idx] -- ADDR_GFX_BASE + 0x8000 + idx (u94, the second third)
			RP_RD1_REQ: begin
				repack_rd_addr_r <= ADDR_GFX_BASE[24:0] + 25'h008000 + {8'b0, repack_idx};
				repack_rd_req_r  <= 1'b1;
				repack_st        <= RP_RD1_WAIT;
			end
			RP_RD1_WAIT: if (sdram_rd2_ack) begin
				repack_wr_data_r    <= repack_b0;      // low byte  = plane0
				repack_wr_data_hi_r <= sdram_rd2_data;  // high byte = plane1
				repack_rd_req_r     <= 1'b0;
				repack_st           <= RP_RD2_REQ;
			end

			// plane2[idx] -- ADDR_GFX_BASE + 0x10000 + idx (u95, the third third)
			RP_RD2_REQ: begin
				repack_rd_addr_r <= ADDR_GFX_BASE[24:0] + 25'h010000 + {8'b0, repack_idx};
				repack_rd_req_r  <= 1'b1;
				repack_st        <= RP_RD2_WAIT;
			end
			RP_RD2_WAIT: if (sdram_rd2_ack) begin
				repack_b2       <= sdram_rd2_data;
				repack_rd_req_r <= 1'b0;
				repack_st       <= RP_WR_REQ;
			end

			// combined word -> ADDR_GFXW_BASE + idx*2 (unchanged from before)
			RP_WR_REQ: begin
				repack_wr_addr_r <= ADDR_GFXW_BASE[24:0] + {repack_idx, 1'b0};
				repack_wr_req_r  <= 1'b1;
				repack_st        <= RP_WR_WAIT;
			end
			RP_WR_WAIT: if (sdram_wr_ack) begin
				repack_wr_req_r <= 1'b0;
				repack_st       <= RP_WR2_REQ;
			end

			// GFXROW word0 = {plane1,plane0} (same content as the GFXW write
			// above) at ADDR_GFXROW_BASE + idx*4. repack_wr_data_hi_r already
			// holds plane1 from RP_RD1_WAIT and is untouched since -- only
			// repack_wr_data_r (plane0) needs re-latching here since it may
			// have been overwritten by the time we get here (it isn't, but
			// re-latching from repack_b0 keeps this state self-contained
			// rather than relying on that fact).
			RP_WR2_REQ: begin
				repack_wr_addr_r <= ADDR_GFXROW_BASE[24:0] + {repack_idx, 2'b00};
				repack_wr_data_r <= repack_b0; // plane0
				repack_wr_req_r  <= 1'b1;
				repack_st        <= RP_WR2_WAIT;
			end
			RP_WR2_WAIT: if (sdram_wr_ack) begin
				repack_wr_req_r <= 1'b0;
				repack_st       <= RP_WR3_REQ;
			end

			// GFXROW word1 = {8'h00, plane2} at ADDR_GFXROW_BASE + idx*4 + 2.
			RP_WR3_REQ: begin
				repack_wr_addr_r    <= ADDR_GFXROW_BASE[24:0] + {repack_idx, 2'b00} + 25'd2;
				repack_wr_data_r    <= repack_b2;  // low byte  = plane2
				repack_wr_data_hi_r <= 8'h00;      // high byte = padding
				repack_wr_req_r     <= 1'b1;
				repack_st           <= RP_WR3_WAIT;
			end
			RP_WR3_WAIT: if (sdram_wr_ack) begin
				repack_wr_req_r <= 1'b0;
				if (repack_idx == REPACK_LEN - 17'd1) begin
					repack_st   <= RP_DONE;
					repack_done <= 1'b1;
				end else begin
					repack_idx  <= repack_idx + 17'd1;
					repack_st   <= RP_RD0_REQ;
				end
			end

			default: ; // RP_DONE: parked here for the rest of time
		endcase
	end
end

//------------------------------------------------------------------
// WP-L3: per-game EEPROM default-content load. Runs once, after the
// gfx repack FSM finishes (repack_done), borrowing the same rd2 channel
// (repack and this FSM are never active at the same time, so this is a
// simple mutually-exclusive extension of the mux below, same pattern as
// repack borrowing rd2/wr from sor_video/the ioctl loader). Reads the
// 128-byte MRA-delivered image at ADDR_EEPROM_BASE (big-endian words,
// hi byte first, matching sor_eeprom_93c46's eeprom_data[o*2+0]=hi/
// [o*2+1]=lo convention) and writes all 64 words into the eeprom
// module's mem[] before the CPUs are released.
//------------------------------------------------------------------
typedef enum logic [2:0] {
	EE_IDLE, EE_RD_HI_REQ, EE_RD_HI_WAIT, EE_RD_LO_REQ, EE_RD_LO_WAIT, EE_WR, EE_DONE
} ee_state_e;

ee_state_e ee_st;
reg  [5:0] ee_idx;
reg  [7:0] ee_hi;
reg        ee_done;

reg        ee_rd_req_r;
reg [24:0] ee_rd_addr_r;
reg        ee_mem_wr_r;
reg  [5:0] ee_mem_wr_addr_r;
reg [15:0] ee_mem_wr_data_r;

wire ee_active = (ee_st != EE_IDLE) && (ee_st != EE_DONE);

always @(posedge clk_sys) begin
	ee_mem_wr_r <= 1'b0;
	if (sdram_init) begin
		ee_st       <= EE_IDLE;
		ee_idx      <= 6'd0;
		ee_done     <= 1'b0;
		ee_rd_req_r <= 1'b0;
	end else begin
		case (ee_st)
			EE_IDLE: if (repack_done) ee_st <= EE_RD_HI_REQ;

			EE_RD_HI_REQ: begin
				ee_rd_addr_r <= ADDR_EEPROM_BASE[24:0] + {18'b0, ee_idx, 1'b0};
				ee_rd_req_r  <= 1'b1;
				ee_st        <= EE_RD_HI_WAIT;
			end
			EE_RD_HI_WAIT: if (sdram_rd2_ack) begin
				ee_hi       <= sdram_rd2_data;
				ee_rd_req_r <= 1'b0;
				ee_st       <= EE_RD_LO_REQ;
			end

			EE_RD_LO_REQ: begin
				ee_rd_addr_r <= ADDR_EEPROM_BASE[24:0] + {18'b0, ee_idx, 1'b0} + 25'd1;
				ee_rd_req_r  <= 1'b1;
				ee_st        <= EE_RD_LO_WAIT;
			end
			EE_RD_LO_WAIT: if (sdram_rd2_ack) begin
				ee_mem_wr_data_r <= {ee_hi, sdram_rd2_data};
				// Latch the destination word index HERE, together with the
				// data, while ee_idx still holds the current word. ee_mem_wr_r
				// is a registered pulse that only lands the cycle AFTER EE_WR,
				// by which point EE_WR has already advanced ee_idx to idx+1.
				// Pairing the pulse with a live `ee_idx` wrote mem[idx+1] <=
				// data[idx] (whole image shifted up one word; mem[0] never
				// written, mem[62] clobbered) -- corrupting the EEPROM. Using
				// this latched, pre-increment index writes mem[idx] <= data[idx].
				ee_mem_wr_addr_r <= ee_idx;
				ee_rd_req_r      <= 1'b0;
				ee_st            <= EE_WR;
			end

			EE_WR: begin
				ee_mem_wr_r <= 1'b1;
				if (ee_idx == 6'd63) begin
					ee_st   <= EE_DONE;
					ee_done <= 1'b1;
				end else begin
					ee_idx <= ee_idx + 6'd1;
					ee_st  <= EE_RD_HI_REQ;
				end
			end

			default: ; // EE_DONE: parked here for the rest of time
		endcase
	end
end

wire        eeprom_mem_wr      = ee_mem_wr_r;
wire  [5:0] eeprom_mem_wr_addr = ee_mem_wr_addr_r;
wire [15:0] eeprom_mem_wr_data = ee_mem_wr_data_r;

// Final muxes: repack and the EEPROM loader borrow rd2/wr while active
// (mutually exclusive -- EE_IDLE only advances once repack_done);
// otherwise the channels behave exactly as before (sor_video's own
// request / the ioctl loader's own write), unchanged.
assign sdram_rd2_req  = repack_active ? repack_rd_req_r  : (ee_active ? ee_rd_req_r  : sdram_rd2_req_v);
assign sdram_rd2_addr = repack_active ? repack_rd_addr_r : (ee_active ? ee_rd_addr_r : sdram_rd2_addr_v);

assign sdram_wr_req      = repack_active ? repack_wr_req_r     : wr_pending;
assign sdram_wr_addr     = repack_active ? repack_wr_addr_r    : sdram_wr_addr_ioctl;
assign sdram_wr_data     = repack_active ? repack_wr_data_r    : sdram_wr_data_ioctl;
assign sdram_wr_data_hi  = repack_active ? repack_wr_data_hi_r : sdram_wr_data_hi_ioctl;

//------------------------------------------------------------------
// Graphics / palette ROMs -- WP-L2: moved off on-chip BRAM (was the
// binding M10K utilization constraint) onto SDRAM, at their canonical
// leland_board_pkg addresses (ADDR_GFX_BASE/ADDR_PROM_BASE). They are
// loaded through the same wfifo->SDRAM-write pipeline as master/slave/
// sound now (see wr_gate_hi above) and fetched by sor_video's fetch_ph
// FSM through the rd2 arbiter channel declared near the top of this
// file. No BRAM arrays, no dedicated video read port, and no special-
// cased ioctl write block remain here -- gfx/prom are "just more SDRAM
// content" from this module's point of view.
//------------------------------------------------------------------

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

sor_dpram #(.ADDR_WIDTH(12), .DATA_WIDTH(8)) wram_m
(
	.clk(clk_sys),
	.addr_a(wram_addr_m), .din_a(wram_din_m), .we_a(wram_we_m), .dout_a(wram_dout_m),
	.addr_b(12'd0), .din_b(8'd0), .we_b(1'b0), .dout_b()
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
	.do_out(eeprom_do),

	.mem_wr(eeprom_mem_wr),
	.mem_wr_addr(eeprom_mem_wr_addr),
	.mem_wr_data(eeprom_mem_wr_data)
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

// WP-M7 (docs/planning_sdram_multichannel.md §12): read-only line cache in
// front of rd0 (master Z80 code fetch) -- replaces the old bare ack_hold
// wrapper (still visible in git history) with rom_line_cache, which *is*
// that wrapper plus a cache. See rtl/rom_line_cache.sv's header comment for
// the design (256 lines x 8 bytes, direct-mapped, sequential single-word
// refill -- deliberately not using WP-M6 burst yet, see that file).
wire master_rom_stall;
// 2026-07-26 sizing, from the MR/MA overlay telemetry this cache now feeds:
// at the default INDEX_BITS=8 this was 256 lines x 8 bytes = 2KB, DIRECT
// MAPPED over a 256KB (18-bit) ROM space -- 128 distinct regions aliasing
// onto every line. Hardware measured a 5.0-5.9% miss rate in gameplay, and
// because each miss costs a flat ~12 CE_6M ticks (8 single-byte SDRAM reads,
// see rom_line_cache.sv's fill FSM), master stall tracked MS = 12 x MR to
// within 5% across a 6x swing in scene load -- i.e. stall is purely miss
// COUNT, not contention. INDEX_BITS=11 gives 2048 lines (16KB) and drops the
// aliasing to 16 regions per line; the tag narrows 7->4 bits as the index
// grows, so the cost is ~141kbit rather than 8x the original.
rom_line_cache #(
	.BASE       (leland_board_pkg::ADDR_MASTER_BASE),
	.ADDR_WIDTH (18),
	.INDEX_BITS (11)
) rd0_cache (
	.clk_sys      (clk_sys),
	.reset        (sdram_init),
	.sdram_ready  (sdram_ready),

	.cpu_req      (master_rom_req),
	.cpu_addr     (master_rom_addr_w),
	.cpu_data     (master_rom_data_r),
	.cpu_stall    (master_rom_stall),

	.sd_req       (sdram_rd0_req),
	.sd_addr      (sdram_rd0_addr),
	.sd_data      (sdram_rd0_data),
	.sd_ack       (sdram_rd0_ack),

	// 2026-07-26: wired to the debug overlay at last. rom_line_cache's header
	// says to "start small, grow only if hit-rate telemetry justifies it" --
	// this is that telemetry. Hardware shows master stall (MS) scaling 6x
	// from level 1 (2.4% of CPU cycles) to a heavy level-3 scene (14.9%
	// peak); since VRAM is on-chip BRAM and cannot contend for SDRAM, the
	// remaining candidate is this cache missing more often as the master
	// ranges over more code, so measure it directly rather than infer.
	.access_pulse (mrom_access_pulse),
	.hit_pulse    (mrom_hit_pulse)
);

//------------------------------------------------------------------
// Master Z80
//------------------------------------------------------------------
wire [15:0] vid_addr_m;
wire        vid_addr_wr_m;
wire [15:0] scroll_x_m, scroll_y_m;
wire  [7:0] gfxbank_m;

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
	// repack_done added (2026-07-22, wider-reads): sor_video must not
	// start requesting rd2 until the gfx-repack FSM above has finished
	// borrowing it, since the repack mux only routes rd2/wr to sor_video/
	// the ioctl loader when it's inactive -- releasing video any earlier
	// would let the two collide on the same channel.
	//
	// WP-L3: the EEPROM load FSM (ee_st) ALSO borrows rd2, sequenced
	// right after repack_done, but is deliberately NOT a video_release
	// gate condition here (unlike repack_done) -- measured in sim at
	// ~29us total (64 words x ~450ns/word), it's short enough that
	// blocking the whole boot handshake on it measurably delayed
	// cpu_release and, on hardware, pushed the 80186 sound board's own
	// boot handshake past its timeout window (silent audio, confirmed
	// regression on real hardware 2026-07-24). If sor_video's fetch FSM
	// happens to request rd2 during that ~29us window while ee_active,
	// the mux below masks its request out (ee_active ? ee_rd_req_r :
	// sdram_rd2_req_v) -- a handful of dropped/delayed gfx-fetch acks,
	// well within the fetch FSM's existing variable-latency/cache-miss
	// tolerance, not a correctness issue. EEPROM content itself is only
	// consumed later in boot (checksum/service-mode reads), long after
	// this negligible window closes.
	else if (!video_release && sdram_ready && dl_settled && repack_done && (ce_z80_cnt == 3'd0))
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

	.io_base(io_base_r),
	.mvram_base(mvram_base_r),
	.dual_io_window(dual_io_window_r),
	.in4_port_en(in4_port_en_r),
	.input_scheme(input_scheme_r),

	.p1_joy(p1_joy),
	.p2_joy(p2_joy),
	.p3_joy(p3_joy),
	.p4_joy(p4_joy),

	.rom_req  (master_rom_req),
	.rom_stall(master_rom_stall),

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

// WP-M7 line cache for rd1 (slave Z80 code fetch) -- see rd0_cache above
// for the design and rtl/rom_line_cache.sv for the module itself.
wire slave_rom_stall;
wire slave_vport_stall; // debug-overlay only, from sor_slave.vport_stall_out
// Same treatment as rd0_cache above, and this one was worse off: 2KB direct
// mapped over 512KB is 256 regions aliasing per line. Slave stall (SS) ran
// 2.7-5.2% with a 7.8% peak on the same captures that drove the master
// change.
rom_line_cache #(
	.BASE       (leland_board_pkg::ADDR_SLAVE_BASE),
	.ADDR_WIDTH (19),
	.INDEX_BITS (11)
) rd1_cache (
	.clk_sys      (clk_sys),
	.reset        (sdram_init),
	.sdram_ready  (sdram_ready),

	.cpu_req      (slave_rom_req),
	.cpu_addr     (slave_rom_addr_w),
	.cpu_data     (slave_rom_data_r),
	.cpu_stall    (slave_rom_stall),

	.sd_req       (sdram_rd1_req),
	.sd_addr      (sdram_rd1_addr),
	.sd_data      (sdram_rd1_data),
	.sd_ack       (sdram_rd1_ack),

	.access_pulse (),
	.hit_pulse    ()
);


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

	.vport_stall_out(slave_vport_stall)
);

//------------------------------------------------------------------
// Sound board (WP10 of docs/planning_80186_sound.md): real s80x86
// Core + i186_periph (WP2-4, WP8 DMA) + leland_sound_board (WP6) +
// leland_dac_mixer (WP7), bundled in rtl/sor_sound.sv.
//------------------------------------------------------------------
wire        sound_rom_req;
wire [19:0] sound_rom_addr_w;
reg   [7:0] sound_rom_data_r;

// WP-M7 line cache for rd3 (80186 sound CPU code fetch) -- see rd0_cache
// above for the design and rtl/rom_line_cache.sv for the module itself.
wire sound_rom_stall;
// Sound (80186) has never been instrumented, so this one is sized on the
// same reasoning rather than on measurement: 2KB direct mapped over 1MB is
// 512 regions per line, the worst ratio of the three. Its misses share the
// same SDRAM, so cutting them reduces pressure on master/slave regardless of
// whether sound itself is ever stall-bound.
rom_line_cache #(
	.BASE       (leland_board_pkg::ADDR_SOUND_BASE),
	.ADDR_WIDTH (20),
	.INDEX_BITS (11)
) rd3_cache (
	.clk_sys      (clk_sys),
	.reset        (sdram_init),
	.sdram_ready  (sdram_ready),

	.cpu_req      (sound_rom_req),
	.cpu_addr     (sound_rom_addr_w),
	.cpu_data     (sound_rom_data_r),
	.cpu_stall    (sound_rom_stall),

	.sd_req       (sdram_rd3_req),
	.sd_addr      (sdram_rd3_addr),
	.sd_data      (sdram_rd3_data),
	.sd_ack       (sdram_rd3_ack),

	.access_pulse (),
	.hit_pulse    ()
);


// SIM_NO_SOUND: stub out the 80186 sound core entirely for the fast
// master/slave-only ModelSim cross-check harness (the s80x86 Core is too
// large to simulate at practical wall-clock speed under ModelSim; rd1/rd3
// never request during the boot window under investigation anyway --
// confirmed via probe, see docs/SESSION_2026-07-23_BOOT_HANG.md point 6).
// Default (undefined) path is completely unchanged -- synthesis and the
// normal Verilator harness still instantiate the real core.
`ifndef SIM_NO_SOUND
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

	.audio_out(audio_out)
);
`else
assign sound_response_data = 8'h00;
assign sound_rom_req       = 1'b0;
assign sound_rom_addr_w    = '0;
assign audio_out           = 16'h0;
`endif

//------------------------------------------------------------------
// Video system
//------------------------------------------------------------------
// Debug-overlay per-frame event pulses (2026-07-25, Pig Out slow-motion
// investigation instrumentation, see docs/SESSION_2026-07-24_PIGOUT_
// INVESTIGATION_HANDOFF.md). All display-only; none feed back into game
// logic. sor_video.sv owns the actual per-frame accumulate/freeze/render
// (it already has VBlank locally); this module only forwards the raw
// one-tick-wide event signals it's in a position to see.
//
//   master_stall_tick/slave_stall_tick: CE_6M ticks where that CPU's
//     wait_n is held low purely by the ROM/SDRAM stall term (rom_req &
//     rom_stall) -- mirrors rtl/sor_master.sv:260's wait_n formula, ROM
//     term only. This is SDRAM-controller cost the real board never
//     pays. It deliberately excludes the VRAM-mailbox stall term (see
//     slave_port_stall_tick below) -- the two are counted separately
//     because only the ROM term is purely our cost; the real board pays
//     a VRAM-port wait too.
//   slave_port_stall_tick: CE_6M ticks where the slave is held in /WAIT
//     by its own VRAM port (vport_stall), regardless of ROM stall.
//     NOT purely our cost -- real hardware/MAME also blocks the slave
//     here (see sor_vram_port.sv's vp_stall comment) -- included for
//     comparison, not as a bug indicator on its own.
//   slave_vram_write_tick: one pulse per completed slave-side VRAM port
//     WRITE op (vp_pop_s with vp_rd_s low). This is a proxy for "how
//     much drawing work the slave is doing this frame", not a byte
//     count -- ops 1/2 (see sor_vram_port.sv header) each perform two
//     physical byte writes but are counted once here, at op-completion
//     granularity, which is what's cheaply visible at this port.
//   rd2_grant_tick: one pulse per SDRAM arbitration cycle admitting a
//     rd2 (video gfx) transaction (sel_rd2 above, already exactly
//     one-cycle-wide by construction of the accept FSM).
wire master_stall_tick      = CE_6M && master_rom_req && master_rom_stall;
wire slave_stall_tick       = CE_6M && slave_rom_req  && slave_rom_stall;
wire slave_port_stall_tick  = CE_6M && slave_vport_stall;
wire slave_vram_write_tick  = vp_pop_s && !vp_rd_s;
wire rd2_grant_tick         = sel_rd2;
//   mrom_access_tick / mrom_miss_tick: master ROM line-cache completions and
//     the subset that missed (rd0_cache above). Every miss is a real SDRAM
//     round-trip contending with the video rd2 stream, so miss COUNT is the
//     volume term and MS/miss is the contention term -- the two things the
//     stall percentage alone cannot separate.
wire mrom_access_pulse, mrom_hit_pulse;
wire mrom_access_tick       = mrom_access_pulse;
wire mrom_miss_tick         = mrom_access_pulse && !mrom_hit_pulse;

sor_video video
(
	.clk_sys(clk_sys),
	.reset(reset | ~video_release), // phase-locked video release -- see video_release/cpu_release above
	.ce_pix(ce_pix),

	.show_overlay          (show_overlay),
	.master_stall_tick     (master_stall_tick),
	.slave_stall_tick      (slave_stall_tick),
	.slave_port_stall_tick (slave_port_stall_tick),
	.slave_vram_write_tick (slave_vram_write_tick),
	.rd2_grant_tick        (rd2_grant_tick),
	.mrom_access_tick      (mrom_access_tick),
	.mrom_miss_tick        (mrom_miss_tick),

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

	.sdram_rd2_req  (sdram_rd2_req_v),
	.sdram_rd2_ack  (sdram_rd2_ack),
	.sdram_rd2_addr (sdram_rd2_addr_v),
	.sdram_rd2_data (sdram_rd2_data),
	.sdram_rd2_data16(sdram_rd2_data16),
	.sdram_rd2_data16_hi(sdram_rd2_data16_hi),
	.fetch_busy     (rd2_fetch_busy),
	.rbuf_count_out (rd2_rbuf_count),

	.raster_line(raster_line)
);

endmodule
