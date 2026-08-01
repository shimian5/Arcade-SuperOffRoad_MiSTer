//============================================================================
//  Super Off Road — Video Timing & Pixel Output
//
//  Leland hardware timing (from MAME leland_v.cpp):
//    Crystal:    14.318181 MHz
//    Pixel CLK:  7.159090 MHz  (÷2)
//    H total:    424 pixels
//    H active:   320 pixels
//    H rate:     16.88 kHz
//    V total:    256 lines
//    V active:   240 lines
//    V rate:     65.95 Hz
//
//  Color:  BGR 2-3-3 palette (8 bits per entry, 1024 entries)
//  The palette byte maps as:
//    [7:6] = Blue  (2 bits → 0..85..170..255)
//    [5:3] = Green (3 bits → 0..36..73..109..146..182..219..255)
//    [2:0] = Red   (3 bits → same as green)
//
//  In the POC (Chunks 1–2) the video RAM is empty/zeroed, so the
//  screen renders with palette entry 0 across the active area.
//  A colour-bar test pattern is overlaid so you can verify timing
//  before the CPUs are wired up.  Define SOR_TEST_PATTERN to enable.
//  Remove or leave undefined for production.
//
// SOR_TEST_PATTERN removed — real VRAM data now flowing (Chunk 3)

import leland_board_pkg::*;

module sor_video
(
	input         clk_sys,
	input         reset,
	input         ce_pix,     // pixel clock enable (~7.16 MHz from 48 MHz)

	output reg    HBlank,
	output reg    HSync,
	output reg    VBlank,
	output reg    VSync,

	// Read port into Video RAM (driven by this module)
	output [16:0] vram_addr,
	input   [7:0] vram_data,

	// Read port into Color RAM
	output  [9:0] cram_addr,
	input   [7:0] cram_data,

	// 24-bit RGB output (registered, aligned to ce_pix)
	output reg [23:0] rgb,

	// Background tilemap scroll registers (Master I/O 0x8C-0x8F/0xCC-0xCF,
	// validated against MAME leland_v.cpp scroll_w)
	input  [15:0] scroll_x,
	input  [15:0] scroll_y,

	// Graphics bank (MAME leland_state::m_gfxbank, set via the AY8910's
	// Port A write callback -- sound_port_w -> gfx_port_w in
	// leland_v.cpp/leland_m.cpp). Driven from sor_master.sv, which
	// tracks the Master's real AY8910 register-select+data write
	// sequence (I/O offsets 0x0A/0x0B) just enough to catch writes to
	// register 0x0E (I/O Port A) -- no actual AY8910 sound synthesis.
	input   [7:0] gfxbank,

	// WP-L2: gfx/prom tile ROM content lives in SDRAM now (not BRAM) --
	// this module owns the fetch_ph FSM below and drives sor_board.sv's
	// rd2 arbiter channel directly (request/ack handshake, variable
	// latency, unlike the old 1-cycle BRAM read). One shared channel,
	// time-multiplexed across the prom lookup byte and the 3 gfx
	// bitplane bytes.
	output        sdram_rd2_req,
	input         sdram_rd2_ack,
	output [24:0] sdram_rd2_addr,
	input   [7:0] sdram_rd2_data,
	// Wider-reads path (2026-07-22): the full 16-bit word the SDRAM
	// controller already captured, unmuxed -- used by FP_GFXROW_REQ/WAIT,
	// which targets the ADDR_GFXROW_BASE packed entry (see
	// rtl/sor_board.sv's gfx-repack FSM) holding plane0/plane1 in word0.
	input  [15:0] sdram_rd2_data16,
	// WP-M8 (2026-07-24): second burst word of the same GFXROW read
	// (BURST_LEN=2 on the shared sdram_ctrl) -- {8'h00, plane2}. See
	// rtl/sor_board.sv's sdram_rd2_data16_hi comment for how this is
	// captured (sd_burst_words[1] at the same ack cycle as data16).
	input  [15:0] sdram_rd2_data16_hi,

	// Whole-tile-fetch-burst-in-progress flag for sor_board.sv's rd2
	// aging/boost logic (2026-07-22): fetch_ph != FP_IDLE, i.e. true from
	// the moment a tile fetch is armed until its last sub-request (gfx
	// plane 2) is acked, spanning the brief 1-cycle req-line gaps BETWEEN
	// the burst's individual sub-requests. The arbiter's own rd2_pending
	// (req && !ack) drops during exactly those gaps, which was silently
	// resetting the aging counter/boost latch every sub-request instead
	// of once per tile -- this lets the arbiter distinguish "between
	// sub-requests of one burst" from "genuinely idle, no fetch pending".
	output        fetch_busy,

	// Ring-buffer occupancy (2026-07-22, root-cause fix for the rd2-only
	// urgency escalation's own regression): rd2_fetch_busy alone can't
	// distinguish "genuinely at risk of underrunning" from "just doing
	// its normal, near-continuous greedy refill" -- the producer re-arms
	// every cycle it has any room at all (see rbuf_has_room below), so
	// fetch_busy is true almost all the time regardless of how full the
	// buffer actually is. sor_board.sv's rd2 urgency escalation now
	// gates on THIS (buffer running low) instead of/in addition to raw
	// busy-time, so it only preempts round-robin when rd2 is actually
	// close to an underrun, not on every ordinary fetch burst.
	output  [3:0] rbuf_count_out,

	// Raster line counter output (for Slave Z80 synchronisation)
	output  [7:0] raster_line,

	//------------------------------------------------------------------
	// Debug overlay (2026-07-25, Pig Out slow-motion investigation
	// instrumentation, see docs/SESSION_2026-07-24_PIGOUT_INVESTIGATION_
	// HANDOFF.md). Display-only: renders seven per-frame hex counters
	// over the top rows of active video when show_overlay is high.
	// Touches no game logic; when show_overlay is low the whole render
	// path below reduces to dead logic feeding a mux that always picks
	// the real pixel, same shape as the pre-b01c24b overlay this
	// restores a minimal version of. Counter update/render logic lives
	// entirely in this module (it already owns VBlank locally); the
	// five *_tick inputs are one-clk_sys-wide event pulses sourced from
	// sor_board.sv, which is in the right place to see them.
	//------------------------------------------------------------------
	input         show_overlay,
	input         master_stall_tick,     // CE_6M tick, master ROM/SDRAM stall
	input         slave_stall_tick,      // CE_6M tick, slave  ROM/SDRAM stall
	input         slave_port_stall_tick, // CE_6M tick, slave VRAM-port stall
	input         mrom_access_tick,      // master ROM line-cache completion
	input         mrom_miss_tick,        // ...that completion was a miss
	input         slave_vram_write_tick, // one pulse per completed slave VRAM write op
	input         rd2_grant_tick         // one pulse per rd2 SDRAM grant
);

//------------------------------------------------------------------
// Timing counters
//------------------------------------------------------------------
// H counter: 0..423 (424 total)
//   0..319   — active pixels
//   320..423 — blanking (104 pixels)
//   HSync:   336..383 (48-pixel pulse, ~6.7 µs, ≈ 4.7 µs NTSC-ish)
//
// V counter: 0..255 (256 total)
//   0..239   — active lines
//   240..255 — blanking (16 lines)
//   VSync:   241..244 (4-line pulse, ~237 µs)

localparam H_TOTAL  = 10'd424;
localparam H_ACTIVE = 10'd320;
localparam H_SYNC_S = 10'd336;
localparam H_SYNC_E = 10'd384;

localparam V_TOTAL  = 9'd256;
localparam V_ACTIVE = 9'd240;
localparam V_SYNC_S = 9'd241;
localparam V_SYNC_E = 9'd245;

reg  [9:0] hc;   // horizontal pixel counter
reg  [8:0] vc;   // vertical line counter

always @(posedge clk_sys) begin
	if (reset) begin
		hc <= 0;
		vc <= 0;
	end else if (ce_pix) begin
		if (hc == H_TOTAL - 1'd1) begin
			hc <= 0;
			vc <= (vc == V_TOTAL - 1'd1) ? 9'd0 : vc + 1'd1;
		end else begin
			hc <= hc + 1'd1;
		end
	end
end

// REVERTED 2026-07-22: a scroll_x/scroll_y frame-latch was added here on
// the theory that mid-frame scroll writes were an unintended tearing bug.
// Checked against the actual MAME reference (docs/reference/mame/
// leland_v.cpp:156-158) before shipping it further: scroll_w() calls
// m_screen->update_partial(vpos()-1) before applying the new value --
// i.e. real Leland hardware/MAME explicitly supports and this driver
// uses mid-frame scroll changes (e.g. a HUD/status split), rendering
// everything before the write with the old value and everything after
// with the new one. That is precisely what reading scroll_x/scroll_y
// live and combinationally (no latch) already does -- the original
// behavior was correct, not a bug. Latching it to once-per-frame would
// have silently broken any real mid-frame scroll split effect the game
// uses. Reverted; do not reintroduce without first confirming the real
// visual symptom against this reference behavior, not assumption.

// Blanking and sync — combinational off counters, registered on clk_sys
always @(posedge clk_sys) begin
	if (ce_pix) begin
		HBlank <= (hc >= H_ACTIVE);
		HSync  <= (hc >= H_SYNC_S) && (hc < H_SYNC_E);
		VBlank <= (vc >= V_ACTIVE);
		VSync  <= (vc >= V_SYNC_S) && (vc < V_SYNC_E);
	end
end

//------------------------------------------------------------------
// Video RAM fetch
//
// The Leland video circuit fetches one byte per pixel during the
// active area; each byte encodes two 4-bit pen values packed as:
//   [7:4] = left/even pixel pen
//   [3:0] = right/odd pixel pen
//
// The Slave Z80 writes 8-bit bytes to the 128 KB video RAM.
// Stride = 256 bytes per line (validated against MAME leland_v.cpp y<<8).
//------------------------------------------------------------------
// VRAM addressing: validated against MAME leland_v.cpp screen_update.
// Each scanline occupies 256 bytes (stride = y<<8).  Active pixels use
// bytes 0-159 (320 pixels × 2 pixels/byte).
// addr = {buffer_0, vc[7:0], hc[8:1]}  =  vc*256 + hc/2
wire [16:0] fetch_addr = {1'b0, vc[7:0], hc[8:1]};
assign vram_addr  = fetch_addr;
assign raster_line = vc[7:0];

// Latch the fetched byte and select the correct nibble
reg  [7:0] vram_latch;
reg        hc_lsb;

always @(posedge clk_sys) begin
	if (ce_pix) begin
		vram_latch <= vram_data;
		hc_lsb     <= hc[0];
	end
end

wire [3:0] fg_pen = hc_lsb ? vram_latch[3:0] : vram_latch[7:4];

//------------------------------------------------------------------
// Background ROM tilemap — validated against MAME leland_v.cpp
// leland_get_tile_info/leland_scan/screen_update.
//
// 256x256 grid of 8x8 tiles (2048x2048 px), scrolled by scroll_x/
// scroll_y and wrapping. Tile lookup byte comes from the "bg_prom"
// PROM (our prom_rom); tile pixel data is 3 bitplanes in "bg_gfx"
// (our gfx_rom), each plane RGN_FRAC(1,3) of the ROM.
//
// One tile's lookup + 3 plane bytes are fetched a tile ahead of when
// they're displayed (armed at the START of the currently-displaying
// tile, giving 8 pixels/~54 clk_sys cycles of slack -- WP-L2: doubled
// from the original LEAD=4 mid-tile arm now that the 4 sequential
// reads go to SDRAM via sor_board.sv's rd2 arbiter channel (variable
// latency, req/ack handshake) instead of 1-cycle BRAM reads. ~54
// clk_sys cycles gives ~2x margin over the ~20-25 cycles four
// sequential 5-cycle SDRAM reads plus rd2's high-priority arbitration
// wait are expected to need).
//------------------------------------------------------------------
wire [10:0] eff_x = hc[9:0] + scroll_x[10:0];
wire [10:0] eff_y = {2'b0, vc} + scroll_y[10:0];
wire  [7:0] tile_col     = eff_x[10:3];
wire  [7:0] tile_row     = eff_y[10:3];
wire  [2:0] col_in_tile  = eff_x[2:0];
wire  [2:0] row_in_tile  = eff_y[2:0];

// leland_scan: idx = (col&0xff) | ((row&0x1f)<<8) | ((row&0xe0)<<9)
function automatic [16:0] leland_tile_idx(input [7:0] col, input [7:0] row);
	leland_tile_idx = {row[7:5], 1'b0, row[4:0], col};
endfunction

reg [3:0] fetch_ph;
// Hardware bring-up debug (2026-07-22): pulses for one clk_sys cycle every
// time the tile commit below finds the fetch STILL incomplete and holds
// the previous tile instead of committing fresh data -- every real
// "stale commit" event. Three fixes to this fetch/scheduling path
// (commit-gate, PROM re-fetch elimination) showed zero visible
// improvement on real hardware despite sim confirming the underlying
// deadline misses are real -- this + the saturating counter/corner
// overlay near the rgb output below gives a direct, real-hardware answer
// to whether this mechanism is firing at any meaningful rate in
// practice, instead of guessing at yet another fix blind.
reg [7:0] fetch_col, fetch_row;
reg [2:0] fetch_riy; // row-in-tile of the TARGET pixel, latched at arm time:
                     // a row-start fetch is armed during the previous row's
                     // blanking (vc not yet incremented), so the live
                     // row_in_tile would be one scanline stale there --
                     // mid-row arms latch the identical live value.
reg [7:0] prom_byte_next;

// PROM re-fetch elimination (2026-07-22, WP-L2 bandwidth fix -- modeled on
// Darius_MiSTer's tile-ROM cache, see rtl/darius/tile_rom_arbiter.sv): the
// PROM byte (tile selection + palette, prom_sdram_addr) is addressed
// purely by (fetch_col, fetch_row, gfxbank[3]/prom_bank) -- NOT by
// fetch_riy (row-within-tile). Since a tile spans 8 scanlines and only
// fetch_riy changes between them, the fetch FSM was re-issuing an
// identical PROM read on 7 out of every 8 scanlines for zero new
// information -- one quarter of ALL rd2 SDRAM traffic was pure waste.
// Caching the last-fetched (col,row,bank3) tuple and skipping straight to
// FP_GFX0_REQ when it still matches cuts total tile-row fetches from 4 to
// 3 for 7/8 of all fetches, with no ROM/MRA layout change and no address-
// math change -- gfxbank[5:4] (which only affects fetch_tile_code's
// post-fetch composition, not PROM addressing) is unaffected since
// fetch_tile_code always recomputes from the live gfxbank every time
// regardless of whether prom_byte_next came from a fresh fetch or the
// cache.
// 2026-07-26 REPLACEMENT: the single-entry cache above achieved a measured
// hit rate of EXACTLY ZERO on real hardware -- the overlay's PM counter read
// 0x2800 = 10240 = 40 tile columns x 256 lines, i.e. one miss per fetch, every
// fetch, all frame. The reason is structural: the fetch cursor advances 8
// pixels per arm, so tile_col_tgt (= eff_x_tgt[10:3]) INCREMENTS between
// consecutive fetches. The same tile is not revisited until 8 scanlines
// later, by which point the one slot has been overwritten ~40 times. The
// "7 out of 8 scanlines" reuse the comment above describes can only happen
// with one entry PER COLUMN, so that is what this is: a 256-entry line cache
// indexed by tile column, tagged with {gfxbank[3], row}.
//
// No invalidation path is needed, for the same reason the gfx-plane cache
// below needs none: ADDR_PROM_BASE (leland_board_pkg.sv:110) is populated
// only by the ioctl ROM loader and is read-only for the session -- the CPU
// has no write path to it, and background changes come from gfxbank banking
// (which is in the tag), not from tilemap writes. Verified before widening
// the cache, since a multi-entry cache holds entries far longer than the
// single-entry one did and would expose any staleness the old one hid by
// never hitting at all.
localparam integer PROM_LC_N = 256;
reg [16:0]           prom_lc_mem [0:PROM_LC_N-1]; // {bank3, row[7:0], byte[7:0]}
reg [16:0]           prom_lc_q;                   // registered read, valid the cycle after
reg [PROM_LC_N-1:0]  prom_lc_valid;               // flops, so reset clears every entry at once
reg [7:0] bg_color_cur;
// Named by which THIRD of bg_gfx they came from, not by plane-bit weight --
// the two are reversed here, see the bg_pen assign below.
reg [7:0] bg_third0_cur, bg_third1_cur, bg_third2_cur;

// 2026-07-22 deep-prefetch redesign (see docs/planning_video_sdram_prefetch.md):
// real hardware measured 40-79 clk_sys cycles per single SDRAM round-trip,
// and a tile-row needs up to 3 serialized round-trips -- 120-237 cycles
// against the old single-tile-ahead 54-cycle budget, a 2.2x-4.4x deficit
// that fully explains the near-100%-of-fetches miss rate seen on real
// hardware (a bring-up-only stale-commit diagnostic, since removed,
// showed this saturating within ~2 frames).
// Shaving reads (PROM cache) or gating garbage commits (commit-gate) both
// remain necessary but cannot close a gap that size -- the fetch pipeline's
// lookahead has to be several tile-widths deep, decoupled from the tight
// per-scanline commit schedule, so a fetch has genuine slack to finish even
// at bad-case real latency. Replaces the old single-slot "armed" scheme
// (fetch_armed + *_next holding regs) with an N-deep ring buffer: the
// producer (the fetch_ph FSM below) keeps it as full as it can, running
// ahead of the display's actual consumption point; the display-side commit
// just pops the next entry instead of checking a single in-flight fetch.
localparam RBUF_N = 4'd8; // ring buffer depth (2-4x margin over the worst
                          // measured 3-read/tile-row deficit above)
reg [7:0] rbuf_color [0:7];   // only [2:0] meaningful, stored as a byte for
                              // simplicity/uniformity with the other planes
reg [7:0] rbuf_third0[0:7];
reg [7:0] rbuf_third1[0:7];
reg [7:0] rbuf_third2[0:7];
reg [2:0] rbuf_wr, rbuf_rd;   // 3-bit pointers into the 8-deep buffer --
                              // wrap naturally on overflow, no modulo needed
reg [3:0] rbuf_count;         // 0..8
assign rbuf_count_out = rbuf_count;

wire [7:0] rbuf_color_rd  = rbuf_color [rbuf_rd];
wire [7:0] rbuf_third0_rd = rbuf_third0[rbuf_rd];
wire [7:0] rbuf_third1_rd = rbuf_third1[rbuf_rd];
wire [7:0] rbuf_third2_rd = rbuf_third2[rbuf_rd];
wire       rbuf_has_data  = (rbuf_count != 4'd0);
wire       rbuf_has_room  = (rbuf_count != RBUF_N);

// The producer's own walk cursor: the (hc,vc) position of the tile-row
// most recently armed/in-flight, i.e. one step behind "the next position
// to target". While the buffer is empty, the walk resyncs to the live
// display position (hc+8, same lead the old single-slot scheme used) so
// a scroll_x/scroll_y/gfxbank change is picked up as fast as possible;
// once entries are queued, each new arm continues the walk from the LAST
// queued target instead, advancing by one more 8-pixel tile-width every
// time -- this is what makes the lookahead run several tiles deep instead
// of just one. (A queued entry's tile position is therefore only as fresh
// as the scroll/gfxbank values live when IT was armed, not when it's
// finally displayed -- for a several-tile-deep queue that means a few
// tiles' worth of transient staleness right after a scroll/bank change,
// same class of tradeoff any deep prefetch makes; the old scheme had this
// too, just with one tile of depth instead of up to eight.)
reg [9:0] walk_hc;
reg [8:0] walk_vc;

// Row-boundary resync (2026-07-23, boot-hang fix): the walk cursor above
// only ever resyncs to the live hc/vc when the ring buffer is fully
// empty, which -- per the drift-tracking comment on base_hc/base_vc
// below -- is rare-to-never in steady state, so a one-time miscount
// (observed: right after the very first post-video_release fill burst)
// has no other opportunity to correct itself and persists losslessly
// forever, exactly the "walk cursor ran ~166 lines ahead of the real
// display" symptom documented there. This forces one resync per real
// display row regardless of buffer occupancy, bounding any such drift
// to under one row as that comment's own proposed fix describes, without
// touching the fine-scroll pop-timing logic that broke on the two prior
// (reverted) attempts at fixing this the other way.
reg [8:0] row_resync_vc_r;
reg       row_resync_pending;

wire [11:0] fetch_tile_code = {gfxbank[5:4], fetch_row[7:6], prom_byte_next};

// SDRAM byte addresses for this tile's 4 fetches -- same tile-code/row
// math the old BRAM-address computation used, just added onto the
// package's canonical region bases instead of driving a BRAM-local
// address. gfxbank[3] selects the PROM's bank-1 half (bit 13 of its
// offset), same as before.
wire [16:0] prom_offset = leland_tile_idx(fetch_col, fetch_row) | {3'b0, gfxbank[3], 13'b0};

wire [24:0] prom_sdram_addr = ADDR_PROM_BASE[24:0] + {8'b0, prom_offset};

// Wider-reads bandwidth optimization (2026-07-22, see docs/
// planning_video_sdram_prefetch.md and rtl/sor_board.sv's gfx-repack
// FSM): planes 0+1 are pre-interleaved into 16-bit words at
// ADDR_GFXW_BASE (word i = {plane1[i], plane0[i]}, i = tile_code*8+riy,
// exactly the same 15-bit index either raw plane used alone). Superseded
// as of WP-M8 by the ADDR_GFXROW_BASE burst read below for the actual
// fetch path, but ADDR_GFXW_BASE itself is left populated/untouched (the
// repack FSM still writes it) in case anything else ever wants a plain
// 16-bit two-plane read without pulling in plane2.
wire [14:0] gfx01_idx = {fetch_tile_code, fetch_riy};

// WP-M8 (2026-07-24, docs/planning_sdram_multichannel.md §12 "Video
// re-encoding"): single burst read of the ADDR_GFXROW_BASE packed entry
// (word0={plane1,plane0}, word1={8'h00,plane2} -- see
// leland_board_pkg.sv's ADDR_GFXROW_BASE comment) replaces the old
// separate FP_GFX01_REQ/WAIT + FP_GFX2_REQ/WAIT sequence -- one SDRAM
// transaction (BURST_LEN=2 burst, sdram_ctrl instantiated accordingly in
// rtl/sor_board.sv) instead of two, safely wired via WP-M8 step 1's
// dout_b0/burst_words[0] migration. Address is inherently BURST_LEN=2-
// column-aligned since ADDR_GFXROW_BASE and the *4 stride are both
// multiples of 4 bytes (2 words).
//
// Third time's the charm (2026-07-24): the first two attempts at this
// exact change were reverted after sim/sor_video_tb.sv + bg_reference.py
// showed a large pixel diff -- root-caused (see base_hc's own comment
// below) to a COMPLETELY UNRELATED pre-existing bug in the ring-buffer's
// row-resync logic (a 7-tile horizontal shift, present with or without
// this GFXROW change). That bug is now fixed. Re-tested clean: race/
// winners/track2 all 0.00% diff with this change applied.
wire [24:0] gfxrow_sdram_addr = ADDR_GFXROW_BASE[24:0] + {gfx01_idx, 2'b00};

// Tile-row gfx-plane cache (2026-07-22, post-hardware-bringup follow-up
// to the wider-reads fix -- see docs/planning_video_sdram_prefetch.md):
// modeled on Darius_MiSTer's tile_rom_arbiter.sv, a 256-entry direct-
// mapped cache over gfx01_idx (= {tile_code, riy}, the same 15-bit
// address wider-reads already uses), storing all 3 plane bytes for a
// tile-row together. A hit skips BOTH remaining SDRAM reads (gfx01 and
// gfx2) entirely -- not just shrinking the transaction like wider reads
// did, but eliminating it -- because this design's own gfxbank[5:4]
// bits are already folded into tile_code (see fetch_tile_code above),
// so distinct banks/tiles naturally land in distinct cache slots with
// no separate invalidation logic needed; and because bg_gfx is a
// read-only ROM for the whole session, a cached entry is valid forever
// once populated -- no coherency/staleness concern at all. Given how
// much a scrolling tilemap reuses the same handful of terrain tiles,
// this should cut rd2 traffic (and therefore its remaining pressure on
// CPU/sound bus time, per the arbiter-boost zero-sum finding) well
// below what wider reads alone achieved.
//
// M10K inference (2026-07-22 follow-up): a single 256x32 array --
// {valid(1), tag(7), b0(8), b1(8), b2(8)} -- with a plain synchronous
// write port and a REGISTERED read port (address presented one cycle,
// data captured the next, standard simple-dual-port BRAM timing), split
// into its own always block below. The first cut of this cache used 5
// separate combinationally-read arrays with a one-shot 256-wide
// synchronous clear, which is a pattern Quartus doesn't map onto M10K
// (no per-bit clear on real block RAM) -- it fell back to registers/
// ALMs (confirmed: block memory bits unchanged in the fit report, ALM
// usage jumped by ~8200, matching the array size almost exactly).
// Clearing now happens by WRITING zero to each address in turn over the
// first 256 cycles of reset instead of an all-at-once clear -- sor_video
// stays in reset for the entire ROM-load + gfx-repack boot window (far
// more than 256 clk_sys cycles), so there's no time pressure. The
// registered read adds exactly one clk_sys cycle of latency per tile
// fetch (FP_GFX_LOOKUP below) regardless of hit or miss -- negligible
// against the ~54-cycle tile budget.
localparam [31:0] GFXCACHE_INVALID = 32'd0; // valid bit is bit 31, 0 = invalid
reg [31:0] gfxcache_mem [0:255];
reg  [7:0] gfxcache_clear_idx;
reg        gfxcache_wr_en;
reg  [7:0] gfxcache_wr_addr;
reg [31:0] gfxcache_wr_data;
reg [31:0] gfxcache_rd_data_r;

wire [7:0] gfx_cache_idx = gfx01_idx[7:0];
wire [6:0] gfx_cache_tag = gfx01_idx[14:8];
wire       gfx_cache_hit  = gfxcache_rd_data_r[31] && (gfxcache_rd_data_r[30:24] == gfx_cache_tag);
wire [7:0] gfxcache_b0_rd = gfxcache_rd_data_r[23:16];
wire [7:0] gfxcache_b1_rd = gfxcache_rd_data_r[15:8];
wire [7:0] gfxcache_b2_rd = gfxcache_rd_data_r[7:0];

always @(posedge clk_sys) begin
	if (reset) begin
		gfxcache_clear_idx <= (gfxcache_clear_idx == 8'hFF) ? gfxcache_clear_idx : (gfxcache_clear_idx + 8'd1);
		gfxcache_mem[gfxcache_clear_idx] <= GFXCACHE_INVALID;
	end else begin
		gfxcache_clear_idx <= 8'd0;
		if (gfxcache_wr_en) gfxcache_mem[gfxcache_wr_addr] <= gfxcache_wr_data;
	end
	// Read port: address is the live (combinational) gfx_cache_idx every
	// cycle; only meaningful the cycle fetch_ph==FP_GFX_LOOKUP presents
	// it, captured here and consumed the FOLLOWING cycle in FP_GFX01_REQ.
	gfxcache_rd_data_r <= gfxcache_mem[gfx_cache_idx];
end

reg         sdram_rd2_req_r;
reg  [24:0] sdram_rd2_addr_r;
assign sdram_rd2_req  = sdram_rd2_req_r;
assign sdram_rd2_addr = sdram_rd2_addr_r;

// Blanking-aware fetch target (2026-07-14 bg-pipeline audit, re-derived
// 2026-07-20 for WP-L2's LEAD=8): the arm fires at col_in_tile==0
// (tile start) and its commit (col_in_tile==0 of the NEXT tile) lands
// exactly 8 hc-ticks later -- doubled from LEAD=4's mid-tile arm/4-tick
// window now that the 4 sequential fetches go to SDRAM (see the arm-
// point comment above). The original bug this logic fixes -- a naive
// "always fetch tile_col+1, same row" rule renders the leftmost
// partial tile of every scanline as stale/wrong -- and its fix are
// unchanged in kind; only the constant changes. The general rule
// (commit position hc+8, three cases) is derived purely from where
// commit_pos falls relative to H_ACTIVE/H_TOTAL, so it generalizes to
// any LEAD without re-deriving the case split itself:
//   active (hc+8 < H_ACTIVE): target hc+8 on the current row --
//     the tile that will occupy screen column hc+8 when its window
//     opens, exactly as LEAD=4 targeted hc+4.
//   blanking (H_ACTIVE <= hc+8 < H_TOTAL): the commit itself is
//     invisible; the tile's only possible visible coverage is the
//     START of the next row, so target (0, vc+1). All blanking arms
//     then redundantly refetch that same row-start tile (harmless) --
//     unchanged from LEAD=4.
//   wrapped (hc+8 >= H_TOTAL): the commit lands at hc+8-H_TOTAL on the
//     next row. H_TOTAL-H_ACTIVE=104 pixels of blanking comfortably
//     exceeds LEAD=8, so a wrapped commit can only ever land once
//     into the next row (hc max 423 + 8 = 431 < 2*H_TOTAL), same as
//     LEAD=4's single-wrap guarantee. The wrapped range widens from
//     LEAD=4's {1..3} to {0..7} (hc 416..423 -> commit 424..431 ->
//     wrapped 0..7): this now OVERLAPS the blanking case's fixed
//     target of column 0 for some scroll_x fine offsets -- both paths
//     agreeing on the same row-start tile in that overlap is harmless
//     redundancy, not a new case; the {0..7} range still resolves to
//     at most two distinct visible tiles via tile_col_tgt's own
//     eff_x_tgt[10:3] division, exactly as LEAD=4's narrower {1..3}
//     range did. No new case beyond the original three is needed.
// base_hc/base_vc: the position the NEXT arm's target is 8 pixels ahead
// of. Empty buffer -> resync to the live display position (hc), exactly
// the old scheme's lead. Non-empty -> continue from the walk cursor (the
// last-armed target), so successive arms step 8 pixels further ahead each
// time instead of all targeting the same spot.
//
// 2026-07-24 fix: row_resync_pending (added 2026-07-23 for the boot-hang
// fix) forced this SAME "hc, zero backlog" reset whenever a new display
// row starts, regardless of whether the buffer still holds queued
// entries -- but direct sim measurement (sim/sor_video_tb.sv +
// bg_reference.py showing a clean, scroll-independent 56px/7-tile
// horizontal shift, confirmed NOT a settle-timing artifact) proved the
// buffer is reliably FULL (rbuf_count==RBUF_N) at the exact instant a new
// row starts in steady state -- this producer greedily fills all the way
// to RBUF_N deep (see the walk-cursor comment above: "makes the lookahead
// run several tiles deep instead of just one"), not just 1 tile ahead.
// The row-resync's "+8" (via commit_pos below) assumed this newly-armed
// entry would be displayed almost immediately (the TRUE empty-buffer
// case's assumption) -- but it actually lands behind however many entries
// are still queued, and isn't displayed until that many pop cycles later,
// by which point the real hc has advanced rbuf_count tile-widths further
// than this entry was computed for. Confirmed numerically: measured
// rbuf_count==7 at the exact resync instant, 7*8=56 == the measured
// shift, exactly. Fix: only assume zero backlog when the buffer is
// GENUINELY empty; when row_resync_pending fires with entries still
// queued, advance past them (+8 per already-queued entry) instead of
// resetting to bare hc.
wire  [9:0] base_hc = (rbuf_has_data && !row_resync_pending) ? walk_hc :
                      rbuf_has_data ? (hc + {3'd0, rbuf_count, 3'd0}) : hc;
wire  [8:0] base_vc = (rbuf_has_data && !row_resync_pending) ? walk_vc : vc;

wire  [9:0] commit_pos      = base_hc + 10'd8;
wire        commit_wraps    = (commit_pos >= H_TOTAL);
wire        commit_in_blank = !commit_wraps && (commit_pos >= H_ACTIVE);
wire  [9:0] hc_tgt  = commit_wraps    ? (commit_pos - H_TOTAL) :
                      commit_in_blank ? 10'd0 : commit_pos;
wire  [8:0] vc_tgt  = (commit_wraps || commit_in_blank)
                      ? ((base_vc == V_TOTAL - 1'd1) ? 9'd0 : base_vc + 1'd1) : base_vc;
wire [10:0] eff_x_tgt = hc_tgt + scroll_x[10:0];
wire [10:0] eff_y_tgt = {2'b0, vc_tgt} + scroll_y[10:0];
wire  [7:0] tile_col_tgt = eff_x_tgt[10:3];
wire  [7:0] tile_row_tgt = eff_y_tgt[10:3];

// walk_hc_store: what gets latched into the walk cursor for the NEXT
// continuation step -- distinct from hc_tgt itself only in the
// commit_in_blank case. hc_tgt==0 there is correct for THIS push (it's
// screen column 0, whatever tile that scrolls to), but a non-zero
// scroll_x fine offset means the real second col_in_tile==0 boundary of
// the new row lands at hc=(8-fine), not hc=8: continuing the chain with
// a flat "0, then +8" loses the fine-offset phase from the second tile
// of every row onward (found via the finex3/5/6/7 fine-scroll-offset
// regression cases -- race/winners/track2 all use 8-aligned scroll_x
// and so never exercised this). (8-fine)&7 reduces to plain 0 when
// fine==0, so the aligned cases are unaffected.
wire [2:0] scroll_x_fine     = scroll_x[2:0];
wire [9:0] blank_wrap_next_hc = (10'd8 - {7'd0, scroll_x_fine}) & 10'd7;
wire [9:0] walk_hc_store = commit_in_blank ? blank_wrap_next_hc : hc_tgt;

// Line-cache lookup. prom_lc_q is registered off the LIVE tile_col_tgt every
// cycle, so during FP_PROM_LOOKUP (one cycle after the arm that latched
// fetch_col <= tile_col_tgt) it holds exactly mem[fetch_col]. The valid bit
// is a flop array rather than a RAM bit so reset clears all 256 at once --
// RAM contents are undefined at power-up and would otherwise false-hit.
wire prom_lc_hit = prom_lc_valid[fetch_col]
                  && (prom_lc_q[16]   == gfxbank[3])
                  && (prom_lc_q[15:8] == fetch_row);

// fetch_ph encoding: 0=idle, odd=issue request for this byte, even=wait
// for sdram_rd2_ack. Unlike the old fixed-latency BRAM scheme (address-
// set phase + dedicated wait phase + capture-on-next-phase), each REQ
// phase holds sdram_rd2_req_r high until sdram_rd2_ack arrives -- the
// SDRAM channel's latency varies with arbitration wait, so this is a
// genuine handshake, not a fixed cycle count.
localparam FP_IDLE        = 4'd0,
           FP_PROM_LOOKUP = 4'd7, // present prom_lc index to the registered-read line-cache RAM
           FP_PROM_REQ    = 4'd1, FP_PROM_WAIT   = 4'd2,
           FP_GFX_LOOKUP  = 4'd3, // present gfx_cache_idx to the registered-read cache RAM this cycle
           FP_GFXROW_REQ  = 4'd4, // gfxcache_rd_data_r now valid -- check hit/miss here
           FP_GFXROW_WAIT = 4'd5, // WP-M8: single burst read replaces old FP_GFX01+FP_GFX2 pair
           FP_GFXCACHED   = 4'd6; // gfx_cache_hit at FP_GFXROW_REQ -- push next cycle, no SDRAM needed

assign fetch_busy = (fetch_ph != FP_IDLE);

// Ring-buffer push/pop: a fetch completes (last gfx byte acked) and a
// display tile-boundary wants the next entry, evaluated independently
// every clk_sys cycle -- can coincide (net buffer occupancy unchanged)
// without conflict since each drives its own pointer.
//
// fifo_pop_req is gated to hc<H_ACTIVE (2026-07-22, found via sim
// pixel-diff regression after the initial ring-buffer draft showed
// ~90%+ diff on every case): col_in_tile==0 keeps firing roughly every
// 8 pixels through the ~104-pixel HBlank tail too (~13 extra events per
// scanline), not just across the 320 active pixels. The OLD single-slot
// scheme could ignore this -- each of those extra events re-armed off
// LIVE hc, which was still inside blanking every time, so they all
// harmlessly re-targeted the exact same (0, vc+1) tile over and over.
// This ring buffer's producer instead chains forward from its own walk
// cursor once armed, so as soon as that cursor's first post-wrap step
// lands on hc_tgt==0 (< H_ACTIVE), every following continuation step is
// the ACTIVE case again -- each of those ~13 "redundant" blanking pops
// was draining one genuinely-distinct, already-produced future tile
// instead of a harmless repeat, so the producer kept re-filling ~13
// tiles further into the NEXT row every single scanline. That's a
// runaway compounding drift (confirmed in sim: the walk cursor's target
// row ran ~166 lines ahead of the real display within under a third of
// a frame) with no self-correction, since the buffer never actually
// empties in steady state. Restricting pops to the 320 real active
// pixels (hc==0,8,...,312 -- exactly the 40 tiles/row that get
// displayed) makes exactly one pop happen per one push needed, keeping
// the walk cursor's lead bounded by the buffer depth as intended.
// KNOWN LIMITATION (2026-07-22, tracked for follow-up): a non-8-aligned
// scroll_x fine offset means a row touches 41 distinct tiles, not 40
// (both screen edges can show a partial tile), and this pop gate as
// written only fires 40 times/row -- the leftmost partial tile's own
// col_in_tile==0 crossing lands during the PREVIOUS row's HBlank (hc>=
// H_ACTIVE), which is deliberately excluded below (see comment) to fix
// a worse bug. Two same-session attempts to also pop once at hc==0 (to
// pick up that 41st tile) were tried and reverted:
//   1st attempt broke the previously-clean 8-aligned cases
//      (switch_race_to_winners regressed from 0.00% to 68% diff).
//   2nd attempt (tagging each ring-buffer entry with its real fetch_col/
//      fetch_row and comparing against ground truth at pop time, rather
//      than inferring correctness from hc/vc alone) proved the ALIGNED
//      cases broke too, and precisely why: at the exact frame-boundary
//      instant, the buffer's front entries were still tiles from ~33
//      columns further into a PRIOR pass over the SAME row (col=74..80
//      instead of the expected 41..47) -- a genuine, non-transient
//      several-tile phase lag between the walk cursor's logical position
//      and the FIFO's actual front, NOT a same-cycle race as first
//      suspected: aggregate push/pop counts balance exactly over any
//      window (confirmed 9839/9839 across a full captured frame), so
//      whatever caused the lag happened once, early, and then persisted
//      losslessly rather than growing or self-correcting. Root cause not
//      yet isolated further (candidate: the walk cursor only ever
//      resyncs to live hc/vc when the buffer is fully empty -- rare in
//      steady state -- so a one-time miscount near reset/warm-up has no
//      other opportunity to correct itself; a fix likely needs the
//      producer to resync to live hc/vc once per ROW, not just on empty,
//      bounding any such lag to under one row instead of letting it
//      persist indefinitely). Net effect of leaving this unresolved: the
// four scroll-register values actually captured from real MAME gameplay
// (race/winners/track2/switch, all 8-aligned scroll_x) are byte-exact
// (0.00% diff, sim/sor_video_tb.sv + sim/bg_reference.py); the
// synthetic fine-offset stress cases (finex3/5/6/7) went from ~94% diff
// (pre-fix) to ~3% diff (leftmost ~13 px/row wrong, everything else
// correct) -- a large improvement, not a full fix. Real hardware scroll
// deltas will hit non-aligned values during normal play, so this must
// be resolved (or explicitly accepted as a narrow, small-glitch
// regression against the pre-WP-L2 BRAM baseline) before calling this
// done -- do not remove this comment until finex3/5/6/7 also hit 0.00%.
// fifo_push covers both ways a tile-row finishes: the normal SDRAM
// completion (FP_GFXROW_WAIT's ack) and a gfx-cache hit (FP_GFXCACHED,
// one cycle after FP_GFXROW_REQ found gfx_cache_hit, no SDRAM wait
// needed at all).
wire fifo_push    = ((fetch_ph == FP_GFXROW_WAIT) && sdram_rd2_ack) || (fetch_ph == FP_GFXCACHED);
wire fifo_pop_req = ce_pix && (col_in_tile == 3'd0) && (hc < H_ACTIVE);
wire fifo_pop     = fifo_pop_req && rbuf_has_data;

//------------------------------------------------------------------
// Debug-overlay instrumentation, part 1: events only this module can
// see directly (prom_cache_valid/fetch_ph are private to the fetch
// FSM below). Two per-tile events:
//
//   prom_cache_miss_ev: fires the same cycle a new tile-row fetch is
//     armed and prom_cache_hit was false, i.e. a real cache miss on
//     the (col,row,gfxbank[3]) key at rtl/sor_video.sv:276-278's
//     prom_cache_valid/_col/_row/_bank3. NOTE (2026-07-25 amendment):
//     this cache's key does not include tile index, so ordinary
//     background content changing frame-to-frame does NOT invalidate
//     it -- only col/row/gfxbank[3] changing does. This counter tests
//     the gfxbank-cycling variant of the starvation hypothesis, not
//     the original (weaker) one.
//   tile_deadline_miss: fires when a complete tile-row fetch (PROM +
//     GFXROW, i.e. arm-to-fifo_push) took more than TILE_DEADLINE_
//     CYCLES clk_sys cycles -- mirrors sim/sor_board_tb.sv's
//     TILE_FETCH_DEADLINE_MISS (RD2_DEADLINE_CYCLES=54) so the two
//     numbers are directly comparable to earlier sim measurements.
//------------------------------------------------------------------
wire fetch_arm = (fetch_ph == FP_IDLE) && rbuf_has_room;
// Evaluated one cycle later than the arm now (the line-cache RAM read is
// registered), so this fires in FP_PROM_LOOKUP rather than at arm time --
// still exactly one event per fetch, so PM stays directly comparable to the
// pre-line-cache readings.
wire prom_cache_miss_ev = (fetch_ph == FP_PROM_LOOKUP) && !prom_lc_hit;

localparam [8:0] TILE_DEADLINE_CYCLES = 9'd54;
reg [8:0] fetch_cyc_cnt;
reg       fetch_active;
wire      tile_deadline_miss = fifo_push && fetch_active && (fetch_cyc_cnt > TILE_DEADLINE_CYCLES);

always @(posedge clk_sys) begin
	if (reset) begin
		fetch_cyc_cnt <= 9'd0;
		fetch_active  <= 1'b0;
	end else begin
		if (fetch_arm) begin
			fetch_active  <= 1'b1;
			fetch_cyc_cnt <= 9'd0;
		end else if (fetch_active && !fifo_push) begin
			fetch_cyc_cnt <= (fetch_cyc_cnt == 9'h1FF) ? fetch_cyc_cnt : fetch_cyc_cnt + 9'd1;
		end
		if (fifo_push) fetch_active <= 1'b0;
	end
end

`ifndef ALTERA_RESERVED_QIS
// Sim-only (2026-07-26): direct hit-rate measurement for the line cache.
// Inferring the hit rate from total rd2 traffic proved unreliable -- measure
// it at the source instead.
integer dbg_arm_cnt = 0, dbg_lc_hit = 0, dbg_lc_miss = 0;
always @(posedge clk_sys) begin
	// !reset is essential: during the long ROM-load reset the FSM is pinned
	// in FP_IDLE with rbuf_has_room true, so fetch_arm is high EVERY cycle
	// and an unmasked count reads ~6.2M against ~19k real fetches.
	if (fetch_arm && !reset) dbg_arm_cnt <= dbg_arm_cnt + 1;
	if (fetch_ph == FP_PROM_LOOKUP) begin
		if (prom_lc_hit) dbg_lc_hit  <= dbg_lc_hit  + 1;
		else             dbg_lc_miss <= dbg_lc_miss + 1;
	end
end
final $display("%m PROM_LC_SUMMARY: arms=%0d hits=%0d misses=%0d", dbg_arm_cnt, dbg_lc_hit, dbg_lc_miss);
`endif

// PROM line cache RAM. Read address is the LIVE tile_col_tgt every cycle, so
// prom_lc_q is always "the entry for whatever column was targeted last
// cycle" -- which during FP_PROM_LOOKUP is precisely fetch_col. Fill fires on
// the same FP_PROM_WAIT ack that latches prom_byte_next, so fetch_col /
// fetch_row / gfxbank[3] / sdram_rd2_data are all live and consistent here.
wire prom_lc_fill = (fetch_ph == FP_PROM_WAIT) && sdram_rd2_ack;

always @(posedge clk_sys) begin
	if (prom_lc_fill) prom_lc_mem[fetch_col] <= {gfxbank[3], fetch_row, sdram_rd2_data};
	prom_lc_q <= prom_lc_mem[tile_col_tgt];
end

always @(posedge clk_sys) begin
	if (reset)             prom_lc_valid <= '0;
	else if (prom_lc_fill) prom_lc_valid[fetch_col] <= 1'b1;
end

always @(posedge clk_sys) begin
	if (reset) begin
		fetch_ph         <= FP_IDLE;
		sdram_rd2_req_r  <= 1'b0;
		rbuf_wr          <= 3'd0;
		rbuf_rd          <= 3'd0;
		rbuf_count       <= 4'd0;
		gfxcache_wr_en   <= 1'b0;
		row_resync_vc_r    <= 9'd0;
		row_resync_pending <= 1'b1;
	end else begin
		gfxcache_wr_en   <= 1'b0;

		// Row-boundary resync tracker: arm a forced live-hc/vc resync the
		// moment vc changes (a new display row has started), so the next
		// producer arm below uses reality instead of blindly continuing
		// the walk cursor -- see the row_resync_pending declaration
		// comment above for why this is needed.
		if (vc != row_resync_vc_r) begin
			row_resync_vc_r    <= vc;
			row_resync_pending <= 1'b1;
		end

		// Producer: keep the ring buffer as full as possible, decoupled
		// from ce_pix/col_in_tile -- runs every clk_sys cycle whenever the
		// FSM is idle and there's room, instead of arming exactly once per
		// tile-display-window like the old single-slot scheme.
		if ((fetch_ph == FP_IDLE) && rbuf_has_room) begin
			fetch_col <= tile_col_tgt;
			fetch_row <= tile_row_tgt;
			fetch_riy <= eff_y_tgt[2:0];
			walk_hc   <= walk_hc_store;
			walk_vc   <= vc_tgt;
			row_resync_pending <= 1'b0;
			// PROM re-fetch elimination: always spend one cycle in
			// FP_PROM_LOOKUP so the line-cache RAM read (registered off
			// tile_col_tgt this cycle) is valid to test next cycle. That one
			// cycle replaces a full SDRAM round-trip on every hit -- measured
			// at 40-79 clk_sys cycles apiece -- so it pays for itself many
			// times over even at a modest hit rate.
			fetch_ph  <= FP_PROM_LOOKUP;
		end

		// Consumer: pop the next queued tile-row at the start of its
		// display window. An empty buffer (real underrun -- should now be
		// rare given the deep lookahead) falls back to holding the
		// previous tile, same benign failure mode the old commit-gate
		// introduced.
		if (fifo_pop_req && rbuf_has_data) begin
			bg_color_cur   <= rbuf_color_rd[7:5];
			bg_third0_cur  <= rbuf_third0_rd;
			bg_third1_cur  <= rbuf_third1_rd;
			bg_third2_cur  <= rbuf_third2_rd;
		end

		if (fifo_push && !fifo_pop) rbuf_count <= rbuf_count + 4'd1;
		else if (!fifo_push && fifo_pop) rbuf_count <= rbuf_count - 4'd1;
		if (fifo_push) rbuf_wr <= rbuf_wr + 3'd1;
		if (fifo_pop)  rbuf_rd <= rbuf_rd + 3'd1;

		case (fetch_ph)
			FP_PROM_LOOKUP: begin
				// prom_lc_q/prom_lc_valid are valid now. A hit reuses the
				// cached PROM byte and skips the SDRAM round-trip entirely.
				if (prom_lc_hit) begin
					prom_byte_next <= prom_lc_q[7:0];
					fetch_ph       <= FP_GFX_LOOKUP;
				end else begin
					fetch_ph       <= FP_PROM_REQ;
				end
			end
			FP_PROM_REQ: begin
				sdram_rd2_addr_r <= prom_sdram_addr;
				sdram_rd2_req_r  <= 1'b1;
				fetch_ph         <= FP_PROM_WAIT;
			end
			FP_PROM_WAIT: if (sdram_rd2_ack) begin
				prom_byte_next   <= sdram_rd2_data;
				// line-cache fill happens in the RAM block below, off the
				// same condition, so it uses the live sdram_rd2_data.
				sdram_rd2_req_r  <= 1'b0;
				fetch_ph         <= FP_GFX_LOOKUP;
			end
			FP_GFX_LOOKUP: begin
				// fetch_tile_code/fetch_riy (via gfx01_idx) are valid this
				// cycle regardless of which arm path got here (prom cache
				// hit: prom_byte_next was already valid; fresh PROM fetch:
				// updated last cycle in FP_PROM_WAIT). Nothing to do but
				// let one cycle pass -- gfx_cache_idx is being presented to
				// the registered-read cache RAM (see its always block)
				// right now, so gfxcache_rd_data_r/gfx_cache_hit are valid
				// starting next cycle, in FP_GFXROW_REQ.
				fetch_ph <= FP_GFXROW_REQ;
			end
			FP_GFXROW_REQ: begin
				// A hit needs no SDRAM access at all -- straight to a
				// one-cycle push.
				if (gfx_cache_hit) begin
					fetch_ph <= FP_GFXCACHED;
				end else begin
					sdram_rd2_addr_r <= gfxrow_sdram_addr;
					sdram_rd2_req_r  <= 1'b1;
					fetch_ph         <= FP_GFXROW_WAIT;
				end
			end
			FP_GFXROW_WAIT: if (sdram_rd2_ack) begin
				// WP-M8: one BURST_LEN=2 transaction returns both packed
				// words at once -- word0 (sdram_rd2_data16) = {plane1,
				// plane0}, word1 (sdram_rd2_data16_hi) = {8'h00, plane2} --
				// see leland_board_pkg.sv's ADDR_GFXROW_BASE comment. Both
				// words are valid the same ack cycle (burst_words[0]/[1]
				// captured back-to-back by sdram_banked, see its WP-M6/M8
				// comments), so the whole tile-row can be pushed directly,
				// no separate gfx0_next/gfx1_next latch needed any more.
				rbuf_color [rbuf_wr] <= prom_byte_next;
				rbuf_third0[rbuf_wr] <= sdram_rd2_data16[7:0];
				rbuf_third1[rbuf_wr] <= sdram_rd2_data16[15:8];
				rbuf_third2[rbuf_wr] <= sdram_rd2_data16_hi[7:0];
				// Populate the gfx-plane cache for next time this exact
				// (tile_code, riy) is needed -- gfx_cache_idx/tag are still
				// valid here (same fetch_tile_code/fetch_riy this whole
				// burst). bg_gfx is a read-only ROM for the session, so
				// this entry never goes stale once written -- no
				// invalidation path needed.
				gfxcache_wr_en   <= 1'b1;
				gfxcache_wr_addr <= gfx_cache_idx;
				gfxcache_wr_data <= {1'b1, gfx_cache_tag, sdram_rd2_data16[7:0], sdram_rd2_data16[15:8], sdram_rd2_data16_hi[7:0]};
				sdram_rd2_req_r      <= 1'b0;
				fetch_ph             <= FP_IDLE;
			end
			FP_GFXCACHED: begin
				// gfx_cache_hit fired last cycle at FP_GFXROW_REQ -- push
				// straight from the cache, no SDRAM involved for this
				// tile-row at all (fifo_push covers this state, see above).
				rbuf_color [rbuf_wr] <= prom_byte_next;
				rbuf_third0[rbuf_wr] <= gfxcache_b0_rd;
				rbuf_third1[rbuf_wr] <= gfxcache_b1_rd;
				rbuf_third2[rbuf_wr] <= gfxcache_b2_rd;
				fetch_ph             <= FP_IDLE;
			end
			default: ;
		endcase
	end
end

// Bit order within a tile row byte: MAME's leland_layout gfx_layout uses
// xoffset = STEP8(0,1) = {0,1,...,7}. MAME's gfx decode (gfxdecode.cpp /
// drawgfxm.h) interprets an xoffset value N as bit position N counted
// from the MSB -- i.e. pixel column x reads bit (7-x) of the byte, so
// column 0 (leftmost) is the byte's MSB (bit 7), not bit 0. The
// straight `[col_in_tile]` index below was LSB-first (column 0 = bit
// 0), which mirrors every 8-pixel tile horizontally: 2026-07-13
// hardware run showed the demo-race track rendering with correct tile
// placement/color but scrambled fine detail within tiles -- exactly
// what a per-tile horizontal mirror looks like at a glance (gross
// shapes made of many tiles stay recognizable; texture within each
// 8px tile is flipped).
//
// Pipeline alignment (2026-07-14 bg-pipeline audit): the bit index
// must use a ONE-ce_pix-DELAYED col_in_tile, not the live one. Every
// other pixel-path signal is one ce_pix behind hc -- HBlank/VBlank
// latch from the pre-edge hc/vc, and the fg path's vram_latch/hc_lsb
// pair latches the byte for pixel hc at the edge that advances hc --
// so during the hc==N ce window the screen is actually presenting
// pixel N-1. The tile commit above (pre-edge col_in_tile==0, taking
// effect from the following ce window onward) already matches that
// convention; the live col_in_tile did not. The mismatch showed up in
// the isolated-TB reference diff (sim/bg_reference.py) as exactly one
// wrong pixel per tile, at every tile's FIRST column: during that
// window the planes still hold the OLD tile (the commit lands at the
// window's closing edge) while the live bit index has already wrapped
// to 7, so the screen showed the PREVIOUS tile's pixel 0 -- a
// 1px-wide vertical streak every 8 pixels, scroll-phase-locked (the
// "faint vertical streaks" hardware symptom), plus a 1px bg-vs-fg
// lateral misalignment. Delaying the bit index restores pixel-exact
// alignment with both the commit timing and the fg/blanking pipeline.
reg [2:0] col_in_tile_d;
always @(posedge clk_sys) if (ce_pix) col_in_tile_d <= col_in_tile;

// Plane-bit weights are REVERSED relative to bg_gfx's ROM order, and this is
// deliberate. MAME's leland_layout (leland_v.cpp) lists its planeoffsets
// ASCENDING -- { RGN_FRAC(0,3), RGN_FRAC(1,3), RGN_FRAC(2,3) } -- while
// gfx_element::decode() weights them `planebit = 1 << (planes-1-plane)`, i.e.
// planeoffset[0] is the MOST significant plane. So the FIRST third of bg_gfx
// (u93) is pixel bit 2 and the LAST third (u95) is pixel bit 0. (The common
// MAME idiom for first-frac-is-LSB is to list fracs DESCENDING, as
// gfx_8x8x6_planar does; leland does not.)
//
// This used to read {..., bit2, bit1, bit0} straight down the ROM order, which
// bit-reversed every background pixel value: 0/2/5/7 survive, but 1<->4 and
// 3<->6 swap. Confirmed empirically against MAME (2026-07-16): in offroad's
// palette the dirt is pen v=3 (145,72,0) brown and v=6 (109,109,170) lavender,
// and the FPGA rendered the track's dirt lavender where MAME renders it brown
// -- exactly the 3<->6 swap. It only showed on the background because the game
// programs every fg!=0 pen to the foreground colour regardless of these bits,
// so all opaque Slave-drawn art (panel, logo, cars) masked the bug.
wire bg_third0 = bg_third0_cur[3'd7 - col_in_tile_d];  // u93 -> pixel bit 2 (MSB)
wire bg_third1 = bg_third1_cur[3'd7 - col_in_tile_d];  // u94 -> pixel bit 1
wire bg_third2 = bg_third2_cur[3'd7 - col_in_tile_d];  // u95 -> pixel bit 0 (LSB)
wire [5:0] bg_pen = {bg_color_cur[2:0], bg_third0, bg_third1, bg_third2};

//------------------------------------------------------------------
// Palette lookup
//
// Composite pen: background is d0-d5 (ROM tilemap), foreground is
// d6-d9 (direct video_ram bitmap) -- validated against MAME
// leland_v.cpp screen_update(). Color RAM has 1024 entries indexed
// by this combined 10-bit pen.
//
// BGR 2-3-3 decode:
//   R = {cram_data[2:0], cram_data[2:0], cram_data[2:1]}  (8-bit)
//   G = {cram_data[5:3], cram_data[5:3], cram_data[5:4]}
//   B = {cram_data[7:6], cram_data[7:6], cram_data[7:6], cram_data[7:6]}
//------------------------------------------------------------------
assign cram_addr = {fg_pen, bg_pen};

// BGR 2-3-3 → RGB 8-8-8 expansion
// R[7:0] = {cram[2:0], cram[2:0], cram[2:1]}  (3-bit → 8-bit, MSB-replicated)
// G[7:0] = {cram[5:3], cram[5:3], cram[5:4]}
// B[7:0] = {cram[7:6], cram[7:6], cram[7:6], cram[7:6]}
wire [7:0] col_r = {cram_data[2:0], cram_data[2:0], cram_data[2:1]};
wire [7:0] col_g = {cram_data[5:3], cram_data[5:3], cram_data[5:4]};
wire [7:0] col_b = {cram_data[7:6], cram_data[7:6], cram_data[7:6], cram_data[7:6]};

//------------------------------------------------------------------
// Debug overlay (2026-07-25) -- see the port-list comment above for
// scope/intent. Part 2: per-frame accumulate/freeze/render.
//
// Each of the seven counters below is a free-running accumulator that
// counts events during the current frame, then is copied into a
// separate "display" register at the VBlank rising edge (so the
// on-screen digits are stable for a whole frame instead of visibly
// ticking mid-scan) and the accumulator is cleared for the next frame.
// All accumulators saturate at 16'hFFFF rather than wrapping, so a
// screen reading FFFF means "at least that many", never a silently
// wrapped small number.
//------------------------------------------------------------------
reg        ov_vblank_prev;
wire       ov_vblank_rise = VBlank && !ov_vblank_prev;

`define OV_ACC_TICK(acc, ev) acc <= (acc == 16'hFFFF) ? acc : (acc + ((ev) ? 16'd1 : 16'd0))
// Restart-of-frame load: clears the accumulator while still counting an
// event landing on the VBlank-rise cycle itself, so no tick is dropped at
// the frame boundary. (2026-07-26: the VBlank branch previously re-used
// OV_ACC_TICK here, so the accumulators were never cleared -- every
// counter was a lifetime total and six of the seven pinned at the FFFF
// saturation within seconds on hardware, hiding all per-frame structure.)
`define OV_ACC_LOAD(acc, ev) acc <= ((ev) ? 16'd1 : 16'd0)

reg [15:0] frame_cnt;
reg [15:0] mstall_acc,  mstall_disp;   // #2 master ROM/SDRAM stall ticks
reg [15:0] sstall_acc,  sstall_disp;   // #6 slave  ROM/SDRAM stall ticks
reg [15:0] mracc_acc,   mracc_disp;    // master ROM cache accesses (MA)
reg [15:0] mrmiss_acc,  mrmiss_disp;   // master ROM cache misses   (MR)
reg [15:0] vwr_acc,     vwr_disp;      // #7 slave VRAM write ops
reg [15:0] fcnt_acc,    fcnt_disp;     // tile-row fetches armed -- denominator for PM/TD
reg [15:0] tdl_acc,     tdl_disp;      // #4 tile-fetch deadline misses
reg [15:0] pmiss_acc,   pmiss_disp;    // #5 PROM cache misses

// Peak hold (2026-07-26): the per-frame values move far too fast to read off
// a moving screen -- the only readings we have came from single-frame
// screenshots. These latch the worst per-frame value seen and hold it, so a
// slow section can be played through and the peak read afterwards at leisure.
// Cleared whenever the overlay is toggled OFF, so flipping it off and back on
// at the OSD re-arms the measurement for the next run.
// pmiss_peak retired 2026-07-26: the PROM line cache settled it (PM tracks
// FC/8 exactly on hardware), so the peak slot is better spent on the master
// ROM cache miss count, which is the open question.
reg [15:0] mstall_peak, sstall_peak, mrmiss_peak;
reg        ov_show_prev;

always @(posedge clk_sys) begin
	if (reset) begin
		ov_vblank_prev <= 1'b0;
		frame_cnt   <= 16'd0;
		mstall_acc  <= 16'd0; mstall_disp <= 16'd0;
		sstall_acc  <= 16'd0; sstall_disp <= 16'd0;
		mracc_acc   <= 16'd0; mracc_disp  <= 16'd0;
		mrmiss_acc  <= 16'd0; mrmiss_disp <= 16'd0;
		vwr_acc     <= 16'd0; vwr_disp    <= 16'd0;
		fcnt_acc    <= 16'd0; fcnt_disp   <= 16'd0;
		tdl_acc     <= 16'd0; tdl_disp    <= 16'd0;
		pmiss_acc   <= 16'd0; pmiss_disp  <= 16'd0;
		mstall_peak <= 16'd0; sstall_peak <= 16'd0; mrmiss_peak <= 16'd0;
		ov_show_prev <= 1'b0;
	end else begin
		ov_vblank_prev <= VBlank;
		ov_show_prev   <= show_overlay;
		if (!show_overlay && ov_show_prev) begin
			mstall_peak <= 16'd0; sstall_peak <= 16'd0; mrmiss_peak <= 16'd0;
		end else if (ov_vblank_rise) begin
			if (mstall_acc > mstall_peak)  mstall_peak <= mstall_acc;
			if (sstall_acc > sstall_peak)  sstall_peak <= sstall_acc;
			if (mrmiss_acc > mrmiss_peak)  mrmiss_peak <= mrmiss_acc;
		end
		if (ov_vblank_rise) begin
			frame_cnt   <= frame_cnt + 16'd1;
			mstall_disp <= mstall_acc; `OV_ACC_LOAD(mstall_acc, master_stall_tick);
			sstall_disp <= sstall_acc; `OV_ACC_LOAD(sstall_acc, slave_stall_tick);
			mracc_disp  <= mracc_acc;  `OV_ACC_LOAD(mracc_acc,  mrom_access_tick);
			mrmiss_disp <= mrmiss_acc; `OV_ACC_LOAD(mrmiss_acc, mrom_miss_tick);
			vwr_disp    <= vwr_acc;    `OV_ACC_LOAD(vwr_acc,    slave_vram_write_tick);
			fcnt_disp   <= fcnt_acc;   `OV_ACC_LOAD(fcnt_acc,   fetch_arm);
			tdl_disp    <= tdl_acc;    `OV_ACC_LOAD(tdl_acc,    tile_deadline_miss);
			pmiss_disp  <= pmiss_acc;  `OV_ACC_LOAD(pmiss_acc,  prom_cache_miss_ev);
		end else begin
			`OV_ACC_TICK(mstall_acc, master_stall_tick);
			`OV_ACC_TICK(sstall_acc, slave_stall_tick);
			`OV_ACC_TICK(mracc_acc,  mrom_access_tick);
			`OV_ACC_TICK(mrmiss_acc, mrom_miss_tick);
			`OV_ACC_TICK(vwr_acc,    slave_vram_write_tick);
			`OV_ACC_TICK(fcnt_acc,   fetch_arm);
			`OV_ACC_TICK(tdl_acc,    tile_deadline_miss);
			`OV_ACC_TICK(pmiss_acc,  prom_cache_miss_ev);
		end
	end
end

`undef OV_ACC_TICK
`undef OV_ACC_LOAD

// 5x7 font, one glyph per 5-bit character code -- minimal subset
// recovered/extended from the pre-b01c24b overlay (commit b01c24b^:
// rtl/sor_video.sv), NOT the byte-test/self-test machinery around it
// (deliberately not restored, per the investigation handoff). Codes
// 10-15 double as both hex A-F and the literal letters used below
// (D/E/F), same idiom the original font used.
function automatic [34:0] font_glyph;
	input [4:0] ch;
	begin
		case (ch)
			5'd0 : font_glyph = {5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
			5'd1 : font_glyph = {5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
			5'd2 : font_glyph = {5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
			5'd3 : font_glyph = {5'b01110,5'b10001,5'b00001,5'b00110,5'b00001,5'b10001,5'b01110};
			5'd4 : font_glyph = {5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
			5'd5 : font_glyph = {5'b11111,5'b10000,5'b11110,5'b00001,5'b00001,5'b10001,5'b01110};
			5'd6 : font_glyph = {5'b00110,5'b01000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
			5'd7 : font_glyph = {5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
			5'd8 : font_glyph = {5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
			5'd9 : font_glyph = {5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00010,5'b01100};
			5'd10: font_glyph = {5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001}; // A
			5'd11: font_glyph = {5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110}; // B
			5'd12: font_glyph = {5'b01110,5'b10001,5'b10000,5'b10000,5'b10000,5'b10001,5'b01110}; // C
			5'd13: font_glyph = {5'b11100,5'b10010,5'b10001,5'b10001,5'b10001,5'b10010,5'b11100}; // D
			5'd14: font_glyph = {5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111}; // E
			5'd15: font_glyph = {5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000}; // F
			5'd16: font_glyph = {5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000}; // P
			5'd18: font_glyph = {5'b00000,5'b00000,5'b00000,5'b11111,5'b00000,5'b00000,5'b00000}; // -
			5'd19: font_glyph = {5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001}; // R
			5'd20: font_glyph = {5'b01111,5'b10000,5'b10000,5'b01110,5'b00001,5'b00001,5'b11110}; // S
			5'd21: font_glyph = {5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100}; // T
			5'd22: font_glyph = {5'b01110,5'b10001,5'b10000,5'b10111,5'b10001,5'b10001,5'b01111}; // G
			5'd23: font_glyph = {5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001}; // M
			5'd24: font_glyph = {5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100}; // V
			5'd25: font_glyph = {5'b10001,5'b10001,5'b10001,5'b10101,5'b10101,5'b10101,5'b01010}; // W
			5'd26: font_glyph = {5'b00000,5'b00100,5'b00000,5'b00000,5'b00000,5'b00100,5'b00000}; // :
			default: font_glyph = 35'd0; // 17 = space, and unused codes
		endcase
	end
endfunction

// Three 8px-tall text rows across the top of active video (vc 0..23),
// 8px-wide character cells, priority order per the investigation
// handoff (most to least valuable): VRAM writes, master stall, slave
// stall, frame counter, PROM misses, tile deadline misses, rd2 grants,
// plus the optional slave port-stall as an 8th if room allows (it did).
//   Row0 (vc  0- 7): "VW:xxxx MS:xxxx SS:xxxx"  per-frame
//   Row1 (vc  8-15): "FR:xxxx PM:xxxx TD:xxxx"  per-frame (FR free-runs)
//   Row2 (vc 16-23): "FC:xxxx MR:xxxx MA:xxxx"  per-frame (FC = PM/TD denom,
//                    MR/MA = master ROM line-cache miss/access)
//   Row3 (vc 24-31): "PM:xxxx PS:xxxx PR:xxxx"  peak hold since overlay on
//                    (peak master stall / slave stall / master ROM misses)
localparam [8:0] OV_ROWS = 9'd32;
wire        in_overlay_area = show_overlay && (vc < OV_ROWS) && (hc < H_ACTIVE);
wire [1:0]  ov_row_sel = vc[4:3];     // 0/1/2/3 selects which text row
wire [5:0]  ov_char_idx = hc[8:3];    // 0..39, 8px per char cell
wire [2:0]  ov_row_y    = vc[2:0];
wire [2:0]  ov_col      = hc[2:0];

reg [4:0] ov_ch;
always @(*) begin
	case (ov_row_sel)
		2'd0: case (ov_char_idx) // "VW:xxxx MS:xxxx SS:xxxx"
			6'd0 : ov_ch = 5'd24; // V
			6'd1 : ov_ch = 5'd25; // W
			6'd2 : ov_ch = 5'd26; // :
			6'd3 : ov_ch = vwr_disp[15:12];
			6'd4 : ov_ch = vwr_disp[11:8];
			6'd5 : ov_ch = vwr_disp[7:4];
			6'd6 : ov_ch = vwr_disp[3:0];
			6'd7 : ov_ch = 5'd17; // space
			6'd8 : ov_ch = 5'd23; // M
			6'd9 : ov_ch = 5'd20; // S
			6'd10: ov_ch = 5'd26; // :
			6'd11: ov_ch = mstall_disp[15:12];
			6'd12: ov_ch = mstall_disp[11:8];
			6'd13: ov_ch = mstall_disp[7:4];
			6'd14: ov_ch = mstall_disp[3:0];
			6'd15: ov_ch = 5'd17;
			6'd16: ov_ch = 5'd20; // S
			6'd17: ov_ch = 5'd20; // S
			6'd18: ov_ch = 5'd26; // :
			6'd19: ov_ch = sstall_disp[15:12];
			6'd20: ov_ch = sstall_disp[11:8];
			6'd21: ov_ch = sstall_disp[7:4];
			6'd22: ov_ch = sstall_disp[3:0];
			default: ov_ch = 5'd17;
		endcase
		2'd1: case (ov_char_idx) // "FR:xxxx PM:xxxx TD:xxxx"
			6'd0 : ov_ch = 5'd15; // F
			6'd1 : ov_ch = 5'd19; // R
			6'd2 : ov_ch = 5'd26; // :
			6'd3 : ov_ch = frame_cnt[15:12];
			6'd4 : ov_ch = frame_cnt[11:8];
			6'd5 : ov_ch = frame_cnt[7:4];
			6'd6 : ov_ch = frame_cnt[3:0];
			6'd7 : ov_ch = 5'd17;
			6'd8 : ov_ch = 5'd16; // P
			6'd9 : ov_ch = 5'd23; // M
			6'd10: ov_ch = 5'd26; // :
			6'd11: ov_ch = pmiss_disp[15:12];
			6'd12: ov_ch = pmiss_disp[11:8];
			6'd13: ov_ch = pmiss_disp[7:4];
			6'd14: ov_ch = pmiss_disp[3:0];
			6'd15: ov_ch = 5'd17;
			6'd16: ov_ch = 5'd21; // T
			6'd17: ov_ch = 5'd13; // D
			6'd18: ov_ch = 5'd26; // :
			6'd19: ov_ch = tdl_disp[15:12];
			6'd20: ov_ch = tdl_disp[11:8];
			6'd21: ov_ch = tdl_disp[7:4];
			6'd22: ov_ch = tdl_disp[3:0];
			default: ov_ch = 5'd17;
		endcase
		2'd2: case (ov_char_idx) // "FC:xxxx MR:xxxx MA:xxxx"
			6'd0 : ov_ch = 5'd15; // F
			6'd1 : ov_ch = 5'd12; // C
			6'd2 : ov_ch = 5'd26; // :
			6'd3 : ov_ch = fcnt_disp[15:12];
			6'd4 : ov_ch = fcnt_disp[11:8];
			6'd5 : ov_ch = fcnt_disp[7:4];
			6'd6 : ov_ch = fcnt_disp[3:0];
			6'd7 : ov_ch = 5'd17;
			6'd8 : ov_ch = 5'd23; // M
			6'd9 : ov_ch = 5'd19; // R
			6'd10: ov_ch = 5'd26; // :
			6'd11: ov_ch = mrmiss_disp[15:12];
			6'd12: ov_ch = mrmiss_disp[11:8];
			6'd13: ov_ch = mrmiss_disp[7:4];
			6'd14: ov_ch = mrmiss_disp[3:0];
			6'd15: ov_ch = 5'd17;
			6'd16: ov_ch = 5'd23; // M
			6'd17: ov_ch = 5'd10; // A
			6'd18: ov_ch = 5'd26; // :
			6'd19: ov_ch = mracc_disp[15:12];
			6'd20: ov_ch = mracc_disp[11:8];
			6'd21: ov_ch = mracc_disp[7:4];
			6'd22: ov_ch = mracc_disp[3:0];
			default: ov_ch = 5'd17;
		endcase
		// Peak-hold row: worst per-frame value since the overlay was last
		// switched on. "P" prefix = peak.
		2'd3: case (ov_char_idx) // "PM:xxxx PS:xxxx PR:xxxx"
			6'd0 : ov_ch = 5'd16; // P
			6'd1 : ov_ch = 5'd23; // M
			6'd2 : ov_ch = 5'd26; // :
			6'd3 : ov_ch = mstall_peak[15:12];
			6'd4 : ov_ch = mstall_peak[11:8];
			6'd5 : ov_ch = mstall_peak[7:4];
			6'd6 : ov_ch = mstall_peak[3:0];
			6'd7 : ov_ch = 5'd17;
			6'd8 : ov_ch = 5'd16; // P
			6'd9 : ov_ch = 5'd20; // S
			6'd10: ov_ch = 5'd26; // :
			6'd11: ov_ch = sstall_peak[15:12];
			6'd12: ov_ch = sstall_peak[11:8];
			6'd13: ov_ch = sstall_peak[7:4];
			6'd14: ov_ch = sstall_peak[3:0];
			6'd15: ov_ch = 5'd17;
			6'd16: ov_ch = 5'd16; // P
			6'd17: ov_ch = 5'd19; // R
			6'd18: ov_ch = 5'd26; // :
			6'd19: ov_ch = mrmiss_peak[15:12];
			6'd20: ov_ch = mrmiss_peak[11:8];
			6'd21: ov_ch = mrmiss_peak[7:4];
			6'd22: ov_ch = mrmiss_peak[3:0];
			default: ov_ch = 5'd17;
		endcase
		default: ov_ch = 5'd17;
	endcase
end

wire [34:0] ov_glyph  = font_glyph(ov_ch);
wire  [5:0] ov_bitpos = {3'd0, ov_row_y} * 6'd5 + {3'd0, ov_col};
wire        ov_pixel  = (ov_col < 3'd5) && (ov_row_y < 3'd7) && ov_glyph[34 - ov_bitpos];

//------------------------------------------------------------------
// Output register
//------------------------------------------------------------------
always @(posedge clk_sys) begin
	if (ce_pix) begin
		if (in_overlay_area) begin
			rgb <= ov_pixel ? 24'hFFFFFF : 24'h000000;
		end else if (HBlank || VBlank) begin
			rgb <= 24'd0;
		end else begin
			rgb <= {col_r, col_g, col_b};
		end
	end
end

endmodule
