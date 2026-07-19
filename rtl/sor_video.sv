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

	// Background tile ROM (gfx_rom in sor_board.sv) — one shared read
	// port, time-multiplexed across the 3 bitplanes by this module.
	output [16:0] gfx_addr,
	input   [7:0] gfx_data,

	// Background tile lookup PROM (prom_rom in sor_board.sv)
	output [16:0] prom_addr,
	input   [7:0] prom_data,

	// Raster line counter output (for Slave Z80 synchronisation)
	output  [7:0] raster_line,

	// Debug overlay inputs
	// dbg_bank[4:0] — repurposed as latched status bits (see sor_board.sv):
	//   [0] red   : sdram_ready
	//   [1] green : slave_reset_n (Master released Slave)
	//   [2] blue  : Master ever issued a VRAM I/O port op
	//   [3] white : vram_we ever fired (Slave wrote VRAM)
	//   [4] orange: Master PC ever fetched from banked ROM (>=0x2000)
	input  [4:0] dbg_bank,
	input        dbg_cpu_active,  // pulses each Master Z80 memory-read cycle
	input [15:0] dbg_pc,          // Master CPU PC latched at each opcode fetch
	input  [7:0] dbg_irq_cnt,     // saturating count of periodic_int_n (raster IRQ) firings
	// Sticky I/O-read indicators: replaces trying to read a fast-
	// changing hex port address off a screenshot. Each latches on
	// permanently the first time the Master reads that port, so any
	// later screenshot still shows the answer.
	input        dbg_read_gin0,   // 0x00/0xC0/0x80 -- GIN0 (nitro buttons)
	input        dbg_read_gin1,   // 0x01/0xC1/0x81 -- GIN1 (coin inputs, slave halt)
	input        dbg_mcont_wr,    // 0x09/0xC9/0x89 -- MCONT write (slave release)
	input        dbg_ever_cram,   // sticky: Master ever wrote Color RAM
	input        dbg_ever_scroll, // sticky: Master ever wrote a scroll register
	input        dbg_ever_vram,   // sticky: Slave ever wrote a VRAM byte
	input        dbg_pc_isr,      // sticky: Master PC ever entered 0x0038-0x0066
	input  [7:0] dbg_rom_byte,    // last byte fetched from SDRAM (Master ROM)
	input  [7:0] dbg_io_addr,     // last I/O port address accessed by Master
	input        dbg_io_rd,       // 1=read, 0=write

	// SDRAM write/readback integrity check (Master ROM)
	input  [7:0] dbg_wr_chk,      // running sum accumulated while writing ROM
	input  [7:0] dbg_rd_chk,      // running sum accumulated while reading it back
	input        dbg_chk_done,    // readback scan finished
	input        dbg_chk_match,   // wr_chk == rd_chk (only meaningful when done)

	// Same integrity check, split by address parity (even/odd byte) to
	// localize whether the DQM low/high-byte write-select is at fault
	input  [7:0] dbg_wr_chk_even,
	input  [7:0] dbg_rd_chk_even,
	input        dbg_chk_match_even,
	input  [7:0] dbg_wr_chk_odd,
	input  [7:0] dbg_rd_chk_odd,
	input        dbg_chk_match_odd,

	// Same integrity check, split into 4 x 64KB quarters matching the
	// Master ROM's 4 physical files — localizes a mismatch to one file.
	input  [3:0] dbg_chk_match_q,
	input        dbg_chk_done_q,

	// Isolated single-byte SDRAM write/readback self-test — completely
	// independent of ROM loading/content and the CPUs.
	input        dbg_bt_done,
	input        dbg_bt_pass,
	input  [7:0] dbg_bt_readback,
	// Second (odd-address) half of the paired byte-test -- see
	// sor_board.sv's BT_PATTERN2 comment. dbg_bt_readback is now the
	// EVEN address (expect 0xA5), this is the ODD address (expect
	// 0x5A); a lane fault makes them read the same value.
	input  [7:0] dbg_bt_readback2,

	// Ground-truth spot check: actual first 4 bytes read back from SDRAM
	// at Master ROM addresses 0-3 (compare directly against a known ROM
	// hex dump, e.g. expect "F3 ED 56 31").
	input  [7:0] dbg_scan_b0,
	input  [7:0] dbg_scan_b1,
	input  [7:0] dbg_scan_b2,
	input  [7:0] dbg_scan_b3,

	// Mirror on the WRITE side: actual bytes the core handed to the
	// SDRAM write port at addresses 0-3, independent of whatever the
	// read-back scan reports -- separates "wrong data going in" from
	// "wrong data coming back out".
	input  [7:0] dbg_wr_b0,
	input  [7:0] dbg_wr_b1,
	input  [7:0] dbg_wr_b2,
	input  [7:0] dbg_wr_b3,

	// Address 0 read back from SDRAM right as the Master ROM finishes
	// loading (before the Slave ROM's ~512KB even starts) -- compare
	// against dbg_scan_b0 (the same address, but read back only at the
	// very end, after everything has loaded) to test whether SDRAM
	// retention over the load's duration is the source of corruption.
	input  [7:0] dbg_early_b0,

	// Address 0 read back IMMEDIATELY after its own write acknowledges
	// -- essentially zero elapsed time. The most decisive of the three:
	// if this is also wrong, the fault is in the actual SDRAM write or
	// in reading it back moments later, not decay over time.
	input  [7:0] dbg_immediate_b0,

	// Running count of real rd2-channel completions (sdram_rd2_ack
	// pulses) seen so far, saturating at 0xFF. Sampled here purely for
	// display -- lets us tell, on the next hardware run, whether
	// dbg_scan_b0 (which came back matching the byte-test's own
	// BT_PATTERN rather than the real ROM byte) reflects a genuinely
	// fresh SDRAM transaction or a stale/reused completion: if the
	// count at the moment scan_b0 latches is not at least 3 (immediate
	// check + byte-test + this read), something upstream of the SDRAM
	// core itself is failing to issue a real new transaction for it.
	input  [7:0] dbg_rd2_ack_cnt,

	// Re-test of the ioctl_addr skew theory at address 2 (not exempted
	// by hps_io's skip_add the way address 0 is). raw_addr = the RAW
	// (undelayed) ioctl_addr's low byte at the moment ioctl_addr_d1
	// first reads 2. If a real one-cycle skew exists, raw_addr should
	// read 3 here (already advanced past); if it reads 2, no skew.
	input  [7:0] dbg_accept2_raw_addr,
	input  [7:0] dbg_accept2_data,

	// Ground truth of the ARM's download-session structure: total
	// number of ioctl_download sessions (falling edges) seen. The MRA
	// was rewritten to a single index="0" download session (see its own
	// header comment for why -- the old five-<rom>-region structure had
	// a real hardware bug), so the expected value is now 01 -- any
	// other number means the ARM's real session structure differs from
	// what sor_board.sv's single-session address-range routing assumes.
	input  [7:0] dbg_dl_sessions,

	//------------------------------------------------------------------
	// Slave-side debug taps (Follow-up 6, docs/SESSION_2026-07-14.md) --
	// repurpose status-row chars 10-16/18-28/30-34 (stale, confirmed-
	// passing diagnostics) for hardware evidence of the organic
	// `ld ($C000),a` bank switch. See sor_slave.sv's port comments.
	//------------------------------------------------------------------
	input [15:0] dbg_s_pc,
	input  [3:0] dbg_s_bank_reg,
	input  [7:0] dbg_s_bank_wr_cnt,
	input  [3:0] dbg_s_bank_max,
	input [15:0] dbg_s_vram_wr_cnt,
	input        dbg_s_banked_read_ever,

	//------------------------------------------------------------------
	// Reset-gating taps (2026-07-15) -- repurpose status-row chars 36-39,
	// which used to show scroll_y[10:8]/scroll_x[10:8]/gfxbank (both
	// mysteries those chased are resolved: gfxbank C7 explained as inert
	// AY8910 bits, and the bg-pipeline audit proved scroll/tile addressing
	// pixel-exact vs MAME). Tests whether the Slave core ever actually
	// leaves reset on this hardware -- one of the standing hypotheses for
	// why 7ec580f's bank-switch fix shows no visual change on hardware
	// despite passing organically in sim (see sor_board.sv's Slave reset
	// input: reset | ~sdram_ready | ~slave_reset_n | ~dl_settled).
	//------------------------------------------------------------------
	//------------------------------------------------------------------
	// Sound-command history (2026-07-17, row-1 chars 30-38, replaces the
	// resolved reset-gating taps that used to live in chars 36-39):
	// 2-deep rolling log of {hi (F4), lo (F2)} sound-command pairs from
	// sor_master.sv, freeze-investigation tap -- see its own comment
	// there. {hi0,lo0,hi1,lo1}, most recent pair last.
	//------------------------------------------------------------------
	input [31:0] dbg_snd_cmd_hist,

	//------------------------------------------------------------------
	// Sound-board audio-activity taps (2026-07-18, "no sound during
	// gameplay on real hardware despite a clean boot pop" investigation,
	// docs/WP10_PROGRESS.md): pure pass-through from sor_sound.sv's own
	// new debug ports (which themselves just expose signals already
	// present on i186_periph/leland_sound_board -- no new logic in
	// either). dbg_dac_activity/dbg_dac9_activity are 1-cycle pulses;
	// this module turns them into rolling counters below (same idiom as
	// the existing dbg_s_bank_wr_cnt/dbg_s_vram_wr_cnt counters).
	//------------------------------------------------------------------
	input        dbg_dac_activity,
	input        dbg_dac9_activity,
	input        dbg_snd_int0_pin,
	input  [7:0] dbg_snd_intc_request,
	input  [7:0] dbg_snd_intc_in_service,
	// Added after round-2 hardware read (intc_request=0x30 stuck,
	// in_service=0x00 stuck, static regardless of game state): a Fable
	// sanity check on i186_periph.sv's request->poll_status pipeline
	// found it structurally sound, but flagged the one legitimate way
	// this exact reading persists -- EXT0/EXT1 still masked (MSK bit,
	// ext0/1_ctrl[3]) at reset default, never cleared by the ROM. These
	// distinguish "still masked" (ctrl reads 0x0F) from "unmasked but
	// never granted" (ctrl's bit3 clear) directly.
	input  [6:0] dbg_snd_intc_ext0_ctrl,
	input  [6:0] dbg_snd_intc_ext1_ctrl,
	input        dbg_snd_intc_poll_pending, // = intc_poll_status[15] (=intr): did a request ever actually reach the CPU-visible poll/intr line at all

	//------------------------------------------------------------------
	// Sound-CPU liveness taps (2026-07-18, same investigation): the
	// decisive next hardware read. A Fable investigation confirmed
	// dac_activity/dac9_activity reading 0 was NOT a wiring bug -- and
	// flagged that dac_write_count=8/dac9_write_count=1 being invariant
	// across every single simulation run but ABSENT on real hardware
	// means the 80186 may never be completing (or even starting) its
	// own boot self-test on real silicon at all. Shown on a new third
	// always-on status row (vc 16..23) below.
	//------------------------------------------------------------------
	input        dbg_snd_audiocpu_reset_n,  // did /RESET ever release
	input        dbg_snd_core_bus_activity, // 1-cycle pulse: Core issued any bus transaction
	input        dbg_snd_rom_fetch_activity,// 1-cycle pulse: one ROM word fetch completed via CH_RD3
	// Round-3 read: reset_n=1, both activity counters SATURATED --
	// 80186 confirmed alive and fetching real ROM. Live IP distinguishes
	// genuine forward progress through boot code from a stuck loop --
	// see dbg_core_ip's own comment in sor_sound.sv.
	input [15:0] dbg_snd_core_ip,

	//------------------------------------------------------------------
	// Runaway-PC trap (2026-07-16) -- shown on its own always-on second
	// status row (vc 8..15). The crash is a runaway, not a loop: the live
	// PC only ever shows where it landed (a NOP sled in dead ROM), which
	// differs every time and says nothing. These freeze on detect and name
	// the jump that caused it. Row 2 layout, 8px per char:
	//   0-3  : jump_from -- PC of the last control-flow transfer before
	//          the runaway, i.e. THE INSTRUCTION THAT CRASHED IT
	//   5-8  : jump_to   -- where it jumped (start of the sled)
	//   10   : 1 = trap fired (values frozen), 0 = not fired (values live)
	//   12   : Master bank_reg (added 2026-07-16, see dbg_m_bank_reg below)
	//------------------------------------------------------------------
	input [15:0] dbg_jump_from,
	input [15:0] dbg_jump_to,
	input        dbg_sled_trapped,

	// Master bank_reg (2026-07-16, docs/SESSION_2026-07-16.md section 8d):
	// added to the runaway-trap row (char 12) so a banked jump_from/PC can
	// be resolved to a specific ROM image instead of being ambiguous
	// across all 7 banks. Same layout row as jump_from/jump_to above.
	input  [2:0] dbg_m_bank_reg,

	// Live fetch-mismatch counter (docs/sdram_plan.md Section 2) -- unlike
	// fgck (title-screen only) this polls a fixed Master-ROM address
	// continuously during gameplay, so it catches SDRAM read corruption
	// live rather than only at boot. Row-2 chars 19-21:
	//   19-20: live_mm_cnt[7:0], saturating -- 00 while clean
	//   21   : live_poll_cnt[3:0] -- must visibly spin (liveness proof)
	input [11:0] dbg_live_mm,

	// Stall detector (2026-07-17, freeze investigation, row-2 chars
	// 23-30): frozen ONCE when the game stops making VRAM-write progress
	// for ~8s -- see sor_board.sv's stall-detector comment. dbg_stall_pc
	// = Master PC at that moment; dbg_stall_flags = {mvport_stall(1),
	// cur_side(1), seq_state(3)} of the VRAM-port sequencer, same
	// instant. All-zero until a stall is ever detected.
	input [15:0] dbg_stall_pc,
	input  [4:0] dbg_stall_flags,
	input  [2:0] dbg_stall_bank, // live bank_reg at the frozen instant -- resolves which
	                            // banked ROM image dbg_stall_pc belongs to

	// WRAM dump at the frozen instant (freeze investigation, row-2 chars
	// 32-38): {e712,e715,e716} -- suspected list-scan loop's flag byte
	// and 16-bit list pointer. All-zero until a stall latches.
	input [23:0] dbg_wram_dump
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
// Foreground-pen region checksum (2026-07-17 instrument).
// Passive tap on the final fg pen stream: per frame, sums fg_pen over
// the copyright-text region of the title screen (visible-pixel box
// x 24..215, y 176..215, counted the same way the board-TB SCANOUT
// capture indexes pixels: x = pens since HBlank fall, y = visible
// lines since VBlank fall). Latched at each VBlank rise and shown as
// 4 hex chars on status row 2 (chars 14-17). Purpose: hardware shows
// the first title draw with character gaps while the sim scan-out is
// pixel-perfect; this splits the divergence in one photo -- if the
// displayed sum equals the sim-expected value while the screen shows
// gaps, corruption enters DOWNSTREAM of the pen stream (palette /
// overlay / framework scaler); if it differs, the fetch or VRAM
// content is already wrong inside the core. Churning digits = per-
// frame variation (live fetch trouble); stable digits = static.
//------------------------------------------------------------------
reg [15:0] fgck_acc, fgck_lat;
reg  [8:0] fgck_x;
reg  [8:0] fgck_y;
reg        fgck_hb_d, fgck_vb_d;
always @(posedge clk_sys) begin
	if (ce_pix) begin
		fgck_hb_d <= HBlank;
		fgck_vb_d <= VBlank;
		if (VBlank && !fgck_vb_d) begin
			fgck_lat <= fgck_acc;
			fgck_acc <= 16'd0;
			fgck_y   <= 9'd0;
		end
		if (HBlank && !fgck_hb_d) begin
			fgck_x <= 9'd0;
			if (!VBlank) fgck_y <= fgck_y + 9'd1;
		end
		if (!VBlank && !HBlank) begin
			if (fgck_y >= 9'd176 && fgck_y < 9'd216 &&
			    fgck_x >= 9'd24  && fgck_x < 9'd216)
				fgck_acc <= fgck_acc + {12'd0, fg_pen};
			fgck_x <= fgck_x + 9'd1;
		end
	end
end

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
// they're displayed (armed when the display reaches the middle of
// the current tile, giving ~4 pixels/~28 clk_sys cycles of slack for
// 4 sequential 1-cycle BRAM reads -- comfortable margin at 48MHz
// clk_sys vs ~7.16MHz ce_pix).
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
reg [7:0] fetch_col, fetch_row;
reg [2:0] fetch_riy; // row-in-tile of the TARGET pixel, latched at arm time:
                     // a row-start fetch is armed during the previous row's
                     // blanking (vc not yet incremented), so the live
                     // row_in_tile would be one scanline stale there --
                     // mid-row arms latch the identical live value.
reg [7:0] prom_byte_next;
reg [7:0] gfx0_next, gfx1_next, gfx2_next;
reg [7:0] bg_color_cur;
// Named by which THIRD of bg_gfx they came from, not by plane-bit weight --
// the two are reversed here, see the bg_pen assign below.
reg [7:0] bg_third0_cur, bg_third1_cur, bg_third2_cur;
reg [16:0] gfx_addr_r, prom_addr_r;
reg        fetch_armed;

wire [11:0] fetch_tile_code = {gfxbank[5:4], fetch_row[7:6], prom_byte_next};

assign gfx_addr  = gfx_addr_r;
assign prom_addr = prom_addr_r;

// Blanking-aware fetch target (2026-07-14 bg-pipeline audit): the arm
// fires at col_in_tile==4 (mid-tile) and its commit (col_in_tile==0)
// lands exactly 4 hc-ticks later. The old code always fetched
// "tile_col+1, current tile_row" -- correct while the commit lands
// inside the same row's active area, but wrong for the last commit of
// each scanline: that committed tile stays displayed across the
// hc-wrap into the FIRST visible pixels of the NEXT row (the tilemap
// source coordinate jumps discontinuously at the wrap -- eff_x from
// ~scroll_x+420 back to scroll_x, eff_y advancing one line -- so no
// single fetched tile can cover both sides; the pre-wrap side is
// blanked, so the next row's side is the one that matters), yet the
// fetch had targeted the old row's blanking-region continuation. The
// isolated-TB frame diff against a software reference model
// (sim/bg_reference.py, MAME leland_v.cpp math recomputed from the
// ROM files) showed exactly this: the entire leftmost partial tile of
// EVERY row rendered as a stale wrong tile, at every scroll_x fine
// offset tested ({0,3,5}).
//
// General rule replacing the "+1, same row" heuristic, three cases on
// the commit position hc+4:
//   active (hc+4 < H_ACTIVE): target hc+4 on the current row --
//     provably identical to the old "tile_col+1, same row" here.
//   blanking (H_ACTIVE <= hc+4 < H_TOTAL): the commit itself is
//     invisible; the tile's only possible visible coverage is the
//     START of the next row, so target (0, vc+1). All blanking arms
//     then redundantly refetch that same row-start tile (harmless).
//   wrapped (hc+4 >= H_TOTAL): the commit lands at hc+4-H_TOTAL on
//     the next row -- which is ACTIVE (0..7), not necessarily 0:
//     scroll_x fine offsets {0,5,6,7} place the final arm of a row at
//     hc 420..423, whose commit covers next-row pixels starting at
//     1..3 -- the second visible tile, NOT the row-start tile (the
//     row-start tile came from the blanking commits before it).
//     Targeting 0 here (a first, simpler version of this fix) left
//     the second visible tile of every row stale for fine offsets
//     {5,6,7}, caught by the finex5 TB case's reference diff.
wire  [9:0] commit_pos      = hc + 10'd4;
wire        commit_wraps    = (commit_pos >= H_TOTAL);
wire        commit_in_blank = !commit_wraps && (commit_pos >= H_ACTIVE);
wire  [9:0] hc_tgt  = commit_wraps    ? (commit_pos - H_TOTAL) :
                      commit_in_blank ? 10'd0 : commit_pos;
wire  [8:0] vc_tgt  = (commit_wraps || commit_in_blank)
                      ? ((vc == V_TOTAL - 1'd1) ? 9'd0 : vc + 1'd1) : vc;
wire [10:0] eff_x_tgt = hc_tgt + scroll_x[10:0];
wire [10:0] eff_y_tgt = {2'b0, vc_tgt} + scroll_y[10:0];
wire  [7:0] tile_col_tgt = eff_x_tgt[10:3];
wire  [7:0] tile_row_tgt = eff_y_tgt[10:3];

always @(posedge clk_sys) begin
	if (reset) begin
		fetch_ph    <= 4'd0;
		fetch_armed <= 1'b0;
	end else begin
		if (ce_pix) begin
			if ((col_in_tile == 3'd4) && !fetch_armed && (fetch_ph == 4'd0)) begin
				fetch_armed <= 1'b1;
				fetch_col   <= tile_col_tgt;
				fetch_row   <= tile_row_tgt;
				fetch_riy   <= eff_y_tgt[2:0];
				fetch_ph    <= 4'd1;
			end else if (col_in_tile != 3'd4) begin
				fetch_armed <= 1'b0;
			end

			// Commit the previously-fetched tile at the start of its display window
			if (col_in_tile == 3'd0) begin
				bg_color_cur   <= prom_byte_next[7:5];
				bg_third0_cur  <= gfx0_next;
				bg_third1_cur  <= gfx1_next;
				bg_third2_cur  <= gfx2_next;
			end
		end

		// Each BRAM read (sor_board.sv's prom_rom/gfx_rom) is a standard
		// single-cycle synchronous read: the address must be stable for
		// one full clk_sys cycle before the registered data output
		// reflects it. Every address-set phase below is therefore
		// followed by a dedicated wait phase before the data is
		// captured -- capturing on the very next cycle (as an earlier
		// version of this FSM did) reads back stale data one fetch
		// behind, since the new address hasn't reached the BRAM's
		// output register yet.
		case (fetch_ph)
			4'd1: begin
				prom_addr_r <= leland_tile_idx(fetch_col, fetch_row) | {3'b0, gfxbank[3], 13'b0};
				fetch_ph    <= 4'd2;
			end
			4'd2: fetch_ph <= 4'd3; // wait: let prom_addr_r settle into the BRAM
			4'd3: begin
				prom_byte_next <= prom_data;
				fetch_ph       <= 4'd4;
			end
			4'd4: begin
				gfx_addr_r <= {2'd0, fetch_tile_code, fetch_riy};
				fetch_ph   <= 4'd5;
			end
			4'd5: fetch_ph <= 4'd6; // wait: let gfx_addr_r (plane 0) settle
			4'd6: begin
				gfx0_next  <= gfx_data;
				gfx_addr_r <= {2'd1, fetch_tile_code, fetch_riy};
				fetch_ph   <= 4'd7;
			end
			4'd7: fetch_ph <= 4'd8; // wait: let gfx_addr_r (plane 1) settle
			4'd8: begin
				gfx1_next  <= gfx_data;
				gfx_addr_r <= {2'd2, fetch_tile_code, fetch_riy};
				fetch_ph   <= 4'd9;
			end
			4'd9: fetch_ph <= 4'd10; // wait: let gfx_addr_r (plane 2) settle
			4'd10: begin
				gfx2_next <= gfx_data;
				fetch_ph  <= 4'd0;
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
// Debug overlay — top 16 rows of the active area
//
// Visible on screen without an IO board. Layout (320 px wide):
//
//  ┌──── 64 px ────┬────────── 128 px ──────────┬── remaining ──┐
//  │  bank_reg     │  CPU activity heartbeat     │  (bars/black) │
//  │  4 bits as    │  8-bit counter incremented  │               │
//  │  16×16 blocks │  by dbg_cpu_active pulses,  │               │
//  │               │  shown as a gradient bar    │               │
//  └───────────────┴─────────────────────────────┴───────────────┘
//
// Bank register blocks (hc 0..63, vc 0..15):
//   Each bit occupies 16 horizontal pixels.
//   Bit clear → dark grey (#333)
//   Bit set   → bright colour (bit0=red, bit1=green, bit2=blue, bit3=white)
//   If bank_reg stays 0 the Z80 has not written port 0xF0 yet.
//   Any non-zero value = CPU is alive and has set a ROM bank.
//
// Activity heartbeat (hc 64..191, vc 0..15):
//   An 8-bit counter driven by dbg_cpu_active.
//   Displayed as 16 px × 16 row blocks for each bit.
//   The counter rolling over continuously = CPU is executing.
//------------------------------------------------------------------
reg [7:0] act_ctr;
always @(posedge clk_sys)
	if (dbg_cpu_active) act_ctr <= act_ctr + 1'd1;

wire in_dbg_row  = (vc < 9'd16);                 // top 16 rows
wire in_bank_col = (hc < 10'd80);                // leftmost 80 px (5 x 16px blocks)
wire in_act_col  = (hc >= 10'd80) && (hc < 10'd208);  // next 128 px

// Second debug row (vc 16..31): live Master CPU PC, 16 bits × 16 px blocks
// = 256 px wide (fits within 320 active). bit15 (MSB) at hc 0..15, down to
// bit0 at hc 240..255. Lit = white, clear = dark grey.
wire in_pc_row = (vc >= 9'd16) && (vc < 9'd32);
wire in_pc_col = (hc < 10'd256);
wire [3:0] pc_bit_idx = hc[7:4];              // 0..15
wire pc_bit_val = dbg_pc[4'd15 - pc_bit_idx]; // MSB-first left to right

// Third debug row (vc 32..47): live ROM byte fetched from SDRAM (Master),
// 8 bits x 16 px blocks = 128 px wide. MSB-first left to right.
wire in_rom_row = (vc >= 9'd32) && (vc < 9'd48);
wire in_rom_col = (hc < 10'd128);
wire [2:0] rom_bit_idx = hc[6:4];                 // 0..7
wire rom_bit_val = dbg_rom_byte[3'd7 - rom_bit_idx];

// Fourth debug row (vc 48..63): last I/O port address accessed by Master.
// 8 bits x 16px blocks = 128 px wide, MSB-first. Cyan = read, magenta = write.
wire in_io_row = (vc >= 9'd48) && (vc < 9'd64);
wire in_io_col = (hc < 10'd128);
wire [2:0] io_bit_idx = hc[6:4];
wire io_bit_val = dbg_io_addr[3'd7 - io_bit_idx];
wire [23:0] io_bit_color = dbg_io_rd ? 24'h00CCCC : 24'hCC00CC;

// Fifth debug row (vc 64..79): SDRAM write/readback integrity check.
//   hc   0..127: write-time checksum (8 bits, MSB-first, amber)
//   hc 144..271: readback checksum   (8 bits, MSB-first, cyan)
//   hc 288..303: match indicator — green once scan done and equal,
//                red once scan done and different, dark grey mid-scan.
wire in_chk_row  = (vc >= 9'd64) && (vc < 9'd80);
wire in_wrchk_col = (hc < 10'd128);
wire in_rdchk_col = (hc >= 10'd144) && (hc < 10'd272);
wire in_match_col = (hc >= 10'd288) && (hc < 10'd304);
wire [2:0] wrchk_bit_idx = hc[6:4];
wire [2:0] rdchk_bit_idx = (hc - 10'd144) >> 4;
wire wrchk_bit_val = dbg_wr_chk[3'd7 - wrchk_bit_idx];
wire rdchk_bit_val = dbg_rd_chk[3'd7 - rdchk_bit_idx];
wire [23:0] match_color = !dbg_chk_done ? 24'h333333 :
                           dbg_chk_match ? 24'h22CC22 : 24'hCC2222;

// Sixth/seventh debug rows (vc 80..95 even, vc 96..111 odd): same
// write/readback checksum layout as row five, split by byte parity.
wire in_chk_even_row = (vc >= 9'd80)  && (vc < 9'd96);
wire in_chk_odd_row  = (vc >= 9'd96)  && (vc < 9'd112);
wire wrchk_even_bit_val = dbg_wr_chk_even[3'd7 - wrchk_bit_idx];
wire rdchk_even_bit_val = dbg_rd_chk_even[3'd7 - rdchk_bit_idx];
wire wrchk_odd_bit_val  = dbg_wr_chk_odd [3'd7 - wrchk_bit_idx];
wire rdchk_odd_bit_val  = dbg_rd_chk_odd [3'd7 - rdchk_bit_idx];
wire [23:0] match_even_color = !dbg_chk_done ? 24'h333333 :
                                 dbg_chk_match_even ? 24'h22CC22 : 24'hCC2222;
wire [23:0] match_odd_color  = !dbg_chk_done ? 24'h333333 :
                                 dbg_chk_match_odd  ? 24'h22CC22 : 24'hCC2222;

// Eighth debug row (vc 112..127): per-64KB-quarter match indicators,
// one per Master ROM file (u58t/u59t/u57t/u56t, left to right).
// 4 blocks, 32px wide each = 128px total. Dark grey mid-scan, green
// on match, red on mismatch once the scan finishes.
wire in_q_row = (vc >= 9'd112) && (vc < 9'd128);
wire in_q_col = (hc < 10'd128);
wire [1:0] q_idx = hc[6:5];   // 0..3, 32px per block
wire [23:0] q_color = !dbg_chk_done_q  ? 24'h333333 :
                       dbg_chk_match_q[q_idx] ? 24'h22CC22 : 24'hCC2222;

// Ninth debug row (vc 128..143): isolated single-byte write/readback
// self-test. hc 0..15: pass/fail indicator (dark grey until done,
// green pass, red fail). hc 32..159: the actual readback byte value,
// 8 bits x 16px, MSB-first, white=1/grey=0 — should read 0xA5
// (10100101) on pass.
wire in_bt_row      = (vc >= 9'd128) && (vc < 9'd144);
wire in_bt_ind_col   = (hc < 10'd16);
wire in_bt_data_col  = (hc >= 10'd32) && (hc < 10'd160);
wire [2:0] bt_bit_idx = (hc - 10'd32) >> 4;
wire bt_bit_val = dbg_bt_readback[3'd7 - bt_bit_idx];
wire [23:0] bt_ind_color = !dbg_bt_done ? 24'h333333 :
                            dbg_bt_pass ? 24'h22CC22 : 24'hCC2222;

//------------------------------------------------------------------
// Tenth debug row (vc 144..151): text readout of the checksum/self-test
// results, so they can be transcribed as plain characters instead of
// having to read color/bit-pattern blocks off a photo (which repeatedly
// proved unreliable in practice). 20 characters x 8px = 160px wide.
//
// Layout: "WW RR - M E O - Q Q Q Q - B"
//   WW = wr_chk hex byte, RR = rd_chk hex byte (combined checksum)
//   M  = combined match (P=pass, F=fail, -=scan not done yet)
//   E  = even-byte-only match, O = odd-byte-only match
//   Q  = per-64KB-quarter match, one char each (u58t/u59t/u57t/u56t order)
//   B  = isolated single-byte self-test (P/F/-)
//------------------------------------------------------------------
// Font: 5x7 glyph per character. char codes 0-9 = digits, 10-15 = hex
// A-F (also doubles as literal letter F for "Fail" -- same glyph shape
// serves both), 16 = 'P', 17 = space, 18 = '-'.
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
			default: font_glyph = 35'd0; // 17 = space, and unused codes
		endcase
	end
endfunction

wire in_text_row = (vc >= 9'd144) && (vc < 9'd152);
wire [4:0] text_char_idx = hc[7:3];   // 0..19, 8px per character cell
wire [2:0] text_col      = hc[2:0];   // 0..7 within each cell
wire [2:0] text_row_y    = vc - 9'd144; // 0..7

reg [4:0] text_ch;
always @(*) begin
	case (text_char_idx)
		5'd0 : text_ch = dbg_wr_chk[7:4];
		5'd1 : text_ch = dbg_wr_chk[3:0];
		5'd2 : text_ch = 5'd17; // space
		5'd3 : text_ch = dbg_rd_chk[7:4];
		5'd4 : text_ch = dbg_rd_chk[3:0];
		5'd5 : text_ch = 5'd17;
		5'd6 : text_ch = 5'd18; // -
		5'd7 : text_ch = 5'd17;
		5'd8 : text_ch = !dbg_chk_done      ? 5'd18 : (dbg_chk_match      ? 5'd16 : 5'd15);
		5'd9 : text_ch = !dbg_chk_done      ? 5'd18 : (dbg_chk_match_even ? 5'd16 : 5'd15);
		5'd10: text_ch = !dbg_chk_done      ? 5'd18 : (dbg_chk_match_odd  ? 5'd16 : 5'd15);
		5'd11: text_ch = 5'd17;
		5'd12: text_ch = 5'd18;
		5'd13: text_ch = 5'd17;
		5'd14: text_ch = !dbg_chk_done_q ? 5'd18 : (dbg_chk_match_q[0] ? 5'd16 : 5'd15);
		5'd15: text_ch = !dbg_chk_done_q ? 5'd18 : (dbg_chk_match_q[1] ? 5'd16 : 5'd15);
		5'd16: text_ch = !dbg_chk_done_q ? 5'd18 : (dbg_chk_match_q[2] ? 5'd16 : 5'd15);
		5'd17: text_ch = !dbg_chk_done_q ? 5'd18 : (dbg_chk_match_q[3] ? 5'd16 : 5'd15);
		5'd18: text_ch = 5'd17;
		5'd19: text_ch = !dbg_bt_done ? 5'd18 : (dbg_bt_pass ? 5'd16 : 5'd15);
		default: text_ch = 5'd17;
	endcase
end

wire [34:0] text_glyph = font_glyph(text_ch);
wire  [5:0] text_bit_pos = {3'd0, text_row_y} * 6'd5 + {3'd0, text_col};
wire        text_pixel = (text_col < 3'd5) && (text_row_y < 3'd7) &&
                          text_glyph[34 - text_bit_pos];

//------------------------------------------------------------------
// Eleventh debug row (vc 152..159): ground-truth first 4 bytes read
// back from SDRAM at Master ROM addresses 0-3, as plain hex text.
// Compare directly against a known ROM hex dump (e.g. expect the fixed
// bank's actual first bytes "F3 ED 56 31") to settle whether the SDRAM
// round-trip itself is correct, independent of the checksum's opaque
// pass/fail.
//------------------------------------------------------------------
wire in_scan_row = (vc >= 9'd152) && (vc < 9'd160);
wire [3:0] scan_char_idx = hc[6:3]; // 0..11, 8px per character cell
wire [2:0] scan_row_y    = vc - 9'd152;

reg [4:0] scan_ch;
always @(*) begin
	case (scan_char_idx)
		4'd0 : scan_ch = dbg_scan_b0[7:4];
		4'd1 : scan_ch = dbg_scan_b0[3:0];
		4'd2 : scan_ch = 5'd17;
		4'd3 : scan_ch = dbg_scan_b1[7:4];
		4'd4 : scan_ch = dbg_scan_b1[3:0];
		4'd5 : scan_ch = 5'd17;
		4'd6 : scan_ch = dbg_scan_b2[7:4];
		4'd7 : scan_ch = dbg_scan_b2[3:0];
		4'd8 : scan_ch = 5'd17;
		4'd9 : scan_ch = dbg_scan_b3[7:4];
		4'd10: scan_ch = dbg_scan_b3[3:0];
		default: scan_ch = 5'd17;
	endcase
end

wire [34:0] scan_glyph = font_glyph(scan_ch);
wire  [5:0] scan_bit_pos = {3'd0, scan_row_y} * 6'd5 + {3'd0, text_col};
wire        scan_pixel = (text_col < 3'd5) && (scan_row_y < 3'd7) &&
                          scan_glyph[34 - scan_bit_pos];

//------------------------------------------------------------------
// Twelfth debug row (vc 160..167): mirror of the scan-byte row, but
// on the WRITE side -- the actual bytes handed to the SDRAM write
// port at addresses 0-3. Compared against the row above (read-back),
// this separates "wrong data went in" from "right data went in, wrong
// data came back out".
//------------------------------------------------------------------
wire in_wrb_row = (vc >= 9'd160) && (vc < 9'd168);
wire [3:0] wrb_char_idx = hc[6:3];
wire [2:0] wrb_row_y    = vc - 9'd160;

reg [4:0] wrb_ch;
always @(*) begin
	case (wrb_char_idx)
		4'd0 : wrb_ch = dbg_wr_b0[7:4];
		4'd1 : wrb_ch = dbg_wr_b0[3:0];
		4'd2 : wrb_ch = 5'd17;
		4'd3 : wrb_ch = dbg_wr_b1[7:4];
		4'd4 : wrb_ch = dbg_wr_b1[3:0];
		4'd5 : wrb_ch = 5'd17;
		4'd6 : wrb_ch = dbg_wr_b2[7:4];
		4'd7 : wrb_ch = dbg_wr_b2[3:0];
		4'd8 : wrb_ch = 5'd17;
		4'd9 : wrb_ch = dbg_wr_b3[7:4];
		4'd10: wrb_ch = dbg_wr_b3[3:0];
		default: wrb_ch = 5'd17;
	endcase
end

wire [34:0] wrb_glyph = font_glyph(wrb_ch);
wire  [5:0] wrb_bit_pos = {3'd0, wrb_row_y} * 6'd5 + {3'd0, text_col};
wire        wrb_pixel = (text_col < 3'd5) && (wrb_row_y < 3'd7) &&
                         wrb_glyph[34 - wrb_bit_pos];

//------------------------------------------------------------------
// Thirteenth debug row (vc 168..175): address 0 read back from SDRAM
// right as the Master ROM finishes loading -- a single 2-digit hex
// value. Compare against slot 0 of the read-back row above
// (dbg_scan_b0, the same address read only at the very end): if this
// early value is correct (0xF3) but the later one isn't, that's direct
// proof of decay over the load's duration rather than a logic bug.
//------------------------------------------------------------------
wire in_early_row = (vc >= 9'd168) && (vc < 9'd176);
wire [3:0] early_char_idx = hc[6:3];
wire [2:0] early_row_y    = vc - 9'd168;

reg [4:0] early_ch;
always @(*) begin
	case (early_char_idx)
		4'd0: early_ch = dbg_early_b0[7:4];
		4'd1: early_ch = dbg_early_b0[3:0];
		default: early_ch = 5'd17;
	endcase
end

wire [34:0] early_glyph = font_glyph(early_ch);
wire  [5:0] early_bit_pos = {3'd0, early_row_y} * 6'd5 + {3'd0, text_col};
wire        early_pixel = (text_col < 3'd5) && (early_row_y < 3'd7) &&
                           early_glyph[34 - early_bit_pos];

//------------------------------------------------------------------
// Fourteenth debug row (vc 176..183): address 0 read back IMMEDIATELY
// after its own write acknowledges -- essentially zero elapsed time.
// The most decisive of the three address-0 readback checks: if this
// is ALSO wrong, decay over time is ruled out entirely and the fault
// is in the actual SDRAM write or in reading it back moments later.
//------------------------------------------------------------------
wire in_imm_row = (vc >= 9'd176) && (vc < 9'd184);
wire [3:0] imm_char_idx = hc[6:3];
wire [2:0] imm_row_y    = vc - 9'd176;

reg [4:0] imm_ch;
always @(*) begin
	case (imm_char_idx)
		4'd0: imm_ch = dbg_immediate_b0[7:4];
		4'd1: imm_ch = dbg_immediate_b0[3:0];
		default: imm_ch = 5'd17;
	endcase
end

wire [34:0] imm_glyph = font_glyph(imm_ch);
wire  [5:0] imm_bit_pos = {3'd0, imm_row_y} * 6'd5 + {3'd0, text_col};
wire        imm_pixel = (text_col < 3'd5) && (imm_row_y < 3'd7) &&
                         imm_glyph[34 - imm_bit_pos];

//------------------------------------------------------------------
// Fifteenth debug row (vc 184..191): running count of real rd2-channel
// completions (saturating hex byte). Tells us whether dbg_scan_b0 --
// which came back matching the byte-test's own BT_PATTERN instead of
// the real ROM byte -- reflects a genuinely fresh SDRAM transaction.
//------------------------------------------------------------------
wire in_rd2cnt_row = (vc >= 9'd184) && (vc < 9'd192);
wire [3:0] rd2cnt_char_idx = hc[6:3];
wire [2:0] rd2cnt_row_y    = vc - 9'd184;

reg [4:0] rd2cnt_ch;
always @(*) begin
	case (rd2cnt_char_idx)
		4'd0: rd2cnt_ch = dbg_rd2_ack_cnt[7:4];
		4'd1: rd2cnt_ch = dbg_rd2_ack_cnt[3:0];
		default: rd2cnt_ch = 5'd17;
	endcase
end

wire [34:0] rd2cnt_glyph = font_glyph(rd2cnt_ch);
wire  [5:0] rd2cnt_bit_pos = {3'd0, rd2cnt_row_y} * 6'd5 + {3'd0, text_col};
wire        rd2cnt_pixel = (text_col < 3'd5) && (rd2cnt_row_y < 3'd7) &&
                            rd2cnt_glyph[34 - rd2cnt_bit_pos];

//------------------------------------------------------------------
// Sixteenth debug row (vc 192..199): re-test of the ioctl_addr skew
// theory at address 2 -- "RR DD", RR = raw (undelayed) ioctl_addr low
// byte at the moment ioctl_addr_d1 first read 2, DD = the byte value
// seen. RR==02 means no skew (matches the delayed value); RR==03 means
// a real one-cycle skew exists here and ioctl_addr_d1 is load-bearing.
//------------------------------------------------------------------
wire in_a2_row = (vc >= 9'd192) && (vc < 9'd200);
wire [3:0] a2_char_idx = hc[7:3];
wire [2:0] a2_row_y    = vc - 9'd192;

reg [4:0] a2_ch;
always @(*) begin
	case (a2_char_idx)
		4'd0 : a2_ch = dbg_accept2_raw_addr[7:4];
		4'd1 : a2_ch = dbg_accept2_raw_addr[3:0];
		4'd2 : a2_ch = 5'd17;
		4'd3 : a2_ch = dbg_accept2_data[7:4];
		4'd4 : a2_ch = dbg_accept2_data[3:0];
		default: a2_ch = 5'd17;
	endcase
end

wire [34:0] a2_glyph = font_glyph(a2_ch);
wire  [5:0] a2_bit_pos = {3'd0, a2_row_y} * 6'd5 + {3'd0, text_col};
wire        a2_pixel = (text_col < 3'd5) && (a2_row_y < 3'd7) &&
                        a2_glyph[34 - a2_bit_pos];

//------------------------------------------------------------------
// Seventeenth debug row (vc 200..207): number of download sessions
// the ARM actually delivered (expected 01 -- see dbg_dl_sessions
// port comment for why this isn't 05 anymore).
//------------------------------------------------------------------
wire in_dls_row = (vc >= 9'd200) && (vc < 9'd208);
wire [3:0] dls_char_idx = hc[6:3];
wire [2:0] dls_row_y    = vc - 9'd200;

reg [4:0] dls_ch;
always @(*) begin
	case (dls_char_idx)
		4'd0: dls_ch = dbg_dl_sessions[7:4];
		4'd1: dls_ch = dbg_dl_sessions[3:0];
		default: dls_ch = 5'd17;
	endcase
end

wire [34:0] dls_glyph = font_glyph(dls_ch);
wire  [5:0] dls_bit_pos = {3'd0, dls_row_y} * 6'd5 + {3'd0, text_col};
wire        dls_pixel = (text_col < 3'd5) && (dls_row_y < 3'd7) &&
                         dls_glyph[34 - dls_bit_pos];

//------------------------------------------------------------------
// Eighteenth debug row (vc 208..215): paired byte-test readback,
// "EE OO" -- EE = even-address byte (expect A5), OO = odd-address
// byte (expect 5A). EE==OO (e.g. "5A 5A") fingerprints a stuck
// DQM/byte-lane; see sor_board.sv's BT_PATTERN2 comment for the full
// fault table.
//------------------------------------------------------------------
wire in_btp_row = (vc >= 9'd208) && (vc < 9'd216);
wire [3:0] btp_char_idx = hc[7:3];
wire [2:0] btp_row_y    = vc - 9'd208;

reg [4:0] btp_ch;
always @(*) begin
	case (btp_char_idx)
		4'd0 : btp_ch = dbg_bt_readback[7:4];
		4'd1 : btp_ch = dbg_bt_readback[3:0];
		4'd2 : btp_ch = 5'd17;
		4'd3 : btp_ch = dbg_bt_readback2[7:4];
		4'd4 : btp_ch = dbg_bt_readback2[3:0];
		default: btp_ch = 5'd17;
	endcase
end

wire [34:0] btp_glyph = font_glyph(btp_ch);
wire  [5:0] btp_bit_pos = {3'd0, btp_row_y} * 6'd5 + {3'd0, text_col};
wire        btp_pixel = (text_col < 3'd5) && (btp_row_y < 3'd7) &&
                         btp_glyph[34 - btp_bit_pos];

// Which bit of bank_reg (each block = 16 px, 5 blocks = 80 px)
wire [2:0] bank_bit_idx = hc[6:4];   // 0..4 within 0..79
wire bank_bit_val = dbg_bank[bank_bit_idx];

wire [23:0] bank_bit_color =
	(bank_bit_idx == 3'd0) ? 24'hCC2222 :   // bit 0 = red
	(bank_bit_idx == 3'd1) ? 24'h22CC22 :   // bit 1 = green
	(bank_bit_idx == 3'd2) ? 24'h2222CC :   // bit 2 = blue
	(bank_bit_idx == 3'd3) ? 24'hCCCCCC :   // bit 3 = white
	                         24'hFF8800;    // bit 4 = orange

// Which bit of act_ctr (each block = 16 px within the 128 px zone)
wire [2:0] act_bit_idx = (hc - 10'd80) >> 4;   // 0..7 within 80..207
wire act_bit_val = act_ctr[act_bit_idx];

wire [23:0] dbg_pixel =
	in_bank_col ? (bank_bit_val ? bank_bit_color : 24'h333333) :
	in_act_col  ? (act_bit_val  ? 24'hFFAA00    : 24'h222222) :
	              24'h111111;   // right side of debug row: near-black border

//------------------------------------------------------------------
// Output register
//
// HARDWARE TIMING-CLOSURE EXPERIMENT (temporary): SHOW_DEBUG_OVERLAY=0
// bypasses the entire debug-row compositing chain below, so none of
// the upstream font-glyph/row logic feeding it has any path to an
// output pin -- synthesis should prune all of it as dead logic. This
// tests whether the overlay's LUT/routing footprint was contributing
// to a timing-closure problem invisible to a zero-delay behavioral
// simulator (every RTL fix this session has verified correctly in sim
// and changed nothing on real hardware, the classic signature of a
// timing problem sim can't show). Set back to 1 to restore the
// overlay once this experiment is answered -- no other logic in this
// file needs to change either way.
//------------------------------------------------------------------
// Diagnostic job is done -- see ENABLE_SDRAM_DIAG_TRAFFIC in
// sor_board.sv for the full rationale. Flip back to 1 to restore the
// overlay if diagnostics are needed again.
localparam bit SHOW_DEBUG_OVERLAY = 1'b0;

// 2026-07-18: WP10 sound bring-up is done (interrupt sticky-request fix,
// tick4 pulse-width fix, ext_int_inhibit priority fix, response_data
// handshake wiring -- see docs/WP10_PROGRESS.md "round 6") and hardware
// testing confirmed sound works through song transitions. The three
// always-on status rows (PC/heartbeat/sound taps, runaway-PC trap,
// sound-CPU liveness) below are display-only diagnostics -- gating them
// here (rather than deleting the rows, their glyph-rendering logic, or
// any of the dbg_* ports/taps feeding them) restores the full game
// picture for real play while keeping every trace point intact for the
// next time hardware-level debugging is needed. Flip back to 1 to
// re-enable; actual RTL cleanup of this debug infrastructure is
// deferred to a later pass, not done here.
localparam bit SHOW_STATUS_ROWS = 1'b0;

//------------------------------------------------------------------
// VSync heartbeat and the sticky I/O-read squares (GIN0/GIN1/MCONT/
// CRAM/SCROLL/VRAM/ISR) have served their diagnostic purpose --
// removed 2026-07-12 once real gameplay was reached and the squares'
// blended-together appearance started reading as a spurious colored
// HUD bar in screenshots, confusing "is this the game's real HUD or
// our overlay" triage. The status text row below (PC/bank/heartbeat/
// IRQ count/last I/O port) is kept -- still useful for troubleshooting
// and reads unambiguously as debug text, not game content.
//------------------------------------------------------------------
reg        vsync_prev;
always @(posedge clk_sys) vsync_prev <= VSync;

// Rolling 8-bit saturating counters for the two audio-activity pulses
// (same idiom as dbg_s_bank_wr_cnt above) -- lets a screenshot show
// "how much" DAC activity has happened since boot, not just a
// snapshot level. Saturates rather than wraps so a screenshot doesn't
// misread a wrapped-back-to-0 count as "no activity".
//
// Deliberately NOT gated on this module's own `reset` input (real
// hardware found the bug: this port is `reset | ~video_release`, a
// VIDEO-pipeline-specific release signal, separate from the sound
// board's own `reset | ~cpu_release` in sor_board.sv -- if the 80186's
// boot self-test (and its DAC pulses, dbg_dac_activity/dbg_dac9_activity)
// completes before video_release ever goes high, this counter's own
// `if (reset)` branch was silently re-clearing it through the entire
// window those pulses arrived in, even though the pulses themselves
// were correctly wired -- a real boot pop was audible on every core
// reset the whole time, proving the sound board itself was fine.
// Matches this file's own `heartbeat` register's convention just above
// (free-running from FPGA power-up default 0, no explicit reset at
// all) rather than inventing a new, still-possibly-wrong reset source.
reg [7:0] dac_activity_cnt, dac9_activity_cnt;
always @(posedge clk_sys) begin
	if (dbg_dac_activity  && (dac_activity_cnt  != 8'hFF)) dac_activity_cnt  <= dac_activity_cnt  + 8'd1;
	if (dbg_dac9_activity && (dac9_activity_cnt != 8'hFF)) dac9_activity_cnt <= dac9_activity_cnt + 8'd1;
end

// Same free-running-counter idiom, for the sound-CPU liveness taps
// (see the port comments above).
reg [7:0] core_bus_activity_cnt, rom_fetch_activity_cnt;
always @(posedge clk_sys) begin
	if (dbg_snd_core_bus_activity && (core_bus_activity_cnt != 8'hFF)) core_bus_activity_cnt <= core_bus_activity_cnt + 8'd1;
	if (dbg_snd_rom_fetch_activity && (rom_fetch_activity_cnt != 8'hFF)) rom_fetch_activity_cnt <= rom_fetch_activity_cnt + 8'd1;
end

//------------------------------------------------------------------
// Minimal always-on status strip: "PC=XXXX BK=X HB=X", a single 8px
// text row at the very top (vc 0..7), independent of
// SHOW_DEBUG_OVERLAY. Not the full overlay -- just enough to tell
// whether the Master CPU is actually executing (PC changing over
// time, matched against a heartbeat that increments once per VSync
// so two screenshots a few seconds apart show a moving digit) versus
// genuinely stuck, since MAME's own boot has no memory-check screen
// to compare against and a single static screenshot can't otherwise
// distinguish "frozen" from "running but the picture just looks
// static/rolling for a video-timing reason".
//------------------------------------------------------------------
reg [3:0] heartbeat;
always @(posedge clk_sys) if (VSync && !vsync_prev) heartbeat <= heartbeat + 4'd1;

wire in_status_row = (vc < 9'd8);
// 6 bits, not 5: active area is hc 0..319 = 40 8px cells, needing bits
// [8:3]. The earlier [7:3] (5 bits) silently dropped hc[8], wrapping
// the pattern back to index 0 at hc=256 and rendering it a second
// time within the same active row -- the "PC 2BDx" repeat visible on
// hardware screenshots was this bug, not a scaler artifact.
wire [5:0] status_char_idx = hc[8:3];
wire [2:0] status_row_y    = vc[2:0];

reg [4:0] status_ch;
always @(*) begin
	case (status_char_idx)
		// 2026-07-12 session: stripped back to a minimal, high-signal
		// set after a run of diagnostic-only builds correlated with the
		// foreground text layer's on-screen position shifting/breaking
		// build to build (this design isn't timing-clean -- every
		// Quartus run reports "not fully constrained for setup/hold
		// requirements" -- so overlay logic footprint appears to matter
		// for text stability even though it can't touch gfx_rom/prom_rom
		// BRAM inference, which is proven identical between builds).
		// Kept: PC + heartbeat (basic liveness) and the new gfx_rom
		// self-test result (see sor_board.sv's GFX_SELFTEST_ADDR
		// comment) -- a synthetic, ioctl-timing-independent write+
		// readback proving whether gfx_rom's BRAM interface itself
		// works at all, the one thing never directly tested before.
		6'd0 : status_ch = 5'd16; // 'P' (reuses the P glyph shape)
		6'd1 : status_ch = 12;    // 'C' glyph index (see font_glyph: 12=C)
		6'd2 : status_ch = 5'd17; // space
		6'd3 : status_ch = dbg_pc[15:12];
		6'd4 : status_ch = dbg_pc[11:8];
		6'd5 : status_ch = dbg_pc[7:4];
		6'd6 : status_ch = dbg_pc[3:0];
		6'd7 : status_ch = 5'd17;
		6'd8 : status_ch = heartbeat;
		6'd9 : status_ch = 5'd17;
		// 2026-07-18: chars 10-29 repurposed from the (stale,
		// already-confirmed-passing -- see docs/SESSION_2026-07-14.md
		// Follow-up 6 for the retired Slave-ROM-bringup layout this
		// replaces) Slave-side debug fields to sound-board audio
		// diagnostics, for the "clean boot pop but no sound during real
		// gameplay" investigation (docs/WP10_PROGRESS.md). dbg_s_pc/
		// dbg_s_bank_reg/etc. ports and their OTHER consumer
		// (sor_board.sv's stall_vram_ref watchdog) are untouched --
		// only what this row displays changed. Layout:
		//   10-11: dac_activity_cnt  -- saturating count of any of the 6
		//          8-bit DAC channel writes since boot/reset (2 hex)
		//   13-14: dac9_activity_cnt -- same, for the dac9 (10-bit,
		//          timer-paced) channel specifically (2 hex). WP0's own
		//          real-hardware capture put dac9 at ~7.5kHz once
		//          actually streaming -- if this count is stuck at 1
		//          (the boot self-test's own single write) while
		//          dac_activity_cnt is also stuck, nothing past boot is
		//          ever driving the DAC at all.
		//   16:    dbg_snd_int0_pin -- live level into i186_periph's
		//          external interrupt 0 (0/1)
		//   18-19: dbg_snd_intc_request -- i186_periph's
		//          intc_request_reg (2 hex; bit4=ext0 pending)
		//   21-22: dbg_snd_intc_in_service -- i186_periph's
		//          intc_in_service_reg (2 hex; bit4=ext0 in service --
		//          per i186_periph.sv's own do_ack = inta || poll_read_ack,
		//          this sets the same way whether the 80186 took a real
		//          vectored interrupt OR serviced it via POLL-register
		//          read, so a nonzero-then-clearing bit4 here means
		//          SOMETHING serviced the request, regardless of which
		//          mechanism)
		// 12/15/17/20/23-29 are spaces/blank.
		6'd10: status_ch = dac_activity_cnt[7:4];
		6'd11: status_ch = dac_activity_cnt[3:0];
		6'd12: status_ch = 5'd17;
		6'd13: status_ch = dac9_activity_cnt[7:4];
		6'd14: status_ch = dac9_activity_cnt[3:0];
		6'd15: status_ch = 5'd17;
		6'd16: status_ch = dbg_snd_int0_pin ? 5'd1 : 5'd0;
		6'd17: status_ch = 5'd17;
		6'd18: status_ch = dbg_snd_intc_request[7:4];
		6'd19: status_ch = dbg_snd_intc_request[3:0];
		6'd20: status_ch = 5'd17;
		6'd21: status_ch = dbg_snd_intc_in_service[7:4];
		6'd22: status_ch = dbg_snd_intc_in_service[3:0];
		// 23-29: EXT0/EXT1 ctrl registers (bit3=MSK, bit4=LTM) +
		// poll_pending (=intr) -- see the port comment above for why
		// these were added (Fable sanity-check finding, round-2
		// hardware read). intc_ext0/1_ctrl are only 7 bits so the
		// top-nibble hex digit only ever shows 0 or 1 (bit6=SFNM).
		6'd23: status_ch = {1'b0, dbg_snd_intc_ext0_ctrl[6:4]};
		6'd24: status_ch = dbg_snd_intc_ext0_ctrl[3:0];
		6'd25: status_ch = 5'd17;
		6'd26: status_ch = {1'b0, dbg_snd_intc_ext1_ctrl[6:4]};
		6'd27: status_ch = dbg_snd_intc_ext1_ctrl[3:0];
		6'd28: status_ch = dbg_snd_intc_poll_pending ? 5'd1 : 5'd0;
		6'd29: status_ch = 5'd17;
		// Sound-command history (2026-07-17, freeze investigation):
		// {hi0,lo0} older command, {hi1,lo1} most recent -- compare
		// directly against the MAME-captured freeze-point sequence
		// (0x71/0x83, 0x71/0x51, 0x71/0x92, then later 0x51/0xd6).
		6'd30: status_ch = dbg_snd_cmd_hist[31:28];
		6'd31: status_ch = dbg_snd_cmd_hist[27:24];
		6'd32: status_ch = dbg_snd_cmd_hist[23:20];
		6'd33: status_ch = dbg_snd_cmd_hist[19:16];
		6'd34: status_ch = 5'd17;
		6'd35: status_ch = dbg_snd_cmd_hist[15:12];
		6'd36: status_ch = dbg_snd_cmd_hist[11:8];
		6'd37: status_ch = dbg_snd_cmd_hist[7:4];
		6'd38: status_ch = dbg_snd_cmd_hist[3:0];
		6'd39: status_ch = 5'd17;
		default: status_ch = 5'd17;
	endcase
end

// Second always-on status row (vc 8..15): the runaway-PC trap. See the
// dbg_jump_from port comment for the layout.
wire       in_status_row2 = (vc >= 9'd8) && (vc < 9'd16);
wire [5:0] s2_char_idx    = hc[8:3];
wire [2:0] s2_row_y       = vc[2:0];

reg [4:0] s2_ch;
always @(*) begin
	case (s2_char_idx)
		6'd0 : s2_ch = dbg_jump_from[15:12];
		6'd1 : s2_ch = dbg_jump_from[11:8];
		6'd2 : s2_ch = dbg_jump_from[7:4];
		6'd3 : s2_ch = dbg_jump_from[3:0];
		6'd5 : s2_ch = dbg_jump_to[15:12];
		6'd6 : s2_ch = dbg_jump_to[11:8];
		6'd7 : s2_ch = dbg_jump_to[7:4];
		6'd8 : s2_ch = dbg_jump_to[3:0];
		6'd10: s2_ch = dbg_sled_trapped ? 5'd1 : 5'd0;
		6'd12: s2_ch = {2'b00, dbg_m_bank_reg}; // Master bank_reg, 1 hex char
		// chars 14-17: per-frame fg-pen checksum of the copyright-text
		// region (see fgck_* block near fg_pen) -- compare against the
		// sim-expected value on the first title screen.
		6'd14: s2_ch = {1'b0, fgck_lat[15:12]};
		6'd15: s2_ch = {1'b0, fgck_lat[11:8]};
		6'd16: s2_ch = {1'b0, fgck_lat[7:4]};
		6'd17: s2_ch = {1'b0, fgck_lat[3:0]};
		6'd19: s2_ch = {1'b0, dbg_live_mm[11:8]};
		6'd20: s2_ch = {1'b0, dbg_live_mm[7:4]};
		6'd21: s2_ch = {1'b0, dbg_live_mm[3:0]};   // poll-count nibble: must visibly spin
		// Stall-detector snapshot (freeze investigation): 23-26 = frozen
		// Master PC, 28 = mvport_stall, 29 = cur_side, 30 = seq_state.
		6'd23: s2_ch = {1'b0, dbg_stall_pc[15:12]};
		6'd24: s2_ch = {1'b0, dbg_stall_pc[11:8]};
		6'd25: s2_ch = {1'b0, dbg_stall_pc[7:4]};
		6'd26: s2_ch = {1'b0, dbg_stall_pc[3:0]};
		6'd28: s2_ch = {4'b0, dbg_stall_flags[4]};   // mvport_stall
		6'd29: s2_ch = {4'b0, dbg_stall_flags[3]};   // cur_side
		6'd30: s2_ch = {2'b0, dbg_stall_flags[2:0]}; // seq_state
		6'd31: s2_ch = {2'b0, dbg_stall_bank};       // bank_reg -- resolves dbg_stall_pc's bank
		// WRAM dump: 32-33 = $E712 (flag byte), 35-38 = $E715:E716
		// (16-bit list pointer, high byte first).
		6'd32: s2_ch = {1'b0, dbg_wram_dump[23:20]};
		6'd33: s2_ch = {1'b0, dbg_wram_dump[19:16]};
		6'd35: s2_ch = {1'b0, dbg_wram_dump[15:12]};
		6'd36: s2_ch = {1'b0, dbg_wram_dump[11:8]};
		6'd37: s2_ch = {1'b0, dbg_wram_dump[7:4]};
		6'd38: s2_ch = {1'b0, dbg_wram_dump[3:0]};
		default: s2_ch = 5'd17; // space
	endcase
end

wire [34:0] s2_glyph    = font_glyph(s2_ch);
wire  [5:0] s2_bit_pos  = {3'd0, s2_row_y} * 6'd5 + {3'd0, hc[2:0]};
wire        s2_pixel    = (hc[2:0] < 3'd5) && (s2_row_y < 3'd7) &&
                           s2_glyph[34 - s2_bit_pos];

wire [34:0] status_glyph = font_glyph(status_ch);
wire  [2:0] status_col   = hc[2:0];
wire  [5:0] status_bit_pos = {3'd0, status_row_y} * 6'd5 + {3'd0, status_col};
wire        status_pixel = (status_col < 3'd5) && (status_row_y < 3'd7) &&
                            status_glyph[34 - status_bit_pos];

// Third always-on status row (vc 16..23): sound-CPU liveness taps (see
// the dbg_snd_audiocpu_reset_n/dbg_snd_core_bus_activity/
// dbg_snd_rom_fetch_activity port comments above). Layout:
//   0: dbg_snd_audiocpu_reset_n (0/1) -- did /RESET ever release
//   2-3: core_bus_activity_cnt (2 hex, saturating) -- is Core issuing
//        ANY bus transaction at all
//   5-6: rom_fetch_activity_cnt (2 hex, saturating) -- is the sound
//        ROM's SDRAM channel (CH_RD3) actually delivering bytes
//   8-11: dbg_snd_core_ip (4 hex) -- the 80186's own live instruction
//        pointer (rtl/s80x86/Core.sv's ip_current, tapped via
//        hierarchical reference in sor_sound.sv). Round-3 read showed
//        reset_n=1 and both activity counters saturated at 0xFF (CPU
//        confirmed alive, ROM fetches confirmed working) but zero DAC
//        activity -- this can't tell "genuine forward progress through
//        boot code" from "stuck looping over a tiny address range"
//        (e.g. a wait/error handler); compare this value across two
//        screenshots a few seconds apart, same convention as row 1's
//        Master PC.
// 1/4/7/12-39 blank.
wire       in_status_row3 = (vc >= 9'd16) && (vc < 9'd24);
wire [5:0] s3_char_idx    = hc[8:3];
wire [2:0] s3_row_y       = vc[2:0];

reg [4:0] s3_ch;
always @(*) begin
	case (s3_char_idx)
		6'd0: s3_ch = dbg_snd_audiocpu_reset_n ? 5'd1 : 5'd0;
		6'd2: s3_ch = core_bus_activity_cnt[7:4];
		6'd3: s3_ch = core_bus_activity_cnt[3:0];
		6'd5: s3_ch = rom_fetch_activity_cnt[7:4];
		6'd6: s3_ch = rom_fetch_activity_cnt[3:0];
		6'd8: s3_ch = dbg_snd_core_ip[15:12];
		6'd9: s3_ch = dbg_snd_core_ip[11:8];
		6'd10: s3_ch = dbg_snd_core_ip[7:4];
		6'd11: s3_ch = dbg_snd_core_ip[3:0];
		default: s3_ch = 5'd17; // space
	endcase
end

wire [34:0] s3_glyph   = font_glyph(s3_ch);
wire  [5:0] s3_bit_pos = {3'd0, s3_row_y} * 6'd5 + {3'd0, hc[2:0]};
wire        s3_pixel   = (hc[2:0] < 3'd5) && (s3_row_y < 3'd7) &&
                          s3_glyph[34 - s3_bit_pos];

always @(posedge clk_sys) begin
	if (ce_pix) begin
		if (HBlank || VBlank) begin
			rgb <= 24'd0;
		end else if (SHOW_STATUS_ROWS && in_status_row) begin
			rgb <= status_pixel ? 24'h00FFFF : 24'd0;
		end else if (SHOW_STATUS_ROWS && in_status_row2) begin
			// Amber while live, red once the trap has fired -- so a crash is
			// obvious at a glance without decoding the hex.
			rgb <= s2_pixel ? (dbg_sled_trapped ? 24'hFF4040 : 24'hFFAA00) : 24'd0;
		end else if (SHOW_STATUS_ROWS && in_status_row3) begin
			rgb <= s3_pixel ? 24'h80FF80 : 24'd0; // pale green -- visually distinct from rows 1/2
		end else if (!SHOW_DEBUG_OVERLAY) begin
			rgb <= {col_r, col_g, col_b};
		end else if (in_dbg_row) begin
			// Debug overlay takes priority over everything in top 16 rows
			rgb <= dbg_pixel;
		end else if (in_pc_row) begin
			rgb <= in_pc_col ? (pc_bit_val ? 24'hFFFFFF : 24'h333333) : 24'd0;
		end else if (in_rom_row) begin
			rgb <= in_rom_col ? (rom_bit_val ? 24'hFFCC00 : 24'h333333) : 24'd0;
		end else if (in_io_row) begin
			rgb <= in_io_col ? (io_bit_val ? io_bit_color : 24'h333333) : 24'd0;
		end else if (in_chk_row) begin
			rgb <= in_wrchk_col ? (wrchk_bit_val ? 24'hFFAA00 : 24'h333333) :
			       in_rdchk_col ? (rdchk_bit_val ? 24'h00CCFF : 24'h333333) :
			       in_match_col ? match_color :
			                      24'd0;
		end else if (in_chk_even_row) begin
			// Solid full-width green/red (not a narrow 16px indicator column)
			// -- the bit-pattern + narrow-indicator style used by the other
			// checksum rows proved impossible to read reliably off a photo
			// of the overlay; this can't be misjudged.
			rgb <= in_q_col ? match_even_color : 24'd0;
		end else if (in_chk_odd_row) begin
			rgb <= in_q_col ? match_odd_color : 24'd0;
		end else if (in_q_row) begin
			rgb <= in_q_col ? q_color : 24'd0;
		end else if (in_bt_row) begin
			rgb <= in_bt_ind_col  ? bt_ind_color :
			       in_bt_data_col ? (bt_bit_val ? 24'hFFFFFF : 24'h333333) :
			                        24'd0;
		end else if (in_text_row) begin
			rgb <= text_pixel ? 24'hFFFFFF : 24'd0;
		end else if (in_scan_row) begin
			rgb <= scan_pixel ? 24'hFFFFFF : 24'd0;
		end else if (in_wrb_row) begin
			rgb <= wrb_pixel ? 24'hFFFF00 : 24'd0;
		end else if (in_early_row) begin
			rgb <= early_pixel ? 24'h00FF88 : 24'd0;
		end else if (in_imm_row) begin
			rgb <= imm_pixel ? 24'hFF00FF : 24'd0;
		end else if (in_rd2cnt_row) begin
			rgb <= rd2cnt_pixel ? 24'h00FFFF : 24'd0;
		end else if (in_a2_row) begin
			rgb <= a2_pixel ? 24'hFF8800 : 24'd0;
		end else if (in_dls_row) begin
			rgb <= dls_pixel ? 24'hAAFF00 : 24'd0;
		end else if (in_btp_row) begin
			rgb <= btp_pixel ? 24'hFF00AA : 24'd0;
		end else begin
			rgb <= {col_r, col_g, col_b};
		end
	end
end

endmodule
