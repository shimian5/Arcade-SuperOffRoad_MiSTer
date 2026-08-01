//============================================================================
//  Leland-family board package
//
//  Shared by all Leland-derived cores (WP-L1, docs/planning_leland_multiboard.md
//  section 3/4): board/input enums, the 16-byte MRA header layout, the
//  canonical SDRAM region layout (sized to the family maxima), and the
//  per-game configuration table. offroad is currently the table's only row.
//============================================================================

package leland_board_pkg;

	//--------------------------------------------------------------
	// board_class: selects structural muxes (sound board, video
	// pipeline, EEPROM model, slave bank scheme). Values match the MRA
	// header's byte 2 encoding exactly (plan section 3).
	//--------------------------------------------------------------
	typedef enum logic [7:0] {
		GEN1          = 8'd0,
		GEN2_REDLINE  = 8'd1,
		GEN2_QB       = 8'd2,
		GEN3_LELANDI  = 8'd3,
		GEN4_ATAXX    = 8'd4,
		GEN4_WSF      = 8'd5
	} board_class_e;

	//--------------------------------------------------------------
	// input_scheme: selects which control sources feed which game
	// ports (plan section 7). Header byte 4.
	//--------------------------------------------------------------
	typedef enum logic [7:0] {
		WHEELS3_PEDALS3 = 8'd0,
		JOY4_DIGITAL    = 8'd1,
		JOY3_DIGITAL    = 8'd2,
		TRACKBALL       = 8'd3,
		ADSTICK         = 8'd4,
		DIAL2_PEDAL2    = 8'd5,
		WHEELS3_MUXED   = 8'd6
	} input_scheme_e;

	//--------------------------------------------------------------
	// 16-byte MRA header (plan section 3), prepended to the index=0
	// download stream ahead of all ROM content.
	//--------------------------------------------------------------
	localparam int HDR_LEN = 16;

	localparam int HDR_OFF_MAGIC        = 0;
	localparam int HDR_OFF_VERSION      = 1;
	localparam int HDR_OFF_BOARD_CLASS  = 2;
	localparam int HDR_OFF_GAME_ID      = 3;
	localparam int HDR_OFF_INPUT_SCHEME = 4;
	localparam int HDR_OFF_FLAGS        = 5;
	// bytes 6-15: reserved (0x00)

	localparam logic [7:0] HDR_MAGIC   = 8'h4C; // 'L'
	localparam logic [7:0] HDR_VERSION = 8'h01;

	// flags byte bit assignments
	localparam int FLAG_DUAL_IO_WINDOW = 0; // dual I/O window map (offroad)
	localparam int FLAG_EEPROM_93C56   = 1; // 0 = 93C46, 1 = 93C56
	localparam int FLAG_XROM_PRESENT   = 2;
	localparam int FLAG_EXTDAC_PRESENT = 3;
	localparam int FLAG_IN4_PORT       = 4; // fixed IN4 @ raw 0x7F (pigout 4th-player port)

	//--------------------------------------------------------------
	// Canonical SDRAM layout (post-header byte addresses, plan
	// section 4). Sized to the whole Leland family's maxima; any one
	// game/board pads the unused tail of its region with zero fill in
	// the MRA. All bases/sizes are byte addresses within the 27-bit
	// ioctl/SDRAM address space used throughout this core.
	//--------------------------------------------------------------
	localparam logic [26:0] ADDR_MASTER_BASE = 27'h000000; // master ROM
	localparam logic [26:0] MASTER_MAX       = 27'h100000; // 1 MB reserved

	localparam logic [26:0] ADDR_SLAVE_BASE  = 27'h100000; // slave ROM
	localparam logic [26:0] SLAVE_MAX        = 27'h200000; // 2 MB reserved

	localparam logic [26:0] ADDR_SOUND_BASE  = 27'h300000; // 80186 sound ROM
	localparam logic [26:0] SOUND_MAX        = 27'h100000; // 1 MB reserved

	localparam logic [26:0] ADDR_GFX_BASE    = 27'h400000; // bg_gfx
	localparam logic [26:0] GFX_MAX          = 27'h200000; // 2 MB reserved

	// Wider-reads bandwidth optimization (2026-07-22): a repacked COPY of
	// bg_gfx planes 0+1, interleaved into 16-bit words (word i = {plane1
	// [i], plane0[i]}), built once at boot from the real ADDR_GFX_BASE
	// content (which stays untouched, still exactly matching the MRA/
	// MAME ROM_LOAD layout -- this is a derived cache, not a relayout).
	// Lets sor_video.sv's fetch FSM read both bitplane bytes needed per
	// tile-row in a single 16-bit SDRAM transaction instead of two 8-bit
	// ones. Sized 2*0x8000=0x10000 bytes (one 16-bit word per byte-offset
	// within a single 32KB plane); placed right after the 3 raw planes
	// (0x18000 bytes) within GFX_BASE's existing 2MB reservation, nowhere
	// near ADDR_PROM_BASE.
	localparam logic [26:0] ADDR_GFXW_BASE   = ADDR_GFX_BASE + 27'h018000;

	// WP-M8 (2026-07-24, docs/planning_sdram_multichannel.md §12 "Video
	// re-encoding"): a further derived cache alongside ADDR_GFXW_BASE --
	// all 3 bg_gfx planes for one tile-row packed into one 4-byte-aligned,
	// burst-friendly entry: word0 = {plane1[i], plane0[i]} (identical
	// content to ADDR_GFXW_BASE's entry i), word1 = {8'h00, plane2[i]}.
	// Same 15-bit index i = tile_code*8+riy as ADDR_GFXW_BASE/gfx01_idx.
	// Built by the same boot-time repack FSM (rtl/sor_board.sv) that
	// builds ADDR_GFXW_BASE, purely additive -- ADDR_GFXW_BASE stays
	// exactly as before for anything still reading it. Sized 4*0x8000 =
	// 0x20000 bytes; placed right after ADDR_GFXW_BASE's 0x10000-byte
	// region, still nowhere near ADDR_PROM_BASE (2MB reserved for GFX_BASE
	// total, only 0x48000 used so far).
	localparam logic [26:0] ADDR_GFXROW_BASE = ADDR_GFXW_BASE + 27'h010000;

	localparam logic [26:0] ADDR_PROM_BASE   = 27'h600000; // bg_prom (gen1-3)
	localparam logic [26:0] PROM_MAX         = 27'h040000; // 256 KB reserved

	localparam logic [26:0] ADDR_XROM_BASE   = 27'h640000; // xrom (gen4 only)
	localparam logic [26:0] XROM_MAX         = 27'h040000; // 256 KB reserved

	localparam logic [26:0] ADDR_EXTDAC_BASE = 27'h680000; // ext DAC samples (GEN4_WSF)
	localparam logic [26:0] EXTDAC_MAX       = 27'h080000; // 512 KB reserved

	localparam logic [26:0] ADDR_EEPROM_BASE = 27'h700000; // EEPROM default image
	localparam logic [26:0] EEPROM_MAX       = 27'h001000; // 4 KB reserved
	// NOTE: EEPROM default-image *loading* is deferred past WP-L1 (scope
	// note in the WP-L1 task) -- this constant reserves the address space
	// only; no load path is wired to it yet.

	//--------------------------------------------------------------
	// Multi-bank open-row controller (WP-M): region → bank mapping.
	//
	// Each gameplay read client is pinned to one physical SDRAM bank so
	// their open rows never evict each other:
	//   bank 0  master Z80 code ROM   (ADDR_MASTER_BASE .. ADDR_SLAVE_BASE)
	//   bank 1  slave  Z80 code ROM   (ADDR_SLAVE_BASE  .. ADDR_SOUND_BASE)
	//   bank 2  80186  sound code ROM  (ADDR_SOUND_BASE  .. ADDR_GFX_BASE)
	//   bank 3  bg gfx / prom / gfxw  (ADDR_GFX_BASE    .. )
	//
	// Both functions accept a 27-bit ioctl/SDRAM byte address and return
	// the 2-bit bank index or the 23-bit region-relative offset within
	// that bank. The write channel uses these to derive bank_sel and the
	// relative address for sdram_banked, mirroring what each read client
	// already provides via its own fixed bank assignment. Keeping both
	// paths here (rather than in sor_board.sv) ensures a single source of
	// truth — if the region layout changes, only this file changes.
	//
	// Each region fits comfortably in one 8 MB bank (23-bit offset):
	//   master 1 MB, slave 2 MB, sound 1 MB, gfx+prom 2 MB+  all < 8 MB.
	//--------------------------------------------------------------
	function automatic [1:0] sdram_addr_to_bank(input logic [26:0] addr);
		if      (addr < ADDR_SLAVE_BASE)  sdram_addr_to_bank = 2'd0;
		else if (addr < ADDR_SOUND_BASE)  sdram_addr_to_bank = 2'd1;
		else if (addr < ADDR_GFX_BASE)    sdram_addr_to_bank = 2'd2;
		else                              sdram_addr_to_bank = 2'd3;
	endfunction

	// Bank virtual base for each region (the offset subtracted to get the
	// region-relative byte address fed to sdram_banked's addr port).
	function automatic [22:0] sdram_bank_base(input logic [26:0] addr);
		if      (addr < ADDR_SLAVE_BASE)  sdram_bank_base = ADDR_MASTER_BASE[22:0]; // 0
		else if (addr < ADDR_SOUND_BASE)  sdram_bank_base = ADDR_SLAVE_BASE[22:0];
		else if (addr < ADDR_GFX_BASE)    sdram_bank_base = ADDR_SOUND_BASE[22:0];
		else                              sdram_bank_base = ADDR_GFX_BASE[22:0];
	endfunction

	//--------------------------------------------------------------
	// Per-game configuration table. game_id (header byte 3) indexes
	// this array. WP-L3 adds offroadt/pigout alongside offroad; all
	// three share the exact same master/slave bank tables (verified
	// against MAME leland_m.cpp's offroad_bankswitch, which is
	// explicitly commented as shared by all three games) -- only the
	// I/O port bases, input scheme, and flags differ per row.
	//--------------------------------------------------------------
	typedef struct packed {
		board_class_e   board_class;
		input_scheme_e  input_scheme;
		logic [7:0]     flags;
		logic [7:0]     io_base;    // leland_master_input_r/output_w window base
		logic [7:0]     mvram_base; // leland_mvram_port_r/w window base
	} game_cfg_t;

	localparam int NUM_GAMES = 3;
	localparam int GAME_OFFROAD  = 0;
	localparam int GAME_OFFROADT = 1;
	localparam int GAME_PIGOUT   = 2;

	function automatic game_cfg_t game_cfg(input int game_id);
		game_cfg_t cfg;
		case (game_id)
			GAME_OFFROAD: cfg = '{
				board_class:  GEN3_LELANDI,
				input_scheme: WHEELS3_PEDALS3,
				flags:        (8'd1 << FLAG_DUAL_IO_WINDOW),
				io_base:      8'hC0, // + dual alias @ 0x80
				mvram_base:   8'h00  // + dual alias @ 0x40
			};
			GAME_OFFROADT: cfg = '{
				board_class:  GEN3_LELANDI,
				input_scheme: WHEELS3_PEDALS3,
				flags:        8'h00, // single window, no alias
				io_base:      8'h40,
				mvram_base:   8'h80
			};
			GAME_PIGOUT: cfg = '{
				board_class:  GEN3_LELANDI,
				input_scheme: JOY4_DIGITAL,
				flags:        (8'd1 << FLAG_IN4_PORT), // single window + fixed IN4@0x7F
				io_base:      8'h40,
				mvram_base:   8'h00
			};
			default: cfg = '{
				board_class:  GEN3_LELANDI,
				input_scheme: WHEELS3_PEDALS3,
				flags:        (8'd1 << FLAG_DUAL_IO_WINDOW),
				io_base:      8'hC0,
				mvram_base:   8'h00
			};
		endcase
		return cfg;
	endfunction

endpackage
