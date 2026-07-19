//============================================================================
//  sor_video_tb.sv — standalone testbench for rtl/sor_video.sv
//
//  Isolates the background ROM-tilemap fetch pipeline from the CPUs/boot
//  sequence entirely: loads the real bg_gfx/bg_prom ROM chip files
//  directly, forces gfxbank/scroll_x/scroll_y, and free-runs clk_sys/
//  ce_pix to let the tile-fetch FSM run through real frames. Dumps
//  bg_pen for a sample of tile positions to see directly whether the
//  fetch pipeline produces non-blank content when given known-good ROM
//  data and inputs -- without waiting for a full CPU boot (a few real
//  frames simulate in seconds here, vs minutes for a CPU-driven run).
//
//  2026-07-12 session, corrected: the original gfxbank=0x03/scroll=0,0
//  values here were WRONG -- traced from an assumption, not verified
//  MAME I/O traffic, and (per docs/planning.md's later review) the
//  original "26,866/26,868 nonzero" pass result they produced was
//  independently a false positive (tile-0's own plane bytes are
//  nonzero regardless of correctness, so that count never actually
//  distinguished working from broken). Both errors are exactly the
//  "false narrative from incorrectly collected MAME behavior" the
//  2026-07-12-afternoon docs review flagged.
//
//  Replaced with values captured directly from a live MAME I/O write
//  watchpoint trace (wpiset on the master CPU's 0x80-0xCF port range,
//  loaded from sta/offroad/s.sta, a mid-gameplay save state) rather
//  than assumption: gfxbank=0x03 (register 0x0E write of 0x03 via the
//  AY8910 address/data port pair, 0xCA/0xCB), scroll_x=0x0140 (BKXL/
//  BKXH writes 0x40/0x01 at 0xCC/0xCD), scroll_y=0x0188 (BKYL/BKYH
//  writes 0x88/0x01 at 0xCE/0xCF). The corresponding MAME screenshot
//  at this state shows a fully-rendered racetrack (checkered road,
//  curbing, dirt texture) -- real, visually rich background content,
//  not the blank/near-blank region the old (0,0) scroll actually
//  pointed at.
//============================================================================

`timescale 1ns / 1ps

module sor_video_tb;

localparam CLK_PERIOD = 20.83; // 48 MHz

reg clk_sys = 0;
always #(CLK_PERIOD/2) clk_sys = ~clk_sys;

reg reset = 1;

// Pixel clock: 7.159090 MHz from 48 MHz -- exact copy of sor_board.sv's
// ce_pix generation, so this testbench matches real timing precisely.
reg [15:0] pix_acc = 16'd0;
reg        ce_pix_r = 0;
always @(posedge clk_sys) begin
	if (pix_acc + 16'd7159 >= 16'd48000) begin
		pix_acc  <= pix_acc + 16'd7159 - 16'd48000;
		ce_pix_r <= 1;
	end else begin
		pix_acc  <= pix_acc + 16'd7159;
		ce_pix_r <= 0;
	end
end
wire ce_pix = ce_pix_r;

wire HBlank, HSync, VBlank, VSync;
wire [16:0] vram_addr;
wire [7:0]  vram_data = 8'h00; // no text/sprite layer needed for this test
wire [9:0]  cram_addr;
wire [7:0]  cram_data = 8'h00; // palette lookup not needed -- probing bg_pen directly
wire [23:0] rgb;

wire [16:0] gfx_addr, prom_addr;
reg  [7:0]  gfx_data, prom_data;

// 2026-07-14 session: promoted from hardwired literals to regs so a
// single simulation run can sweep multiple (scroll_x, scroll_y,
// gfxbank) cases -- the original one-shot version only ever exercised
// the demo-race state (0x140/0x188/0x03), which never touched the
// bank-1 PROM screens (gfxbank bit3=1), screen-edge tile columns, or a
// non-zero scroll_x fine offset (eff_x[2:0]!=0). See run_case task
// below for the case list.
reg [15:0] tb_scroll_x = 16'h0140;
reg [15:0] tb_scroll_y = 16'h0188;
reg [7:0]  tb_gfxbank  = 8'h03;

// Real bg_gfx (96KB, 3x32KB planes) and bg_prom (128KB sparse, 4x16KB +
// gaps) ROM content, loaded exactly like sor_board.sv's flat-image
// layout (see its ADDR_GFX_LO/ADDR_PROM_LO comments and load order).
reg [7:0] gfx_rom  [0:17'h17FFF];
reg [7:0] prom_rom [0:17'h1FFFF];

integer fd, rd_count, i;
localparam BUF_LEN = 32768;
reg [7:0] file_buf [0:BUF_LEN-1];

task load_file(input [8*32-1:0] fname, input integer base, input integer len);
	integer j;
	begin
		fd = $fopen(fname, "rb");
		if (fd == 0) begin
			$display("ERROR: could not open %0s", fname);
			$finish;
		end
		rd_count = $fread(file_buf, fd);
		$fclose(fd);
		if (rd_count != len) begin
			$display("ERROR: %0s read %0d bytes, expected %0d", fname, rd_count, len);
			$finish;
		end
	end
endtask

initial begin
	// bg_gfx: u93+u94+u95, 32KB each, contiguous (matches
	// ADDR_GFX_LO..ADDR_PROM_LO layout in sor_board.sv)
	load_file("03-22105-02.u93", 0, 32768);
	for (i = 0; i < 32768; i = i + 1) gfx_rom[i] = file_buf[i];
	load_file("03-22106-02.u94", 0, 32768);
	for (i = 0; i < 32768; i = i + 1) gfx_rom[32768 + i] = file_buf[i];
	load_file("03-22107-02.u95", 0, 32768);
	for (i = 0; i < 32768; i = i + 1) gfx_rom[65536 + i] = file_buf[i];

	// bg_prom: sparse layout -- u70 empty (0x0000-0x3FFF), u92
	// (0x4000-0x7FFF), u69 (0x8000-0xBFFF), u91+u68 empty
	// (0xC000-0x13FFF), u90 (0x14000-0x17FFF), u67 (0x18000-0x1BFFF),
	// u89 empty (0x1C000-0x1FFFF). Matches the offsets documented in
	// mra/SuperOffRoad.mra and this project's own ROM-load comments.
	for (i = 0; i < 131072; i = i + 1) prom_rom[i] = 8'h00; // NOT 17'h20000 -- overflows a 17-bit literal (needs 18 bits), silently truncates to 0, loop never runs
	load_file("03-22104-01.u92", 0, 16384);
	for (i = 0; i < 16384; i = i + 1) prom_rom[17'h4000 + i] = file_buf[i];
	load_file("03-22102-01.u69", 0, 16384);
	for (i = 0; i < 16384; i = i + 1) prom_rom[17'h8000 + i] = file_buf[i];
	load_file("03-22103-02.u90", 0, 16384);
	for (i = 0; i < 16384; i = i + 1) prom_rom[17'h14000 + i] = file_buf[i];
	load_file("03-22101-02.u67", 0, 16384);
	for (i = 0; i < 16384; i = i + 1) prom_rom[17'h18000 + i] = file_buf[i];

	$display("=== ROMs loaded ===");
	$display("prom_rom[0]=%02x prom_rom[0x4000]=%02x prom_rom[0x8000]=%02x",
	          prom_rom[0], prom_rom[17'h4000], prom_rom[17'h8000]);
	$display("prom_rom[0..7] = %02x %02x %02x %02x %02x %02x %02x %02x",
	          prom_rom[0], prom_rom[1], prom_rom[2], prom_rom[3],
	          prom_rom[4], prom_rom[5], prom_rom[6], prom_rom[7]);
	$display("gfx_rom[0..7]  = %02x %02x %02x %02x %02x %02x %02x %02x",
	          gfx_rom[0], gfx_rom[1], gfx_rom[2], gfx_rom[3],
	          gfx_rom[4], gfx_rom[5], gfx_rom[6], gfx_rom[7]);
end

always @(posedge clk_sys) begin
	gfx_data  <= gfx_rom[gfx_addr];
	prom_data <= prom_rom[prom_addr];
end

sor_video dut
(
	.clk_sys(clk_sys),
	.reset(reset),
	.ce_pix(ce_pix),

	.HBlank(HBlank), .HSync(HSync), .VBlank(VBlank), .VSync(VSync),

	.vram_addr(vram_addr), .vram_data(vram_data),
	.cram_addr(cram_addr), .cram_data(cram_data),
	.rgb(rgb),

	.scroll_x(tb_scroll_x), // driven per-case by run_case (see below)
	.scroll_y(tb_scroll_y),
	.gfxbank(tb_gfxbank),

	.gfx_addr(gfx_addr), .gfx_data(gfx_data),
	.prom_addr(prom_addr), .prom_data(prom_data),

	.raster_line(),

	.dbg_bank(5'd0), .dbg_cpu_active(1'b0), .dbg_pc(16'd0), .dbg_irq_cnt(8'd0),
	.dbg_read_gin0(1'b0), .dbg_read_gin1(1'b0), .dbg_mcont_wr(1'b0),
	.dbg_ever_cram(1'b0), .dbg_ever_scroll(1'b0), .dbg_ever_vram(1'b0),
	.dbg_pc_isr(1'b0), .dbg_rom_byte(8'd0), .dbg_io_addr(8'd0), .dbg_io_rd(1'b0),
	.dbg_wr_chk(8'd0), .dbg_rd_chk(8'd0), .dbg_chk_done(1'b0), .dbg_chk_match(1'b0),
	.dbg_wr_chk_even(8'd0), .dbg_rd_chk_even(8'd0), .dbg_chk_match_even(1'b0),
	.dbg_wr_chk_odd(8'd0), .dbg_rd_chk_odd(8'd0), .dbg_chk_match_odd(1'b0),
	.dbg_chk_match_q(4'd0), .dbg_chk_done_q(1'b0),
	.dbg_bt_done(1'b0), .dbg_bt_pass(1'b0), .dbg_bt_readback(8'd0),

	.dbg_s_pc(16'd0), .dbg_s_bank_reg(4'd0), .dbg_s_bank_wr_cnt(8'd0),
	.dbg_s_bank_max(4'd0), .dbg_s_vram_wr_cnt(16'd0), .dbg_s_banked_read_ever(1'b0),

	.dbg_snd_cmd_hist(32'd0),
	.dbg_stall_pc(16'd0), .dbg_stall_flags(5'd0), .dbg_stall_bank(3'd0),
	.dbg_wram_dump(24'd0)
);

// Sample bg_pen (and the raw fetch internals) once per tile, at the
// start of each tile's display window (col_in_tile==0, matching when
// dut commits bg_color_cur/bg_plane*_cur), across two full frames --
// logs only non-blank tiles plus a running total, so a real bug (every
// tile blank) is immediately obvious without scrolling through 8960
// identical zero lines.
integer nonzero_count = 0;
integer sample_count  = 0;
integer frame_count   = 0;
reg vblank_prev;
// Diversity counters (2026-07-12 session, corrected validation metric):
// nonzero_count alone was proven a false positive last time -- tile 0's
// own plane bytes are nonzero regardless of whether addressing/fetch is
// actually correct, since tile 0 isn't "blank" in gfx_rom, it's real
// PROM content (see rtl/sor_board.sv's gfx_fp_8003 comment: expected
// 0xFF at tile-0/plane-1/row-3). A real racetrack scene should exercise
// many distinct tile codes and pen values, not just repeatedly land on
// one. seen_tile_code/seen_pen are presence bitmaps; distinct_* are the
// popcounts, computed once at the end.
reg [0:255] seen_tile_code;
reg [0:63]  seen_pen;
always @(posedge clk_sys) begin
	vblank_prev <= VBlank;
	if (VBlank && !vblank_prev) frame_count = frame_count + 1;
	if (ce_pix && dut.col_in_tile == 3'd0 && !HBlank && !VBlank) begin
		sample_count = sample_count + 1;
		seen_tile_code[dut.prom_byte_next] = 1'b1;
		seen_pen[dut.bg_pen] = 1'b1;
		if (dut.bg_pen != 6'd0) begin
			nonzero_count = nonzero_count + 1;
			if (nonzero_count <= 40)
				$display("t=%0t NONZERO bg_pen tile_col=%0d tile_row=%0d bg_pen=%02x bg_color=%0d prom_byte=%02x gfx0=%02x gfx1=%02x gfx2=%02x",
				          $time, dut.tile_col, dut.tile_row, dut.bg_pen, dut.bg_color_cur,
				          dut.prom_byte_next, dut.bg_plane0_cur, dut.bg_plane1_cur, dut.bg_plane2_cur);
		end
	end
end

// Raw fetch-pipeline diagnostics: does fetch_ph ever leave 0 (i.e. does
// the arm condition ever fire), does prom_byte_next/gfx*_next ever hold
// real non-zero ROM content, and do the committed bg_color_cur/
// bg_plane*_cur regs ever actually change from whatever they start as.
integer fetch_ph_nonzero_count = 0;
integer commit_count = 0;
integer fetch_ph_change_count = 0;
reg [3:0] fetch_ph_prev = 4'd0;
reg [7:0] bg_color_prev, bg_plane0_prev;
always @(posedge clk_sys) begin
	if (dut.fetch_ph != 4'd0) fetch_ph_nonzero_count = fetch_ph_nonzero_count + 1;
	if (dut.fetch_ph !== fetch_ph_prev) begin
		fetch_ph_change_count = fetch_ph_change_count + 1;
		if (fetch_ph_change_count <= 60)
			$display("t=%0t FETCH_PH %0d->%0d ce_pix=%b col_in_tile=%0d fetch_armed=%b prom_addr_r=%05x gfx_addr_r=%05x",
			          $time, fetch_ph_prev, dut.fetch_ph, ce_pix, dut.col_in_tile, dut.fetch_armed, dut.prom_addr_r, dut.gfx_addr_r);
		fetch_ph_prev = dut.fetch_ph;
	end
	if (ce_pix && dut.col_in_tile == 3'd0 && !HBlank && !VBlank) begin
		commit_count = commit_count + 1;
		if (commit_count <= 20)
			$display("t=%0t COMMIT #%0d tile_col=%0d tile_row=%0d prom_byte_next=%02x gfx0_next=%02x gfx1_next=%02x gfx2_next=%02x fetch_armed=%b",
			          $time, commit_count, dut.tile_col, dut.tile_row,
			          dut.prom_byte_next, dut.gfx0_next, dut.gfx1_next, dut.gfx2_next, dut.fetch_armed);
	end
end

integer distinct_tile_code, distinct_pen;

//------------------------------------------------------------------
// Full-frame PPM dump (2026-07-13 session): the diversity metrics
// above (distinct_tile_code/distinct_pen) prove the fetch pipeline
// produces varied, non-degenerate output, but can't detect scrambling
// (e.g. tiles fetched in the wrong order, or pixels within a tile
// read out in the wrong bit order) -- a scrambled frame can still hit
// plenty of distinct tile codes and pens. Dump one full rendered
// frame as a binary PPM (P6) so it can be visually diffed against a
// MAME screenshot of the identical (scroll_x, scroll_y, gfxbank)
// state (demo-race frame, mame/snap/offroad/0006.png).
//
// NOTE: cram_data is hardwired to 8'h00 in this TB (no CRAM/palette
// loaded), so the DUT's real `rgb` output would be uniformly black --
// useless for a structural comparison. Instead this dump visualizes
// bg_pen directly (grayscale, pen<<2, 0..252) sampled at the same
// point sor_video commits it (posedge clk_sys, valid across the
// pixel's full active window) -- this carries exactly the "which
// tile, which pixel-within-tile" structural information a scrambling
// bug would corrupt, without needing a working palette.
integer ppm_fd;
reg [7:0] frame_buf [0:240*320-1];
reg       ppm_capturing;
reg       ppm_done;
// capture_target_frame: which frame_count value must be current when the
// next hc==0/vc==0 boundary arrives before capture is allowed to start.
// run_case (below) sets this per-case: a normal case sets it two frames
// ahead of the current count (full warm-up); the "scroll changes
// mid-stream" case sets it to the frame_count value already in effect
// at the moment scroll is changed, so capture starts at the very next
// frame boundary -- i.e. the FIRST frame rendered after the change,
// exactly the scenario under test.
integer capture_target_frame;
initial begin
	ppm_capturing = 0;
	ppm_done      = 0;
	capture_target_frame = 2;
end
// NOTE: gate on the DUT's raw hc/vc counters being in-range, NOT on
// the HBlank/VBlank *outputs* -- those are registered one clk_sys
// cycle behind hc/vc (see sor_video.sv's "Blanking and sync" block:
// `HBlank <= (hc >= H_ACTIVE)` uses the pre-edge hc), so the exact
// combination "hc==0 && vc==0 && !HBlank && !VBlank" is never
// simultaneously true -- HBlank/VBlank at that instant still reflect
// the *previous* (blanked) position. Gating the start trigger on that
// combination silently meant ppm_capturing never turned on and the
// whole dump was uninitialized (X, written out as 0) -- a full black
// frame that looked like real data because nonzero_bg_pen (a
// different always block, unaffected) kept passing.
always @(posedge clk_sys) begin
	if (!reset && ce_pix) begin
		if (!ppm_capturing && !ppm_done && frame_count == capture_target_frame &&
		    dut.hc == 10'd0 && dut.vc == 9'd0)
			ppm_capturing = 1;
		// Screen-coordinate indexing (2026-07-14): the pixel pipeline
		// presents screen pixel N-1 during the hc==N ce window (HBlank/
		// VBlank/fg vram_latch are all latched from the pre-edge hc, and
		// the bg path now matches via col_in_tile_d) -- so sample windows
		// hc 1..320 into x 0..319. The old `frame_buf[hc]` indexing was
		// off by one against the DUT's own blanking alignment, which is
		// exactly the kind of misalignment this dump exists to catch.
		if (ppm_capturing && dut.hc >= 10'd1 && dut.hc <= 10'd320 && dut.vc < 9'd240)
			frame_buf[dut.vc * 320 + dut.hc - 1] = {dut.bg_pen, 2'b00};
		if (ppm_capturing && dut.hc == 10'd320 && dut.vc == 9'd239) begin
			ppm_capturing = 0;
			ppm_done      = 1;
		end
	end
end

task dump_ppm(input [8*64-1:0] fname);
	integer x, y, fd2;
	reg [7:0] v;
	begin
		fd2 = $fopen(fname, "wb");
		if (fd2 == 0) begin
			$display("ERROR: could not open %0s for writing", fname);
		end else begin
			$fwrite(fd2, "P6\n320 240\n255\n");
			for (y = 0; y < 240; y = y + 1) begin
				for (x = 0; x < 320; x = x + 1) begin
					v = frame_buf[y * 320 + x];
					$fwrite(fd2, "%c%c%c", v, v, v);
				end
			end
			$fclose(fd2);
			$display("=== PPM DUMP WRITTEN: %0s (320x240, bg_pen grayscale) ===", fname);
		end
	end
endtask

// run_case: sets scroll_x/scroll_y/gfxbank, lets the fetch pipeline warm
// up for `settle` full frames, then captures and dumps the next full
// frame. Used for the four steady-state cases (race/winners/track2/
// fine-offset) -- each waits settle=2 frames from a freshly-zeroed
// frame_count, matching the original single-case script's warm-up.
task run_case(input [8*40-1:0] label, input [15:0] sx, input [15:0] sy,
              input [7:0] gb, input [8*64-1:0] fname);
	begin
		tb_scroll_x = sx;
		tb_scroll_y = sy;
		tb_gfxbank  = gb;
		frame_count = 0;
		ppm_capturing = 0;
		ppm_done      = 0;
		capture_target_frame = 2;
		wait (ppm_done);
		dump_ppm(fname);
		$display("=== CASE %0s DONE: scroll_x=%04x scroll_y=%04x gfxbank=%02x -> %0s ===",
		          label, sx, sy, gb, fname);
	end
endtask

// run_switch_case: does NOT reset frame_count or wait for warm-up --
// changes scroll/gfxbank immediately (mid-stream, as if the Master
// wrote new scroll registers between frames) and captures the very
// next frame boundary. This is the "scroll changes between frames"
// case from the task brief: the fetch pipeline's internal fetch_row/
// bg_*_cur registers still hold state computed under the OLD scroll
// from the tail of the previous frame, so this directly exercises
// whatever transition behavior exists (or doesn't).
task run_switch_case(input [8*40-1:0] label, input [15:0] sx, input [15:0] sy,
                      input [7:0] gb, input [8*64-1:0] fname);
	begin
		// +1, not frame_count itself: frame_count increments at each
		// VBlank RISE, i.e. before the next hc==0/vc==0 frame boundary
		// arrives -- a target equal to the current count can never match
		// at a boundary and the capture (and this task) would hang. The
		// previous case's dump completes at (vc==239, hc==320), so the
		// very next boundary -- the first frame rendered under the new
		// scroll values -- carries frame_count+1.
		capture_target_frame = frame_count + 1;
		ppm_capturing = 0;
		ppm_done      = 0;
		tb_scroll_x = sx;
		tb_scroll_y = sy;
		tb_gfxbank  = gb;
		wait (ppm_done);
		dump_ppm(fname);
		$display("=== SWITCH-CASE %0s DONE: scroll_x=%04x scroll_y=%04x gfxbank=%02x -> %0s ===",
		          label, sx, sy, gb, fname);
	end
endtask

initial begin
	repeat (10) @(posedge clk_sys);
	reset = 0;

	// Case 1: race (x=0140 y=0188 gfx=03) -- MAME snap 0006.png. Also
	// the baseline used by every prior single-case run this branch has
	// been comparing against.
	run_case("race", 16'h0140, 16'h0188, 8'h03, "frame_dump_race.ppm");

	// Case 2: winners circle (x=0140 y=01d8 gfx=3b) -- MAME snap
	// 0035.png. First-ever exercise of gfxbank bit3=1 (bank-1 PROM
	// half, prom_bank=gfxbank[3]<<13) in this isolated TB.
	run_case("winners", 16'h0140, 16'h01d8, 8'h3b, "frame_dump_winners.ppm");

	// Case 3: second demo track "Huevos Grande" (x=0280 y=0190 gfx=1b)
	// -- MAME snap 0059.png. Also bit3=1, different tile_col range.
	run_case("track2", 16'h0280, 16'h0190, 8'h1b, "frame_dump_track2.ppm");

	// Case 4: scroll_x fine offset non-zero (eff_x[2:0] = 3, i.e.
	// scroll_x[2:0]=3'd3). Per the row-wrap-lookahead fix's own
	// analysis (see sor_video.sv comment above fetch_col_p4/tile_row_p4),
	// scroll_x[2:0]=3 is one of the phases where the OLD "+1, same row"
	// heuristic was NOT actually buggy (only {0,5,6,7} were) -- so this
	// case is a differential check: same track content, offset by 3
	// pixels, should show the SAME clean structure as the race case
	// (case 1) just shifted, with no additional corruption.
	run_case("finex3", 16'h0143, 16'h0188, 8'h03, "frame_dump_finex3.ppm");

	// Cases 5-7: scroll_x fine offsets {5,6,7} -- the class where the
	// final arm of each row wraps into the NEXT row's active area
	// (commit lands at hc 1..3, covering the second visible tile).
	// This class caught a real gap in the first version of the
	// blanking-aware fetch-target fix (it sent all wrapped commits to
	// pixel 0, leaving the second visible tile of every row stale --
	// found by finex5's reference diff), so all three phases are kept
	// as regression cases alongside fine=0 (race/winners/track2).
	run_case("finex5", 16'h0145, 16'h0188, 8'h03, "frame_dump_finex5.ppm");
	run_case("finex6", 16'h0146, 16'h0188, 8'h03, "frame_dump_finex6.ppm");
	run_case("finex7", 16'h0147, 16'h0188, 8'h03, "frame_dump_finex7.ppm");

	// Case 6: scroll changes BETWEEN frames (race -> winners, mid-
	// stream, no settle wait) -- exercises the fetch pipeline's
	// transition behavior across a scroll-register write that a real
	// CPU could issue at any point, not just frame boundaries.
	run_switch_case("race_to_winners", 16'h0140, 16'h01d8, 8'h3b,
	                 "frame_dump_switch_race_to_winners.ppm");

	distinct_tile_code = 0;
	for (i = 0; i < 256; i = i + 1) if (seen_tile_code[i]) distinct_tile_code = distinct_tile_code + 1;
	distinct_pen = 0;
	for (i = 0; i < 64; i = i + 1) if (seen_pen[i]) distinct_pen = distinct_pen + 1;

	$display("=== ALL CASES DONE: total_frames=%0d tile_samples=%0d nonzero_bg_pen=%0d fetch_ph_nonzero_cycles=%0d distinct_tile_codes=%0d distinct_pens=%0d ===",
	          frame_count, sample_count, nonzero_count, fetch_ph_nonzero_count, distinct_tile_code, distinct_pen);
	$finish;
end

endmodule
