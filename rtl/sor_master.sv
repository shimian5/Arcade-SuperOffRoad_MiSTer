//============================================================================
//  Super Off Road — Master Z80 CPU
//
//  Leland Master Z80 memory map (validated against MAME leland.cpp):
//    0x0000-0x1FFF  ROM fixed bank (always master_rom[0x00000..0x01FFF])
//    0x2000-0x9FFF  Banked ROM window (32 KB, bank register selects offset)
//    0xA000-0xDFFF  Fixed ROM (region offset 0xA000, unbanked) UNLESS
//                   bank_reg==1, in which case it's the battery-backed
//                   RAM view instead (see master_redline_map_program /
//                   offroad_bankswitch's update_battery_ram_view call)
//    0xE000-0xEFFF  Work RAM (4 KB, PRIVATE -- never shared with Slave;
//                   see sor_board.sv's wram_m/wram_s comment)
//    0xF000-0xF3FF  Color RAM (1 KB, palette entries BGR 2-3-3)
//    0xF800-0xF801  Video address latches (slave triggers sprite blit)
//
//  I/O port map (validated against MAME leland.cpp master_map_io and
//  redline_state::init_offroad):
//    0xF0        ROM bank register (write, bits [2:0] used)
//    0xF8        Player 3 wheel read (offroad_wheel_3_r, dial-encoded)
//    0xF9        Player 1 wheel read (offroad_wheel_1_r, dial-encoded)
//    0xFB        Player 2 wheel read (offroad_wheel_2_r, dial-encoded)
//    0xFD        Player 1 pedal read (raw, AN0)
//    0xFE        Player 2 pedal read (raw, AN1)
//    0xFF        Player 3 pedal read (raw, AN2)
//
//  Banking (validated against MAME leland_m.cpp offroad_bankswitch):
//    bank_reg [2:0] indexes a fixed lookup table of ROM base offsets.
//    The 48 KB window at 0x2000-0xDFFF maps to master_rom[bank_offset +
//    (cpu_addr - 0x2000)].  bank_list (MAME):
//      0 → 0x02000   1 → 0x02000   2 → 0x10000   3 → 0x18000
//      4 → 0x20000   5 → 0x28000   6 → 0x30000   7 → 0x38000
//============================================================================

import leland_board_pkg::*;

module sor_master
(
	input         clk_sys,
	input         reset,
	input         CE_6M,       // 6 MHz clock enable

	// Master program ROM (read-only)
	output [17:0] rom_addr,
	input   [7:0] rom_data,

	// Work RAM — dual-port: master writes, slave reads (Chunk 3)
	output [11:0] wram_addr,   // 4 KB
	output  [7:0] wram_din,
	output        wram_we,
	input   [7:0] wram_dout,

	// Battery-backed RAM (0xA000-0xDFFF, 16 KB) -- validated against
	// MAME leland.cpp master_redline_map_program: this range is
	// normally FIXED, unbanked ROM (region "master" offset 0xA000,
	// unaffected by bank_reg), overlaid by leland_m.cpp's
	// m_battery_ram_view only when offroad_bankswitch()'s
	// update_battery_ram_view((m_alternate_bank & 7) == 1) selects it,
	// i.e. exactly when bank_reg == 1. Master-private (Slave's own
	// separate 0xE000-0xEFFF map never touches this range).
	output [13:0] battram_addr,
	output  [7:0] battram_din,
	output        battram_we,
	input   [7:0] battram_dout,

	// Color RAM — master writes palette; video reads (sor_video)
	output  [9:0] cram_addr,
	output  [7:0] cram_din,
	output        cram_we,

	// VRAM I/O port op stream (to sor_board's VRAM sequencer) --
	// leland_mvram_port_r/w, installed by init_master_ports at
	// mvram_base=0x00 and 0x40 (see leland.cpp init_offroad()).
	output        vp_req,
	output        vp_rd,
	output        vp_trans,
	output [15:0] vp_addr,
	output  [7:0] vp_data,
	input         vp_pop,
	input   [7:0] vp_rdata,

	// /MCONT control outputs (I/O port 0x09, validated MAME leland_master_output_w)
	//   bit0: slave_reset_n  (0=hold Slave in reset, 1=run)
	//   bit2: slave_nmi_n    (0=assert NMI)
	//   bit3: slave_int_req  (0=assert Slave INT)
	output        slave_reset_n,
	output        slave_nmi_n,
	output        slave_int_req, // held level: asserted while /MCONT bit3=0

	// WP10: 80186 sound-board control latch. Real hardware shares this
	// exact write with the graphics bank-switch register -- port 0xF0/
	// 0x00 (`io_bank`, already captured into `bank_reg` below): MAME's
	// `redline_master_alt_bankswitch_w` (leland_m.cpp, confirmed by
	// reading the source, NOT the earlier WP0 trace doc which mistakenly
	// pointed at /MCONT/0xC9 instead -- see docs/WP10_PROGRESS.md)
	// forwards the SAME full byte to both `leland_master_alt_bankswitch_w`
	// (bank_reg) and `leland_80186_control_w` (the sound board's own
	// /RESET|ZNMI|INT0|/TEST|INT1 bits, [7:3]) unconditionally, every
	// write. `sound_ctrl_data` is that full byte; `sound_ctrl_wr` is a
	// 1-cycle strobe on the same write, for `leland_sound_board.sv`'s
	// existing `control_data`/`control_wr` port pair.
	output  [7:0] sound_ctrl_data,
	output        sound_ctrl_wr,

	// WP10: sound command latch (leland_a.cpp command_lo_w/command_hi_w,
	// ports 0xF2/0xF4 -- already decoded as io_cmd/io_snd_hi below,
	// previously only feeding this module's own private
	// sound_cmd_lo_r/sound_cmd_hi_r shadow regs for the echo stub).
	// cmd_wr_data matches leland_sound_board.sv's own 16-bit
	// cmd_wr_data/cmd_wr_lo/cmd_wr_hi port shape exactly (that module
	// only ever consumes [7:0] on cmd_wr_lo or [15:8] on cmd_wr_hi, so
	// the same byte is simply replicated into both halves here -- no
	// separate lo/hi bus is needed).
	output [15:0] cmd_wr_data,
	output        cmd_wr_lo,
	output        cmd_wr_hi,

	// 2026-07-18 (real-hardware "song transition hangs the 80186"
	// investigation, docs/WP10_PROGRESS.md): the real 80186 response
	// latch (leland_sound_board.sv's own response_data output, always
	// correct -- this was the missing wire, not a bug in that module).
	// Port 0xF2 reads now return this directly instead of the old
	// echo-stub register (retired -- see its own former declaration
	// site's comment, now just historical narrative, for why it existed
	// and what it was superseded by). Plain wire, no synchronizer needed -- this whole
	// design shares one physical clk_sys domain; leland_sound_board.sv
	// registers response_data synchronously on that same clock, so it's
	// already glitch-free by construction, same as every other
	// board-level status signal already wired this way.
	input  [7:0]  response_data,

	// Video address latch (triggers Slave sprite blit, Chunk 3)
	output [15:0] vid_addr,
	output        vid_addr_wr,

	// Background tilemap scroll registers (validated against MAME
	// leland_v.cpp scroll_w, invoked from leland_master_output_w at
	// I/O offsets 0x0C-0x0F relative to io_base; for offroad io_base
	// is 0xC0 and 0x80, i.e. absolute ports 0xCC-0xCF / 0x8C-0x8F).
	output [15:0] scroll_x,
	output [15:0] scroll_y,

	// Graphics bank (MAME leland_state::m_gfxbank / sor_video.sv's
	// gfxbank input) -- set via the AY8910's Port A write callback
	// (sound_port_w -> gfx_port_w in leland_v.cpp/leland_m.cpp), NOT
	// directly by name. The AY8910 itself isn't emulated (no audio
	// synthesis here), but the Master ROM drives it with a real
	// register-select-then-data 2-step sequence (I/O offsets 0x0A
	// /OGIA = address latch, 0x0B /OGID = data write, both dynamically
	// relocated the same way as MCONT -- see io_mcont) that this
	// design was previously dropping entirely, permanently stuck at
	// gfxbank==0. Confirmed via sim boot trace: real boot code already
	// writes register 0x0E (I/O Port A, standard AY-3-8910 register
	// map) within the first few instructions. Only register 14 is
	// tracked here -- every other AY8910 register (tone/noise/envelope/
	// mixer) only affects audio, irrelevant without real sound anyway.
	output  [7:0] gfxbank,

	// Slave HALT status (wired to GIN1 bit 0, active-low)
	input         slave_halt_n,

	// EEPROM (93C46, sor_eeprom_93c46) -- DI/CLK/CS are /MCONT bits
	// 4/5/6 (leland_master_output_w), DO feeds GIN3 bit0 above.
	output        eeprom_di,
	output        eeprom_clk,
	output        eeprom_cs,
	input         eeprom_do,

	// VBlank for GIN3 bit 1 timing sync
	input         vblank,

	// Raster line counter (from sor_video), used to generate the periodic
	// "VA10" interrupt every 16 scanlines starting at line 8 (validated
	// against MAME leland_m.cpp leland_interrupt_callback).
	input   [7:0] raster_line,

	// Pedal (MAME AN0/AN1/AN2, IPT_PEDAL): raw value, no encoding, read
	// directly at ports 0xFD/0xFE/0xFF. 2026-07-18: WP-controls found this
	// was previously wired to the (misidentified) wheel signal, and these
	// ports were dead code; see p1_wheel comment below for the real port map.
	input   [7:0] p1_pedal,
	input   [7:0] p2_pedal,
	input   [7:0] p3_pedal,

	// Wheel (MAME AN3/AN4/AN5, IPT_DIAL): a genuine free-spinning rotary
	// encoder, NOT an absolute position -- real hardware/MAME never reads an
	// absolute wheel angle, only the direction+magnitude of motion since the
	// last read (leland_m.cpp dial_compute_value(), see the io_wheel1/2/3
	// decode below). p1_wheel/p2_wheel/p3_wheel here are the free-running
	// mod-256 "virtual dial" position (combining analog stick + d-pad +
	// spinner upstream, see steering_input.sv) that this module samples on
	// each real CPU read to compute that encoding -- exactly mirroring how
	// MAME's ioport itself is just such an accumulated position.
	input   [7:0] p1_wheel,
	input   [7:0] p2_wheel,
	input   [7:0] p3_wheel,

	// Digital inputs (active-high from MiSTer)
	input   [3:0] p1_btn,      // [3]=coin [2]=btn2 [1]=btn1 [0]=btn0
	input   [3:0] p2_btn,
	input   [3:0] p3_btn,
	input         service,
	input         free_play,

	// WP-L3: per-game I/O port base parameterization (leland_board_pkg::
	// game_cfg, driven by the MRA header's game_id). io_base/mvram_base
	// are the leland_master_input_r/output_w and leland_mvram_port_r/w
	// window bases (MAME init_master_ports(mvram_base, io_base)).
	// dual_io_window reproduces offroad's real double-install call
	// (init_master_ports called twice, at base and base^0x40) --
	// offroadt/pigout each use a single window and leave this deasserted.
	// in4_port_en gates the pigout-only fixed IN4 @ raw 0x7F
	// (install_read_port, outside the io_base window). input_scheme
	// selects GIN0-3 bit content.
	input   [7:0] io_base,
	input   [7:0] mvram_base,
	input         dual_io_window,
	input         in4_port_en,
	input  leland_board_pkg::input_scheme_e input_scheme,

	// WP-L3: 4-player digital joystick (JOY4_DIGITAL only); bit layout:
	// [0]=right [1]=left [2]=down [3]=up [4]=btn1 [5]=btn2 [6]=start
	// [7]=coin.
	input   [7:0] p1_joy, p2_joy, p3_joy, p4_joy,

	// SDRAM stall: level-high during ROM read; stall when not ready
	output        rom_req,    // high during any ROM read machine cycle
	input         rom_stall  // high = SDRAM not ready; insert wait states
);

//------------------------------------------------------------------
// tv80s_ce (synchronous Z80, Verilog, with clock enable)
//------------------------------------------------------------------
wire        m1_n, mreq_n, iorq_n, rd_n, wr_n, rfsh_n, halt_n, busak_n;
wire [15:0] cpu_addr;
wire  [7:0] cpu_dout;
reg   [7:0] cpu_din;

// rom_req / wait_n wiring (declared after mem-decode wires below, but needed here)
// Forward-declare: these reference in_fixed/in_banked defined further down.
// SystemVerilog allows forward use of wires within the same module.

// int_n_final declared here (moved up from below its own periodic-interrupt
// logic) — some SystemVerilog tools (ModelSim) implicitly infer a net at
// first use in a module port connection, which then conflicts with an
// explicit `wire ... = ...` declaration later; Quartus tolerated this,
// ModelSim doesn't. Declare before use to avoid the ambiguity.
wire int_n_final;

// mvport_stall (from sor_vram_port below, forward-declared same as
// int_n_final above) -- holds the CPU with /WAIT instead of ever
// silently overwriting a not-yet-drained VRAM port op. See
// sor_vram_port.sv's vp_stall comment.
wire mvport_stall;

tv80s_ce #(.Mode(0), .T2Write(1), .IOWait(1)) master_cpu
(
	.reset_n(~reset),
	.clk    (clk_sys),
	.cen    (CE_6M),
`ifdef NO_STALL_CONTROL
	// Measurement-only control knob (docs/SESSION_2026-07-24_..._HANDOFF.md):
	// neutralize the ROM-fetch stall term so the master Z80 never waits on
	// rom_stall, isolating how much wall-clock progress SDRAM contention
	// costs. Absent this define, behavior is bit-identical to before.
	.wait_n (~mvport_stall),
`else
	.wait_n (~((rom_req & rom_stall) | mvport_stall)),
`endif
	.int_n  (int_n_final),
	.nmi_n  (1'b1),
	.busrq_n(1'b1),
	.m1_n   (m1_n),
	.mreq_n (mreq_n),
	.iorq_n (iorq_n),
	.rd_n   (rd_n),
	.wr_n   (wr_n),
	.rfsh_n (rfsh_n),
	.halt_n (halt_n),
	.busak_n(busak_n),
	.A      (cpu_addr),
	.di     (cpu_din),
	.dout   (cpu_dout)
);

//------------------------------------------------------------------
// Periodic "VA10" interrupt — validated against MAME leland_m.cpp
// leland_interrupt_callback(): fires every 16 scanlines starting at
// line 8. This is the only Master INT source in MAME.
//------------------------------------------------------------------
reg [7:0] raster_line_prev;
reg       periodic_int_n;

always @(posedge clk_sys) begin
	raster_line_prev <= raster_line;
	if (reset)
		periodic_int_n <= 1'b1;
	else if ((raster_line != raster_line_prev) && (raster_line[3:0] == 4'd8))
		periodic_int_n <= 1'b0;
	else if (CE_6M && ~iorq_n & ~m1_n)  // INT acknowledge cycle
		periodic_int_n <= 1'b1;
end

// The periodic "VA10" raster interrupt (leland_interrupt_callback) is
// the only Master INT source in MAME -- there is no Slave->Master
// command-port interrupt (see sor_slave.sv header comment: the two
// CPUs only communicate through the shared VRAM mailbox and the
// SLAVEHALT/GIN1 poll).
assign int_n_final = periodic_int_n;

//------------------------------------------------------------------
// ROM banking — validated against MAME offroad_bankswitch()
//
// bank_reg [2:0] selects one of 8 entries in the bank_list table.
// Each entry is the byte offset into the flat 256 KB master ROM
// where the 48 KB banked window (Z80 0x2000-0xDFFF) begins.
//
// rom_addr for banked access = bank_offset[bank_reg] + (cpu_addr - 0x2000)
// rom_addr for fixed  access = cpu_addr[12:0]  (8 KB at master_rom[0])
//------------------------------------------------------------------
reg [2:0] bank_reg;

// bank_list from MAME offroad_bankswitch (offsets into master_rom region)
reg [17:0] bank_offset;
always @(*) begin
	case (bank_reg)
		3'd0: bank_offset = 18'h02000;
		3'd1: bank_offset = 18'h02000;
		3'd2: bank_offset = 18'h10000;
		3'd3: bank_offset = 18'h18000;
		3'd4: bank_offset = 18'h20000;
		3'd5: bank_offset = 18'h28000;
		3'd6: bank_offset = 18'h30000;
		3'd7: bank_offset = 18'h38000;
	endcase
end

wire mem_access = ~mreq_n & rfsh_n;
wire in_fixed      = (cpu_addr[15:13] == 3'b000);       // 0x0000-0x1FFF
wire in_banked_lo  = (cpu_addr[15:13] >= 3'd1) &&
                     (cpu_addr[15:13] <= 3'd4);          // 0x2000-0x9FFF (truly bank-switched)
wire in_high       = (cpu_addr[15:13] == 3'b101) ||
                     (cpu_addr[15:13] == 3'b110);        // 0xA000-0xDFFF
wire in_battram    = in_high & (bank_reg == 3'd1);       // battery RAM view selected
wire in_fixed_high = in_high & ~in_battram;              // falls through to fixed ROM
wire in_wram       = (cpu_addr[15:12] == 4'hE);          // 0xE000-0xEFFF

// ROM read indicator used by sor_board to drive SDRAM stall logic --
// excludes in_battram, which is real RAM, not SDRAM-backed ROM.
assign rom_req = mem_access & ~rd_n & (in_fixed | in_banked_lo | in_fixed_high);
wire in_cram   = (cpu_addr[15:10] == 6'b111100);        // 0xF000-0xF3FF
wire in_vidlat = (cpu_addr[15:1]  == 15'h7C00);         // 0xF800-0xF801

// rom_addr: fixed region uses cpu_addr directly. fixed-high (0xA000-
// 0xDFFF when the battery RAM view is NOT selected) is unbanked --
// master_redline_map_program maps it straight to region "master"
// offset 0xA000, i.e. rom_addr == cpu_addr with no bank_offset
// involved at all.
//
// Banked-low region (0x2000-0x9FFF): directly verified LIVE against
// MAME's debugger (breakpoint at a known Z80 address, dasm the actual
// bytes MAME itself is executing at that instant -- not a static file
// guess) that this is NOT a uniform formula:
//   bank_reg 0/1 (bank_offset==0x2000): rom_addr == cpu_addr exactly
//     (verified: Z80 $2C29 live-disassembles as "LD HL,$01F4"'s
//     continuation, matching flat ROM offset 0x2C29 exactly)
//   bank_reg >=2: rom_addr == bank_offset + cpu_addr, NOT
//     window-relative (cpu_addr - 0x2000) as originally implemented
//     (verified: Z80 $2012 with bank_reg==2 live-disassembles as
//     "JP $5D4B", which only exists in the ROM at flat offset
//     0x12012 == bank_offset(0x10000) + cpu_addr(0x2012), not
//     0x10012 which the old uniform "-0x2000" formula computed and
//     is a completely different, non-jump instruction)
// The asymmetry is real per MAME's own live execution even though its
// exact internal cause (some detail of how offroad_bankswitch's
// set_base/bankr interact for bank_list==0x2000 specifically) wasn't
// fully traced through the C++ source -- this fix follows the
// directly-observed ground truth rather than the C++ reading that
// produced the wrong original formula. This was THE root cause of the
// Master's periodic (~136ms) restart: reaching bank 2 at a "common"
// low address under the old formula fetched garbage non-code bytes,
// which eventually executed a RET popping a garbage stack address.
//
// 2026-07-12 correction: cpu_addr bit 15 is NOT actually part of the
// bank_reg>=2 sum -- addresses $8000-$9FFF (the top 8 KB of the 32 KB
// banked window) alias straight back onto $0000-$1FFF of the SAME
// bank, they don't continue past it. Verified via two independent live
// MAME captures: a real mid-gameplay Master WRAM+register snapshot
// (docs/reference/mame/traces/gameplay_snapshot/) matched
// bank_offset+cpu_addr with zero byte diffs for cpu_addr $2000-$7FFF,
// but the $8000-$9FFF portion only matched at flat offset
// bank_offset+(cpu_addr&$7FFF) -- i.e. bank_offset+$0000 at cpu $8000,
// not bank_offset+$8000 as the un-masked formula computed (confirmed
// byte-exact, 0 diffs, against a live $BDAB-breakpoint capture too).
// Physically consistent with a real bank-switch ROM decode that only
// wires cpu_addr[14:0] (a 32 KB/15-bit span) into the banked chip,
// leaving A15 disconnected/ignored for this window. The bank_reg 0/1
// case does NOT need this mask -- that branch was independently
// verified byte-exact across the full $2000-$9FFF span with no masking
// (it's really just cpu_addr passed straight through, not a genuine
// bank-chip access).
//
// 2026-07-16: attempted a rotate_memory("master")-compensating fix here
// (docs/SESSION_2026-07-15.md, Follow-up 23-24), verified byte-exact in
// isolation (262144/262144 bytes matching a live MAME "master" region
// dump after applying one rotate_memory pass to a naive reconstruction)
// -- but reverted after hardware testing showed it breaks something
// upstream: with this change plus the (independently good) Slave fix,
// the Slave never reaches a single banked ROM read at all
// (SLAVE_DBG_FINAL bank_max=0, banked_read_ever=0 -- previously this
// reached bank 8 at least once), and VRAM fill collapsed back to
// baseline-broken levels (fg_nonzero=609/61440, matching the original
// unfixed 529, not the Slave-only fix's 26120). The Master fix is
// wrong in some way not yet understood (a width-truncation concern was
// checked and ruled out as the sole cause -- an explicit 15-bit
// intermediate wire made no difference) -- reverted to the original,
// previously-verified formula below pending further investigation.
// Do not re-apply the rotation compensation here without first finding
// why it breaks Master execution before the Slave ever gets a real
// command.
// 2026-07-16 conclusion (docs/SESSION_2026-07-15.md, Follow-up 24-25):
// the rotation-compensation attempt above was reverted, and a Fable/
// Opus review resolved WHY it broke things -- this formula (cpu_addr[14:0],
// not window-relative cpu_addr-0x2000) is ALREADY mathematically
// identical to "window-relative offset, rotated by +0x2000":
// cpu_addr[14:0] == cpu_addr mod 0x8000 == ((cpu_addr-0x2000)+0x2000) mod 0x8000.
// The 2026-07-12 session's empirically-discovered "use cpu_addr directly,
// not cpu_addr-0x2000" fix was, unknowingly, already the correct
// rotate_memory("master") compensation. No further Master fix is needed
// -- adding another "+0x2000" on top (as the reverted attempt did) double-
// rotates by 0x4000 (the SLAVE's amount, not the Master's), which is
// exactly why it broke Master execution. Do not touch this formula again
// without re-deriving from this note.
assign rom_addr = in_fixed      ? {5'b0, cpu_addr[12:0]} :
                  in_fixed_high ? {2'b0, cpu_addr} :
                  (bank_offset == 18'h02000) ? {2'b0, cpu_addr} :
                                                bank_offset + {3'b0, cpu_addr[14:0]};

//------------------------------------------------------------------
// Battery-backed RAM (0xA000-0xDFFF, selected when bank_reg==1)
//------------------------------------------------------------------
assign battram_addr = cpu_addr - 16'hA000;
assign battram_din  = cpu_dout;
assign battram_we   = mem_access & ~wr_n & in_battram;

//------------------------------------------------------------------
// Work RAM
//------------------------------------------------------------------
assign wram_addr = cpu_addr[11:0];
assign wram_din  = cpu_dout;
assign wram_we   = mem_access & ~wr_n & in_wram;

//------------------------------------------------------------------
// Color RAM
//------------------------------------------------------------------
assign cram_addr = cpu_addr[9:0];
assign cram_din  = cpu_dout;
// cram_we itself is assigned further down, gated on mcont_r[1] --
// see the comment there (mcont_r isn't declared until later in this
// file, and this module doesn't pre-declare regs used by later assigns).

//------------------------------------------------------------------
// Video address latch (0xF800-0xF801)
//------------------------------------------------------------------
reg [15:0] vid_addr_r;
reg        vid_addr_wr_r;

always @(posedge clk_sys) begin
	vid_addr_wr_r <= 0;
	if (CE_6M && mem_access && ~wr_n && in_vidlat) begin
		if (!cpu_addr[0]) vid_addr_r[7:0]  <= cpu_dout;
		else              vid_addr_r[15:8] <= cpu_dout;
		vid_addr_wr_r <= 1;
	end
end

assign vid_addr    = vid_addr_r;
assign vid_addr_wr = vid_addr_wr_r;

//------------------------------------------------------------------
// I/O decode
//------------------------------------------------------------------
wire io_rd = ~iorq_n & ~rd_n;
wire io_wr = ~iorq_n & ~wr_n;

// WP-L3: leland_mvram_port_r/w range, parameterized by mvram_base
// (MAME init_master_ports(mvram_base, io_base)). offroad's real driver
// call installs this handler TWICE (mvram_base=0x00 and 0x40, dual_io_
// window asserted); offroadt/pigout each install it once at their own
// base. Declared here, before first use below, so tools that don't
// tolerate forward references to implicit nets (ModelSim) don't choke
// on a later explicit `wire io_mvram = ...` colliding with an
// implicitly-inferred one.
wire [7:0] mvram_base_alt = mvram_base ^ 8'h40;
wire io_mvram = (cpu_addr[7:5] == mvram_base[7:5]) ||
                (dual_io_window && (cpu_addr[7:5] == mvram_base_alt[7:5]));

//------------------------------------------------------------------
// VRAM I/O port (leland_mvram_port_r/w). TRANS_EN=0: the Master is
// num=0 in MAME's vram_port_w, so its bit-4 transparency bit is
// ignored (transparency is Slave-only).
//------------------------------------------------------------------
wire [7:0] vram_rd_data;

sor_vram_port #(.TRANS_EN(1'b0)) mvport
(
	.clk_sys(clk_sys),
	.reset(reset),
	.CE_6M(CE_6M),

	.cpu_addr(cpu_addr),
	.cpu_dout(cpu_dout),
	.io_wr(io_wr),
	.io_rd(io_rd),
	.io_vram_sel(io_mvram),
	.vidlat_wr(CE_6M && mem_access && ~wr_n && in_vidlat),
	.vidlat_hi(cpu_addr[0]),

	.rd_data(vram_rd_data),
	.vp_stall(mvport_stall),

	.vp_req(vp_req),
	.vp_rd(vp_rd),
	.vp_trans(vp_trans),
	.vp_addr(vp_addr),
	.vp_data(vp_data),
	.vp_pop(vp_pop),
	.vp_rdata(vp_rdata)
);

// Real driver source (mamedev leland.cpp, redline_state::init_offroad):
//   init_master_ports(0x00, 0xc0);
//   init_master_ports(0x40, 0x80);   /* yes, this is intentional */
// This dynamically installs the shared leland_master_input_r /
// leland_master_output_w handlers (bank/cmd/stat/pal/mcont/GIN logic)
// at physical address io_base+offset, aliased at BOTH io_base=0xC0
// and io_base=0x80 -- NOT at the raw offset directly. Our own /MCONT
// comment said "MAME offset 0x09", which is the offset *within* that
// handler, not the physical port address the real Z80 code actually
// uses (0xC0+0x09=0xC9, mirrored at 0x80+0x09=0x89). Confirmed via
// sim (sor_board_tb.sv): the boot code's real /MCONT release sequence
// (bit0 0->1, hold-then-run) was showing up as writes to port 0xC9
// the entire session, which this design was silently ignoring because
// it only ever listened on raw 0x09. Alias each of these dynamically-
// relocated ports to all three addresses (raw, +0xC0, +0x80) so real
// boot code -- which uses the relocated addresses -- is recognized.
// Bank/cmd/stat/pal are NOT part of this dynamic scheme (they're
// static entries in master_redline_map_io at 0xF0/0xF2/0xF3/0xF4) and
// are unaffected.
wire io_bank  = (cpu_addr[7:0] == 8'hF0);   // bank register write (static map entry)
wire io_adc1  = (cpu_addr[7:0] == 8'hFD);   // P1 pedal, raw (MAME port 0xFD)
wire io_adc2  = (cpu_addr[7:0] == 8'hFE);   // P2 pedal, raw
wire io_adc3  = (cpu_addr[7:0] == 8'hFF);   // P3 pedal, raw
// Real wheel ports (MAME redline_state::init_offroad/init_offroadt):
// dedicated, fixed dial-encoded reads, entirely separate from the pedal
// ports above -- offroad_wheel_1_r=0xF9 (P1), offroad_wheel_2_r=0xFB (P2),
// offroad_wheel_3_r=0xF8 (P3). Fixed for both wheel-scheme games
// (offroad/offroadt); unused/unconsumed for JOY4_DIGITAL (pigout).
wire io_wheel1 = (cpu_addr[7:0] == 8'hF9);  // P1 wheel (offroad_wheel_1_r)
wire io_wheel2 = (cpu_addr[7:0] == 8'hFB);  // P2 wheel (offroad_wheel_2_r)
wire io_wheel3 = (cpu_addr[7:0] == 8'hF8);  // P3 wheel (offroad_wheel_3_r)

// WP-L3: the dynamically-relocated leland_master_input_r/output_w window
// (GIN0/1/3, mcont, ay, scroll) is now parameterized by io_base (MAME
// init_master_ports(mvram_base, io_base)) instead of offroad's hardcoded
// 0xC0/0x80. dual_io_window reproduces offroad's real double-install
// (base and base^0x40); offroadt/pigout install once at their own single
// base. Offsets within the window (0x00/0x01 GIN0/1, 0x09 mcont, 0x0A/0x0B
// ay, 0x0C-0x0F scroll, 0x11 GIN3) are fixed by leland_master_input_r/
// output_w itself and identical across all three games.
wire [7:0] io_base_alt = io_base ^ 8'h40;
function automatic io_win(input [7:0] offset);
	io_win = (cpu_addr[7:0] == (io_base + offset)) ||
	         (dual_io_window && (cpu_addr[7:0] == (io_base_alt + offset)));
endfunction

wire io_mcont = io_win(8'h09);
// AY8910 register-select (/OGIA, offset 0x0A) and data (/OGID, offset
// 0x0B) writes -- see gfxbank output declaration above for why these
// matter despite no audio synthesis being implemented.
wire io_ay_addr = io_win(8'h0A);
wire io_ay_data = io_win(8'h0B);
// Real MAME master_redline_map_io (leland.cpp): 0xF2 = 80186 sound CPU
// response_r/command_lo_w, 0xF4 = command_hi_w (write-only). Neither is
// a palette register -- offroad has no palette-bank register at all;
// real hardware only ever gates CRAM writes on/off via /MCONT bit1
// (m_palette_view, see leland_master_output_w case 0x09 and cram_we's
// mcont_r[1] gate below). Wired to the real sound board since WP10;
// command_lo_w/command_hi_w feed leland_sound_board.sv's own command
// latch, and response_r reads that module's real response_data output
// (see this file's own response_data port comment near the top).
wire io_cmd   = (cpu_addr[7:0] == 8'hF2);   // sound command_lo_w / response_r
wire io_snd_hi = (cpu_addr[7:0] == 8'hF4);  // sound command_hi_w (write-only)

// Background scroll registers (MAME scroll_w, offset 0x0C-0x0F relative
// to io_base -- see leland.cpp init_offroad()/init_offroadt()/init_pigout()).
wire io_scroll_xlo = io_win(8'h0C);
wire io_scroll_xhi = io_win(8'h0D);
wire io_scroll_ylo = io_win(8'h0E);
wire io_scroll_yhi = io_win(8'h0F);

// Wheel dial encoder — MAME leland_m.cpp dial_compute_value(), reproduced
// exactly: each read reports the direction (bit7) and magnitude (bits4:0,
// clamped to 0x1F, accumulated mod 32) of wheel motion since the LAST read
// of that same port, never an absolute angle. wheelN_now is computed
// combinationally from the live p*_wheel input so the value returned on
// THIS read already reflects it (matching MAME's synchronous read-and-update
// call); wheelN_last_input/wheelN_result latch once per read so the next
// read starts its delta from here. Latching is safe even if io_rd stays
// asserted across more than one CE_6M tick within a single Z80 IN
// instruction: p*_wheel won't have moved, so delta recomputes to 0 on any
// repeat tick, same self-idempotent pattern as this module's other
// once-per-instruction write strobes (see cmd_wr_lo_r etc. above).
function automatic [7:0] dial_compute(input [7:0] new_val, input [7:0] last_val, input [7:0] last_result);
	reg signed [8:0] delta;
	reg        [7:0] result;
	begin
		delta  = $signed({1'b0, new_val}) - $signed({1'b0, last_val});
		result = last_result & 8'h80;
		if (delta > 9'sd128)       delta = delta - 9'sd256;
		else if (delta < -9'sd128) delta = delta + 9'sd256;
		if (delta < 0) begin
			result = 8'h80;
			delta  = -delta;
		end else if (delta > 0) begin
			result = 8'h00;
		end
		if (delta > 9'sd31) delta = 9'sd31;
		result = result | ((last_result + delta[7:0]) & 8'h1F);
		dial_compute = result;
	end
endfunction

reg [7:0] wheel1_last_input, wheel2_last_input, wheel3_last_input;
reg [7:0] wheel1_result,     wheel2_result,     wheel3_result;

wire [7:0] wheel1_now = dial_compute(p1_wheel, wheel1_last_input, wheel1_result);
wire [7:0] wheel2_now = dial_compute(p2_wheel, wheel2_last_input, wheel2_result);
wire [7:0] wheel3_now = dial_compute(p3_wheel, wheel3_last_input, wheel3_result);

always @(posedge clk_sys) begin
	if (reset) begin
		wheel1_last_input <= 8'h00; wheel1_result <= 8'h00;
		wheel2_last_input <= 8'h00; wheel2_result <= 8'h00;
		wheel3_last_input <= 8'h00; wheel3_result <= 8'h00;
	end else if (CE_6M && io_rd) begin
		if (io_wheel1) begin wheel1_result <= wheel1_now; wheel1_last_input <= p1_wheel; end
		if (io_wheel2) begin wheel2_result <= wheel2_now; wheel2_last_input <= p2_wheel; end
		if (io_wheel3) begin wheel3_result <= wheel3_now; wheel3_last_input <= p3_wheel; end
	end
end

// GIN input ports — validated against MAME INPUT_PORTS and
// leland_master_input_r (leland_m.cpp), parameterized by io_base (see
// io_win() above). GIN0/1 sit at io_base+0/+1, GIN3 at io_base+0x11.
wire io_gin0  = io_win(8'h00);
wire io_gin1  = io_win(8'h01);
wire io_gin3  = io_win(8'h11);

// WHEELS3_PEDALS3 (offroad/offroadt) GIN0: nitro buttons, active-low
// (0=pressed). bit4=P1BTN1, bit5=P2BTN1, bit6=P3BTN1; other bits float high.
wire [7:0] gin0_wheels = {1'b1,
                          ~p3_btn[1],  // bit6: P3 nitro
                          ~p2_btn[1],  // bit5: P2 nitro
                          ~p1_btn[1],  // bit4: P1 nitro
                          4'hF};

// WHEELS3_PEDALS3 GIN1: slave HALT (bit0 active-low), coin inputs
// (bits1-3 active-low). bit1=COIN3, bit2=COIN2, bit3=COIN1 (MAME offroad
// INPUT_PORTS IN1 order is NOT P1/P2/P3).
wire [7:0] gin1_wheels = {4'hF,
                          ~p1_btn[3],  // bit3: coin P1
                          ~p2_btn[3],  // bit2: coin P2
                          ~p3_btn[3],  // bit1: coin P3
                          slave_halt_n}; // bit0: SLAVEHALT

// JOY4_DIGITAL (pigout) GIN0 (leland.cpp INPUT_PORTS_START(pigout), IN0 @
// io_base+0): bit1=P3BTN2, bit2=P3right, bit3=P3down, bit5=P2BTN2,
// bit6=P2left, bit7=P2up. p*_joy bit layout: [0]=right [1]=left [2]=down
// [3]=up [4]=btn1 [5]=btn2 [6]=start [7]=coin.
// All bits below are active-low on the real bus (0=pressed); p*_joy from
// MiSTer is active-high (1=pressed), hence the ~ on every mapped bit.
wire [7:0] gin0_joy4 = {~p2_joy[3],  // bit7: P2 up
                        ~p2_joy[1],  // bit6: P2 left
                        ~p2_joy[5],  // bit5: P2 btn2
                        1'b1,        // bit4: unused
                        ~p3_joy[2],  // bit3: P3 down
                        ~p3_joy[0],  // bit2: P3 right
                        ~p3_joy[5],  // bit1: P3 btn2
                        1'b1};       // bit0: unused

// JOY4_DIGITAL GIN1 (pigout IN1 @ io_base+1): bit0=SLAVEHALT, bit1=COIN1,
// bit3=COIN2 (bit2 read but never referenced by the real game).
wire [7:0] gin1_joy4 = {4'hF,
                        ~p2_joy[7],  // bit3: coin P2
                        1'b1,        // bit2: unreferenced
                        ~p1_joy[7],  // bit1: coin P1
                        slave_halt_n}; // bit0: SLAVEHALT

wire [7:0] gin0_data = (input_scheme == JOY4_DIGITAL) ? gin0_joy4 : gin0_wheels;
wire [7:0] gin1_data = (input_scheme == JOY4_DIGITAL) ? gin1_joy4 : gin1_wheels;

// GIN2 (io_base+0x10, JOY4_DIGITAL only -- pigout IN2): bit0=START3,
// bit1=P3BTN1, bit2=P3left, bit3=P3up, bit4=START2, bit5=P2BTN1,
// bit6=P2right, bit7=P2down. Unused for WHEELS3_PEDALS3 (never read).
wire io_gin2 = io_win(8'h10);
wire [7:0] gin2_data = {~p2_joy[2],  // bit7: P2 down
                        ~p2_joy[0],  // bit6: P2 right
                        ~p2_joy[4],  // bit5: P2 btn1
                        ~p2_joy[6],  // bit4: start2
                        ~p3_joy[3],  // bit3: P3 up
                        ~p3_joy[1],  // bit2: P3 left
                        ~p3_joy[4],  // bit1: P3 btn1
                        ~p3_joy[6]}; // bit0: start3

// GIN3 (port io_base+0x11): EEPROM DO (bit0), VBlank (bit1), service
// (bit2 for JOY4_DIGITAL/pigout, bit3 for WHEELS3_PEDALS3/offroad --
// bit width of PORT_SERVICE_NO_TOGGLE differs between the two real
// INPUT_PORTS blocks).
wire [7:0] gin3_wheels = {4'hF,
                          ~service,    // bit3: service (active-low)
                          1'b1,        // bit2: unused, float high
                          ~vblank,     // bit1: VBlank (active-low)
                          eeprom_do};  // bit0: real 93C46 DO
wire [7:0] gin3_joy4 = {5'h1F,       // bits7:3: unused, float high
                        ~service,    // bit2: service (active-low)
                        ~vblank,     // bit1: VBlank (active-low)
                        eeprom_do};  // bit0: real 93C46 DO
wire [7:0] gin3_data = (input_scheme == JOY4_DIGITAL) ? gin3_joy4 : gin3_wheels;

// WP-L3 pigout: fixed IN4 @ raw 0x7F (install_read_port, outside the
// io_base window entirely -- MAME init_pigout()). P1 full digital
// joystick + 2 buttons + start (leland.cpp INPUT_PORTS_START(pigout) IN4).
wire io_gin4 = in4_port_en && (cpu_addr[7:0] == 8'h7F);
wire [7:0] gin4_data = {~p1_joy[6],  // bit7: start1
                        ~p1_joy[5],  // bit6: P1 btn2
                        ~p1_joy[4],  // bit5: P1 btn1
                        ~p1_joy[0],  // bit4: P1 right
                        ~p1_joy[1],  // bit3: P1 left
                        ~p1_joy[2],  // bit2: P1 down
                        ~p1_joy[3],  // bit1: P1 up
                        1'b1};       // bit0: unused

// I/O writes
// Sound command latch (leland_a.cpp command_lo_w/command_hi_w): a
// 16-bit register the Master writes, shadowed here and also forwarded
// via cmd_wr_data/cmd_wr_lo/cmd_wr_hi to the real 80186 sound board's
// own command latch (leland_sound_board.sv, since WP10). The shadow
// regs below aren't read back on this side.
reg  [7:0] sound_cmd_lo_r, sound_cmd_hi_r;
// leland_a.cpp m_sound_response: retired 2026-07-18. Was a 4-iteration
// echo stub (sequence-bit-echo, see docs/WP10_PROGRESS.md history)
// standing in for the real 80186 response latch before the sound board
// existed. Now wired to the real thing (response_data port, see its
// own comment above) -- this WAS the actual seam WP10 always intended
// to close here; retiring it is what unblocked the real ROM's
// song-transition handshake (it posts a real response byte and waits
// for the Z80 to see it, which the stub could never produce).
reg  [7:0] mcont_r;       // /MCONT shadow register

reg [15:0] scroll_x_r, scroll_y_r;

// Minimal AY8910 register-select + Port A (register 0x0E) shadow --
// see gfxbank output declaration for why. ay_addr_r only needs to
// hold a register index (0-15); reset to a value that is NOT 0x0E so
// a stray/early io_ay_data write before the first io_ay_addr write
// (shouldn't happen on real hardware, but costs nothing to guard)
// can't accidentally latch garbage into gfxbank.
reg [3:0] ay_addr_r;
reg [7:0] gfxbank_r;

// WP10: sound-board control latch (see the sound_ctrl_data/sound_ctrl_wr
// port declarations above for the real-hardware rationale). sound_ctrl_wr
// is a plain 1-cycle strobe on every io_bank write, unconditional --
// MAME's own `diff==0 -> return` early-out in leland_80186_control_w is
// a software de-dup optimization, not real bus behavior; a real write
// strobe fires every time regardless of whether the value changed, and
// leland_sound_board.sv's control_wr handling is already idempotent.
reg [7:0] sound_ctrl_data_r;
reg       sound_ctrl_wr_r;
assign sound_ctrl_data = sound_ctrl_data_r;
assign sound_ctrl_wr   = sound_ctrl_wr_r;

// WP10: sound command latch outputs (same registered-strobe convention).
reg [7:0] cmd_wr_data_r;
reg       cmd_wr_lo_r, cmd_wr_hi_r;
assign cmd_wr_data = {cmd_wr_data_r, cmd_wr_data_r};
assign cmd_wr_lo   = cmd_wr_lo_r;
assign cmd_wr_hi   = cmd_wr_hi_r;

always @(posedge clk_sys) begin
	if (reset) begin
		mcont_r          <= 8'h00;  // slave_reset_n=0: hold Slave in reset
		bank_reg         <= 3'd0;
		sound_cmd_lo_r   <= 8'h00;
		sound_cmd_hi_r   <= 8'h00;
		scroll_x_r       <= 16'd0;
		scroll_y_r       <= 16'd0;
		ay_addr_r        <= 4'hF;
		gfxbank_r        <= 8'h00;  // matches MAME machine_reset()'s sound_port_w(0)
		sound_ctrl_data_r <= 8'h00;
		sound_ctrl_wr_r   <= 1'b0;
		cmd_wr_data_r     <= 8'h00;
		cmd_wr_lo_r       <= 1'b0;
		cmd_wr_hi_r       <= 1'b0;
	end else begin
		sound_ctrl_wr_r <= 1'b0;
		cmd_wr_lo_r     <= 1'b0;
		cmd_wr_hi_r     <= 1'b0;
		if (CE_6M && io_wr) begin
			if (io_bank) begin
				bank_reg          <= cpu_dout[2:0];
				sound_ctrl_data_r <= cpu_dout;
				sound_ctrl_wr_r   <= 1'b1;
			end
			if (io_cmd) begin
				sound_cmd_lo_r   <= cpu_dout;
				cmd_wr_data_r    <= cpu_dout;
				cmd_wr_lo_r      <= 1'b1;
			end
			if (io_snd_hi) begin
				sound_cmd_hi_r <= cpu_dout;
				cmd_wr_data_r  <= cpu_dout;
				cmd_wr_hi_r    <= 1'b1;
			end
			if (io_mcont) begin
				mcont_r         <= cpu_dout;
			end
			if (io_scroll_xlo) scroll_x_r[7:0]  <= cpu_dout;
			if (io_scroll_xhi) scroll_x_r[15:8] <= cpu_dout;
			if (io_scroll_ylo) scroll_y_r[7:0]  <= cpu_dout;
			if (io_scroll_yhi) scroll_y_r[15:8] <= cpu_dout;
			if (io_ay_addr) ay_addr_r <= cpu_dout[3:0];
			if (io_ay_data && (ay_addr_r == 4'hE)) gfxbank_r <= cpu_dout;
		end
	end
end

assign scroll_x      = scroll_x_r;
assign scroll_y      = scroll_y_r;
assign gfxbank        = gfxbank_r;
assign slave_reset_n = mcont_r[0];  // 1=run, 0=hold in reset
// MAME leland_master_output_w: set_input_line(NMI, BIT(data,2) ? CLEAR_LINE : ASSERT_LINE)
// i.e. bit2=1 -> NMI cleared (inactive), bit2=0 -> NMI asserted. slave_nmi_n
// is active-low, so it must equal mcont_r[2] directly, NOT its inverse --
// the previous `~mcont_r[2]` fired a spurious falling edge (NMI) on the tv80
// core at the exact moment the Master released the Slave from reset (the
// boot-time /MCONT write's bit2=1 was meant to clear/no-op the NMI line,
// not assert it), derailing the Slave into its NMI vector instead of
// continuing normal post-reset execution.
assign slave_nmi_n   = mcont_r[2]; // bit2=1 → clear NMI (active-low, no invert)
// MAME leland_master_output_w: set_input_line(INPUT_LINE_IRQ0, BIT(data,3) ?
// CLEAR_LINE : ASSERT_LINE) -- a held LEVEL, not a pulse. The Slave's INT
// line stays asserted for as long as /MCONT bit3=0 and is only cleared when
// the Master's firmware explicitly rewrites bit3=1; a masked/late EI on the
// Slave must still observe the pending INT. Previously this was a one-cycle
// pulse (slave_int_req_r, defaulting to 0 every cycle and only overridden
// for the single io_mcont-write cycle), which could not reproduce that
// held/repeating-INT behavior. mcont_r is already the registered /MCONT
// shadow, so deriving the level directly from its bit3 is both simpler and
// faithful to MAME.
assign slave_int_req = ~mcont_r[3];
// leland_master_output_w: m_eeprom->di_write(BIT(data,4)); clk_write(BIT(data,5)); cs_write(BIT(data,6));
assign eeprom_di  = mcont_r[4];
assign eeprom_clk = mcont_r[5];
assign eeprom_cs  = mcont_r[6];
// MAME leland_master_output_w: BIT(data,1) ? m_palette_view.select(0) :
// m_palette_view.disable() -- real hardware drops any 0xF000-0xF3FF write
// outright while the view is disabled (mcont_r[1]=0, the machine_reset()-
// time default) rather than landing it in Color RAM regardless. Ungated, a
// stray write during that window would permanently corrupt palette entries
// the fg/bg pen concatenation (sor_video.sv's cram_addr = {fg_pen, bg_pen})
// indexes into -- corrupting exactly the fg-nonzero rows that sprites/
// portrait/flag pixels read.
assign cram_we = mem_access & ~wr_n & in_cram & mcont_r[1];

//------------------------------------------------------------------
// CPU data input mux
//------------------------------------------------------------------
always @(*) begin
	cpu_din = 8'hFF;  // default: bus float
	if (mem_access && ~rd_n) begin
		if      (in_fixed || in_banked_lo || in_fixed_high) cpu_din = rom_data;
		else if (in_battram)             cpu_din = battram_dout;
		else if (in_wram)               cpu_din = wram_dout;
		else if (in_cram)               cpu_din = 8'hFF;  // palette write-only
	end else if (io_rd) begin
		if      (io_gin0) cpu_din = gin0_data;
		else if (io_gin1) cpu_din = gin1_data;
		else if (io_gin2) cpu_din = gin2_data;
		else if (io_gin3) cpu_din = gin3_data;
		else if (io_gin4) cpu_din = gin4_data;
		else if (io_adc1) cpu_din = p1_pedal;
		else if (io_adc2) cpu_din = p2_pedal;
		else if (io_adc3) cpu_din = p3_pedal;
		else if (io_wheel1) cpu_din = wheel1_now;
		else if (io_wheel2) cpu_din = wheel2_now;
		else if (io_wheel3) cpu_din = wheel3_now;
		// Port 0xF2 read = leland_80186_sound_device::response_r
		// (leland_a.cpp). This is a SEPARATE register from the command
		// latch written at the same address -- NOT an echo of the last
		// write. 2026-07-18: now the real 80186 response latch
		// (response_data port, see its own comment above) -- the
		// sound_response_r echo stub is retired (its own declaration
		// comment documents why it existed and what superseded it).
		else if (io_cmd)  cpu_din = response_data;
		else if (io_mvram) cpu_din = vram_rd_data;
	end
end

endmodule
