//============================================================================
//  Super Off Road — Slave Z80 CPU  (Chunk 3)
//
//  Slave Z80 memory map (Follow-up 5, docs/SESSION_2026-07-14.md:
//  offroad's machine config is `lelandi`, which installs MAME leland.cpp's
//  slave_large_map_program -- NOT slave_small_map_program, which an earlier
//  session had wrongly assumed. Validated against leland.cpp lines 315-323
//  and leland_m.cpp slave_large_banksw_w):
//    0x0000-0x1FFF  ROM bank 0 (fixed, slave_rom[0x00000..0x01FFF])
//    0x2000-0x3FFF  Unmapped (reads return 0xFF; no code/data lives here --
//                   confirmed via docs/reference/mame/traces/slavedump.asm,
//                   no read references this range)
//    0x4000-0xBFFF  Banked ROM window (32 KB)
//                   bank_base = 0x10000 + 0x8000*bank_reg, bank_reg>=14 ->
//                   0x10000 (MAME's out-of-range fallback)
//    0xC000         ROM bank register (write, bits[3:0]; exact single
//                   address -- NOT F803. slavedump.asm has nine
//                   `ld ($C000),a` bank-switch sites and zero references
//                   to F803.)
//    0xC001-0xDFFF  Unmapped (reads return 0xFF; no code/data lives here)
//    0xE000-0xEFFF  Work RAM (shared with Master, dual-port)
//    0xF800         VRAM address low byte  (write)
//    0xF801         VRAM address high byte (write; bit7=buffer select=addr[16])
//    0xF802         Raster line counter    (read)
//
//  Slave I/O map (validated against MAME slave_map_io):
//    0x00-0x1F (mirror 0x40): leland_svram_port_r/w -- full op semantics
//    live in sor_vram_port.sv (op = addr[2:0], inc = addr[3],
//    transparency = addr[4]).
//
//  Inter-CPU (validated against MAME -- there is NO dedicated command
//  port between the two Z80s):
//    Master -> Slave: /MCONT bit3 asserts the Slave INT line; commands
//    and data travel through a mailbox at the top of shared VIDEO RAM
//    (>= 0xF000), written/read by both CPUs through their own VRAM
//    I/O ports (see LOG_COMM in MAME leland_v.cpp vram_port_r/w).
//    Slave -> Master: the Slave executes HALT; the Master polls GIN1
//    bit0 (SLAVEHALT, via slave_halt_n).
//============================================================================

module sor_slave
(
	input         clk_sys,
	input         reset,
	input         CE_6M,

	// Slave program ROM
	output [18:0] rom_addr,
	input   [7:0] rom_data,

	// VRAM I/O port op stream (to sor_board's VRAM sequencer)
	output        vp_req,
	output        vp_rd,
	output        vp_trans,
	output [15:0] vp_addr,
	output  [7:0] vp_data,
	input         vp_pop,
	input   [7:0] vp_rdata,

	// Shared work RAM (same 4 KB as master)
	output [11:0] wram_addr,
	output  [7:0] wram_din,
	output        wram_we,
	input   [7:0] wram_dout,

	// Master asserted /MCONT bit3 → assert Slave INT (the only Slave INT
	// source in MAME; there is no Master->Slave command port)
	input         slave_int_req,

	// Master-driven NMI level (/MCONT bit2, MAME leland_master_output_w:
	// set_input_line(INPUT_LINE_NMI, BIT(data,2) ? CLEAR_LINE : ASSERT_LINE)).
	// Active-low, passed straight through to tv80's nmi_n -- tv80_core
	// does its own internal falling-edge detection (Oldnmi_n/NMI_s), so
	// no extra edge logic is needed here.
	input         nmi_n,

	output        slave_halt_n,  // Z80 HALT output → Master GIN1 bit0 (SLAVEHALT)

	// Raster line counter from video circuit
	input   [7:0] raster_line,

	// SDRAM stall: high while ROM byte is being fetched from SDRAM
	output        rom_req,    // level: high during any ROM read machine cycle
	input         rom_stall,  // high = data not ready; insert Z80 wait states

	// Debug-overlay use only (2026-07-25): raw level of the internal
	// vport_stall wire below, exposed so sor_board.sv can build a
	// per-frame "slave VRAM-port stall ticks" counter distinct from the
	// ROM-fetch stall above -- see vport_stall's own comment for why
	// this is a real board cost, not purely ours. Not consumed by any
	// game-logic path; safe to leave unconnected/ignored by any other
	// instantiation.
	output        vport_stall_out
);

//------------------------------------------------------------------
// Z80 bus
//------------------------------------------------------------------
wire        m1_n, mreq_n, iorq_n, rd_n, wr_n, rfsh_n, halt_n, busak_n;
wire [15:0] cpu_addr;
wire  [7:0] cpu_dout;
reg   [7:0] cpu_din;

//------------------------------------------------------------------
// Slave INT: /MCONT bit3 (leland_master_output_w case 0x09) is the
// only Slave INT source in MAME.
//------------------------------------------------------------------
reg int_n;
always @(posedge clk_sys) begin
	if (reset)
		int_n <= 1'b1;
	else if (slave_int_req)
		int_n <= 1'b0;
	else if (CE_6M && ~iorq_n && ~m1_n)  // INT acknowledge cycle
		int_n <= 1'b1;
end

// Memory region decode (moved up from below — mem_access/in_fixed/
// in_banked are used just below by rom_read_cyc; some SystemVerilog
// tools require declare-before-use even though Quartus tolerated the
// forward reference).
wire mem_access = ~mreq_n & rfsh_n;
wire in_fixed   = (cpu_addr[15:13] == 3'b000);            // 0x0000-0x1FFF
wire in_banked  = (cpu_addr >= 16'h4000) && (cpu_addr <= 16'hBFFF);

// ROM read: any access to fixed or banked ROM regions
wire rom_read_cyc = mem_access & ~rd_n & (in_fixed | in_banked);
assign rom_req = rom_read_cyc;

// vport_stall (from sor_vram_port below, forward-declared -- see
// sor_master.sv's identical mvport_stall for why this is safe) --
// holds the CPU with /WAIT instead of ever silently overwriting a
// not-yet-drained VRAM port op. See sor_vram_port.sv's vp_stall comment.
wire vport_stall;
assign vport_stall_out = vport_stall;

// Insert wait states when SDRAM has not yet returned ROM data, or the
// VRAM port isn't ready to accept a new op yet.
`ifdef NO_STALL_CONTROL
	// Measurement-only control knob (see sor_master.sv's identical guard and
	// docs/SESSION_2026-07-24_..._HANDOFF.md): neutralize the ROM-fetch
	// stall term for the slave Z80 too. Absent this define, bit-identical.
	wire cpu_wait_n = ~vport_stall;
`else
	wire cpu_wait_n = ~((rom_read_cyc & rom_stall) | vport_stall);
`endif

tv80s_ce #(.Mode(0), .T2Write(1), .IOWait(1)) slave_cpu
(
	.reset_n(~reset),
	.clk    (clk_sys),
	.cen    (CE_6M),
	.wait_n (cpu_wait_n),
	.int_n  (int_n),
	.nmi_n  (nmi_n),
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

assign slave_halt_n = halt_n;

//------------------------------------------------------------------
// ROM banking (slave_large_map_program / slave_large_banksw_w)
// Bank register at MMIO 0xC000 (memory write, bits[3:0]).
// 32 KB banked window at Z80 0x4000-0xBFFF → slave_rom offset via
// arithmetic (MAME leland_m.cpp slave_large_banksw_w):
//   bankaddress = 0x10000 + 0x8000 * (data & 0x0f)
//   if (bankaddress >= slave region length (0x80000)) bankaddress = 0x10000;
// i.e. bank_reg >= 14 falls back to 0x10000 (same as bank_reg==0).
//
// Slave ROM layout (from mra/SuperOffRoad.mra, single index="0" session),
// unchanged by this fix -- these are offsets WITHIN the slave ROM region
// (rtl/leland_board_pkg.sv ADDR_SLAVE_BASE, canonically 0x100000 in the
// post-header SDRAM address space; sor_board.sv adds that base to this
// module's rom_addr output -- the arithmetic below is a pure relative
// offset and does not change with the region base):
//   0x00000: u3   (8 KB)  ← fixed bank, not reached by banked window
//   0x02000-0x2FFFF: zero fill (184 KB gap — stored in SDRAM, reads return 0)
//   0x30000: u4t  (64 KB) ← bank_reg 4 (low half), 5 (high half)
//   0x40000: u5t  (64 KB) ← bank_reg 6 (low half), 7 (high half)
//   0x50000: u6t  (64 KB) ← bank_reg 8 (low half), 9 (high half)
//   0x60000: u7t  (64 KB) ← bank_reg 10 (low half), 11 (high half)
//   0x70000: u8t  (64 KB) ← bank_reg 12 (low half), 13 (high half)
// bank_reg 0-3 land in the zero-fill gap (0x10000-0x2FFFF) -- faithful
// to MAME, since the real slave ROM region is unpopulated there too.
//
// rom_addr is a byte offset relative to the slave ROM region's own base
// (added externally by sor_board.sv's sdram_rd1_addr); bank_base is that
// relative offset of the selected 32 KB bank.
//------------------------------------------------------------------
reg [3:0] bank_reg;

wire [18:0] bank_base = (bank_reg >= 4'd14) ? 19'h10000
                                             : (19'h10000 + {bank_reg, 15'd0});

//------------------------------------------------------------------
// Memory region decode (mem_access/in_fixed/in_banked declared above,
// near rom_read_cyc)
//------------------------------------------------------------------
wire in_wram    = (cpu_addr[15:12] == 4'hE);              // 0xE000-0xEFFF
wire in_f8xx    = mem_access && (cpu_addr[15:8] == 8'hF8); // 0xF800-0xF8FF

// ROM address mux
//
// Follow-up 23 (docs/SESSION_2026-07-15.md): MAME's redland_state::
// init_offroad() calls rotate_memory("slave") TWICE at driver-init time.
// rotate_memory left-rotates every 0x8000 (32KB) block (starting at
// region offset 0x10000, i.e. exactly this banked window) by 0x2000
// (8KB) per call -- two calls give a net rotation of 0x4000 (half the
// block). For a 15-bit within-block offset, "+0x4000 mod 0x8000" is
// mathematically identical to XORing bit 14 (0x4000): if bit14 is 0,
// adding 0x4000 sets it with no carry; if bit14 is 1, adding 0x4000
// overflows by exactly 0x4000, which is the same as clearing it. So a
// single XOR reproduces the double-rotate exactly. Verified byte-exact
// against a live dump of MAME's real "slave" ROM region (two passes of
// the actual rotate_memory C code applied to our un-rotated ROM image
// matched MAME's region dump 524288/524288 bytes, zero differences).
// The zero-fill gap (bank_reg 0-3) and the bank_reg>=14 fallback are
// also covered by MAME's rotation loop, but rotating an all-zero region
// is a no-op, so applying this XOR unconditionally to every banked read
// is safe and correct for all bank_reg values.
assign rom_addr = in_fixed  ? {6'b0, cpu_addr[12:0]}
                            : bank_base + ((19'(cpu_addr) - 19'h4000) ^ 19'h4000);

//------------------------------------------------------------------
// Work RAM
//------------------------------------------------------------------
assign wram_addr = cpu_addr[11:0];
assign wram_din  = cpu_dout;
assign wram_we   = mem_access & ~wr_n & in_wram;

//------------------------------------------------------------------
// VRAM I/O port (leland_svram_port_r/w + slave_video_addr_w) -- full
// MAME semantics live in sor_vram_port.sv. TRANS_EN=1: the Slave's
// bit-4 transparency ops are honored (num=1 in MAME's vram_port_w).
//------------------------------------------------------------------
wire io_rd   = ~iorq_n & ~rd_n;
wire io_wr   = ~iorq_n & ~wr_n;
// MAME slave_map_io installs this handler at 0x00-0x1F with .mirror(0x40),
// i.e. bit 6 is a don't-care -- the same port also answers at 0x40-0x5F.
wire io_vram = (cpu_addr[7] == 1'b0) && (cpu_addr[5] == 1'b0); // I/O 0x00-0x1F, 0x40-0x5F

// Memory-mapped VRAM address latch: exactly 0xF800/0xF801
wire vidlat_wr = CE_6M && mem_access && ~wr_n && in_f8xx && (cpu_addr[7:1] == 7'd0);

wire [7:0] vram_rd_data;

sor_vram_port #(.TRANS_EN(1'b1)) vport
(
	.clk_sys(clk_sys),
	.reset(reset),
	.CE_6M(CE_6M),

	.cpu_addr(cpu_addr),
	.cpu_dout(cpu_dout),
	.io_wr(io_wr),
	.io_rd(io_rd),
	.io_vram_sel(io_vram),
	.vidlat_wr(vidlat_wr),
	.vidlat_hi(cpu_addr[0]),

	.rd_data(vram_rd_data),
	.vp_stall(vport_stall),

	.vp_req(vp_req),
	.vp_rd(vp_rd),
	.vp_trans(vp_trans),
	.vp_addr(vp_addr),
	.vp_data(vp_data),
	.vp_pop(vp_pop),
	.vp_rdata(vp_rdata)
);

//------------------------------------------------------------------
// Bank register write: exactly 0xC000 (MAME slave_large_banksw_w).
// Reset value 0 matches MAME machine_reset(), which sets the slave
// bankslot base to slave_base[0x10000] -- the same address bank_reg==0
// produces via the bank_base arithmetic above.
//------------------------------------------------------------------
wire bank_wr = CE_6M && mem_access && ~wr_n && (cpu_addr == 16'hC000);

always @(posedge clk_sys) begin
	if (reset)
		bank_reg <= 4'd0;
	else if (bank_wr)
		bank_reg <= cpu_dout[3:0]; // 0xC000 bank switch
end

//------------------------------------------------------------------
// CPU data input mux
//------------------------------------------------------------------
always @(*) begin
	cpu_din = 8'hFF;
	if (mem_access && ~rd_n) begin
		if      (in_fixed || in_banked) cpu_din = rom_data;
		else if (in_wram)               cpu_din = wram_dout;
		else if (in_f8xx && cpu_addr[1:0] == 2'b10) cpu_din = raster_line; // 0xF802
	end else if (io_rd) begin
		if (io_vram) cpu_din = vram_rd_data;
	end
end

endmodule
