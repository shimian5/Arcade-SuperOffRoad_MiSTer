// i186_periph -- 80186 internal peripheral register block (WP2 of
// docs/planning_80186_sound.md): relocation register + chip-select
// unit (CSU). Net-new RTL, spec'd from MAME's i186.cpp (behavioral
// reference, not code reuse -- see docs/reference/mame/i186.cpp) and
// validated against real observed values from WP0
// (docs/reference/mame/traces/wp0_offroad_80186_sound_config.md).
//
// Sits inline on the s80x86 Core's DATA bus only (instruction fetch
// never targets the internal page in practice, and MAME's own
// interception is data-bus-only -- see docs/WP2_PROGRESS.md): when an
// access hits the relocation-selected internal-register page, this
// module services it directly (no downstream forwarding); everything
// else passes through untouched to the system bus.
//
// Registers implemented: RELOC (0xFE), UMCS (0xA0), LMCS (0xA2),
// PACS (0xA4), MMCS (0xA6), MPCS (0xA8), Timers 0/1/2 (0x50-0x66,
// WP3 -- see the "Timer block" section below). Interrupt-controller/
// DMA registers are WP4/WP8 -- unimplemented offsets here just
// read back 0 and ignore writes (own TODO markers, not silent bit-rot:
// grep `WP4`/`WP8` in this file for exactly which offsets those
// packages need to claim).
//
// Timer block (WP3, spec'd line-by-line from i186.cpp's timer_elapsed/
// restart_timer/inc_timer/internal_timer_update -- docs/WP3_PROGRESS.md
// has the full derivation): all three timers tick on a shared CLKOUT/4
// enable (`tick4`, a free-running divide-by-4 of `clk`, which this
// module treats as the 80186's internal CLKOUT per WP0's cross-validated
// ~7.5kHz TMROUT0 measurement). Unlike MAME's event-scheduled model
// (which recomputes an attotime-until-next-elapse on every register
// write), count/maxA/maxB are live registers here and comparisons are
// evaluated every tick -- functionally identical, no "sync" bookkeeping
// needed since there is no scheduled-event state to keep consistent.
// Timer 2 has no maxB/ALT/TMROUT (control-write resbits force those
// bits to always read back their reset value of 0, matching i186.cpp's
// 0x1fde resbits mask for timer 2) but drives timer 0/1's optional
// prescale-chain input (control bits [3:2]=="10") on its own terminal
// count, exactly mirroring i186.cpp's `timer_elapsed(2)` calling
// `inc_timer(0)`/`inc_timer(1)` directly instead of scheduling them.
//
// UMCS/LMCS/MMCS have no further side effect in this system (no real
// external chip-select pins to drive -- P3 priority per the plan's §2
// table): plain read/write-with-forced-bits stubs.
//
// Interrupt controller (WP4, offsets 0x10-0x1f, spec'd from i186.cpp's
// update_interrupt_state/int_callback/handle_eoi/external_int --
// docs/WP4_PROGRESS.md has the full derivation). **Non-iRMX only**
// (reloc bit14 is hardwired off per WP0/plan -- every BIT(m_reloc,14)
// branch in the MAME source is the dead branch here and is simply
// omitted, not conditionally compiled). Cascade/SFNM slave-vector-read
// modes are likewise omitted (assert-not-configured, per the plan's
// §2 table) -- if a ROM ever programs CASCADE mode this model just
// won't behave like real hardware there, a documented simplification.
//
// Vector-output stability contract (the actual reason this is its own
// section, not just "port more MAME code"): WP1's G4 seam-gate
// (docs/WP1_PROGRESS.md) validated that the s80x86 core requires
// `irq[7:0]` to stay stable from the moment `intr` asserts until the
// `inta` acknowledge pulse -- corrupting it even one cycle early can
// change the vector taken. MAME's own C++ model does NOT preserve this
// property literally (update_interrupt_state() is called eagerly on
// every register write and will silently overwrite a pending-but-not-
// yet-acked vector if a higher-priority source shows up in the
// meantime -- harmless in a single-threaded event-driven emulator
// where the CPU always samples "soon enough", but a real bug if
// ported literally into concurrent RTL). This module instead follows
// the plan's own §1.3 contract directly: the presented vector
// (`intc_poll_status`/`irq_out`) only ever gets recomputed while
// `intr` is currently low, or exactly at the `inta` (or poll-register-
// read-with-a-pending-request, which MAME treats identically -- see
// the POLL/0x12 handling below) acknowledge event.

module i186_periph(
    input  logic        clk,
    input  logic        reset,
    input  logic        ce_8m,      // WP10: real ~8MHz-equivalent clock
                                     // enable, gates ONLY the internal
                                     // timer tick (tick4_div below) --
                                     // tie high in any bench/integration
                                     // that wants this module's timers
                                     // to tick every clk (all existing
                                     // testbenches do exactly that)

    // CPU-side data bus (from Core.sv)
    input  logic [19:1] cpu_data_m_addr,
    output logic [15:0] cpu_data_m_data_in,
    input  logic [15:0] cpu_data_m_data_out,
    input  logic        cpu_data_m_access,
    output logic        cpu_data_m_ack,
    input  logic        cpu_data_m_wr_en,
    input  logic [1:0]  cpu_data_m_bytesel,
    input  logic        cpu_d_io,

    // System-side data bus (pass-through for non-internal accesses)
    output logic [19:1] sys_data_m_addr,
    input  logic [15:0] sys_data_m_data_in,
    output logic [15:0] sys_data_m_data_out,
    output logic        sys_data_m_access,
    input  logic        sys_data_m_ack,
    output logic        sys_data_m_wr_en,
    output logic [1:0]  sys_data_m_bytesel,
    output logic        sys_d_io,

    // CSU outputs for WP6 (external PCS0-5 window placement)
    output logic [19:0] ext_window_base,
    output logic        ext_window_is_mem,
    output logic        ext_window_valid,

    // Register readback for Stage-B directed tests / debug
    output logic [15:0] reloc_reg,
    output logic [15:0] umcs_reg,
    output logic [15:0] lmcs_reg,
    output logic [15:0] pacs_reg,
    output logic [15:0] mmcs_reg,
    output logic [15:0] mpcs_reg,

    // --- WP3: timer pins/interrupts ---
    output logic         tmrout0,
    output logic         tmrout1,
    output logic [2:0]   timer_irq,       // 1-cycle pulse per timer on
                                           // int-enabled terminal count
                                           // (WP4 latches these into the
                                           // sticky interrupt-status reg)
    output logic [2:0]   timer_tc_pulse,  // 1-cycle pulse per timer on
                                           // *every* terminal count,
                                           // unconditional on the
                                           // interrupt-enable bit --
                                           // i186.cpp's timer_elapsed()
                                           // drives m_out_tmrout0_func(1)
                                           // on every TC regardless of
                                           // control&0x2000 (that gate
                                           // only affects m_intr.status).
                                           // tmrout0/1 above model the
                                           // *pin level*, which in
                                           // non-ALT mode asserts once
                                           // and never deasserts on its
                                           // own -- board-level DAC
                                           // clock-service logic (WP6)
                                           // needs the repeated event,
                                           // not a level edge, so it
                                           // consumes this instead.
    output logic [15:0]  t_count  [0:2],
    output logic [15:0]  t_maxA   [0:2],
    output logic [15:0]  t_maxB   [0:2],
    output logic [15:0]  t_control[0:2],

    // --- WP4: interrupt controller / Core adapter ---
    output logic         intr,       // level, to Core.sv
    output logic [7:0]   irq_out,    // vector, to Core.sv (sampled at inta)
    input  logic         inta,       // 1-cycle ack pulse, from Core.sv
    input  logic         int0_pin,   // board-level INT0 (async, synchronized here)
    input  logic         int1_pin,   // board-level INT1 (async, synchronized here)
    input  logic         dma0_irq_req, // 1-cycle pulse stub for WP8 (DMA0 TC)
    input  logic         dma1_irq_req, // 1-cycle pulse stub for WP8 (DMA1 TC)

    // Register readback for Stage-B directed tests / debug
    output logic [7:0]   intc_request_reg,
    output logic [7:0]   intc_in_service_reg,
    output logic [2:0]   intc_status_reg,
    output logic [3:0]   intc_timer0_ctrl_reg,
    output logic [3:0]   intc_dma0_ctrl_reg,
    output logic [3:0]   intc_dma1_ctrl_reg,
    output logic [6:0]   intc_ext0_ctrl_reg,
    output logic [6:0]   intc_ext1_ctrl_reg,

    // --- WP8: internal DMA channels 0/1 ---
    input  logic         drq0_pit_level, drq1_pit_level, // raw PIT0 OUT0/OUT1 levels (leland_sound_board)
    input  logic         drq0_clear,     drq1_clear,     // 1-cycle software-clear strobes (leland_sound_board's dac_w decode)

    // Register readback for Stage-B/D directed tests / debug
    output logic [19:0]  dma_src     [0:1],
    output logic [19:0]  dma_dst     [0:1],
    output logic [15:0]  dma_count   [0:1],
    output logic [15:0]  dma_control [0:1],
    output logic [1:0]   dma_active,       // per-channel: FSM currently servicing this channel
    output logic         dma_byte_done,    // 1-cycle pulse: one byte fully transferred (either channel)
    output logic         dma_byte_done_ch  // which channel dma_byte_done just serviced (valid same cycle)
);

// --- Relocation register ---
// Reset default 0x20FF: I/O mode (bit12=0), page 0xFF -> window
// 0xFF00-0xFFFF. Confirmed by WP0 as the value BEFORE the ROM
// reprograms it (i186.cpp device_reset: m_reloc = 0x20ff).
reg [15:0] reloc;
assign reloc_reg = reloc;

// --- CSU registers ---
reg [15:0] umcs, lmcs, pacs, mmcs, mpcs;
assign umcs_reg = umcs;
assign lmcs_reg = lmcs;
assign pacs_reg = pacs;
assign mmcs_reg = mmcs;
assign mpcs_reg = mpcs;

// External PCS window, recomputed continuously from PACS/MPCS (matches
// leland_a.cpp peripheral_ctrl: temp = (PACS & 0xffc0) << 4, space
// chosen by MPCS bit 6 of the *stored* (forced-bits-included) value).
assign ext_window_base = {pacs[15:6], 10'b0}; // (pacs & 0xffc0) << 4, 20 bits total
assign ext_window_is_mem = mpcs[6];
reg ext_window_written;
assign ext_window_valid = ext_window_written;

// --- WP4: interrupt controller registers (declared here, ahead of the
// read mux below, since this ModelSim requires strict declare-before-
// use for wires/regs referenced in an always_comb -- see the WP1
// tooling notes in docs/WP1_PROGRESS.md) ---
reg [7:0]  intc_vector;        // 0x10, iRMX-only functionally in real
                                // hardware -- plain RW stub here
reg [3:0]  intc_timer0_ctrl;   // 0x19 TCUCON: [2:0]=priority, [3]=MSK
reg [3:0]  intc_dma0_ctrl;     // 0x1a
reg [3:0]  intc_dma1_ctrl;     // 0x1b
reg [6:0]  intc_ext0_ctrl;     // 0x1c: [2:0]=pri,[3]=MSK,[4]=LTM,[5]=CASCADE(unused),[6]=SFNM
reg [6:0]  intc_ext1_ctrl;     // 0x1d
reg [4:0]  intc_ext2_ctrl;     // 0x1e: [2:0]=pri,[3]=MSK (no LTM/CASCADE/SFNM -- pin unconnected anyway)
reg [4:0]  intc_ext3_ctrl;     // 0x1f
reg [2:0]  intc_priority_mask; // 0x15 PRIMSK
reg [7:0]  intc_in_service;    // 0x16 INSERV
reg [7:0]  intc_request;       // bit0 unused/stored (real value is
                                // read-computed, see 0x17 above),
                                // bit1 unused, [3:2]=dma0/1, [7:4]=ext0-3
reg [2:0]  intc_status;        // 0x18 INTSTS low 3 bits: sticky
                                // per-timer terminal-count flags
reg [15:0] intc_poll_status;   // bit15=pending, [7:0]=vector -- the
                                // actual intr/irq_out source, per the
                                // stability contract described in this
                                // file's header comment
reg [7:0]  intc_ack_mask;      // latched alongside poll_status: which
                                // request/in_service bit(s) the
                                // *currently presented* vector maps to
reg [3:0]  intc_ext_state;     // current (synchronized) level of
                                // INT0-3 -- only bits 0/1 ever change,
                                // 2/3 have no pin and stay 0
reg        rescan_trigger_d;   // 1-cycle-delayed mutation_event (see
                                // the deferred-rescan note near the
                                // main interrupt-controller always_ff)
reg        rescan_force_d;     // 1-cycle-delayed do_ack -- allows the
                                // deferred rescan to override a still-
                                // pending poll_status

assign intr    = intc_poll_status[15];
assign irq_out = intc_poll_status[7:0];
assign intc_request_reg     = intc_request;
assign intc_in_service_reg  = intc_in_service;
assign intc_status_reg      = intc_status;
assign intc_timer0_ctrl_reg = intc_timer0_ctrl;
assign intc_dma0_ctrl_reg   = intc_dma0_ctrl;
assign intc_dma1_ctrl_reg   = intc_dma1_ctrl;
assign intc_ext0_ctrl_reg   = intc_ext0_ctrl;
assign intc_ext1_ctrl_reg   = intc_ext1_ctrl;

// --- WP8: DMA channel registers (0x60-0x6d) ---
// dma_control bit map (i186.cpp, DMA control register -- see this
// module's own case-statement comments below for the exact bit
// positions used): [15]=DEST_MIO [14]=DEST_DEC [13]=DEST_INC
// [12]=SRC_MIO [11]=SRC_DEC [10]=SRC_INC [9]=TERMINATE_ON_ZERO
// [8]=INTERRUPT_ON_ZERO [7:6]=SYNC_MASK(00=unsync,01=src-sync,10=dst-sync)
// [5]=CHANNEL_PRIORITY [4]=TIMER_DRQ [2]=CHG_NOCHG(write-only, self-clears)
// [1]=ST_STOP(1=running) [0]=BYTE_WORD(1=word transfer).
// dma_src/dma_dst/dma_count/dma_control are themselves the output ports
// (declared in the port list above) -- same convention as t_count/
// t_maxA/t_control's own readback in the WP3 timer block, no separate
// shadow register needed.

// DRQ edge-latch (external PIT-driven channels) + internal-timer-2
// terminal-count latch (TIMER_DRQ mode) -- both sticky, cleared once
// the FSM below actually services that channel's request.
reg drq0_pit_d, drq1_pit_d;
reg drq0_latch, drq1_latch;
reg [1:0] timer2_latch; // [0]=ch0, [1]=ch1 -- both can independently select TIMER_DRQ

// DMA bus-master (feeds the internal second-level MemArbiter below)
reg [1:0]  dma_state; // 0=idle 1=read(source) 2=write(dest) 3=update
localparam DMA_IDLE = 2'd0, DMA_READ = 2'd1, DMA_WRITE = 2'd2, DMA_UPDATE = 2'd3;
reg        dma_active_ch;
reg [7:0]  dma_byte_val;
reg [15:0] dma_word_val;
reg        dma0_irq_pulse, dma1_irq_pulse;
assign dma_active = {dma_active_ch & (dma_state != DMA_IDLE), ~dma_active_ch & (dma_state != DMA_IDLE)};
assign dma_byte_done_ch = dma_active_ch;

// --- Internal-page hit decode ---
wire io_mode_hit  = !reloc[12] && cpu_d_io  && (cpu_data_m_addr[15:8] == reloc[7:0]);
wire mem_mode_hit = reloc[12]  && !cpu_d_io && (cpu_data_m_addr[19:8] == reloc[11:0]);
wire internal_hit_now = io_mode_hit || mem_mode_hit;

// internal_hit is latched for the duration of a transaction, not
// recomputed live every delta: a write to RELOC itself can change
// io_mode_hit/mem_mode_hit's *own* outcome (the relocation register
// controls whether an access is "internal" at all, including the very
// write transaction that's changing it), and reloc's new value becomes
// visible via NBA one delta after `internal_ack` commits -- if
// internal_hit were still combinational at that point, it flips
// mid-transaction, which flips `cpu_data_m_ack`'s mux source away from
// the internal_ack that just fired and toward sys_data_m_ack (0, since
// the far side hasn't even seen the access yet), which flips Core's own
// `access` (rtl/s80x86/LoadStore.sv computes it combinationally from
// `ack`), which flips internal_hit's downstream consumers again --
// closes into a genuine zero-delay oscillation. Never visible to any
// BFM-driven bench (a BFM's `access` is a plain testbench register, not
// itself a combinational function of `ack`) -- only surfaced once the
// real Core was wired in (`+autofindloop` caught it, see
// docs/WP8_PROGRESS.md). Latching at the moment `access` first asserts
// (mirroring this file's own `internal_ack` "hold while access is
// asserted" idiom) makes the hit decision a stable snapshot for the
// rest of the transaction, exactly matching how a real chip's decode
// can't retroactively un-claim a cycle it already committed to.
reg access_prev;
reg internal_hit_latched;
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        access_prev <= 1'b0;
        internal_hit_latched <= 1'b0;
    end else begin
        access_prev <= cpu_data_m_access;
        if (cpu_data_m_access && !access_prev)
            internal_hit_latched <= internal_hit_now;
    end
end
wire internal_hit = (cpu_data_m_access && !access_prev) ? internal_hit_now : internal_hit_latched;

// Word-offset within the page (7 bits: byte_addr[7:1], i.e. addr>>1 &
// 0x7f -- a direct slice, addr[7:1] is already exactly 7 bits wide).
wire [6:0] word_offset = cpu_data_m_addr[7:1];

// Byte-swap quirk (see docs/WP2_PROGRESS.md): bytesel==10 (high lane
// only) is the "port&1" (odd byte address) case in i186.cpp, targeting
// the register word's HIGH byte; bytesel==01 is the low byte.
// bytesel==11 only ever arises from an already-aligned word access
// (LoadStore.sv splits any misaligned CPU-level word access into two
// byte transactions before they reach this module), so it never needs
// swap handling.
wire byte_hi_only = cpu_data_m_bytesel == 2'b10;
wire byte_lo_only = cpu_data_m_bytesel == 2'b01;
wire word_access   = cpu_data_m_bytesel == 2'b11;

// --- Register read mux (combinational; internal registers have no
// extra read latency beyond the bus's own single-cycle-ack shape) ---
reg [15:0] reg_rd_val;
always_comb begin
    case (word_offset)
        7'h50: reg_rd_val = umcs;
        7'h51: reg_rd_val = lmcs;
        7'h52: reg_rd_val = pacs;
        7'h53: reg_rd_val = mmcs;
        7'h54: reg_rd_val = mpcs;
        7'h7f: reg_rd_val = reloc;
        7'h28: reg_rd_val = t_count[0];
        7'h29: reg_rd_val = t_maxA[0];
        7'h2a: reg_rd_val = t_maxB[0];
        7'h2b: reg_rd_val = t_control[0];
        7'h2c: reg_rd_val = t_count[1];
        7'h2d: reg_rd_val = t_maxA[1];
        7'h2e: reg_rd_val = t_maxB[1];
        7'h2f: reg_rd_val = t_control[1];
        7'h30: reg_rd_val = t_count[2];
        7'h31: reg_rd_val = t_maxA[2];
        7'h33: reg_rd_val = t_control[2];
        7'h10: reg_rd_val = intc_vector;
        7'h11: reg_rd_val = 16'h0000; // write-only (EOI); i186.cpp reads log an error and return nothing
        7'h12: reg_rd_val = intc_poll_status; // side effect (ack-equivalent) handled sequentially below
        7'h13: reg_rd_val = intc_poll_status;
        7'h14: reg_rd_val = {8'h00,
                              intc_ext3_ctrl[3], intc_ext2_ctrl[3], intc_ext1_ctrl[3], intc_ext0_ctrl[3],
                              intc_dma1_ctrl[3], intc_dma0_ctrl[3], 1'b0, intc_timer0_ctrl[3]};
        7'h15: reg_rd_val = {13'h0, intc_priority_mask};
        7'h16: reg_rd_val = {8'h00, intc_in_service};
        7'h17: reg_rd_val = {8'h00, intc_request[7:1], (intc_status != 3'b000)};
        7'h18: reg_rd_val = {13'h0, intc_status};
        7'h19: reg_rd_val = {12'h0, intc_timer0_ctrl};
        7'h1a: reg_rd_val = {12'h0, intc_dma0_ctrl};
        7'h1b: reg_rd_val = {12'h0, intc_dma1_ctrl};
        7'h1c: reg_rd_val = {9'h0, intc_ext0_ctrl};
        7'h1d: reg_rd_val = {9'h0, intc_ext1_ctrl};
        7'h1e: reg_rd_val = {11'h0, intc_ext2_ctrl};
        7'h1f: reg_rd_val = {11'h0, intc_ext3_ctrl};
        // WP8: DMA channel 0 (0x60-0x65) / channel 1 (0x68-0x6d).
        // Hi-address (source/dest upper 4 bits) registers read back with
        // the unused upper 12 bits as 0, matching i186.cpp's `source >> 16`
        // (source stored as a plain integer wider than the real 20-bit bus).
        7'h60: reg_rd_val = dma_src[0][15:0];
        7'h61: reg_rd_val = {12'h0, dma_src[0][19:16]};
        7'h62: reg_rd_val = dma_dst[0][15:0];
        7'h63: reg_rd_val = {12'h0, dma_dst[0][19:16]};
        7'h64: reg_rd_val = dma_count[0];
        7'h65: reg_rd_val = dma_control[0];
        7'h68: reg_rd_val = dma_src[1][15:0];
        7'h69: reg_rd_val = {12'h0, dma_src[1][19:16]};
        7'h6a: reg_rd_val = dma_dst[1][15:0];
        7'h6b: reg_rd_val = {12'h0, dma_dst[1][19:16]};
        7'h6c: reg_rd_val = dma_count[1];
        7'h6d: reg_rd_val = dma_control[1];
        default: reg_rd_val = 16'h0000;
    endcase
end

wire [7:0] reg_rd_byte = byte_hi_only ? reg_rd_val[15:8] : reg_rd_val[7:0];

// --- WP8: CPU pass-through is now master "a" of a second-level
// MemArbiter (docs/planning_80186_sound.md §1.2 Option A); the new DMA
// engine (declared further below) is master "b", higher priority,
// matching the vendored MemArbiter's own "b" = higher-priority
// convention and WP1's G2/G3-validated arb2 topology
// (docs/WP1_PROGRESS.md). The arbiter's combined output becomes this
// module's sys_data_m_* port -- leland_sound_board downstream sees a
// single bus master exactly as before, whether a given transaction
// originated from the CPU or from DMA. See docs/WP8_PROGRESS.md
// decision #2/#3/#4 for the full rationale, including why this
// insertion does not reopen the ack-mux zero-delay-loop hazard class
// documented in docs/SOUND_SMOKETEST_PROGRESS.md. Declared here, ahead
// of the ack/data-in mux below that consumes cpu_pass_ack/
// cpu_pass_data_in, per this ModelSim's declare-before-use requirement
// (docs/WP1_PROGRESS.md tooling notes).
wire [19:1] cpu_pass_addr    = cpu_data_m_addr;
wire [15:0] cpu_pass_data_out = cpu_data_m_data_out;
wire        cpu_pass_access  = cpu_data_m_access && !internal_hit;
wire        cpu_pass_wr_en   = cpu_data_m_wr_en;
wire [1:0]  cpu_pass_bytesel = cpu_data_m_bytesel;
wire        cpu_pass_ack;
wire [15:0] cpu_pass_data_in;

wire [19:1] dma_m_addr;
wire [15:0] dma_m_data_out;
wire        dma_m_access;
wire        dma_m_wr_en;
wire [1:0]  dma_m_bytesel;
wire        dma_m_ack;
wire [15:0] dma_m_data_in;
wire        dma_m_d_io;

wire        sys_arb_grant_b; // 1 = DMA currently granted the shared bus

MemArbiter u_dma_arb(
    .clk(clk), .reset(reset),
    .a_m_addr(cpu_pass_addr), .a_m_data_in(cpu_pass_data_in), .a_m_data_out(cpu_pass_data_out),
    .a_m_access(cpu_pass_access), .a_m_ack(cpu_pass_ack), .a_m_wr_en(cpu_pass_wr_en), .a_m_bytesel(cpu_pass_bytesel),
    .b_m_addr(dma_m_addr), .b_m_data_in(dma_m_data_in), .b_m_data_out(dma_m_data_out),
    .b_m_access(dma_m_access), .b_m_ack(dma_m_ack), .b_m_wr_en(dma_m_wr_en), .b_m_bytesel(dma_m_bytesel),
    .q_m_addr(sys_data_m_addr), .q_m_data_in(sys_data_m_data_in), .q_m_data_out(sys_data_m_data_out),
    .q_m_access(sys_data_m_access), .q_m_ack(sys_data_m_ack), .q_m_wr_en(sys_data_m_wr_en), .q_m_bytesel(sys_data_m_bytesel),
    .q_b(sys_arb_grant_b));

// d_io has no arbiter port (MemArbiter is vendored, addr/data/access/
// ack/wr_en/bytesel only) -- mux it externally on the same grant basis
// (q_b) the arbiter itself uses for everything else, so there is no
// separate/inconsistent classification basis to go stale.
assign sys_d_io = sys_arb_grant_b ? dma_m_d_io : cpu_d_io;

// --- Ack + data-in (1-cycle latency, matching the rest of this
// system's bus convention) ---
reg internal_ack;
assign cpu_data_m_ack = internal_hit ? internal_ack : cpu_pass_ack;
assign cpu_data_m_data_in = internal_hit ?
    (word_access ? reg_rd_val : {8'h00, reg_rd_byte}) : cpu_pass_data_in;

// --- WP3: timer datapath (combinational side) ---
// Generic bus-write value (word or RMW'd-byte, same byte-swap-quirk
// convention as the CSU registers) -- reused for every timer register.
wire [15:0] bus_wr_val = word_access ? cpu_data_m_data_out :
    byte_hi_only ? {cpu_data_m_data_out[7:0], reg_rd_val[7:0]} :
                   {reg_rd_val[15:8], cpu_data_m_data_out[7:0]};

wire do_write = internal_hit && cpu_data_m_access && !internal_ack && cpu_data_m_wr_en;

wire wr_t0cnt  = do_write && (word_offset == 7'h28);
wire wr_t0cmpa = do_write && (word_offset == 7'h29);
wire wr_t0cmpb = do_write && (word_offset == 7'h2a);
wire wr_t0con  = do_write && (word_offset == 7'h2b);
wire wr_t1cnt  = do_write && (word_offset == 7'h2c);
wire wr_t1cmpa = do_write && (word_offset == 7'h2d);
wire wr_t1cmpb = do_write && (word_offset == 7'h2e);
wire wr_t1con  = do_write && (word_offset == 7'h2f);
wire wr_t2cnt  = do_write && (word_offset == 7'h30);
wire wr_t2cmpa = do_write && (word_offset == 7'h31);
wire wr_t2con  = do_write && (word_offset == 7'h33);

// Shared CLKOUT/4 tick enable (free-running divide-by-4 of clk, gated
// by ce_8m -- WP10). This module's own bus/register/DMA/interrupt logic
// deliberately runs un-gated at the caller's full clk rate (no audio
// correctness depends on that -- see docs/WP10_PROGRESS.md's clocking
// section), but the internal timers' own tick genuinely paces dac9's
// real sample rate on real hardware (T0CMPA/TMROUT0, WP0's
// cross-validated ~7.5kHz finding), so it must tick at the real
// CLKOUT/4 rate regardless of what rate the surrounding bus logic
// happens to run at. Every existing testbench ties ce_8m=1'b1
// (unconditional tick, byte-for-byte identical to this module's
// pre-WP10 behavior); only the real board integration feeds a genuine
// ~8MHz-equivalent clock enable.
reg [1:0] tick4_div;
// 2026-07-18 (real-hardware "sound plays 6x too fast, crackling"
// investigation, docs/WP10_PROGRESS.md): pulse-width bug found by
// Fable. `tick4_div` only ADVANCES on `ce_8m` (1-in-6 of clk_sys), so
// each value it holds -- including 3 -- is held for 6 CONSECUTIVE
// clk_sys cycles. But `tick4` was sampled every clk_sys edge
// (unconditionally, not itself gated by ce_8m), so the old
// `tick4_div == 2'd3` alone read true for all 6 of those held cycles,
// not just the one cycle `ce_8m` actually advanced into it -- the
// downstream timer-increment logic (direct_tick0/1/2) then incremented
// 6 TIMES per intended tick instead of once, running the internal
// timers (hence dac9's T0CMPA-derived sample rate, and the firmware's
// own timer-interrupt-paced servicing cadence) at 6x real speed --
// exactly the reported symptom, and exact arithmetic match (6/24 *
// 48MHz = 12MHz vs. the correct CLKOUT/4 = 2MHz = 6.0x). Invisible in
// every simulation run because every existing testbench ties
// ce_8m=1'b1 unconditionally (see this wire's own comment above,
// "every existing testbench ties ce_8m=1'b1"), which makes tick4_div
// advance every single cycle -- so tick4_div==3 only ever holds for
// exactly 1 cycle there, and the bug can't manifest. Fixed by ANDing
// ce_8m directly into tick4 itself, so it's a genuine single-cycle
// pulse regardless of how long tick4_div holds any given value.
wire tick4 = (tick4_div == 2'd3) && ce_8m;

// Timer 2 (no maxB/ALT/TMROUT; drives timer 0/1's optional prescale
// chain on its own terminal count -- declared first since timer 0/1
// reference elapse2, and this ModelSim requires strict
// declare-before-use for wires, see docs/WP1_PROGRESS.md tooling notes).
wire ext2         = t_control[2][2];
wire direct_tick2 = t_control[2][15] && !ext2 && tick4;
wire [15:0] next_count2 = t_count[2] + 16'd1;
wire elapse2       = direct_tick2 && (next_count2 == t_maxA[2]);

// Timer 0
wire ext0   = t_control[0][2];
wire presc0 = t_control[0][3] && !ext0; // prescaled by timer 2
wire alt0   = t_control[0][1];
wire riu0   = t_control[0][12];
// Selection is by RIU, not by "ALT enabled" (i186.cpp: `(control &
// 0x1000) ? maxB : maxA`) -- when ALT=0, RIU is always forced 0 by the
// write-masking/reprime rules below, so this reduces to maxA in that
// case without needing a separate ALT check here.
wire [15:0] target0 = riu0 ? t_maxB[0] : t_maxA[0];
wire direct_tick0 = t_control[0][15] && !ext0 && !presc0 && tick4;
wire presc_tick0  = t_control[0][15] && presc0 && elapse2;
wire inc0 = direct_tick0 || presc_tick0;
wire [15:0] next_count0 = t_count[0] + 16'd1;
wire elapse0 = inc0 && (next_count0 == target0);

// Timer 1 (mirror of timer 0)
wire ext1   = t_control[1][2];
wire presc1 = t_control[1][3] && !ext1;
wire alt1   = t_control[1][1];
wire riu1   = t_control[1][12];
wire [15:0] target1 = riu1 ? t_maxB[1] : t_maxA[1];
wire direct_tick1 = t_control[1][15] && !ext1 && !presc1 && tick4;
wire presc_tick1  = t_control[1][15] && presc1 && elapse2;
wire inc1 = direct_tick1 || presc_tick1;
wire [15:0] next_count1 = t_count[1] + 16'd1;
wire elapse1 = inc1 && (next_count1 == target1);

// Control-register write masking: bits in `resbits` are hardware-owned
// (preserved from the old value, never settable by a CPU write); INH
// (bit14) gates whether EN (bit15) actually updates from this write,
// and always reads back 0 (i186.cpp internal_timer_update).
function automatic [15:0] mask_control(input [15:0] old_ctrl,
                                        input [15:0] new_raw,
                                        input [15:0] resbits);
    logic [15:0] merged;
    begin
        merged = (new_raw & ~resbits) | (old_ctrl & resbits);
        if (!merged[14])
            merged = (merged & ~16'h8000) | (old_ctrl & 16'h8000);
        merged[14] = 1'b0;
        mask_control = merged;
    end
endfunction

wire [15:0] t0con_new = mask_control(t_control[0], bus_wr_val, 16'h1fc0);
wire [15:0] t1con_new = mask_control(t_control[1], bus_wr_val, 16'h1fc0);
wire [15:0] t2con_new = mask_control(t_control[2], bus_wr_val, 16'h1fde);

// Post-elapse control mutation (i186.cpp timer_elapsed): MC (bit5)
// always sets; CONT (bit0) or mid-ALT-cycle (ALT && !RIU) reprimes
// (toggling RIU on the ALT case) and keeps EN running, otherwise the
// timer stops (EN and RIU both clear).
function automatic [15:0] control_after_elapse01(input [15:0] ctrl);
    logic reprime, new_riu, new_en;
    begin
        reprime = ctrl[0] || (ctrl[1] && !ctrl[12]);
        if (reprime) begin
            // i186.cpp: RIU sets only when (ALT && !RIU) [starting the B
            // half]; every other reprime case (not ALT, or ALT&&RIU
            // finishing the B half under CONT) unconditionally clears it
            // back to 0 -- NOT "leave unchanged".
            new_riu = (ctrl[1] && !ctrl[12]) ? 1'b1 : 1'b0;
            new_en  = ctrl[15];
        end else begin
            new_riu = 1'b0;
            new_en  = 1'b0;
        end
        control_after_elapse01 = ctrl;
        control_after_elapse01[5]  = 1'b1;
        control_after_elapse01[12] = new_riu;
        control_after_elapse01[15] = new_en;
    end
endfunction

// Timer 2 has no ALT (resbits force bit1 to always read 0), so reprime
// reduces to CONT alone and RIU never sets.
function automatic [15:0] control_after_elapse2(input [15:0] ctrl);
    logic new_en;
    begin
        new_en = ctrl[0] ? ctrl[15] : 1'b0;
        control_after_elapse2 = ctrl;
        control_after_elapse2[5]  = 1'b1;
        control_after_elapse2[15] = new_en;
    end
endfunction

// --- WP3: timer datapath (sequential side) ---
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        tick4_div <= 2'd0;
        t_count[0] <= 16'h0000; t_maxA[0] <= 16'h0000;
        t_maxB[0]  <= 16'h0000; t_control[0] <= 16'h0000;
        t_count[1] <= 16'h0000; t_maxA[1] <= 16'h0000;
        t_maxB[1]  <= 16'h0000; t_control[1] <= 16'h0000;
        t_count[2] <= 16'h0000; t_maxA[2] <= 16'h0000;
        t_maxB[2]  <= 16'h0000; t_control[2] <= 16'h0000;
        tmrout0        <= 1'b0;
        tmrout1        <= 1'b0;
        timer_irq      <= 3'b000;
        timer_tc_pulse <= 3'b000;
    end else begin
        if (ce_8m) tick4_div <= tick4_div + 2'd1;

        // Timer 0
        if (wr_t0cnt) t_count[0] <= bus_wr_val;
        else if (elapse0) t_count[0] <= 16'h0000;
        else if (inc0) t_count[0] <= next_count0;

        if (wr_t0cmpa) t_maxA[0] <= bus_wr_val;
        if (wr_t0cmpb) t_maxB[0] <= bus_wr_val;

        if (wr_t0con) t_control[0] <= t0con_new;
        else if (elapse0) t_control[0] <= control_after_elapse01(t_control[0]);

        tmrout0           <= elapse0 ? (!alt0 ? 1'b1 : riu0) : tmrout0;
        timer_irq[0]      <= elapse0 && t_control[0][13];
        timer_tc_pulse[0] <= elapse0;

        // Timer 1 (mirror)
        if (wr_t1cnt) t_count[1] <= bus_wr_val;
        else if (elapse1) t_count[1] <= 16'h0000;
        else if (inc1) t_count[1] <= next_count1;

        if (wr_t1cmpa) t_maxA[1] <= bus_wr_val;
        if (wr_t1cmpb) t_maxB[1] <= bus_wr_val;

        if (wr_t1con) t_control[1] <= t1con_new;
        else if (elapse1) t_control[1] <= control_after_elapse01(t_control[1]);

        tmrout1           <= elapse1 ? (!alt1 ? 1'b1 : riu1) : tmrout1;
        timer_irq[1]      <= elapse1 && t_control[1][13];
        timer_tc_pulse[1] <= elapse1;

        // Timer 2 (no maxB, no TMROUT)
        if (wr_t2cnt) t_count[2] <= bus_wr_val;
        else if (elapse2) t_count[2] <= 16'h0000;
        else if (direct_tick2) t_count[2] <= next_count2;

        if (wr_t2cmpa) t_maxA[2] <= bus_wr_val;

        if (wr_t2con) t_control[2] <= t2con_new;
        else if (elapse2) t_control[2] <= control_after_elapse2(t_control[2]);

        timer_irq[2]      <= elapse2 && t_control[2][13];
        timer_tc_pulse[2] <= elapse2;
    end
end

// --- WP4: interrupt controller combinational support (registers
// themselves are declared earlier, ahead of the read mux) ---

// INT0/INT1 synchronizers + edge detect (async board pins -- WP10
// decides the final clocking plan per R11 of the planning doc; a plain
// 2-FF synchronizer is the safe default in the meantime).
reg [1:0] int0_sync, int1_sync;
wire int0_level = int0_sync[1];
wire int1_level = int1_sync[1];
wire int0_edge  = int0_sync[1] != intc_ext_state[0];
wire int1_edge  = int1_sync[1] != intc_ext_state[1];

// --- Priority resolver (i186.cpp update_interrupt_state, non-iRMX
// scan order: for each priority 0..7, check timer, then dma0, then
// dma1, then ext0..ext3; masked sources never match since their
// stored [3:0] field with MSK=1 can never equal a 0-7 priority value).
// A source at the current priority level that is ALREADY in-service
// aborts the ENTIRE scan (mirrors MAME's literal `return;` from inside
// the loop, not `continue` -- once something is in service at a given
// priority tier, nothing at that tier or any lower-urgency tier further
// down the loop can be granted, only a strictly higher-priority source
// checked earlier in the loop already would have been).
typedef struct packed {
    logic       found;
    logic [7:0] vector;
    logic [7:0] ack_mask;
} resolve_t;

function automatic resolve_t resolve_irq(
    input [3:0] timer0_ctrl, input [2:0] timer_status,
    input [3:0] dma0_ctrl, input [3:0] dma1_ctrl,
    input dma0_req, input dma1_req,
    input [6:0] ext0_ctrl, input [6:0] ext1_ctrl,
    input [4:0] ext2_ctrl, input [4:0] ext3_ctrl,
    input ext0_req, input ext1_req, input ext2_req, input ext3_req,
    input [7:0] in_service
);
    resolve_t none, r;
    integer p;
    begin
        none.found = 1'b0; none.vector = 8'h00; none.ack_mask = 8'h00;

        for (p = 0; p < 8; p = p + 1) begin
            // Timer (irq bit 0x01 -> in_service[0])
            if (timer0_ctrl[3:0] == p[3:0]) begin
                if (in_service[0]) return none;
                if (timer_status != 3'b000) begin
                    r.found = 1'b1;
                    r.vector = timer_status[0] ? 8'h08 : (timer_status[1] ? 8'h12 : 8'h13);
                    r.ack_mask = 8'h01;
                    return r;
                end
            end
            // DMA0 (0x04 -> in_service[2])
            if (dma0_ctrl[3:0] == p[3:0]) begin
                if (in_service[2]) return none;
                if (dma0_req) begin
                    r.found = 1'b1; r.vector = 8'h0a; r.ack_mask = 8'h04;
                    return r;
                end
            end
            // DMA1 (0x08 -> in_service[3])
            if (dma1_ctrl[3:0] == p[3:0]) begin
                if (in_service[3]) return none;
                if (dma1_req) begin
                    r.found = 1'b1; r.vector = 8'h0b; r.ack_mask = 8'h08;
                    return r;
                end
            end
            // EXT0 (0x10 -> in_service[4]); SFNM (bit6) exempts an
            // in-service source from blocking the scan (i186.cpp's
            // extra "in_service && SFNM" branch also just returns --
            // net effect is the same either way, SFNM never actually
            // grants a nested request here since cascade/SFNM support
            // is otherwise omitted per the plan's §2 scoping).
            if (ext0_ctrl[3:0] == p[3:0]) begin
                if (in_service[4] && !ext0_ctrl[6]) return none;
                if (ext0_req) begin
                    r.found = 1'b1; r.vector = 8'h0c; r.ack_mask = 8'h10;
                    return r;
                end
                if (in_service[4] && ext0_ctrl[6]) return none;
            end
            // EXT1 (0x20 -> in_service[5])
            if (ext1_ctrl[3:0] == p[3:0]) begin
                if (in_service[5] && !ext1_ctrl[6]) return none;
                if (ext1_req) begin
                    r.found = 1'b1; r.vector = 8'h0d; r.ack_mask = 8'h20;
                    return r;
                end
                if (in_service[5] && ext1_ctrl[6]) return none;
            end
            // EXT2 (0x40 -> in_service[6]) -- no pin wired, so ext2_req
            // is permanently 0, but the priority/in-service bookkeeping
            // is still implemented per the plan ("implement the
            // registers, tie the pins low").
            if (ext2_ctrl[3:0] == p[3:0]) begin
                if (in_service[6]) return none;
                if (ext2_req) begin
                    r.found = 1'b1; r.vector = 8'h0e; r.ack_mask = 8'h40;
                    return r;
                end
            end
            // EXT3 (0x80 -> in_service[7])
            if (ext3_ctrl[3:0] == p[3:0]) begin
                if (in_service[7]) return none;
                if (ext3_req) begin
                    r.found = 1'b1; r.vector = 8'h0f; r.ack_mask = 8'h80;
                    return r;
                end
            end
        end
        return none;
    end
endfunction

resolve_t rescan_result;
assign rescan_result = resolve_irq(
    intc_timer0_ctrl, intc_status,
    intc_dma0_ctrl, intc_dma1_ctrl, intc_request[2], intc_request[3],
    intc_ext0_ctrl, intc_ext1_ctrl, intc_ext2_ctrl, intc_ext3_ctrl,
    intc_request[4], intc_request[5], intc_request[6], intc_request[7],
    intc_in_service);

// --- WP4: bus-write hit decode (same do_write/bus_wr_val convention
// as the timer registers) ---
wire wr_ivec   = do_write && (word_offset == 7'h10);
wire wr_eoi    = do_write && (word_offset == 7'h11);
wire wr_mask   = do_write && (word_offset == 7'h14);
wire wr_primsk = do_write && (word_offset == 7'h15);
wire wr_inserv = do_write && (word_offset == 7'h16);
wire wr_reqst  = do_write && (word_offset == 7'h17);
wire wr_intsts = do_write && (word_offset == 7'h18);
wire wr_tcucon = do_write && (word_offset == 7'h19);
wire wr_dma0con= do_write && (word_offset == 7'h1a);
wire wr_dma1con= do_write && (word_offset == 7'h1b);
wire wr_i0con  = do_write && (word_offset == 7'h1c);
wire wr_i1con  = do_write && (word_offset == 7'h1d);
wire wr_i2con  = do_write && (word_offset == 7'h1e);
wire wr_i3con  = do_write && (word_offset == 7'h1f);
wire intc_reg_write = wr_mask || wr_primsk || wr_inserv || wr_reqst || wr_intsts ||
                       wr_tcucon || wr_dma0con || wr_dma1con ||
                       wr_i0con || wr_i1con || wr_i2con || wr_i3con;

// --- WP8: DMA register bus-write decode (same do_write/bus_wr_val
// convention as every other internal register block in this file) ---
wire wr_dma0srclo = do_write && (word_offset == 7'h60);
wire wr_dma0srchi = do_write && (word_offset == 7'h61);
wire wr_dma0dstlo = do_write && (word_offset == 7'h62);
wire wr_dma0dsthi = do_write && (word_offset == 7'h63);
wire wr_dma0cnt   = do_write && (word_offset == 7'h64);
wire wr_dma0ctl   = do_write && (word_offset == 7'h65);
wire wr_dma1srclo = do_write && (word_offset == 7'h68);
wire wr_dma1srchi = do_write && (word_offset == 7'h69);
wire wr_dma1dstlo = do_write && (word_offset == 7'h6a);
wire wr_dma1dsthi = do_write && (word_offset == 7'h6b);
wire wr_dma1cnt   = do_write && (word_offset == 7'h6c);
wire wr_dma1ctl   = do_write && (word_offset == 7'h6d);

// Reading POLL (0x12) while a request is pending is treated identically
// to a real `inta` pulse (i186.cpp internal_port_r case 0x12: `if
// (poll_status & 0x8000) inta_callback(...)`) -- this is the mechanism
// behind WP0's observed poll-mode servicing (frequent POLLSTS/POLL
// reads instead of, or alongside, vectored dispatch).
wire poll_read_ack = internal_hit && cpu_data_m_access && !internal_ack &&
                      !cpu_data_m_wr_en && (word_offset == 7'h12) &&
                      intc_poll_status[15];

wire do_ack = inta || poll_read_ack;

// Rescan trigger: every event that would call update_interrupt_state()
// in i186.cpp (register writes, EOI, an INT0/1 edge, a WP3 timer_irq
// pulse, a WP8 DMA stub pulse, or the inta/poll-read ack itself).
// Deliberately does NOT feed resolve() combinationally in the same
// cycle the underlying status/request/in_service registers are being
// mutated by that same event -- those registers only commit their new
// value via nonblocking assignment, so a same-cycle combinational read
// would see last cycle's (pre-mutation) value, one cycle stale. Instead
// this trigger is *registered* (`rescan_trigger_d`/`rescan_force_d`
// below) and the actual resolve()-driven update happens the following
// cycle, once the mutation has genuinely settled -- avoids needing a
// bespoke "effective value" combinational patch for every one of the
// several distinct mutation paths (a first version of this file tried
// exactly that for the timer/DMA/ext-pin paths alone and still missed
// the EOI/ack in_service-clearing case; see docs/WP4_PROGRESS.md).
// WP8: OR the new DMA engine's own TC-interrupt pulses into the same
// signals i186_intc_tb.sv already drives directly as external stimulus
// -- backward-compatible (that testbench's own dma0_irq_req/dma1_irq_req
// stimulus still works unmodified), while real integration (this
// module's own DMA engine) now also feeds the same path.
wire dma0_irq_req_eff = dma0_irq_req || dma0_irq_pulse;
wire dma1_irq_req_eff = dma1_irq_req || dma1_irq_pulse;

wire mutation_event = do_ack || wr_eoi || intc_reg_write ||
                       int0_edge || int1_edge ||
                       timer_irq[0] || timer_irq[1] || timer_irq[2] ||
                       dma0_irq_req_eff || dma1_irq_req_eff;

// EOI (i186.cpp handle_eoi, non-iRMX branches only): specific form
// clears one in_service bit by explicit vector; nonspecific form
// (bit15 set) scans priority 0..7 and clears the first in-service
// source whose *priority field's low 3 bits* (mask bit excluded here,
// unlike the request-scan's 4-bit compare -- a real, faithfully-ported
// distinction from i186.cpp) match.
function automatic [7:0] eoi_clear_mask(
    input [15:0] data,
    input [2:0] timer0_pri, input [2:0] dma0_pri, input [2:0] dma1_pri,
    input [2:0] ext0_pri, input [2:0] ext1_pri, input [2:0] ext2_pri, input [2:0] ext3_pri,
    input [7:0] in_service
);
    integer p;
    begin
        eoi_clear_mask = 8'h00;
        if (!data[15]) begin
            case (data[4:0])
                5'h08, 5'h12, 5'h13: eoi_clear_mask = 8'h01;
                5'h0a: eoi_clear_mask = 8'h04;
                5'h0b: eoi_clear_mask = 8'h08;
                5'h0c: eoi_clear_mask = 8'h10;
                5'h0d: eoi_clear_mask = 8'h20;
                5'h0e: eoi_clear_mask = 8'h40;
                5'h0f: eoi_clear_mask = 8'h80;
                default: eoi_clear_mask = 8'h00;
            endcase
        end else begin
            for (p = 0; p < 8; p = p + 1) begin
                if (eoi_clear_mask == 8'h00) begin
                    if (timer0_pri == p[2:0] && in_service[0]) eoi_clear_mask = 8'h01;
                    else if (dma0_pri == p[2:0] && in_service[2]) eoi_clear_mask = 8'h04;
                    else if (dma1_pri == p[2:0] && in_service[3]) eoi_clear_mask = 8'h08;
                    else if (ext0_pri == p[2:0] && in_service[4]) eoi_clear_mask = 8'h10;
                    else if (ext1_pri == p[2:0] && in_service[5]) eoi_clear_mask = 8'h20;
                    else if (ext2_pri == p[2:0] && in_service[6]) eoi_clear_mask = 8'h40;
                    else if (ext3_pri == p[2:0] && in_service[7]) eoi_clear_mask = 8'h80;
                end
            end
        end
    end
endfunction

// --- WP4: interrupt controller sequential logic ---
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        intc_vector        <= 8'h00;
        intc_timer0_ctrl   <= 4'hf;
        intc_dma0_ctrl     <= 4'hf;
        intc_dma1_ctrl     <= 4'hf;
        intc_ext0_ctrl     <= 7'hf;
        intc_ext1_ctrl     <= 7'hf;
        intc_ext2_ctrl     <= 5'hf;
        intc_ext3_ctrl     <= 5'hf;
        intc_priority_mask <= 3'h7;
        intc_in_service    <= 8'h00;
        intc_request       <= 8'h00;
        intc_status        <= 3'h0;
        intc_poll_status   <= 16'h0000;
        intc_ack_mask      <= 8'h00;
        intc_ext_state     <= 4'h0;
        int0_sync          <= 2'b00;
        int1_sync          <= 2'b00;
        rescan_trigger_d   <= 1'b0;
        rescan_force_d     <= 1'b0;
    end else begin
        int0_sync <= {int0_sync[0], int0_pin};
        int1_sync <= {int1_sync[0], int1_pin};

        // --- Timer status latch (WP3's per-timer strobe -> sticky
        // status bits, i186.cpp timer_elapsed: `m_intr.status |= 0x01
        // << which`) ---
        if (timer_irq[0]) intc_status[0] <= 1'b1;
        if (timer_irq[1]) intc_status[1] <= 1'b1;
        if (timer_irq[2]) intc_status[2] <= 1'b1;

        // --- INT0/INT1 edge -> sticky request bit + ext_state ---
        // 2026-07-18 ("clean boot pop but no sound during real gameplay"
        // investigation, docs/WP10_PROGRESS.md): deliberately diverges
        // from a literal port of i186.cpp's external_int(), which
        // unconditionally sets/clears the request bit on every level
        // change. That's harmless in MAME's own execution model --
        // every I/O write is an atomic, instantaneous event between CPU
        // instructions, so a transition can never be missed -- but real
        // clocked silicon has no such guarantee, and this specific pin
        // is packed into the same byte as the master Z80's own
        // high-frequency graphics bank-switch register (leland_m.cpp's
        // redline_master_alt_bankswitch_w, wired at port 0xF0 -- see
        // this repo's own earlier WP10 finding). Most of those writes
        // aren't deliberately managing INT0 at all; bit5 can plausibly
        // go high then low again across two nearby bank-switch writes
        // before anything downstream (the 2-FF synchronizer above, this
        // controller's own resolve/rescan, the 80186 ROM's own poll-loop
        // cadence) ever samples the pending state -- a real command
        // notification silently vanishing with nothing having serviced
        // it, indistinguishable on the 80186 side from "nothing
        // happened".
        //
        // Fix, scoped to EDGE-triggered mode only (LTM=0, the reset
        // default -- i.e. what this board actually uses): a rising edge
        // latches a STICKY request that only clears on a genuine ack
        // (already correctly LTM-gated in the ack block below), matching
        // how real edge-triggered interrupt controllers actually behave
        // in silicon -- latch-and-hold, not "track the live level". A
        // falling edge still updates ext_state (still needed for that
        // same ack-time LTM check) but no longer clears the request.
        // LEVEL-triggered mode (LTM=1) is UNCHANGED from MAME's literal
        // behavior: the request bit continues to track the live pin
        // level exactly as external_int() does, which is correct for
        // level-triggered semantics (a still-asserted level stays
        // visible/re-detectable regardless of any one sample's timing,
        // so there's no analogous race to fix there).
        if (int0_edge) begin
            intc_ext_state[0] <= int0_level;
            if (intc_ext0_ctrl[4]) intc_request[4] <= int0_level; // LTM=1: MAME's literal level-tracking behavior
            else if (int0_level)   intc_request[4] <= 1'b1;       // LTM=0: rising edge sets; falling edge never clears here
        end
        if (int1_edge) begin
            intc_ext_state[1] <= int1_level;
            if (intc_ext1_ctrl[4]) intc_request[5] <= int1_level;
            else if (int1_level)   intc_request[5] <= 1'b1;
        end

        // --- DMA request pulses (external test stimulus OR'd with the
        // real WP8 DMA engine's own TC-interrupt pulses) ---
        if (dma0_irq_req_eff) intc_request[2] <= 1'b1;
        if (dma1_irq_req_eff) intc_request[3] <= 1'b1;

        // --- EOI (0x11 write): mutate in_service, then always rescan
        // regardless of current pending state -- EOI is one of the
        // explicit allowed-update points per this module's stability
        // contract (a freshly-cleared in_service slot may unblock a
        // previously-blocked lower-priority source). ---
        if (wr_eoi) begin
            automatic logic [7:0] clear_mask;
            clear_mask = eoi_clear_mask(bus_wr_val,
                intc_timer0_ctrl[2:0], intc_dma0_ctrl[2:0], intc_dma1_ctrl[2:0],
                intc_ext0_ctrl[2:0], intc_ext1_ctrl[2:0], intc_ext2_ctrl[2:0], intc_ext3_ctrl[2:0],
                intc_in_service);
            intc_in_service <= intc_in_service & ~clear_mask;
        end

        // --- Register writes (plain field stores; every one of these
        // triggers a rescan in i186.cpp, folded into the shared rescan
        // block below via `intc_reg_write`/`wr_eoi`) ---
        if (wr_ivec)    intc_vector        <= bus_wr_val[7:0];
        if (wr_mask) begin
            intc_timer0_ctrl[3] <= bus_wr_val[0];
            intc_dma0_ctrl[3]   <= bus_wr_val[2];
            intc_dma1_ctrl[3]   <= bus_wr_val[3];
            intc_ext0_ctrl[3]   <= bus_wr_val[4];
            intc_ext1_ctrl[3]   <= bus_wr_val[5];
            intc_ext2_ctrl[3]   <= bus_wr_val[6];
            intc_ext3_ctrl[3]   <= bus_wr_val[7];
        end
        if (wr_primsk)  intc_priority_mask <= bus_wr_val[2:0];
        if (wr_inserv)  intc_in_service    <= bus_wr_val[7:0];
        if (wr_reqst)   intc_request       <= (intc_request & ~8'h0c) | (bus_wr_val[7:0] & 8'h0c);
        if (wr_intsts)  intc_status        <= bus_wr_val[2:0];
        if (wr_tcucon)  intc_timer0_ctrl   <= bus_wr_val[3:0];
        if (wr_dma0con) intc_dma0_ctrl     <= bus_wr_val[3:0];
        if (wr_dma1con) intc_dma1_ctrl     <= bus_wr_val[3:0];
        if (wr_i0con)   intc_ext0_ctrl     <= bus_wr_val[6:0];
        if (wr_i1con)   intc_ext1_ctrl     <= bus_wr_val[6:0];
        if (wr_i2con)   intc_ext2_ctrl     <= bus_wr_val[4:0];
        if (wr_i3con)   intc_ext3_ctrl     <= bus_wr_val[4:0];

        // --- Ack processing (real `inta` pulse from Core, or an
        // equivalent POLL-register read) -- i186.cpp int_callback,
        // non-iRMX branch. Mutates status/request/in_service only;
        // does NOT touch intc_poll_status here (see the deferred
        // rescan below for why) -- `intr` therefore stays asserted,
        // continuously, with the just-acked vector still showing for
        // exactly one extra cycle until the deferred rescan overwrites
        // it, which is safe under G4's contract (property (c): only
        // corruption starting *after* the inta cycle is guaranteed not
        // to affect the vector already taken -- this is a same-or-
        // later-cycle change, and no future dispatch samples irq again
        // until the core reaches a fresh instruction boundary, long
        // after this settles).
        if (do_ack) begin
            if (intc_ack_mask == 8'h01) begin
                case (intc_poll_status[7:0])
                    8'h08: intc_status[0] <= 1'b0;
                    8'h12: intc_status[1] <= 1'b0;
                    8'h13: intc_status[2] <= 1'b0;
                    default: ;
                endcase
            end
            // Clear the request bit(s) this ack covers, UNLESS it's an
            // LTM-mode external source (0xf0 of ack_mask) -- a
            // level-triggered request stays sticky until the pin
            // itself deasserts, matching i186.cpp's LTM check exactly.
            if (intc_ack_mask[7:4] != 4'h0) begin
                automatic logic is_ltm;
                is_ltm = (intc_ack_mask[4] && intc_ext0_ctrl[4]) ||
                         (intc_ack_mask[5] && intc_ext1_ctrl[4]);
                         // ext2/ext3 have no LTM bit (unconnected pins,
                         // no external device to hold the level anyway)
                if (!is_ltm)
                    intc_request <= intc_request & ~intc_ack_mask;
            end else begin
                intc_request <= intc_request & ~intc_ack_mask;
            end
            intc_in_service <= intc_in_service | intc_ack_mask;
            intc_ack_mask   <= 8'h00;
        end

        // --- Deferred rescan: `rescan_trigger_d`/`rescan_force_d` were
        // set last cycle by whatever mutation just happened above (see
        // the module-header note on why this is a cycle late on
        // purpose -- resolve_irq's combinational inputs are now the
        // *settled* post-mutation register values, not stale ones).
        // Normal (non-force) rescans only take effect while nothing is
        // currently pending (the stability contract); an ack-forced
        // rescan is always allowed to load the next winner, since the
        // ack itself is one of the two explicitly-permitted update
        // points and `intc_poll_status` was deliberately left
        // unmodified above for exactly this cycle to overwrite.
        if (rescan_trigger_d && (rescan_force_d || !intc_poll_status[15])) begin
            if (rescan_result.found) begin
                intc_poll_status <= {1'b1, 7'h0, rescan_result.vector};
                intc_ack_mask    <= rescan_result.ack_mask;
            end else if (rescan_force_d) begin
                intc_poll_status <= 16'h0000;
            end
        end

        rescan_trigger_d <= mutation_event;
        rescan_force_d   <= do_ack;
    end
end

// --- Write + ack sequencing ---
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        reloc <= 16'h20ff;
        umcs  <= 16'h0000;
        lmcs  <= 16'h0000;
        pacs  <= 16'h0000;
        mmcs  <= 16'h0000;
        mpcs  <= 16'h0000;
        ext_window_written <= 1'b0;
        internal_ack <= 1'b0;
    end else begin
        // Ack holds high for as long as `access` stays asserted, only
        // clearing once access itself drops -- NOT a fixed 1-cycle
        // pulse. A fixed-pulse ack (ack<=1 one cycle, then
        // unconditionally <=0 the next even if access is still held)
        // has a real race: if the far side hasn't yet dropped `access`
        // by the cycle after the pulse, `access && !ack` becomes true
        // again and the transaction gets serviced a second time --
        // invisible for the CSU registers above (idempotent
        // overwrites) but corrupts the WP3 timer control register's
        // old-value-dependent masking (docs/WP3_PROGRESS.md has the
        // full bug writeup). Holding ack while access remains high
        // closes the window entirely: `do_write`/`wr_t*` below can
        // only ever pulse once per assertion.
        if (internal_hit && cpu_data_m_access) begin
            if (!internal_ack) begin
                internal_ack <= 1'b1;
                if (cpu_data_m_wr_en) begin
                    automatic logic [15:0] wr_val;
                    automatic logic [15:0] cur_val;
                    cur_val = reg_rd_val;
                    if (word_access)
                        wr_val = cpu_data_m_data_out;
                    else if (byte_hi_only)
                        // Read-modify-write: preserve the low byte,
                        // overwrite the high byte -- see the byte-swap
                        // quirk note above.
                        wr_val = {cpu_data_m_data_out[7:0], cur_val[7:0]};
                    else // byte_lo_only
                        wr_val = {cur_val[15:8], cpu_data_m_data_out[7:0]};

                    case (word_offset)
                        7'h50: umcs <= wr_val | 16'hc038;
                        7'h51: lmcs <= (wr_val & 16'h3fff) | 16'h0038;
                        7'h52: pacs <= wr_val | 16'h0038;
                        7'h53: mmcs <= wr_val | 16'h01f8;
                        7'h54: begin
                            mpcs <= wr_val | 16'h8038;
                            ext_window_written <= 1'b1;
                        end
                        7'h7f: reloc <= wr_val;
                        // WP8: DMA registers (0x60-0x6d) are NOT written
                        // here -- dma_src/dma_dst/dma_count/dma_control
                        // are also mutated autonomously by the DMA FSM
                        // itself (auto-clearing ST_STOP on TC), so they
                        // have exactly one owning always_ff (the WP8
                        // section near the end of this file), not two,
                        // to avoid a multi-driver race between a CPU
                        // register write and an in-flight FSM update
                        // landing in the same cycle. wr_dma0*/wr_dma1*
                        // (declared with the other WP8 decode wires) and
                        // bus_wr_val are consumed there instead.
                        default: ; // WP4/WP8 offsets handled in their own always_ff blocks
                    endcase
                end
            end
            // else: already serviced this assertion, hold ack high.
        end else begin
            internal_ack <= 1'b0;
        end
    end
end

// =====================================================================
// WP8: DMA channel engine (bus master "b" of u_dma_arb above).
// docs/WP8_PROGRESS.md has the design rationale; this is the RTL half.
// =====================================================================

// Per-channel "is this channel's DRQ source currently asserted"
// (§2 table's "DMA ch 0/1" row + i186.cpp's drq_callback/
// update_dma_control): TIMER_DRQ (bit4) selects the internal timer-2
// terminal-count latch over the external PIT-driven pin; SYNC_MASK==0
// (and not TIMER_DRQ) is "unsync" -- unconditionally pending whenever
// the channel is running (see docs/WP8_PROGRESS.md decision #7).
wire ch0_running   = dma_control[0][1];
wire ch1_running   = dma_control[1][1];
wire ch0_timer_drq = dma_control[0][4];
wire ch1_timer_drq = dma_control[1][4];
wire ch0_unsync    = (dma_control[0][7:6] == 2'b00) && !ch0_timer_drq;
wire ch1_unsync    = (dma_control[1][7:6] == 2'b00) && !ch1_timer_drq;

wire ch0_pending = ch0_running && (ch0_timer_drq ? timer2_latch[0] : (ch0_unsync ? 1'b1 : drq0_latch));
wire ch1_pending = ch1_running && (ch1_timer_drq ? timer2_latch[1] : (ch1_unsync ? 1'b1 : drq1_latch));

// Scheduler: CHANNEL_PRIORITY (bit5) breaks a tie in favor of whichever
// channel has it set; channel 0 wins by default otherwise (documented
// approximation -- the plan itself expects this bit to be untested by
// the real ROM, §2 table's DMA row).
wire ch0_prio_hi = dma_control[0][5];
wire ch1_prio_hi = dma_control[1][5];
wire pick_ch1    = ch1_pending && (!ch0_pending || (ch1_prio_hi && !ch0_prio_hi));

// Bus-master combinational datapath for whichever channel is currently
// active (dma_active_ch, set at the IDLE->READ transition below).
wire [19:0] cur_src_addr  = dma_src[dma_active_ch];
wire [19:0] cur_dst_addr  = dma_dst[dma_active_ch];
wire        cur_byte_word = dma_control[dma_active_ch][0]; // BYTE_WORD: 1=word transfer

assign dma_m_addr     = (dma_state == DMA_READ) ? cur_src_addr[19:1] : cur_dst_addr[19:1];
assign dma_m_access   = (dma_state == DMA_READ) || (dma_state == DMA_WRITE);
assign dma_m_wr_en    = (dma_state == DMA_WRITE);
assign dma_m_data_out = cur_byte_word ? dma_word_val : {dma_byte_val, dma_byte_val};
assign dma_m_bytesel  = cur_byte_word ? 2'b11 :
                         (dma_state == DMA_READ) ? (cur_src_addr[0] ? 2'b10 : 2'b01)
                                                   : (cur_dst_addr[0] ? 2'b10 : 2'b01);
// M/IO bit semantics per i186.cpp (SRC_MIO/DEST_MIO set == memory
// space): this bus's d_io is the opposite sense (1 == I/O).
assign dma_m_d_io = (dma_state == DMA_READ) ? !dma_control[dma_active_ch][12]
                                              : !dma_control[dma_active_ch][15];

// --- DRQ edge-latch + internal-timer-2 TC latch (separate always_ff:
// touches only drq0_latch/drq1_latch/timer2_latch, never dma_src/dst/
// count/control, so no multi-driver overlap with the block below). ---
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        drq0_pit_d   <= 1'b0; drq1_pit_d <= 1'b0;
        drq0_latch   <= 1'b0; drq1_latch <= 1'b0;
        timer2_latch <= 2'b00;
    end else begin
        drq0_pit_d <= drq0_pit_level;
        drq1_pit_d <= drq1_pit_level;

        if (drq0_pit_level && !drq0_pit_d) drq0_latch <= 1'b1;
        else if (drq0_clear)               drq0_latch <= 1'b0;

        if (drq1_pit_level && !drq1_pit_d) drq1_latch <= 1'b1;
        else if (drq1_clear)               drq1_latch <= 1'b0;

        // Both channels may independently select TIMER_DRQ; the pulse
        // is global (one shared internal timer 2), so both latches set
        // together and each clears independently once its own channel
        // services the request (in the FSM block below).
        if (timer_tc_pulse[2]) timer2_latch <= timer2_latch | 2'b11;
        if (dma_state == DMA_UPDATE && dma_control[dma_active_ch][4])
            timer2_latch[dma_active_ch] <= 1'b0;
    end
end

// --- DMA register storage + bus-master FSM: ONE owning always_ff for
// dma_src/dma_dst/dma_count/dma_control (CPU register writes below,
// FSM auto-update further down -- both live here so a same-cycle CPU
// write and an FSM end-of-transfer update can never race across two
// separate blocks driving the same array element; when both target the
// same channel in the same cycle, the FSM's own assignment executes
// later in program order and wins, a documented, deterministic
// tie-break rather than an undefined multi-driver race). ---
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        dma_src[0] <= 20'h0; dma_dst[0] <= 20'h0; dma_count[0] <= 16'h0; dma_control[0] <= 16'h0;
        dma_src[1] <= 20'h0; dma_dst[1] <= 20'h0; dma_count[1] <= 16'h0; dma_control[1] <= 16'h0;
        dma_state      <= DMA_IDLE;
        dma_active_ch  <= 1'b0;
        dma_byte_val   <= 8'h0;
        dma_word_val   <= 16'h0;
        dma0_irq_pulse <= 1'b0;
        dma1_irq_pulse <= 1'b0;
        dma_byte_done  <= 1'b0;
    end else begin
        dma0_irq_pulse <= 1'b0;
        dma1_irq_pulse <= 1'b0;
        dma_byte_done  <= 1'b0;

        // --- CPU register writes (do_write-gated, same bus_wr_val/
        // word_offset convention as every other register block in this
        // file) ---
        if (wr_dma0srclo) dma_src[0][15:0]  <= bus_wr_val;
        if (wr_dma0srchi) dma_src[0][19:16] <= bus_wr_val[3:0];
        if (wr_dma0dstlo) dma_dst[0][15:0]  <= bus_wr_val;
        if (wr_dma0dsthi) dma_dst[0][19:16] <= bus_wr_val[3:0];
        if (wr_dma0cnt)   dma_count[0]      <= bus_wr_val;
        if (wr_dma0ctl) begin
            // update_dma_control (i186.cpp): CHG_NOCHG (bit2) clear ->
            // preserve the old ST_STOP (bit1) instead of taking the
            // write's; CHG_NOCHG itself always reads back 0 afterward.
            automatic logic [15:0] newctl0;
            newctl0 = bus_wr_val;
            if (!newctl0[2])
                newctl0 = (newctl0 & ~16'h0002) | (dma_control[0] & 16'h0002);
            newctl0[2] = 1'b0;
            dma_control[0] <= newctl0;
        end

        if (wr_dma1srclo) dma_src[1][15:0]  <= bus_wr_val;
        if (wr_dma1srchi) dma_src[1][19:16] <= bus_wr_val[3:0];
        if (wr_dma1dstlo) dma_dst[1][15:0]  <= bus_wr_val;
        if (wr_dma1dsthi) dma_dst[1][19:16] <= bus_wr_val[3:0];
        if (wr_dma1cnt)   dma_count[1]      <= bus_wr_val;
        if (wr_dma1ctl) begin
            automatic logic [15:0] newctl1;
            newctl1 = bus_wr_val;
            if (!newctl1[2])
                newctl1 = (newctl1 & ~16'h0002) | (dma_control[1] & 16'h0002);
            newctl1[2] = 1'b0;
            dma_control[1] <= newctl1;
        end

        // --- Bus-master FSM: idle -> read(source) -> write(dest) ->
        // update -> idle. One shared port services whichever channel
        // the scheduler above picks each time it returns to idle. ---
        case (dma_state)
            DMA_IDLE: begin
                if (pick_ch1) begin
                    dma_active_ch <= 1'b1;
                    dma_state     <= DMA_READ;
                end else if (ch0_pending) begin
                    dma_active_ch <= 1'b0;
                    dma_state     <= DMA_READ;
                end
            end

            DMA_READ: begin
                if (dma_m_ack) begin
                    if (cur_byte_word)
                        dma_word_val <= dma_m_data_in;
                    else
                        dma_byte_val <= cur_src_addr[0] ? dma_m_data_in[15:8] : dma_m_data_in[7:0];
                    dma_state <= DMA_WRITE;
                end
            end

            DMA_WRITE: begin
                if (dma_m_ack) dma_state <= DMA_UPDATE;
            end

            DMA_UPDATE: begin
                // Pointer increment/decrement, count decrement, TC
                // stop/interrupt -- i186.cpp drq_callback, ported
                // directly (see docs/reference/mame/i186.cpp lines
                // ~1571-1650).
                automatic logic [15:0] ctrl, ctrl_after;
                automatic logic [19:0] new_src, new_dst;
                automatic logic [15:0] new_count;
                automatic logic [3:0]  incdec_size;

                ctrl        = dma_control[dma_active_ch];
                incdec_size = ctrl[0] ? 4'd2 : 4'd1; // BYTE_WORD ? word : byte
                new_src     = cur_src_addr;
                new_dst     = cur_dst_addr;

                case (ctrl[11:10]) // {SRC_DECREMENT, SRC_INCREMENT}
                    2'b10: new_src = cur_src_addr - incdec_size;
                    2'b01: new_src = cur_src_addr + incdec_size;
                    default: ; // 00 = fixed; 11 = SRC_NO_CHANGE alias
                endcase
                case (ctrl[14:13]) // {DEST_DECREMENT, DEST_INCREMENT}
                    2'b10: new_dst = cur_dst_addr - incdec_size;
                    2'b01: new_dst = cur_dst_addr + incdec_size;
                    default: ;
                endcase
                new_count  = dma_count[dma_active_ch] - 16'd1;
                ctrl_after = ctrl;

                // Terminate: explicit TERMINATE_ON_ZERO, or unsync mode
                // (which always self-stops on TC regardless of that bit
                // -- i186.cpp: `(control&TERMINATE_ON_ZERO) ||
                // !(control&SYNC_MASK)`).
                if ((ctrl[9] || (ctrl[7:6] == 2'b00)) && (new_count == 16'h0000))
                    ctrl_after[1] = 1'b0; // clear ST_STOP

                dma_src[dma_active_ch]     <= new_src;
                dma_dst[dma_active_ch]     <= new_dst;
                dma_count[dma_active_ch]   <= new_count;
                dma_control[dma_active_ch] <= ctrl_after;

                if (ctrl[8] && (new_count == 16'h0000)) begin // INTERRUPT_ON_ZERO
                    if (dma_active_ch == 1'b0) dma0_irq_pulse <= 1'b1;
                    else                       dma1_irq_pulse <= 1'b1;
                end

                dma_byte_done <= 1'b1;
                dma_state     <= DMA_IDLE;
            end
        endcase
    end
end

endmodule
