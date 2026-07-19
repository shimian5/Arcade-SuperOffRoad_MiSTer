// sor_sound -- WP10 of docs/planning_80186_sound.md: MiSTer-integration
// wrapper bundling the whole 80186 sound-board chain (WP1-WP9) into one
// module `sor_board.sv` can instantiate in place of the old "Chunk 5"
// TODO placeholder. Owns:
//   - the real vendored s80x86 Core, run un-gated at the full clk_sys
//     rate (docs/WP10_PROGRESS.md's clocking decision -- no PLL/CDC,
//     no core patch; only the internal timer tick is re-paced via ce_8m)
//   - the first-level bus arbiter (Core's instr_m vs data_m ports),
//     the other half of planning_80186_sound.md §1.2's topology that
//     WP1-WP9 never needed (every bench so far fed instr_m straight to
//     a flat bench-only memory model)
//   - i186_periph (WP2-4, WP8: CSU/timers/interrupt-controller/DMA +
//     the internal DMA-vs-CPU second-level arbiter)
//   - leland_sound_board (WP6) + leland_dac_mixer (WP7)
//   - the 80186's own 16 KB RAM (self-contained, on-chip -- nothing
//     else touches it, unlike Master/Slave's externally-shared WRAM)
//   - a byte-wide SDRAM read adapter for the ROM window (0x20000-
//     0xFFFFF of the 80186's own address space), matching Master/
//     Slave's existing rom_req/rom_addr/rom_data/rom_stall shape
//     exactly so sor_board.sv's SDRAM arbiter gets a 5th channel built
//     the same way as its already-hardware-validated rd0/rd1 channels,
//     not a new invented protocol.
//
// Ground truth: docs/reference/mame/leland_a.cpp
// leland_80186_map_program (RAM 0x00000-0x1FFFF mirrored, ROM
// 0x20000-0xFFFFF identity-mapped into the "audiocpu" region).

module sor_sound(
    input  logic        clk_sys,   // 48 MHz
    input  logic        reset,     // board reset
    input  logic        ce_8m,     // ~8MHz-equivalent CE -- paces ONLY
                                    // i186_periph's internal timer tick
                                    // (dac9's real sample rate); every
                                    // other signal in this chain runs
                                    // un-gated at clk_sys (see file header)

    // --- Master-Z80-side control/command latch (sor_master.sv) ---
    input  logic [7:0]  sound_ctrl_data,
    input  logic         sound_ctrl_wr,
    input  logic [15:0] cmd_wr_data,
    input  logic         cmd_wr_lo, cmd_wr_hi,
    output logic [7:0]  response_data,

    // --- SDRAM read port (sound ROM), same shape as sor_master/
    //     sor_slave's own rom_req/rom_addr/rom_data/rom_stall ---
    output logic         rom_req,
    output logic [19:0] rom_addr,   // byte address, 0-0xFFFFF (80186's own address space)
    input  logic [7:0]  rom_data,
    input  logic         rom_stall,

    // --- Audio output ---
    output logic signed [15:0] audio_out,

    // --- Debug/diagnostic taps (2026-07-18, "no sound during gameplay
    // on real hardware" investigation, docs/WP10_PROGRESS.md): all
    // sourced from signals that already exist as ports on i186_periph/
    // leland_sound_board -- pure pass-through, no new logic in either
    // of those modules. Intended consumer: sor_video.sv's always-on
    // status overlay, replacing some of its now-stale Slave-ROM-bringup
    // fields (see that file's own comment at the edit site). ---
    output logic         dbg_dac_activity,   // 1-cycle pulse: any of the 6 8-bit DAC channels wrote
    output logic         dbg_dac9_activity,  // 1-cycle pulse: the dac9 (10-bit, timer-paced) channel wrote
    output logic         dbg_int0_pin,       // live level into i186_periph's external interrupt 0
    output logic [7:0]   dbg_intc_request,   // i186_periph's intc_request_reg (bit4 = ext0 pending)
    output logic [7:0]   dbg_intc_in_service,// i186_periph's intc_in_service_reg (bit4 = ext0 in service)

    // Added after round-2 hardware read (intc_request=0x30 stuck,
    // in_service=0x00 stuck): a Fable medium-effort sanity check on
    // i186_periph.sv's request->poll_status pipeline found it
    // structurally sound, but flagged the one legitimate way this
    // exact static reading persists -- EXT0/EXT1 still masked (MSK bit,
    // ext0/1_ctrl[3]) at reset default (7'hf = masked), never cleared
    // by the ROM. These three let the overlay tell "still masked" apart
    // from "unmasked but never granted" directly, without guessing.
    output logic [6:0]   dbg_intc_ext0_ctrl, // i186_periph's intc_ext0_ctrl_reg (bit3=MSK, bit4=LTM)
    output logic [6:0]   dbg_intc_ext1_ctrl, // same, ext1
    output logic          dbg_intc_poll_pending, // = intc_poll_status[15] (= intr) -- did a request ever actually reach the CPU-visible poll/intr line at all

    // Added after a Fable investigation found dac_activity/dac9_activity
    // reading 0 was NOT a wiring bug -- it traced the audible reset pop
    // to a DC-offset artifact in leland_dac_mixer.sv (now fixed
    // separately) requiring zero real dac_wr activity, and flagged that
    // dac_write_count=8/dac9_write_count=1 being invariant across every
    // single simulation run but ABSENT on real hardware means the
    // 80186 may never actually be completing (or even starting) its own
    // boot self-test on real silicon at all. These are the decisive
    // "is the CPU actually alive" taps: audiocpu_reset_n (did /RESET
    // ever release), a Core bus-activity pulse (is Core issuing ANY bus
    // transaction, independent of whether SDRAM/ROM works), and a
    // ROM-word-fetch-completion pulse (is the CH_RD3 SDRAM channel --
    // "never hardware-validated, unlike rd0/rd1" per this session's own
    // earlier WP10_PROGRESS.md note -- actually delivering real bytes).
    output logic          dbg_audiocpu_reset_n,
    output logic          dbg_core_bus_activity,  // 1-cycle pulse: Core asserted instr_m_access or data_m_access
    output logic          dbg_rom_fetch_activity, // 1-cycle pulse: one 16-bit ROM word fetch completed (rom_phase ROM_HI->ROM_DONE)

    // Round-3 hardware read: audiocpu_reset_n=1, core_bus_activity and
    // rom_fetch_activity both SATURATED at 0xFF -- the 80186 is
    // genuinely alive and successfully fetching real ROM words via
    // CH_RD3, ruling out "CPU never runs". But that activity counter
    // can't tell "making real forward progress through boot code" apart
    // from "stuck looping over a small address range forever" (e.g. an
    // error/wait handler) -- the live IP is the direct answer. Tapped
    // via hierarchical reference to the vendored Core's own internal
    // `ip_current` (rtl/s80x86/Core.sv) -- no changes to that file
    // itself, same technique sim/sor_sound_tb.sv already uses for
    // register readback.
    output logic [15:0]   dbg_core_ip
);

// =====================================================================
// Core interrupt/debug ports -- no master-Z80 INT0/INT1 harness wired
// through this module's own ports (leland_sound_board decodes those
// straight from control_data below); NMI unused per leland_a.cpp
// (deliberately not wired, see planning doc §1.3).
// =====================================================================
wire        nmi = 1'b0;
wire        intr, inta;
wire [7:0]  irq;

wire        debug_stopped;
wire [15:0] debug_val;

// =====================================================================
// Core bus ports
// =====================================================================
wire [19:1] instr_m_addr;
wire [15:0] instr_m_data_in;
wire        instr_m_access;
wire        instr_m_ack;

wire [19:1] data_m_addr;
wire [15:0] data_m_data_in;
wire [15:0] data_m_data_out;
wire        data_m_access;
wire        data_m_ack;
wire        data_m_wr_en;
wire [1:0]  data_m_bytesel;
wire        data_m_d_io;
wire        lock;

// --- audiocpu /RESET: board reset OR the control latch's own /RESET
// bit (leland_sound_board's decoded audiocpu_reset_n, active-high
// "running") ---
wire audiocpu_reset_n;
wire core_reset = reset | ~audiocpu_reset_n;

wire [15:0] core_ip_dbg;

Core cpu(
    .clk(clk_sys), .reset(core_reset),
    .nmi(nmi), .intr(intr), .irq(irq), .inta(inta),
    .instr_m_addr(instr_m_addr), .instr_m_data_in(instr_m_data_in),
    .instr_m_access(instr_m_access), .instr_m_ack(instr_m_ack),
    .data_m_addr(data_m_addr), .data_m_data_in(data_m_data_in),
    .data_m_data_out(data_m_data_out), .data_m_access(data_m_access),
    .data_m_ack(data_m_ack), .data_m_wr_en(data_m_wr_en),
    .data_m_bytesel(data_m_bytesel), .d_io(data_m_d_io), .lock(lock),
    .debug_stopped(debug_stopped), .debug_seize(1'b0),
    .debug_addr(8'h0), .debug_run(1'b0), .debug_val(debug_val),
    .debug_wr_val(16'h0), .debug_wr_en(1'b0),
    .ip_out(core_ip_dbg));

// =====================================================================
// First-level arbiter: instr_m (lower priority, "a") vs data_m (higher
// priority, "b") -- planning_80186_sound.md §1.2's own MemArbiter usage
// pattern (data port has priority over prefetch), the half of that
// topology no prior WP bench needed (WP1's Stage-A/arb benches and the
// WP8 smoke test all fed instr_m straight to a flat bench memory). d_io
// is synthesized on the same q_b grant basis MemArbiter already exposes
// (instruction fetches are always memory-space, d_io=0) -- identical
// technique to the DMA arbiter's own d_io mux in i186_periph.sv.
// =====================================================================
wire [19:1] cpu_m_addr;
wire [15:0] cpu_m_data_in;
wire [15:0] cpu_m_data_out;
wire        cpu_m_access;
wire        cpu_m_ack;
wire        cpu_m_wr_en;
wire [1:0]  cpu_m_bytesel;
wire        cpu_arb_grant_b;
wire        cpu_m_d_io = cpu_arb_grant_b ? data_m_d_io : 1'b0;

MemArbiter u_cpu_arb(
    .clk(clk_sys), .reset(core_reset),
    .a_m_addr(instr_m_addr), .a_m_data_in(instr_m_data_in), .a_m_data_out(16'h0),
    .a_m_access(instr_m_access), .a_m_ack(instr_m_ack), .a_m_wr_en(1'b0), .a_m_bytesel(2'b11),
    .b_m_addr(data_m_addr), .b_m_data_in(data_m_data_in), .b_m_data_out(data_m_data_out),
    .b_m_access(data_m_access), .b_m_ack(data_m_ack), .b_m_wr_en(data_m_wr_en), .b_m_bytesel(data_m_bytesel),
    .q_m_addr(cpu_m_addr), .q_m_data_in(cpu_m_data_in), .q_m_data_out(cpu_m_data_out),
    .q_m_access(cpu_m_access), .q_m_ack(cpu_m_ack), .q_m_wr_en(cpu_m_wr_en), .q_m_bytesel(cpu_m_bytesel),
    .q_b(cpu_arb_grant_b));

// =====================================================================
// Combinational-loop break (docs/WP10_PROGRESS.md, "Combinational-loop
// fix mechanism"): u_cpu_arb's q_m_* outputs (addr/access/wr_en/
// bytesel/data_out/d_io) are only genuinely stable once granted --
// during the one cycle before grant_active registers, q_b (hence
// cpu_m_d_io) and q_m_access are live combinational functions of
// Core's own instr_m_access/data_m_access. i186_periph.sv's internal
// hit-classification (internal_hit) and leland_sound_board.sv's own
// (local_hit, keyed off cpu_d_io) each latch-for-duration-of-transaction,
// but that pattern alone doesn't remove the live *selector* edge from
// Quartus's structural (gate-level) loop check -- confirmed via a full
// Quartus build (299-node combinational loop, root node sys_d_io,
// i186_periph.sv:101; -3.69ns worst-case setup slack on clk_sys).
//
// Fix: register the whole forward request bundle here, once, before it
// ever reaches i186_periph -- everything downstream (i186_periph's own
// internal_hit latch, the internal DMA-vs-CPU u_dma_arb, and
// leland_sound_board's local_hit) then only ever sees an
// already-registered, already-stable view of the transaction, with no
// remaining live (same-cycle, Core-driven) term anywhere in that
// combinational cone. The *return* path (cpu_m_ack/cpu_m_data_in) is
// deliberately left unregistered/combinational, matching this whole
// system's existing "ack can arrive on any cycle" contract (WP1's G1
// stall-tolerance finding) -- only the forward leg is structurally
// required to break the cycle; registering the return leg too would
// need explicit stale-ack gating (the registered ack would otherwise
// linger one cycle after Core's own access-drop) for no correctness
// benefit, so it's intentionally not done.
//
// Net effect: exactly +1 cycle of added, uniform, bounded latency
// (trivially still satisfies WP1's G3 "bounded grant latency" contract)
// on every transaction that reaches i186_periph/leland_sound_board --
// nothing here is transaction-type-dependent, so this cannot skew
// leland_sound_board.sv's own do_write (its own header comment already
// requires cpu_d_io to be "a stable-per-transaction Core output"; this
// register is what actually makes that true again once two chained
// MemArbiters sit between Core and it).
// =====================================================================
reg  [19:1] cpu_req_addr_r;
reg  [15:0] cpu_req_data_out_r;
reg         cpu_req_access_r;
reg         cpu_req_wr_en_r;
reg  [1:0]  cpu_req_bytesel_r;
reg         cpu_req_d_io_r;

always_ff @(posedge clk_sys or posedge core_reset) begin
    if (core_reset) begin
        cpu_req_access_r <= 1'b0;
    end else begin
        cpu_req_addr_r     <= cpu_m_addr;
        cpu_req_data_out_r <= cpu_m_data_out;
        cpu_req_access_r   <= cpu_m_access;
        cpu_req_wr_en_r    <= cpu_m_wr_en;
        cpu_req_bytesel_r  <= cpu_m_bytesel;
        cpu_req_d_io_r     <= cpu_m_d_io;
    end
end

// =====================================================================
// i186_periph (WP2-4, WP8)
// =====================================================================
wire [19:1] sys_addr;
wire [15:0] sys_data_in;
wire [15:0] sys_data_out;
wire        sys_access;
wire        sys_ack;
wire        sys_wr_en;
wire [1:0]  sys_bytesel;
wire        sys_d_io;

wire [19:0] ext_window_base;
wire        ext_window_is_mem, ext_window_valid;
wire [15:0] reloc_reg, umcs_reg, lmcs_reg, pacs_reg, mmcs_reg, mpcs_reg;
wire        tmrout0, tmrout1;
wire [2:0]  timer_irq, timer_tc_pulse;
wire [15:0] t_count[0:2], t_maxA[0:2], t_maxB[0:2], t_control[0:2];
wire [7:0]  intc_request_reg, intc_in_service_reg;
wire [2:0]  intc_status_reg;
wire [3:0]  intc_timer0_ctrl_reg, intc_dma0_ctrl_reg, intc_dma1_ctrl_reg;
wire [6:0]  intc_ext0_ctrl_reg, intc_ext1_ctrl_reg;

wire        int0_pin, int1_pin; // from leland_sound_board's control_data decode
wire        drq0_pit_level, drq1_pit_level, drq0_clear, drq1_clear;
wire [19:0] dma_src[0:1], dma_dst[0:1];
wire [15:0] dma_count[0:1], dma_control[0:1];
wire [1:0]  dma_active;
wire        dma_byte_done, dma_byte_done_ch;

i186_periph periph(
    .clk(clk_sys), .reset(core_reset), .ce_8m(ce_8m),
    .cpu_data_m_addr(cpu_req_addr_r), .cpu_data_m_data_in(cpu_m_data_in),
    .cpu_data_m_data_out(cpu_req_data_out_r), .cpu_data_m_access(cpu_req_access_r),
    .cpu_data_m_ack(cpu_m_ack), .cpu_data_m_wr_en(cpu_req_wr_en_r),
    .cpu_data_m_bytesel(cpu_req_bytesel_r), .cpu_d_io(cpu_req_d_io_r),
    .sys_data_m_addr(sys_addr), .sys_data_m_data_in(sys_data_in),
    .sys_data_m_data_out(sys_data_out), .sys_data_m_access(sys_access),
    .sys_data_m_ack(sys_ack), .sys_data_m_wr_en(sys_wr_en),
    .sys_data_m_bytesel(sys_bytesel), .sys_d_io(sys_d_io),
    .ext_window_base(ext_window_base), .ext_window_is_mem(ext_window_is_mem),
    .ext_window_valid(ext_window_valid),
    .reloc_reg(reloc_reg), .umcs_reg(umcs_reg), .lmcs_reg(lmcs_reg),
    .pacs_reg(pacs_reg), .mmcs_reg(mmcs_reg), .mpcs_reg(mpcs_reg),
    .tmrout0(tmrout0), .tmrout1(tmrout1), .timer_irq(timer_irq),
    .timer_tc_pulse(timer_tc_pulse),
    .t_count(t_count), .t_maxA(t_maxA), .t_maxB(t_maxB), .t_control(t_control),
    .intr(intr), .irq_out(irq), .inta(inta),
    .int0_pin(int0_pin), .int1_pin(int1_pin),
    .dma0_irq_req(1'b0), .dma1_irq_req(1'b0),
    .intc_request_reg(intc_request_reg), .intc_in_service_reg(intc_in_service_reg),
    .intc_status_reg(intc_status_reg),
    .intc_timer0_ctrl_reg(intc_timer0_ctrl_reg), .intc_dma0_ctrl_reg(intc_dma0_ctrl_reg),
    .intc_dma1_ctrl_reg(intc_dma1_ctrl_reg),
    .intc_ext0_ctrl_reg(intc_ext0_ctrl_reg), .intc_ext1_ctrl_reg(intc_ext1_ctrl_reg),
    .drq0_pit_level(drq0_pit_level), .drq1_pit_level(drq1_pit_level),
    .drq0_clear(drq0_clear), .drq1_clear(drq1_clear),
    .dma_src(dma_src), .dma_dst(dma_dst), .dma_count(dma_count), .dma_control(dma_control),
    .dma_active(dma_active), .dma_byte_done(dma_byte_done), .dma_byte_done_ch(dma_byte_done_ch));

// =====================================================================
// leland_sound_board (WP6) -- PIT clock enable: real hardware clocks
// both discrete PIT8254s at 4 MHz (leland_a.cpp `set_clk<N>(4000000)`);
// clk_sys/12 = 4 MHz exactly at 48 MHz, matching CE_6M/CE_8M's own
// divide-ratio convention in rtl/sor_board.sv.
// =====================================================================
reg [3:0] pit_ce_div;
wire      pit_ce = (pit_ce_div == 4'd11);
always @(posedge clk_sys or posedge reset) begin
    if (reset) pit_ce_div <= 4'd0;
    else pit_ce_div <= (pit_ce_div == 4'd11) ? 4'd0 : pit_ce_div + 4'd1;
end

wire [19:1] mem_addr;
wire [15:0] mem_data_in;
wire [15:0] mem_data_out;
wire        mem_access;
wire        mem_ack;
wire        mem_wr_en;
wire [1:0]  mem_bytesel;
wire        mem_d_io;

wire        audiocpu_test_n; // decoded but unused -- WP9 found no WAIT usage, no /TEST wiring needed
wire [7:0]  dac_sample[0:5];
wire [7:0]  dac_vol[0:5];
wire        dac_wr[0:5];
wire [9:0]  dac9_sample;
wire        dac9_wr;
wire [6:0]  clock_active;
wire        response_wr;

// =====================================================================
// Combinational-loop break, second boundary (docs/WP10_PROGRESS.md):
// the first pipeline stage above (u_cpu_arb -> i186_periph) cut the
// original 299-node loop down to a smaller, separately-closed 42-node
// loop entirely contained between `u_dma_arb` (inside i186_periph) and
// leland_sound_board -- confirmed via a real Quartus re-run after the
// first fix. Root cause is the exact same bug class, one level deeper:
// leland_sound_board.sv's own `win_hit`/`fresh_access` latch (its own
// header comment there already explains the "why latched" half of the
// story, mirroring i186_periph's `internal_hit`) still reads live
// `cpu_access` (=sys_access=u_dma_arb.q_m_access, which itself depends
// combinationally on `cpu_ack`=sys_ack via MemArbiter's own
// `q_m_access = ~q_m_ack & (...)` formula) directly in its selector --
// the same "selector must not read any live access/request signal, and
// latching-with-a-live-first-cycle-branch doesn't remove that edge"
// finding from the first fix, just recurring on this inner boundary.
// Same fix, same shape: register the whole forward bundle here, once,
// leaving the return leg (cpu_ack/cpu_data_in) unregistered.
// =====================================================================
reg  [19:1] board_req_addr_r;
reg  [15:0] board_req_data_out_r;
reg         board_req_access_r;
reg         board_req_wr_en_r;
reg  [1:0]  board_req_bytesel_r;
reg         board_req_d_io_r;

always_ff @(posedge clk_sys or posedge core_reset) begin
    if (core_reset) begin
        board_req_access_r <= 1'b0;
    end else begin
        board_req_addr_r     <= sys_addr;
        board_req_data_out_r <= sys_data_out;
        board_req_access_r   <= sys_access;
        board_req_wr_en_r    <= sys_wr_en;
        board_req_bytesel_r  <= sys_bytesel;
        board_req_d_io_r     <= sys_d_io;
    end
end

// NOTE: leland_sound_board is reset by the plain board-level `reset`,
// NOT `core_reset` -- core_reset is DERIVED from this module's own
// audiocpu_reset_n output (~audiocpu_reset_n), so gating this module's
// own reset input on core_reset would create a self-perpetuating lock
// (its own reset-value default for audiocpu_reset_n is 0/asserted,
// which would hold core_reset high forever, which would hold this
// module in reset forever, which keeps re-asserting the 0 default --
// found by this session's own first sim attempt: reloc never left its
// power-on value after 300k cycles). Everything that should genuinely
// stay held while the control latch's own /RESET bit is asserted
// (Core, i186_periph) uses core_reset; the thing that DECIDES that bit
// must not. (The new pipeline register above stays on core_reset,
// same as its sibling for the outer boundary -- it's purely combinational
// -loop plumbing between two core_reset-domain modules, not part of
// leland_sound_board's own reset-ordering concern.)
leland_sound_board board(
    .clk(clk_sys), .reset(reset),
    .cpu_addr(board_req_addr_r), .cpu_data_in(sys_data_in), .cpu_data_out(board_req_data_out_r),
    .cpu_access(board_req_access_r), .cpu_ack(sys_ack), .cpu_wr_en(board_req_wr_en_r),
    .cpu_bytesel(board_req_bytesel_r), .cpu_d_io(board_req_d_io_r),
    .ext_window_base(ext_window_base), .ext_window_is_mem(ext_window_is_mem),
    .ext_window_valid(ext_window_valid),
    .mem_addr(mem_addr), .mem_data_in(mem_data_in), .mem_data_out(mem_data_out),
    .mem_access(mem_access), .mem_ack(mem_ack), .mem_wr_en(mem_wr_en),
    .mem_bytesel(mem_bytesel), .mem_d_io(mem_d_io),
    .t0_tc_pulse(timer_tc_pulse[0]),
    .pit_ce(pit_ce),
    .cmd_wr_data(cmd_wr_data), .cmd_wr_lo(cmd_wr_lo), .cmd_wr_hi(cmd_wr_hi),
    .response_data(response_data), .response_wr(response_wr),
    .control_data(sound_ctrl_data), .control_wr(sound_ctrl_wr),
    .audiocpu_reset_n(audiocpu_reset_n), .audiocpu_test_n(audiocpu_test_n),
    .int0_pin(int0_pin), .int1_pin(int1_pin),
    .dac_sample(dac_sample), .dac_vol(dac_vol), .dac_wr(dac_wr),
    .dac9_sample(dac9_sample), .dac9_wr(dac9_wr),
    .drq0_pit_level(drq0_pit_level), .drq1_pit_level(drq1_pit_level),
    .drq0_clear(drq0_clear), .drq1_clear(drq1_clear),
    .clock_active(clock_active));

// =====================================================================
// leland_dac_mixer (WP7)
// =====================================================================
leland_dac_mixer mixer(
    .clk(clk_sys), .reset(reset),
    .dac_sample(dac_sample), .dac_vol(dac_vol), .dac9_sample(dac9_sample),
    .audio_out(audio_out));

// =====================================================================
// 80186's own memory: RAM (16 KB, mirrored x8 across 0x00000-0x1FFFF)
// self-contained here (nothing else touches it, unlike Master/Slave's
// externally-shared WRAM); ROM (0x20000-0xFFFFF) via the byte-wide
// SDRAM adapter below. leland_80186_map_program: RAM occupies word
// address < 19'h10000 (byte < 0x20000); ROM is everything else up to
// the region's own 1MB size.
// =====================================================================
wire mem_is_ram = (mem_addr < 19'h10000);

// --- RAM (16 KB = 8192 words, byte-addressable via bytesel) ---
reg [7:0] ram_lo [0:8191]; // even bytes
reg [7:0] ram_hi [0:8191]; // odd bytes
wire [12:0] ram_word_addr = mem_addr[13:1]; // mirror: only the low 13 bits of the word address matter

reg        ram_ack;
reg [15:0] ram_data_in_r;
always @(posedge clk_sys or posedge core_reset) begin
    if (core_reset) begin
        ram_ack <= 1'b0;
    end else begin
        if (mem_access && mem_is_ram && !ram_ack) begin
            ram_ack <= 1'b1;
            ram_data_in_r <= {ram_hi[ram_word_addr], ram_lo[ram_word_addr]};
            if (mem_wr_en) begin
                if (mem_bytesel[0]) ram_lo[ram_word_addr] <= mem_data_out[7:0];
                if (mem_bytesel[1]) ram_hi[ram_word_addr] <= mem_data_out[15:8];
            end
        end else if (!mem_access) begin
            ram_ack <= 1'b0;
        end
    end
end

// --- ROM (byte-wide SDRAM adapter): fetches BOTH bytes of the
// containing word for every access (regardless of bytesel -- simpler
// than branching per-bytesel, and the extra byte is negligible traffic
// for a background sound CPU), presenting a full 16-bit word once both
// arrive. Explicit 1-cycle req-low gap between the two byte requests --
// required by rtl/sor_board.sv's own SDRAM-arbiter client contract
// ("clients deassert req for >=1 cycle after ack" -- holding req high
// continuously across both byte fetches would starve that release and
// hang the arbiter, the same duplicate-transaction hazard class
// documented in docs/WP10_PROGRESS.md's clocking section, here on the
// rom_req/rom_stall boundary instead of the internal bus).
localparam ROM_IDLE = 3'd0, ROM_LO = 3'd1, ROM_GAP = 3'd2, ROM_HI = 3'd3, ROM_DONE = 3'd4;
reg [2:0]  rom_phase;
reg [18:0] rom_word_addr_latch;
reg [7:0]  rom_lo_byte, rom_hi_byte;

wire rom_hit = mem_access && !mem_is_ram;

assign rom_req  = (rom_phase == ROM_LO) || (rom_phase == ROM_HI);
assign rom_addr = (rom_phase == ROM_HI) ? {rom_word_addr_latch, 1'b1} : {rom_word_addr_latch, 1'b0};

always @(posedge clk_sys or posedge core_reset) begin
    if (core_reset) begin
        rom_phase <= ROM_IDLE;
    end else begin
        case (rom_phase)
            ROM_IDLE: if (rom_hit) begin
                rom_word_addr_latch <= mem_addr;
                rom_phase <= ROM_LO;
            end
            ROM_LO: if (!rom_stall) begin
                rom_lo_byte <= rom_data;
                rom_phase  <= ROM_GAP;
            end
            ROM_GAP: rom_phase <= ROM_HI; // one cycle, rom_req low
            ROM_HI: if (!rom_stall) begin
                rom_hi_byte <= rom_data;
                rom_phase  <= ROM_DONE;
            end
            ROM_DONE: if (!mem_access) rom_phase <= ROM_IDLE;
        endcase
    end
end

wire        rom_done_ack  = (rom_phase == ROM_DONE);
wire [15:0] rom_data_word = {rom_hi_byte, rom_lo_byte};

assign mem_ack     = mem_is_ram ? ram_ack      : rom_done_ack;
assign mem_data_in = mem_is_ram ? ram_data_in_r : rom_data_word;

// --- Debug/diagnostic taps (see port declarations above) ---
assign dbg_dac_activity  = dac_wr[0] | dac_wr[1] | dac_wr[2] | dac_wr[3] | dac_wr[4] | dac_wr[5];
assign dbg_dac9_activity = dac9_wr;
assign dbg_int0_pin      = int0_pin;
assign dbg_intc_request  = intc_request_reg;
assign dbg_intc_in_service = intc_in_service_reg;
assign dbg_intc_ext0_ctrl = intc_ext0_ctrl_reg;
assign dbg_intc_ext1_ctrl = intc_ext1_ctrl_reg;
assign dbg_intc_poll_pending = intr;
assign dbg_audiocpu_reset_n  = audiocpu_reset_n;
// instr_m_ack/data_m_ack (not the *_access levels, which stay high for
// a transaction's full multi-cycle duration) -- matches this whole
// system's "ack fires for exactly one cycle" convention, so this is a
// true one-shot-per-transaction pulse, safe to feed a rolling counter
// without over-counting a single long transaction multiple times.
assign dbg_core_bus_activity = instr_m_ack | data_m_ack;
assign dbg_rom_fetch_activity = (rom_phase == ROM_HI) && !rom_stall;
assign dbg_core_ip = core_ip_dbg;

endmodule
