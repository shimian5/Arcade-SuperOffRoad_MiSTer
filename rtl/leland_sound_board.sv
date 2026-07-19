// leland_sound_board -- WP6 of docs/planning_80186_sound.md ("board glue").
//
// Sits downstream of i186_periph's *sys*-side data bus (i.e. everything
// i186_periph didn't claim as an internal 80186 register): decodes the
// external PCS peripheral window (command/response latch, PIT0/PIT1,
// dac9, clock-active status -- leland_a.cpp's peripheral_r/w, `select =
// offset >> 6`) and the whole-64K I/O-space DAC write path (dac_w),
// and passes everything else through to system RAM/ROM untouched.
//
// Scope is the **3rd-generation ("leland"/SOR) board only** -- 6x
// 8-bit DAC (indices 0-5) + one 10-bit DAC (dac9, PCS select 4) + two
// discrete PIT8254s (PCS select 2/3). Redline (3 PITs, 8 DACs, no
// dac9), Ataxx/WSF (YM2151, external-DAC control block) are explicitly
// out of scope here -- risk R13 in the plan flags per-generation decode
// as a parameterization to add later, not attempted in this pass.
//
// Addressing convention: this repo's bus signals (`*_addr[19:1]`) are
// WORD addresses (bit 0 of the byte address is implicit/dropped, byte
// lane selected separately via `bytesel`). MAME's peripheral_r/w and
// dac_w handlers are installed as 16-bit (`read16s`/`write16s`)
// delegates, whose `offset` parameter is therefore *also* a word
// offset in exactly the same units -- confirmed by cross-checking
// WP0's real trace (PIT0 programming observed at absolute byte address
// 0x20100/0x20106, which is word-offset 0x80/0x83 from the window base
// 0x20000, `>>6` = select 2, matching leland_a.cpp's PIT0 case) against
// the source's `select = offset >> 6` / `offset & 3` decode. This lets
// every offset/mask constant below be lifted directly from leland_a.cpp
// without a byte<->word re-derivation at each site.
//
// clock_active[6:2] (leland_a.cpp's `m_clock_active`, PCS select-0
// status read) is modeled as an edge-triggered latch, not a literal
// level follower of the PIT/timer output pins: real MAME only invokes
// `set_clock_line()` on a pin *transition* event, and bit 6 (TMROUT0)
// specifically only ever transitions high in this board's non-ALT
// timer configuration (WP0 Q5; i186.cpp's timer_elapsed() calls
// `m_out_tmrout0_func(1)` on *every* terminal count regardless of
// pin's previous value -- a repeated "event", not a level change) --
// see i186_periph.sv's `timer_tc_pulse` addition (this same commit)
// for why the pulse output exists. Bits 2-5 (real PIT8254 outputs,
// which do genuinely transition both ways) are approximated as
// rising-edge-sets; a dac_w write to the matching index clears the bit
// immediately, same priority order MAME's single-threaded callback
// sequencing gives Automatically (write-after-set is the only order
// firmware can observe). This is a deliberate simplification (falling
// PIT edges are not separately modeled) consistent with WP3's own
// pulse-abstraction-over-pin-level approach; Stage C's rate-lock
// criterion is about long-run DAC-write rate, not literal pin fidelity.
module leland_sound_board(
    input  logic        clk,
    input  logic        reset,

    // --- CPU-side data bus (i186_periph's sys_data_m_* pass-through) ---
    input  logic [19:1] cpu_addr,
    output logic [15:0] cpu_data_in,
    input  logic [15:0] cpu_data_out,
    input  logic         cpu_access,
    output logic         cpu_ack,
    input  logic         cpu_wr_en,
    input  logic [1:0]   cpu_bytesel,
    input  logic         cpu_d_io,

    // --- external PCS window decode (from i186_periph's CSU, PACS/MPCS) ---
    input  logic [19:0] ext_window_base,
    input  logic         ext_window_is_mem,
    input  logic         ext_window_valid,

    // --- pass-through to system RAM/ROM for anything not claimed here ---
    output logic [19:1] mem_addr,
    input  logic [15:0] mem_data_in,
    output logic [15:0] mem_data_out,
    output logic         mem_access,
    input  logic         mem_ack,
    output logic         mem_wr_en,
    output logic [1:0]   mem_bytesel,
    output logic         mem_d_io,

    // --- timer 0 terminal-count event (i186_periph.sv's timer_tc_pulse[0]) ---
    input  logic         t0_tc_pulse,

    // --- PIT0/PIT1 counter clock enable (4 MHz per WP0; caller derives
    //     this from the system clock -- both PITs' three counters are
    //     all clocked at the same 4 MHz in leland_a.cpp) ---
    input  logic         pit_ce,

    // --- Z80-facing command/response/control seam (WP10 wires the real
    //     master-Z80 latches/synchronizers to these) ---
    input  logic [15:0] cmd_wr_data,
    input  logic         cmd_wr_lo, cmd_wr_hi,   // 1-cycle strobes (command_lo_w/command_hi_w)
    output logic [7:0]  response_data,           // m_sound_response
    output logic         response_wr,             // 1-cycle strobe: new response latched
    input  logic [7:0]  control_data,             // leland_80186_control_w's byte
    input  logic         control_wr,
    output logic         audiocpu_reset_n,
    output logic         audiocpu_test_n,
    output logic         int0_pin,
    output logic         int1_pin,

    // --- DAC/mixer datapath (WP7 consumes) ---
    output logic [7:0]  dac_sample [0:5],
    output logic [7:0]  dac_vol    [0:5],
    output logic         dac_wr     [0:5],        // 1-cycle strobe per channel
    output logic [9:0]  dac9_sample,
    output logic         dac9_wr,

    // --- DMA seam (WP8 consumes) ---
    output logic         drq0_pit_level, drq1_pit_level, // raw PIT0 out0/out1
    output logic         drq0_clear,     drq1_clear,      // 1-cycle strobes (dac_w's software clear)

    // --- debug/status ---
    output logic [6:0]  clock_active
);

// --- external PCS window hit decode ---
wire [19:1] window_base_word = ext_window_base[19:1];
wire [19:1] window_top_word  = window_base_word + 19'h180; // 0x300 bytes = 0x180 words
// Deliberately NOT gated on `cpu_access` (a second, independent bug
// from the ext_window_base/valid race described below): `cpu_ack`'s
// mux selector is built from `local_hit`/`pcs2_hit`/`pcs3_hit`, which
// all trace back to this wire. If it included `&& cpu_access`, the
// selector itself would change the instant `cpu_access` drops in
// response to `ack` -- swinging the mux away from the very branch that
// just presented the ack Core is trying to sample, making Core see
// ack fall, re-assert `access`, re-select the branch, see ack rise
// again, forever, in zero simulated time. i186_periph.sv's own
// `internal_hit` never gated on access either, for the same reason --
// classification of "what kind of access is this" must depend only on
// address/d_io (stable for the whole transaction), never on the
// access/ack handshake state itself. Only genuine bus-activity signals
// (`mem_access`, `pit0_access`, `do_write`, ...) may legitimately AND
// in `cpu_access`.
wire win_hit_now = ext_window_valid && ext_window_is_mem && !cpu_d_io &&
               (cpu_addr >= window_base_word) && (cpu_addr < window_top_word);
wire [8:0] window_word_offset_now = cpu_addr - window_base_word; // 0..0x17F

// win_hit/window_word_offset are latched for the duration of a
// transaction, not recomputed live every delta: ext_window_base/valid
// (this module's inputs, driven by i186_periph's PACS/MPCS) can change
// while this module's OWN currently-in-flight transaction is still
// pending ack -- e.g. an MPCS write's internal_ack commits one cycle,
// `ext_window_written` takes effect the next, and if this module's own
// next transaction starts combinationally in that same narrow window,
// win_hit's classification of THAT brand-new transaction is unstable
// across a couple of deltas right as it begins. That instability
// flipping `cpu_ack`'s mux source (local_ack vs mem_ack) mid-transaction
// closes the same class of zero-delay access<->ack oscillation through
// the real s80x86 Core that i186_periph.sv's `internal_hit` latch (see
// that file's own comment) fixes on its side -- this is the board-level
// half of the same fix, needed for the same reason (only a real Core's
// `access`, combinationally coupled to `ack`, can expose it; no
// BFM-driven bench ever could). Latched at the moment `access` first
// asserts, same idiom as i186_periph.sv.
reg access_prev2;
reg win_hit_latched;
reg [8:0] window_word_offset_latched;
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        access_prev2 <= 1'b0;
        win_hit_latched <= 1'b0;
        window_word_offset_latched <= 9'b0;
    end else begin
        access_prev2 <= cpu_access;
        if (cpu_access && !access_prev2) begin
            win_hit_latched <= win_hit_now;
            window_word_offset_latched <= window_word_offset_now;
        end
    end
end
wire fresh_access = cpu_access && !access_prev2;
wire win_hit = fresh_access ? win_hit_now : win_hit_latched;
wire [8:0] window_word_offset = fresh_access ? window_word_offset_now : window_word_offset_latched;

wire [2:0] pcs_select = window_word_offset[8:6];
wire [5:0] pcs_offset = window_word_offset[5:0];

wire pcs0_hit = win_hit && (pcs_select == 3'd0); // clock-active status
wire pcs1_hit = win_hit && (pcs_select == 3'd1); // command(read)/response(write) latch
wire pcs2_hit = win_hit && (pcs_select == 3'd2); // PIT0
wire pcs3_hit = win_hit && (pcs_select == 3'd3); // PIT1
wire pcs4_hit = win_hit && (pcs_select == 3'd4); // dac9 (word-write only)
// pcs5 (Ataxx-only) intentionally unclaimed -- falls through to the
// "unimplemented read returns 0xFFFF, write ignored" default below,
// same as leland_a.cpp's own `m_type <= TYPE_REDLINE` guard skipping it.

// Any PCS-window hit that isn't PIT0/PIT1 (pcs0/1/4, plus the
// never-really-hit pcs5/select-6/7 fallthrough) is serviced locally via
// the registered `local_ack` below -- see the "why no immediate ack"
// note at `cpu_ack`'s assignment for why this must NOT include a
// same-cycle-as-access ack path.
wire local_reg_hit = win_hit && !pcs2_hit && !pcs3_hit;

// --- I/O-space dac_w hit decode (whole 64K word-addressed io space) ---
// Also deliberately not gated on cpu_access -- see win_hit_now's
// comment above. cpu_d_io is a stable-per-transaction Core output, so
// this needs no separate latch (unlike win_hit, it has no externally-
// changing input to race against).
wire io_hit = cpu_d_io;
// cpu_addr is a WORD address ([19:1], bit0 implicit); io space's word
// offset equals cpu_addr directly (base 0), so "offset & 7" reads
// cpu_addr's three LSBs starting at bit 1, and "offset & 0x60" (bits
// [6:5] of the word offset) reads cpu_addr[7:6].
wire [2:0] dac_index  = cpu_addr[3:1];       // offset & 7
wire [1:0] drq_region = cpu_addr[7:6];       // offset & 0x60, bits [6:5]
wire wr_lo = cpu_bytesel[0]; // ACCESSING_BITS_0_7  (low byte lane or word)
wire wr_hi = cpu_bytesel[1]; // ACCESSING_BITS_8_15 (high byte lane or word)

wire claimed_hit = io_hit || win_hit;

// --- ack (same "hold while access asserted" idiom as i186_periph.sv /
// kf8253_leland_bus.sv, needed so a multi-cycle-held `access` from the
// caller can't re-trigger the write side effect a second time) ---
reg local_ack;
wire local_hit = io_hit || local_reg_hit;
wire do_write = local_hit && cpu_access && !local_ack && cpu_wr_en;

logic pit0_access, pit0_ack; logic [15:0] pit0_data_out;
logic pit1_access, pit1_ack; logic [15:0] pit1_data_out;
assign pit0_access = pcs2_hit && cpu_access;
assign pit1_access = pcs3_hit && cpu_access;

// No branch here may ack in the same cycle `access` first asserts: the
// real s80x86 Core (rtl/s80x86/LoadStore.sv) combinationally drops its
// own `access` the instant `ack` arrives (`m_access = ... & ~m_ack`),
// so a same-cycle ack here would close a zero-delay access<->ack
// oscillation loop through the Core -- caught by ModelSim's
// `+autofindloop` when this module was first wired to the real Core
// (a BFM-driven bench can never see this, since a BFM's `access` isn't
// itself a combinational function of `ack`). Every branch below is
// therefore either a registered ack (`local_ack`/`pit0_ack`/`pit1_ack`)
// or the caller-owned `mem_ack` for genuine passthrough.
assign cpu_ack = local_hit ? local_ack :
                  pcs2_hit  ? pit0_ack  :
                  pcs3_hit  ? pit1_ack  :
                  mem_ack;

assign mem_addr    = cpu_addr;
assign mem_data_out = cpu_data_out;
assign mem_access  = cpu_access && !claimed_hit;
assign mem_wr_en   = cpu_wr_en;
assign mem_bytesel = cpu_bytesel;
assign mem_d_io    = cpu_d_io;

// --- registers ---
reg [15:0] sound_command; // written by cmd_wr_lo/hi (Z80 side), read at pcs1
reg [7:0]  sound_response;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        sound_command  <= 16'h0000;
        sound_response <= 8'h00;
        response_wr    <= 1'b0;
    end else begin
        response_wr <= 1'b0;
        if (cmd_wr_lo) sound_command[7:0]  <= cmd_wr_data[7:0];
        if (cmd_wr_hi) sound_command[15:8] <= cmd_wr_data[15:8];
        if (do_write && pcs1_hit) begin
            sound_response <= cpu_data_out[7:0];
            response_wr    <= 1'b1;
        end
    end
end
assign response_data = sound_response;

// --- control register (/RESET, /TEST, INT0, INT1 -- leland_80186_control_w) ---
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        audiocpu_reset_n <= 1'b0; // control reset value 0xF8 -> bit7=0 -> RESET asserted
        audiocpu_test_n  <= 1'b1;
        int0_pin         <= 1'b0;
        int1_pin         <= 1'b0;
    end else if (control_wr) begin
        audiocpu_reset_n <= control_data[7];
        audiocpu_test_n  <= control_data[4];
        int0_pin         <= control_data[5];
        int1_pin         <= control_data[3];
    end
end

// --- PIT0/PIT1 (KF8253, WP5) ---
logic pit0_c0_out, pit0_c1_out, pit0_c2_out;
logic pit1_c0_out, pit1_c1_out, pit1_c2_out;

kf8253_leland_bus u_pit0(
    .clk(clk), .reset(reset),
    .local_addr(pcs_offset[1:0]), .data_in(cpu_data_out), .data_out(pit0_data_out),
    .access(pit0_access), .ack(pit0_ack), .wr_en(cpu_wr_en), .bytesel(cpu_bytesel),
    .counter_0_clock(pit_ce), .counter_0_gate(1'b1), .counter_0_out(pit0_c0_out),
    .counter_1_clock(pit_ce), .counter_1_gate(1'b1), .counter_1_out(pit0_c1_out),
    .counter_2_clock(pit_ce), .counter_2_gate(1'b1), .counter_2_out(pit0_c2_out));

kf8253_leland_bus u_pit1(
    .clk(clk), .reset(reset),
    .local_addr(pcs_offset[1:0]), .data_in(cpu_data_out), .data_out(pit1_data_out),
    .access(pit1_access), .ack(pit1_ack), .wr_en(cpu_wr_en), .bytesel(cpu_bytesel),
    .counter_0_clock(pit_ce), .counter_0_gate(1'b1), .counter_0_out(pit1_c0_out),
    .counter_1_clock(pit_ce), .counter_1_gate(1'b1), .counter_1_out(pit1_c1_out),
    .counter_2_clock(pit_ce), .counter_2_gate(1'b1), .counter_2_out(pit1_c2_out));

assign drq0_pit_level = pit0_c0_out;
assign drq1_pit_level = pit0_c1_out;

// --- clock_active status (select-0 read; bits [1:0] unused -- DMA-fed
// dac 0/1 never SET a clock_active bit in leland_a.cpp, only dac_w's
// unconditional set_clock_line(dac,0) clear ever touches them, which is
// a permanent no-op since they start and stay at 0) ---
reg pit0_c2_out_d, pit1_c0_out_d, pit1_c1_out_d, pit1_c2_out_d;
wire pit0_c2_rise = pit0_c2_out & ~pit0_c2_out_d;
wire pit1_c0_rise = pit1_c0_out & ~pit1_c0_out_d;
wire pit1_c1_rise = pit1_c1_out & ~pit1_c1_out_d;
wire pit1_c2_rise = pit1_c2_out & ~pit1_c2_out_d;

wire dac_write_now = do_write && io_hit && wr_lo;
wire dac9_write_now = do_write && pcs4_hit && (cpu_bytesel == 2'b11); // mem_mask==0xffff, R10

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        pit0_c2_out_d <= 1'b0; pit1_c0_out_d <= 1'b0;
        pit1_c1_out_d <= 1'b0; pit1_c2_out_d <= 1'b0;
        clock_active  <= 7'b0;
    end else begin
        pit0_c2_out_d <= pit0_c2_out;
        pit1_c0_out_d <= pit1_c0_out;
        pit1_c1_out_d <= pit1_c1_out;
        pit1_c2_out_d <= pit1_c2_out;

        if (pit0_c2_rise) clock_active[2] <= 1'b1;
        if (pit1_c0_rise) clock_active[3] <= 1'b1;
        if (pit1_c1_rise) clock_active[4] <= 1'b1;
        if (pit1_c2_rise) clock_active[5] <= 1'b1;
        if (t0_tc_pulse)  clock_active[6] <= 1'b1;

        // dac_w's unconditional set_clock_line(dac,0) clear -- takes
        // priority over a same-cycle set (see module header comment).
        if (dac_write_now && dac_index < 3'd6) clock_active[dac_index] <= 1'b0;
        if (dac9_write_now) clock_active[6] <= 1'b0;
    end
end

// --- DAC array writes (dac_w, whole I/O space) ---
genvar gi;
generate
    for (gi = 0; gi < 6; gi = gi + 1) begin : g_dac
        always_ff @(posedge clk or posedge reset) begin
            if (reset) begin
                // 2026-07-18 (real-hardware "pop on every reset" investigation,
                // docs/WP10_PROGRESS.md): dac_sample is offset-binary centered
                // at 128 (leland_dac_mixer.sv's own header comment) -- 8'h00
                // is -128 from center, not silence. Harmless *today* only
                // because dac_vol also resets to 0 (prod = sdiff*vol = 0
                // regardless of sdiff), but that's an accidental mute via a
                // different register, not this one actually being at its own
                // neutral value -- fixed for correctness/consistency with the
                // dac9 fix below, which does NOT have an independent volume
                // gate and was audibly wrong for real (see that reset block).
                dac_sample[gi] <= 8'h80;
                dac_vol[gi]    <= 8'h00;
                dac_wr[gi]     <= 1'b0;
            end else begin
                dac_wr[gi] <= 1'b0;
                if (dac_write_now && dac_index == gi[2:0]) begin
                    dac_sample[gi] <= cpu_data_out[7:0];
                    dac_wr[gi]     <= 1'b1;
                end
                if (do_write && io_hit && wr_hi && dac_index == gi[2:0])
                    dac_vol[gi] <= cpu_data_out[15:8];
            end
        end
    end
endgenerate

// --- DRQ software-clear strobes (dac_w's `(offset&0x60)==0x40/0x60`) ---
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        drq0_clear <= 1'b0;
        drq1_clear <= 1'b0;
    end else begin
        drq0_clear <= dac_write_now && (drq_region == 2'b10);
        drq1_clear <= dac_write_now && (drq_region == 2'b11);
    end
end

// --- dac9 (10-bit DAC, PCS select 4, full-word writes only) ---
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        // 2026-07-18 (real-hardware "pop on every reset, but the
        // boot-self-test DAC-activity counters read 0" investigation,
        // docs/WP10_PROGRESS.md): dac9 has NO independent volume
        // register (leland_dac_mixer.sv's header comment: dac9 "IS the
        // 10-bit sample", gain fixed at 1.0) -- unlike the 8-bit
        // channels above, there's nothing else gating this value to
        // silence. dac9_sample is offset-binary centered at 512
        // (leland_dac_mixer.sv: `sdiff9 = dac9_sample - 512`); the old
        // 10'h000 reset value is -512 from center, i.e. near-full-scale
        // negative, not silence. A Fable investigation traced this
        // precisely as the real, sole cause of the audible reset pop
        // (a DC step through the mixer math on every reset release,
        // requiring zero real dac9_wr activity) -- and, more
        // importantly, as a real, permanent ~-32.5k DC offset bug that
        // was silently eating almost all of dac9's negative headroom
        // even during normal operation, independent of the pop itself.
        dac9_sample <= 10'd512;
        dac9_wr     <= 1'b0;
    end else begin
        dac9_wr <= 1'b0;
        if (dac9_write_now) begin
            dac9_sample <= cpu_data_out[9:0];
            dac9_wr     <= 1'b1;
        end
    end
end

// --- local read mux (pcs0/1; pcs4/5 and io-space are write-only, read
// as 0xFFFF matching leland_a.cpp's fallthrough-to-default warn+0xffff) ---
reg [15:0] local_rd_val;
always_comb begin
    if (pcs0_hit)
        local_rd_val = {8'h00, ((({1'b0, clock_active}) >> 1) & 8'h3e)};
    else if (pcs1_hit)
        local_rd_val = sound_command;
    else
        local_rd_val = 16'hFFFF;
end

// (local_hit now covers pcs5/select-6-7 and io-space reads too -- see
// local_reg_hit's definition above -- so there is no remaining
// claimed-but-not-local case to special-case here.)
assign cpu_data_in = local_hit ? local_rd_val :
                      pcs2_hit ? pit0_data_out :
                      pcs3_hit ? pit1_data_out :
                      mem_data_in;

always_ff @(posedge clk or posedge reset) begin
    if (reset) local_ack <= 1'b0;
    else if (local_hit && cpu_access) local_ack <= 1'b1;
    else local_ack <= 1'b0;
end

endmodule
