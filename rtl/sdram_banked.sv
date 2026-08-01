// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Kevin Coleman
//
// Multi-bank open-row SDR SDRAM controller.
//
// Derived from sdram_simple (rtl/sdram.sv) by adding:
//   - Explicit bank_sel input: each caller pins its traffic to one bank
//   - Per-bank open-row state: one active row kept per bank
//   - Page-hit path: skip ACTIVATE when same row is open (~CAS_LAT cost)
//   - Page-miss path: PRECHARGE bank → ACTIVATE new row
//   - PRECHARGE-ALL before AUTO_REFRESH (see §4.4, planning_sdram_multichannel.md)
//   - tRAS honored: S_WRITE completes TWR_CYC+1 cycles to give TRCD+TWR ≥ tRAS
//
// Address layout (no bank field — bank is supplied via bank_sel):
//   addr[0]                         = byte lane
//   addr[COL_BITS:1]                = column (COL_BITS bits)
//   addr[COL_BITS+ROW_BITS:COL_BITS+1] = row (ROW_BITS bits)
//   ADDR_W = ROW_BITS + COL_BITS + 1
//
// Physical layer preserved verbatim from sdram_simple:
//   - Init FSM and mode register
//   - read_capture_sr scheduling and rdata_reg capture (do not touch)
//   - Full-word write / DQM behavior
//   - req_done pulse semantics
//   - Registered command/address/DQ outputs
//
// Rollback: revert sor_board.sv instantiation to sdram_simple; nothing else
// depends on this module.

`timescale 1ns/1ps

module sdram_banked #(
    parameter int CLK_MHZ    = 100,
    parameter int ROW_BITS   = 13,
    parameter int COL_BITS   = 9,
    parameter int BANK_BITS  = 2,
    parameter int CAS_LAT    = 2,
    parameter int READ_CAP_EXTRA = 0,
    // WP-M6 (2026-07-24): burst-read length. Must be 1, 2, 4, or 8 --
    // matches the chip's sequential-burst mode register encoding
    // ($clog2(BURST_LEN) into the BL field, see MODE_REG below). Default 1
    // reproduces today's single-word read path bit-for-bit (every existing
    // caller/instantiation is unaffected); no currently-wired client passes
    // anything else yet -- see docs/planning_sdram_multichannel.md §11 and
    // the WP-M6 plan (burst is a standalone, unwired capability this pass).
    // Writes are NOT affected by this parameter at any value: the mode
    // register's WBurst bit (bit 9) stays 1, i.e. write bursts are always
    // single-location-access regardless of BURST_LEN (plan: "burst reads
    // only, not writes").
    parameter int BURST_LEN  = 1,
    parameter int TRCD_NS    = 18,
    parameter int TRP_NS     = 18,
    parameter int TRAS_NS    = 42,   // new: minimum row-active time
    parameter int TRFC_NS    = 66,
    parameter int TWR_NS     = 12,
    parameter int TREFI_NS   = 7800,
    parameter int TINIT_NS   = 100_000,
    parameter int TMRD_CYC   = 2,

    // Derived — no bank field in addr
    parameter int WORD_ADDR_W = ROW_BITS + COL_BITS,
    parameter int ADDR_W      = WORD_ADDR_W + 1
) (
    // --- SDRAM pins ---------------------------------------------------
    output logic                         sd_cke,
    output logic                         sd_cs_n,
    output logic                         sd_ras_n,
    output logic                         sd_cas_n,
    output logic                         sd_we_n,
    output logic [BANK_BITS-1:0]         sd_ba,
    output logic [ROW_BITS-1:0]          sd_a,
    output logic [1:0]                   sd_dqm,
    output logic [15:0]                  sd_dq_out,
    output logic                         sd_dq_oe,
    input  logic [15:0]                  sd_dq_in,

    // --- Client -------------------------------------------------------
    input  logic                         clk,
    input  logic                         rst_n,
    input  logic [ADDR_W-1:0]            addr,
    input  logic [BANK_BITS-1:0]         bank_sel,   // fixed bank for this client
    input  logic [7:0]                   din,
    output logic [7:0]                   dout,
    output logic [15:0]                  dout16,
    input  logic                         rd,
    input  logic                         we,
    output logic                         ready,

    input  logic [7:0]                   din_hi,
    input  logic                         we_word,

    output logic                         req_done,

    // WP-M5 hardware diagnostic (2026-07-24): pulses one clk_sys cycle,
    // combinationally, on the exact S_IDLE cycle a request is accepted --
    // classified against bank_sel's currently-open row (page_hit below).
    // sor_board.sv gates this by which channel's sel_* was asserted that
    // same cycle (bank_sel is set combinationally off sel_*, so the two
    // always agree) to build a per-channel page-hit-rate counter, answering
    // whether the open-row design is actually landing hits on real
    // hardware or whether every access is still paying the miss/cold
    // ACTIVATE+PRECHARGE cost despite the per-bank pinning.
    output logic                         req_hit,

    // WP-M6 (2026-07-24): burst-read data. Valid for one cycle alongside
    // req_done when the read that just completed was issued under
    // BURST_LEN>1 -- fixed width (max supported BURST_LEN=8) regardless of
    // this instance's actual BURST_LEN, so the port shape never changes
    // across instantiations; only burst_words[0 +: BURST_LEN] is
    // meaningful. dout/dout16 remain the single-word interface and are
    // unaffected by BURST_LEN (existing callers keep using them exactly as
    // today). Not driven by anything yet this pass -- no client reads it.
    output logic [7:0][15:0]             burst_words,

    // WP-M8 (2026-07-24): burst-SAFE single-byte read data -- see dout's
    // comment above for why dout itself is unsafe once BURST_LEN>1
    // (rdata_reg is overwritten by every burst word, ending up holding the
    // LAST word, not the one a single-byte caller actually wanted). bw0 is
    // written exactly once per transaction (only at burst_cnt==0), so it
    // is stable/correct at any BURST_LEN. dout_b0 applies the same
    // rq_byte_sel gating dout does, sourced from bw0 instead of rdata_reg
    // -- at BURST_LEN=1, bw0 captures identically to rdata_reg (same
    // read_capture_sr[0] pulse, same cycle), so dout_b0 == dout bit-for-bit
    // today; this port exists so callers can migrate off dout/dout16 onto
    // a burst-safe path BEFORE BURST_LEN is ever raised above 1 for their
    // channel, resolving WP-M6's open risk instead of leaving it latent.
    output logic [7:0]                   dout_b0
);

    // --- Derived cycle counts ----------------------------------------
    localparam int TRCD_CYC  = (TRCD_NS  * CLK_MHZ + 999) / 1000;
    localparam int TRP_CYC   = (TRP_NS   * CLK_MHZ + 999) / 1000;
    localparam int TRAS_CYC  = (TRAS_NS  * CLK_MHZ + 999) / 1000;
    localparam int TRFC_CYC  = (TRFC_NS  * CLK_MHZ + 999) / 1000;
    localparam int TWR_CYC   = (TWR_NS   * CLK_MHZ + 999) / 1000;
    localparam int TREFI_CYC = (TREFI_NS * CLK_MHZ + 999) / 1000;
    localparam int TINIT_CYC = (TINIT_NS * CLK_MHZ + 999) / 1000;
    localparam int CNT_W     = 16;

    // --- SDRAM command encoding {CS,RAS,CAS,WE}, active low ----------
    localparam [3:0] CMD_NOP          = 4'b0111;
    localparam [3:0] CMD_ACTIVE       = 4'b0011;
    localparam [3:0] CMD_READ         = 4'b0101;
    localparam [3:0] CMD_WRITE        = 4'b0100;
    localparam [3:0] CMD_PRECHARGE    = 4'b0010;
    localparam [3:0] CMD_AUTO_REFRESH = 4'b0001;
    localparam [3:0] CMD_LOAD_MODE    = 4'b0000;
    localparam [3:0] CMD_DESELECT     = 4'b1111;

    // --- FSM ----------------------------------------------------------
    typedef enum logic [3:0] {
        S_INIT_WAIT, S_INIT_PRE, S_INIT_REF1, S_INIT_REF2, S_INIT_MRS,
        S_IDLE,
        S_PRE,       // PRECHARGE specific bank (page-miss path)
        S_ACT,       // ACTIVATE row (cold or after miss precharge)
        S_READ,      // READ with A10=0 (hit and miss paths share this)
        S_WRITE,     // WRITE with A10=0 (hit and miss paths share this)
        S_PRE_ALL,   // PRECHARGE ALL before AUTO_REFRESH
        S_REFRESH
    } state_e;

    state_e            state;
    logic [CNT_W-1:0]  wait_cnt;
    logic [CNT_W-1:0]  refresh_cnt;
    logic              refresh_pending;

    // tRAS gate: counts down from TRAS_CYC on every ACTIVATE (any bank),
    // conservatively tracking only the most-recent one. AUTO_REFRESH's
    // PRECHARGE-ALL must not close a row before tRAS has elapsed since
    // its own ACTIVATE (real protocol requirement -- planning_sdram_
    // multichannel.md §4.5 flagged this as required and TRAS_CYC was
    // computed but never wired to anything, leaving a real gap: a cold
    // ACTIVATE->WRITE sequence only holds a row open TRCD_CYC+TWR_CYC
    // cycles (2 at 48MHz) before returning to S_IDLE, one cycle short of
    // TRAS_CYC (3) -- if refresh_pending is already true at that exact
    // moment, S_PRE_ALL fired immediately, violating tRAS).
    logic [CNT_W-1:0]  ras_wait_cnt;
    wire               tras_ok = (ras_wait_cnt == 0);

    // --- Per-bank open-row state -------------------------------------
    // Explicit scalar registers per bank rather than unpacked arrays:
    // Quartus 17.0 missynths dynamic-index unpacked arrays — banks whose
    // 2-bit index has equal bits (2'b00=0, 2'b11=3) get wrong enable
    // demux and never update.  Constant-index ops (reset loops, S_ACT
    // case) are unaffected; only the dynamic reads/writes needed fixing.
    localparam int NBANK = 1 << BANK_BITS;

    logic [ROW_BITS-1:0] or0, or1, or2, or3; // open_row per bank 0..3
    logic                rv0, rv1, rv2, rv3;  // row_valid per bank 0..3

    // Combinational read muxes (bank_sel selects current-cycle bank).
    logic [ROW_BITS-1:0] cur_open_row;
    logic                cur_row_valid;
    always_comb begin
        case (bank_sel)
            2'd0: begin cur_open_row = or0; cur_row_valid = rv0; end
            2'd1: begin cur_open_row = or1; cur_row_valid = rv1; end
            2'd2: begin cur_open_row = or2; cur_row_valid = rv2; end
            default: begin cur_open_row = or3; cur_row_valid = rv3; end
        endcase
    end

    // Latched request
    logic              rq_write;
    logic              rq_write_word;
    logic              rq_byte_sel;
    logic [ROW_BITS-1:0]  rq_row;
    logic [BANK_BITS-1:0] rq_bank;
    logic [COL_BITS-1:0]  rq_col;
    logic [7:0]           rq_din;
    logic [7:0]           rq_din_hi;

    // WP-M6: burst word index within the current read (0..BURST_LEN-1),
    // reset explicitly at each new S_READ entry (both the page-hit and
    // post-S_ACT paths below), incremented once per read_capture_sr[0]
    // capture pulse. Compared against constants only (BURST_LEN-1, and the
    // case statement below uses constant case labels) -- deliberately NOT
    // used as a dynamic array index (see the or0/or1/or2/or3 comment above:
    // Quartus 17.0 missynths dynamic-index unpacked arrays on this exact
    // class of per-bank/per-word register file; bw0..bw7 below follow that
    // same proven-safe named-scalar-plus-case pattern instead).
    logic [3:0]           burst_cnt;

    // Address field extraction from the client's byte address (no bank field).
    wire                       a_byte_sel = addr[0];
    wire [COL_BITS-1:0]        a_col      = addr[1 +: COL_BITS];
    wire [ROW_BITS-1:0]        a_row      = addr[COL_BITS + 1 +: ROW_BITS];

    // Page-hit/miss/cold classification (combinational, against bank_sel's row).
    wire page_hit  = cur_row_valid && (cur_open_row == a_row);
    wire page_cold = !cur_row_valid;
    // page_miss = row_valid && open_row != a_row (implicit)

    // Mode register: BL=BURST_LEN (sequential burst type, bit3=0), CL
    // parameterized. WBurst (bit 9) stays 1 -- write bursts are always
    // single-location-access regardless of BURST_LEN; only read bursts use
    // the BL field (see BURST_LEN comment on the module port list).
    // BURST_LEN=1 -> $clog2(1)=0 -> 3'b000, byte-for-byte the same MODE_REG
    // value this module has always programmed.
    localparam int BURST_ORDER = $clog2(BURST_LEN);
    localparam logic [12:0] MODE_REG = {3'b000, 1'b1, 2'b00, CAS_LAT[2:0], 1'b0, BURST_ORDER[2:0]};

    // WP-M6: total wait_cnt load for S_READ, covering CAS_LAT+READ_CAP_EXTRA
    // cycles until the FIRST word's capture plus BURST_LEN-1 more cycles for
    // the remaining words in the burst (chip streams them out one per
    // cycle, sequential burst mode). BURST_LEN=1 reduces this to exactly
    // the original CAS_LAT+READ_CAP_EXTRA-1 expression.
    localparam int READ_WAIT_LOAD = CAS_LAT + READ_CAP_EXTRA + BURST_LEN - 2;

    // Read capture shift register — preserved verbatim from sdram_simple.
    logic [15:0] read_capture_sr;

    // =================================================================
    assign ready = (state == S_IDLE) && !refresh_pending;

    // req_hit: combinational, true exactly on the cycle a request is
    // accepted out of S_IDLE (rd/we/we_word high, no refresh pending) AND
    // that request turned out to be a page hit. Mirrors the acceptance
    // condition in the S_IDLE case below exactly (do not let the two
    // drift apart).
    wire req_accept = (state == S_IDLE) && !(refresh_pending && tras_ok) && (rd || we || we_word);
    assign req_hit = req_accept && page_hit;

    // =================================================================
    // Main FSM + refresh counter
    // =================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_INIT_WAIT;
            wait_cnt         <= TINIT_CYC[CNT_W-1:0];
            refresh_cnt      <= '0;
            refresh_pending  <= 1'b0;
            read_capture_sr  <= '0;
            rq_write         <= 1'b0;
            rq_write_word    <= 1'b0;
            rq_byte_sel      <= 1'b0;
            rq_row           <= '0;
            rq_bank          <= '0;
            rq_col           <= '0;
            rq_din           <= '0;
            rq_din_hi        <= '0;
            burst_cnt        <= '0;
            or0 <= '0; or1 <= '0; or2 <= '0; or3 <= '0;
            rv0 <= 1'b0; rv1 <= 1'b0; rv2 <= 1'b0; rv3 <= 1'b0;
            ras_wait_cnt     <= '0;
        end else begin
            if (wait_cnt != 0) wait_cnt <= wait_cnt - 1'b1;
            if (ras_wait_cnt != 0) ras_wait_cnt <= ras_wait_cnt - 1'b1;

            // Shift-right read-capture pulse — bit 0 triggers DQ sample.
            read_capture_sr <= {1'b0, read_capture_sr[15:1]};

            // WP-M6: advance the burst word index on every capture pulse.
            // Safe against collision with the explicit burst_cnt<='0' resets
            // below (S_IDLE hit-path / S_ACT entry into S_READ): those only
            // fire when a NEW read is starting, which cannot coincide with
            // a capture pulse from the PREVIOUS read (the controller never
            // starts a new request until the previous one's req_done --
            // gated on this same read_capture_sr mechanism -- has fired).
            if (read_capture_sr[0]) burst_cnt <= burst_cnt + 4'd1;

            if (refresh_cnt == TREFI_CYC[CNT_W-1:0]) begin
                refresh_cnt     <= '0;
                refresh_pending <= 1'b1;
            end else begin
                refresh_cnt <= refresh_cnt + 1'b1;
            end

            unique case (state)
                // --- Init sequence ------------------------------------
                S_INIT_WAIT: if (wait_cnt == 0) begin
                    state    <= S_INIT_PRE;
                    wait_cnt <= TRP_CYC[CNT_W-1:0] - 1'b1;
                end

                S_INIT_PRE: if (wait_cnt == 0) begin
                    state    <= S_INIT_REF1;
                    wait_cnt <= TRFC_CYC[CNT_W-1:0] - 1'b1;
                end

                S_INIT_REF1: if (wait_cnt == 0) begin
                    state    <= S_INIT_REF2;
                    wait_cnt <= TRFC_CYC[CNT_W-1:0] - 1'b1;
                end

                S_INIT_REF2: if (wait_cnt == 0) begin
                    state    <= S_INIT_MRS;
                    wait_cnt <= TMRD_CYC[CNT_W-1:0] - 1'b1;
                end

                S_INIT_MRS: if (wait_cnt == 0) state <= S_IDLE;

                // --- Normal operation ---------------------------------
                S_IDLE: begin
                    if (refresh_pending && tras_ok) begin
                        // PRECHARGE ALL before AUTO_REFRESH: close every open row.
                        // Gated on tras_ok -- see ras_wait_cnt comment above; do not
                        // close a row before tRAS has elapsed since its ACTIVATE.
                        state           <= S_PRE_ALL;
                        wait_cnt        <= TRP_CYC[CNT_W-1:0] - 1'b1;
                        refresh_pending <= 1'b0;
                        rv0 <= 1'b0; rv1 <= 1'b0; rv2 <= 1'b0; rv3 <= 1'b0;
                    end else if (rd || we || we_word) begin
                        rq_write      <= we || we_word;
                        rq_write_word <= we_word;
                        rq_byte_sel   <= a_byte_sel;
                        rq_bank       <= bank_sel;
                        rq_col        <= a_col;
                        rq_din        <= din;
                        rq_din_hi     <= din_hi;

                        if (page_hit) begin
                            // Hit: go straight to READ/WRITE, no ACTIVATE.
                            // row stays open; sr set here mirrors S_ACT→S_READ pattern.
                            rq_row <= a_row; // not strictly needed but keep consistent
                            if (we || we_word) begin
                                state    <= S_WRITE;
                                // TWR_CYC cycles (not TWR_CYC-1) to satisfy tRAS:
                                // no TRCD was spent so tRAS window starts from the
                                // original ACTIVATE; extra cycle keeps us safe.
                                wait_cnt <= TWR_CYC[CNT_W-1:0];
                            end else begin
                                state     <= S_READ;
                                wait_cnt  <= READ_WAIT_LOAD[CNT_W-1:0];
                                burst_cnt <= '0;
                                for (int bi = 0; bi < BURST_LEN; bi++)
                                    read_capture_sr[CAS_LAT + READ_CAP_EXTRA + bi] <= 1'b1;
                            end
                        end else begin
                            // Miss or cold: latch row, go to precharge (miss) or activate (cold).
                            rq_row <= a_row;
                            if (page_cold) begin
                                state    <= S_ACT;
                                wait_cnt <= TRCD_CYC[CNT_W-1:0] - 1'b1;
                            end else begin
                                // Miss: precharge open row first.
                                state    <= S_PRE;
                                wait_cnt <= TRP_CYC[CNT_W-1:0] - 1'b1;
                                case (bank_sel)
                                    2'd0: rv0 <= 1'b0;
                                    2'd1: rv1 <= 1'b0;
                                    2'd2: rv2 <= 1'b0;
                                    default: rv3 <= 1'b0;
                                endcase
                            end
                        end
                    end
                end

                // Specific-bank precharge (page-miss path).
                S_PRE: if (wait_cnt == 0) begin
                    state    <= S_ACT;
                    wait_cnt <= TRCD_CYC[CNT_W-1:0] - 1'b1;
                end

                S_ACT: if (wait_cnt == 0) begin
                    // Update open-row state on ACTIVATE.
                    case (rq_bank)
                        2'd0: begin or0 <= rq_row; rv0 <= 1'b1; end
                        2'd1: begin or1 <= rq_row; rv1 <= 1'b1; end
                        2'd2: begin or2 <= rq_row; rv2 <= 1'b1; end
                        default: begin or3 <= rq_row; rv3 <= 1'b1; end
                    endcase
                    // Reload tRAS gate -- this ACTIVATE just (re)opened a row;
                    // block AUTO_REFRESH's PRECHARGE-ALL until tRAS elapses.
                    ras_wait_cnt <= TRAS_CYC[CNT_W-1:0];
                    if (rq_write) begin
                        state    <= S_WRITE;
                        // TWR_CYC cycles — gives tRAS margin (TRCD + TWR ≥ tRAS at 48MHz).
                        wait_cnt <= TWR_CYC[CNT_W-1:0];
                    end else begin
                        state     <= S_READ;
                        wait_cnt  <= READ_WAIT_LOAD[CNT_W-1:0];
                        burst_cnt <= '0;
                        for (int bi = 0; bi < BURST_LEN; bi++)
                            read_capture_sr[CAS_LAT + READ_CAP_EXTRA + bi] <= 1'b1;
                    end
                end

                S_WRITE:   if (wait_cnt == 0) state <= S_IDLE;
                S_READ:    if (wait_cnt == 0) state <= S_IDLE;

                // PRECHARGE ALL (before refresh) — row_valid cleared in S_IDLE above.
                S_PRE_ALL: if (wait_cnt == 0) begin
                    state    <= S_REFRESH;
                    wait_cnt <= TRFC_CYC[CNT_W-1:0] - 1'b1;
                end

                S_REFRESH: if (wait_cnt == 0) state <= S_IDLE;

                default:   state <= S_IDLE;
            endcase
        end
    end

    // read_done_d: read_capture_sr[0] delayed by one more cycle -- fires
    // exactly when rdata_reg's registered update (rdata_reg<=sd_dq_in,
    // triggered by read_capture_sr[0]) has actually become visible.
    // Without this, req_done's READ condition (wait_cnt==0, registered)
    // lands on the SAME cycle read_capture_sr[0] itself fires -- one cycle
    // too early, since rdata_reg's new value isn't visible until the cycle
    // after. That race was confirmed directly via ModelSim (DQSNOOP probe,
    // 2026-07-23 session): sd_dq_in/the chip's own Dq_reg are correct at
    // the capture cycle, but the ack captures the stale pre-update
    // rdata_reg/dout value one cycle later. Deliberately does NOT touch
    // wait_cnt or the S_READ comb block's READ-command-issue timing (those
    // are correct, proven by the same probe) or `ready`'s state==S_IDLE
    // timing (sor_board.sv's own in_flight gate, not the controller's
    // `ready`, is what actually blocks a new request until req_done fires,
    // so delaying only req_done here is safe).
    // WP-M6: for a burst, read_capture_sr[0] fires once per word (BURST_LEN
    // times total) -- req_done must only ack once, after the LAST word's
    // capture becomes visible, not after every word. burst_cnt is the
    // pre-increment index (see the "advance the burst word index" block
    // above), so it still reads BURST_LEN-1 on the cycle the last pulse
    // fires. BURST_LEN=1: burst_cnt is always 0 at the (only) pulse, so
    // last_word_capture reduces to exactly read_capture_sr[0], unchanged.
    wire last_word_capture = read_capture_sr[0] && (burst_cnt == BURST_LEN[3:0] - 4'd1);

    logic read_done_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) read_done_d <= 1'b0;
        else        read_done_d <= last_word_capture;
    end

    // req_done: fires same cycle ready re-asserts for WRITE; for READ,
    // fires one cycle after read_capture_sr[0] (see read_done_d above) so
    // the ack always captures the correctly-updated rdata_reg/dout.
    // Fires only for client transactions, not for refresh or precharge cycles.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) req_done <= 1'b0;
        else req_done <= ((state == S_WRITE) && (wait_cnt == 0)) || read_done_d;
    end

    // =================================================================
    // Read data — preserved verbatim from sdram_simple.
    // DO NOT modify read_capture_sr scheduling or rdata_reg capture;
    // see sdram_simple's long comment at the equivalent block.
    // =================================================================
    logic [15:0] rdata_reg;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) rdata_reg <= '0;
        else if (read_capture_sr[0]) rdata_reg <= sd_dq_in;
    end
    assign dout   = rq_byte_sel ? rdata_reg[15:8] : rdata_reg[7:0];
    assign dout16 = rdata_reg;

    // =================================================================
    // WP-M6: burst word capture -- purely additive alongside rdata_reg
    // above (untouched). Named scalars + a constant-case-label switch,
    // NOT a dynamically-indexed array (see burst_cnt's declaration comment
    // for why: this file already hit a real Quartus 17.0 dynamic-index
    // synthesis bug once, on the per-bank open-row registers). At
    // BURST_LEN=1 this still runs (bw0 gets written every read, alongside
    // rdata_reg) but nothing reads bw0 unless a caller uses burst_words,
    // which no currently-wired client does.
    // =================================================================
    logic [15:0] bw0, bw1, bw2, bw3, bw4, bw5, bw6, bw7;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bw0 <= '0; bw1 <= '0; bw2 <= '0; bw3 <= '0;
            bw4 <= '0; bw5 <= '0; bw6 <= '0; bw7 <= '0;
        end else if (read_capture_sr[0]) begin
            case (burst_cnt)
                4'd0: bw0 <= sd_dq_in;
                4'd1: bw1 <= sd_dq_in;
                4'd2: bw2 <= sd_dq_in;
                4'd3: bw3 <= sd_dq_in;
                4'd4: bw4 <= sd_dq_in;
                4'd5: bw5 <= sd_dq_in;
                4'd6: bw6 <= sd_dq_in;
                4'd7: bw7 <= sd_dq_in;
                default: ;
            endcase
        end
    end
    assign dout_b0        = rq_byte_sel ? bw0[15:8] : bw0[7:0];
    assign burst_words[0] = bw0;
    assign burst_words[1] = bw1;
    assign burst_words[2] = bw2;
    assign burst_words[3] = bw3;
    assign burst_words[4] = bw4;
    assign burst_words[5] = bw5;
    assign burst_words[6] = bw6;
    assign burst_words[7] = bw7;

    // WP-M6 sim-only diagnostic: a burst read must stay within one row (the
    // chip wraps the column counter within the row on overflow, per
    // planning_sdram_multichannel.md §11's alignment constraint) -- flag a
    // request whose column isn't BURST_LEN-aligned instead of silently
    // producing wrapped/wrong data. Display-only (matches this project's
    // existing style of unconditional $display diagnostics in
    // sim/mt48lc16m16a2.v), not a hard stop.
`ifndef ALTERA_RESERVED_QIS
    // ALIGN_BITS floors at 1 (rather than BURST_ORDER, which is 0 when
    // BURST_LEN=1) purely so the part-select below always elaborates to a
    // valid, non-negative width regardless of BURST_LEN -- the runtime
    // `BURST_LEN > 1` guard already ensures this never actually fires at
    // BURST_LEN=1, so the slice's content doesn't matter there.
    localparam int ALIGN_BITS = (BURST_ORDER > 0) ? BURST_ORDER : 1;
    // 2026-07-24: was an unconditional $display per event, which fired 141,522
    // times in one 790 ms ModelSim run (14 MB of log) and made up ~99% of the
    // output of a Verilated run. That is a real tax on long runs, and the
    // name overstates it: a misaligned burst only corrupts burst word1 (word0
    // is always the addressed word), and the ONLY consumer of word1 is rd2's
    // GFXROW read, which is column-aligned by construction (see
    // rtl/sor_video.sv's ADDR_GFXROW_BASE comment). Every firing observed so
    // far is a single-word PROM / line-cache refill that never reads word1 --
    // i.e. benign. Now counts them and prints a handful of examples plus a
    // final total, so the diagnostic keeps all of its value at none of the
    // cost. A NONZERO total is still worth understanding; a nonzero total
    // combined with wrong rd2 tile graphics is the case that actually matters.
    integer burst_align_err_cnt = 0;
    always_ff @(posedge clk) begin
        // 2026-07-26: was qualified on `(state == S_IDLE) && rd`, which is NOT
        // the accept condition (see req_accept above) -- a read held pending
        // across refresh or an un-accepted idle cycle was re-counted every
        // cycle it waited, so the total counted CYCLES, not REQUESTS, and
        // overstated by more than an order of magnitude (50,825 reported
        // against a run whose entire misaligned-capable read population was a
        // few thousand). Qualifying on req_accept counts each accepted
        // misaligned read exactly once.
        if (rst_n && (BURST_LEN > 1) && req_accept && rd
            && (a_col[ALIGN_BITS-1:0] != '0)) begin
            burst_align_err_cnt <= burst_align_err_cnt + 1;
            if (burst_align_err_cnt < 5)
                $display("%0t: %m BURST_ALIGN_NOTE: rd col=%0d not aligned to BURST_LEN=%0d (example %0d; word0 unaffected, only word1 wraps)",
                          $time, a_col, BURST_LEN, burst_align_err_cnt);
        end
    end
    final begin
        if (burst_align_err_cnt != 0)
            $display("%m BURST_ALIGN_SUMMARY: %0d misaligned burst reads (benign unless rd2 GFXROW tile data is wrong -- only burst word1 is affected)",
                      burst_align_err_cnt);
    end
`endif

    // =================================================================
    // Combinational next-command / address / data
    // =================================================================
    logic [3:0]               cmd_nxt;
    logic [BANK_BITS-1:0]     sd_ba_nxt;
    logic [ROW_BITS-1:0]      sd_a_nxt;
    logic [1:0]               sd_dqm_nxt;
    logic [15:0]              sd_dq_out_nxt;
    logic                     sd_dq_oe_nxt;

    always_comb begin
        cmd_nxt       = CMD_NOP;
        sd_ba_nxt     = '0;
        sd_a_nxt      = '0;
        sd_dqm_nxt    = 2'b00;
        sd_dq_out_nxt = '0;
        sd_dq_oe_nxt  = 1'b0;

        unique case (state)
            S_INIT_WAIT: begin
                cmd_nxt    = CMD_DESELECT;
                sd_dqm_nxt = 2'b11;
            end

            S_INIT_PRE: begin
                if (wait_cnt == TRP_CYC[CNT_W-1:0] - 1'b1) begin
                    cmd_nxt     = CMD_PRECHARGE;
                    sd_a_nxt    = '0;
                    sd_a_nxt[10]= 1'b1;   // all banks
                end
                sd_dqm_nxt = 2'b11;
            end

            S_INIT_REF1, S_INIT_REF2: begin
                if (wait_cnt == TRFC_CYC[CNT_W-1:0] - 1'b1)
                    cmd_nxt = CMD_AUTO_REFRESH;
                sd_dqm_nxt = 2'b11;
            end

            S_INIT_MRS: begin
                if (wait_cnt == TMRD_CYC[CNT_W-1:0] - 1'b1) begin
                    cmd_nxt        = CMD_LOAD_MODE;
                    sd_a_nxt[12:0] = MODE_REG;
                    sd_ba_nxt      = '0;
                end
                sd_dqm_nxt = 2'b11;
            end

            // PRECHARGE specific bank (page-miss: close old row).
            // A10=0 means single-bank precharge; sd_ba selects the bank.
            S_PRE: begin
                if (wait_cnt == TRP_CYC[CNT_W-1:0] - 1'b1) begin
                    cmd_nxt      = CMD_PRECHARGE;
                    sd_ba_nxt    = rq_bank;
                    sd_a_nxt     = '0;
                    sd_a_nxt[10] = 1'b0;  // single bank
                end
            end

            S_ACT: begin
                if (wait_cnt == TRCD_CYC[CNT_W-1:0] - 1'b1) begin
                    cmd_nxt   = CMD_ACTIVE;
                    sd_ba_nxt = rq_bank;
                    sd_a_nxt  = rq_row;
                end
            end

            S_WRITE: begin
                if (wait_cnt == TWR_CYC[CNT_W-1:0]) begin
                    cmd_nxt              = CMD_WRITE;
                    sd_ba_nxt            = rq_bank;
                    sd_a_nxt             = '0;
                    sd_a_nxt[COL_BITS-1:0] = rq_col;
                    sd_a_nxt[10]         = 1'b0;     // no auto-precharge; row stays open
                end
                // Drive DQ while WRITE in progress (wait_cnt != 0: active write cycles only).
                if (wait_cnt != 0) begin
                    sd_dq_oe_nxt  = 1'b1;
                    if (rq_write_word) begin
                        // Full word write — DQM=00, no per-byte masking.
                        // Preserved verbatim from sdram_simple (DQM has no effect
                        // on this board's hardware per the proven rtl/sdram.sv comment).
                        sd_dq_out_nxt = {rq_din_hi, rq_din};
                        sd_dqm_nxt    = 2'b00;
                    end else begin
                        sd_dq_out_nxt = {rq_din, rq_din};
                        sd_dqm_nxt    = rq_byte_sel ? 2'b01 : 2'b10;
                    end
                end
            end

            S_READ: begin
                if (wait_cnt == READ_WAIT_LOAD[CNT_W-1:0]) begin
                    cmd_nxt              = CMD_READ;
                    sd_ba_nxt            = rq_bank;
                    sd_a_nxt             = '0;
                    sd_a_nxt[COL_BITS-1:0] = rq_col;
                    sd_a_nxt[10]         = 1'b0;  // no auto-precharge; row stays open
                end
                sd_dqm_nxt = 2'b00;
            end

            // PRECHARGE ALL (A10=1) — closes every open bank before AUTO_REFRESH.
            S_PRE_ALL: begin
                if (wait_cnt == TRP_CYC[CNT_W-1:0] - 1'b1) begin
                    cmd_nxt      = CMD_PRECHARGE;
                    sd_a_nxt     = '0;
                    sd_a_nxt[10] = 1'b1;  // all banks
                end
            end

            S_REFRESH: begin
                if (wait_cnt == TRFC_CYC[CNT_W-1:0] - 1'b1)
                    cmd_nxt = CMD_AUTO_REFRESH;
            end

            default: ;
        endcase
    end

    // --- Registered SDRAM outputs ------------------------------------
    logic [3:0] cmd_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_r      <= CMD_NOP;
            sd_ba      <= '0;
            sd_a       <= '0;
            sd_dqm     <= 2'b11;
            sd_dq_out  <= '0;
            sd_dq_oe   <= 1'b0;
        end else begin
            cmd_r      <= cmd_nxt;
            sd_ba      <= sd_ba_nxt;
            sd_a       <= sd_a_nxt;
            sd_dqm     <= sd_dqm_nxt;
            sd_dq_out  <= sd_dq_out_nxt;
            sd_dq_oe   <= sd_dq_oe_nxt;
        end
    end

    assign sd_cs_n  = cmd_r[3];
    assign sd_ras_n = cmd_r[2];
    assign sd_cas_n = cmd_r[1];
    assign sd_we_n  = cmd_r[0];

    assign sd_cke = rst_n;

endmodule
