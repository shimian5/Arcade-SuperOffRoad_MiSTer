//============================================================================
//  Generic True Dual-Port Block RAM
//  Used for: Video RAM (128 KB), Color RAM (1 KB), work RAM, etc.
//============================================================================

module sor_dpram #(
	parameter ADDR_WIDTH = 17,
	parameter DATA_WIDTH = 8
)(
	input                    clk,

	// Port A (typically CPU write)
	input  [ADDR_WIDTH-1:0]  addr_a,
	input  [DATA_WIDTH-1:0]  din_a,
	input                    we_a,
	output reg [DATA_WIDTH-1:0] dout_a,

	// Port B (typically video read)
	input  [ADDR_WIDTH-1:0]  addr_b,
	input  [DATA_WIDTH-1:0]  din_b,
	input                    we_b,
	output reg [DATA_WIDTH-1:0] dout_b
);

// 2026-07-12 session: removed write-forwarding (dout <= din on write),
// a non-native M10K read-during-write pattern flagged in
// docs/m10k_to_review.md and confirmed against a real, working
// reference core (MiSTer-devel/Arcade-SNK_TripleZ80_MiSTer's
// SRAM_dual_sync.v): "Q0 <= mem[ADDR0]; if(we0) mem[ADDR0] <= DATA0;"
// -- unconditional read-old-data every cycle, write applied
// afterward/independently, no forwarding. ramstyle="no_rw_check"
// matches that same reference core's declaration exactly, telling
// Quartus to skip generating read-during-write hazard-detection/
// compatibility logic this design never actually needs (no port here
// ever reads and writes the same address in the same cycle by
// design), rather than leaving that decision to the synthesizer.
reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];

always @(posedge clk) begin
	dout_a <= mem[addr_a];
	if (we_a) mem[addr_a] <= din_a;
end

// NOTE (2026-07-17): mixed-port read-during-write was investigated as
// a sim-vs-silicon divergence candidate for the on-hardware text-gap
// artifact and RULED OUT: Quartus map already infers altsyncram with
// READ_DURING_WRITE_MODE_MIXED_PORTS=OLD_DATA for every sor_dpram
// instance (vram/cram/wram), which matches this behavioral model's
// old-data semantics on silicon. Do not add a collision bypass here --
// it would change semantics (new data) for no benefit.
always @(posedge clk)
begin
    dout_b <= mem[addr_b];
end

endmodule
