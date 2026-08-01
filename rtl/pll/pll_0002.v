`timescale 1ns/10ps
module  pll_0002(

	// interface 'refclk'
	input wire refclk,

	// interface 'reset'
	input wire rst,

	// interface 'outclk0'
	output wire outclk_0,

	// interface 'outclk1'
	output wire outclk_1,

	// interface 'locked'
	output wire locked
);

	// WP-L3 (96 MHz dedicated SDRAM clock domain + async CDC bridge)
	// REVERTED WHOLESALE 2026-07-22. Full history preserved in git-tracked
	// prose form for anyone reviving this: WP-L3 chased a real problem
	// (TRCD_CYC/TRP_CYC rounding to a zero-slack 1 cycle at 48 MHz once
	// WP-L2 added gfx/prom-via-SDRAM traffic) but the fix -- moving
	// sdram_simple to a separate 96 MHz clock behind an async
	// toggle-handshake CDC bridge (rtl/sdram_cdc_bridge.sv) -- introduced
	// its own, larger problem: the bridge's round-trip (2-3 flop sync each
	// direction, plus FSM issue/wait states) roughly DOUBLED the
	// per-transaction latency on the single-outstanding SDRAM port versus
	// running sdram_simple directly on clk_sys. That net-negative on bus
	// throughput reproduced the exact symptom (deterministic playfield
	// garbling, sound-CPU slowdown/freeze under bus contention) the 96MHz
	// margin bump was meant to prevent -- confirmed on real hardware,
	// unchanged across two different SDRAM_CLK phase values (3472ps and
	// 5208ps), which is what proved it wasn't an analog capture-margin
	// problem at all but a bus-bandwidth one. Separately, getting
	// sdram_simple's actual read-capture point correct on its own new
	// 96MHz clock took real hardware iteration (phase_shift proportional-
	// rescale reasoning was wrong -- physical board delays are fixed
	// absolute-ns, not a period fraction; and the integer clk_b-cycle
	// capture count needed READ_CAP_EXTRA=1 to account for the ~8.5ns
	// real SDRAM_CLK forwarding delay, which sim's zero-delay ideal model
	// could never reveal) -- all of that complexity is now moot with the
	// revert. Back to the single, proven-on-real-hardware-with-sound
	// 48 MHz outclk_0 (clk_sys) + phase-shifted outclk_1 (SDRAM_CLK)
	// scheme below; the actual bandwidth fix for WP-L2's added gfx/prom
	// traffic is instead a rtl/sor_video.sv change (stop wasting rd2
	// fetches on off-screen VBlank tiles), applied on top of this
	// simpler, lower-risk baseline.
	altera_pll #(
		.fractional_vco_multiplier("false"),
		.reference_clock_frequency("50.0 MHz"),
		.operation_mode("direct"),
		.number_of_clocks(2),
		.output_clock_frequency0("48.000000 MHz"),
		.phase_shift0("0 ps"),
		.duty_cycle0(50),
		// outclk_1: feeds SDRAM_CLK, phase-referenced to clk_sys/outclk_0
		// (both 48 MHz, same domain). 3472 ps = 60 degrees of the
		// 20.833 ns period -- docs/SDRAM_TIMING_INVESTIGATION.md's B6
		// result, the best of a real hardware phase ladder (command
		// +2.95ns / data +4.88ns margin), proven on deployed hardware
		// with sound working. Do not touch without a fresh hardware
		// ladder if this period ever changes again.
		.output_clock_frequency1("48.000000 MHz"),
		.phase_shift1("3472 ps"),
		.duty_cycle1(50),
		.output_clock_frequency3("0 MHz"),
		.phase_shift3("0 ps"),
		.duty_cycle3(50),
		.output_clock_frequency4("0 MHz"),
		.phase_shift4("0 ps"),
		.duty_cycle4(50),
		.output_clock_frequency5("0 MHz"),
		.phase_shift5("0 ps"),
		.duty_cycle5(50),
		.output_clock_frequency6("0 MHz"),
		.phase_shift6("0 ps"),
		.duty_cycle6(50),
		.output_clock_frequency7("0 MHz"),
		.phase_shift7("0 ps"),
		.duty_cycle7(50),
		.output_clock_frequency8("0 MHz"),
		.phase_shift8("0 ps"),
		.duty_cycle8(50),
		.output_clock_frequency9("0 MHz"),
		.phase_shift9("0 ps"),
		.duty_cycle9(50),
		.output_clock_frequency10("0 MHz"),
		.phase_shift10("0 ps"),
		.duty_cycle10(50),
		.output_clock_frequency11("0 MHz"),
		.phase_shift11("0 ps"),
		.duty_cycle11(50),
		.output_clock_frequency12("0 MHz"),
		.phase_shift12("0 ps"),
		.duty_cycle12(50),
		.output_clock_frequency13("0 MHz"),
		.phase_shift13("0 ps"),
		.duty_cycle13(50),
		.output_clock_frequency14("0 MHz"),
		.phase_shift14("0 ps"),
		.duty_cycle14(50),
		.output_clock_frequency15("0 MHz"),
		.phase_shift15("0 ps"),
		.duty_cycle15(50),
		.output_clock_frequency16("0 MHz"),
		.phase_shift16("0 ps"),
		.duty_cycle16(50),
		.output_clock_frequency17("0 MHz"),
		.phase_shift17("0 ps"),
		.duty_cycle17(50),
		.pll_type("General"),
		.pll_subtype("General")
	) altera_pll_i (
		.rst	(rst),
		.outclk	({outclk_1, outclk_0}),
		.locked	(locked),
		.fboutclk	( ),
		.fbclk	(1'b0),
		.refclk	(refclk)
	);
endmodule

