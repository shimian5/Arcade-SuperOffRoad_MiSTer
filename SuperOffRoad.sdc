## Super Off Road — Timing Constraints
## clk_sys = 48 MHz from PLL

derive_pll_clocks
derive_clock_uncertainty

## SDRAM I/O: REVERTED (2026-07-17) to the blanket-false-path scheme.
## docs/sdram_plan.md Section 3c's real SDC (create_generated_clock +
## set_input/output_delay + set_multicycle_path) was started on B6's
## winning config (outclk_1 @ 60deg, capture at T4) but never finished
## converging -- see docs/SDRAM_TIMING_INVESTIGATION.md Step 4 for the
## exact state (DQ path reached -10.1ns setup slack, still violated;
## command path -3.3ns, root cause not found). Left in that half-done
## state while the investigation pivoted to the race-freeze bug, then
## built as-is -- and the resulting build showed real, heavy SDRAM
## corruption (live_mm counter saturated at 0xFF, core wouldn't boot
## across 3-4 reboots) that B6 never showed under the OLD blanket
## false-path SDC. Suspected cause: Quartus's fitter can use SDC
## constraints to guide retiming/placement even when the constraints
## are violated, so a half-correct real SDC can produce WORSE placement
## than a blanket exemption that tells Quartus not to touch SDRAM
## timing at all. Reverted here to isolate/confirm; do not re-attempt
## Section 3c until this is retested and the multicycle relationships
## are worked out to a clean, fully-converged (zero violations) state
## BEFORE building on real hardware again -- verify via report_timing
## against the compiled db first, same as the original investigation
## discipline (docs/SDRAM_TIMING_INVESTIGATION.md Section 1.3/Step 0).
## Matches the pattern used by Arcade-IGSPGM_MiSTer and other Sorgelig cores.
set_false_path -to   [get_ports {SDRAM_CLK SDRAM_A[*] SDRAM_BA[*] SDRAM_nCS SDRAM_nRAS SDRAM_nCAS SDRAM_nWE SDRAM_DQML SDRAM_DQMH SDRAM_DQ[*]}]
set_false_path -from [get_ports {SDRAM_DQ[*]}]
