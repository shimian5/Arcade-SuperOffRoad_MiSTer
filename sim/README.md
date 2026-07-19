# SDRAM controller + board integration testbenches

Standalone simulation of `rtl/sdram.sv` against Micron's official
`mt48lc16m16a2` behavioral SDRAM model — no HPS, no CPUs, no video, no
Quartus vendor simulation libraries required. Exists because a full
debug cycle on real hardware (Quartus rebuild → flash → read the debug
overlay by eye) takes minutes per iteration; this reproduces the same
write/read patterns in seconds.

## Files

- `sdram_tb.sv` — the testbench (see its header comment for what it checks)
- `mt48lc16m16a2.v` — Micron's behavioral SDRAM model (`4Meg x 16 x 4 Banks`,
  same part family as the DE10-Nano's stock 32MB chip; also supports 64MB/
  8MB density via a compile-time define — see below)
- `timescale.v`, `test-defines.v` — small stub includes the Micron model
  expects; both are effectively empty for our purposes

`rtl/sdram.sv` itself needed one small addition to support this: a
`USE_ALTDDIO` parameter (default `1`, unchanged hardware behavior). The
testbench instantiates it with `USE_ALTDDIO(0)`, which swaps the
`altddio_out` Cyclone V primitive for a plain `assign SDRAM_CLK = ~clk`
— avoiding any need for Altera's simulation libraries (which normally
requires a Quartus install + `quartus_sh --simlib_comp` to obtain).

## Running in ModelSim / Questa

From the `sim/` directory:

```sh
vlib work
vmap work work

# Compile the behavioral model + DUT + testbench.
# +incdir+. lets mt48lc16m16a2.v find timescale.v / test-defines.v.
# -suppress 2244 silences vlog's "implicit static" warning for two
# local regs in sdram.sv that have inline initializers (last_a,
# init_old) — this matches upstream's own file exactly, and adding an
# explicit `static` keyword to satisfy this ModelSim-only warning
# actually breaks Quartus synthesis (`static` + an array initializer
# isn't fully supported there), so don't "fix" it in the RTL.
vlog -suppress 2244 +incdir+. mt48lc16m16a2.v ../rtl/sdram.sv sdram_tb.sv

# Run headless, let it finish, print all $display output.
vsim -c work.sdram_tb -do "run -all; quit"
```

Look for the final line:

```
=== SUMMARY: <N> checks, <N> errors ===
=== PASS ===        (or === FAIL === with per-check FAIL lines above it)
```

Any `FAIL [...]` line names which check failed, the address, and
expected vs. actual byte — that pinpoints the exact failing address
without needing to eyeball a debug overlay on real hardware.

### Testing against the 64MB/128MB-class chip geometry

The debugging session raised a real question: MiSTer boards can have a
32MB (stock, 9-bit column), 64MB, or 128MB (`AS4C32M16SB`-based
upgrade boards use two 64MB chips, each 10-bit column) SDRAM installed,
and `rtl/sdram.sv` doesn't currently know or care which. Run the exact
same testbench against a 64MB-shaped model to see whether that
distinction matters for this design:

```sh
vlog -suppress 2244 +incdir+. +define+MT48LC32M16 mt48lc16m16a2.v ../rtl/sdram.sv sdram_tb.sv
vsim -c work.sdram_tb -do "run -all; quit"
```

If both runs come back `=== PASS ===`, the column-width mismatch
(covered in the session notes) is confirmed harmless for this
project's address footprint, as reasoned through at the time — the
row-boundary test cases in `sdram_tb.sv` specifically straddle both
the 9-bit-column and 10-bit-column row boundaries to exercise this
directly rather than just take that reasoning on faith.

## If it fails

- A `FAIL [isolated-byte] ...` failure means the bug reproduces on the
  simplest possible case — look hard at `sdram.sv`'s FSM/DQM logic
  directly; a waveform (`vsim` without `-c`, add signals, `run -all`)
  will show exactly which SDRAM command sequence went wrong.
- A `FAIL [seq-fwd-rd0] ...` or `[seq-rev-rd2] ...` failure that
  *doesn't* also fail `isolated-byte` means the bug is specific to
  back-to-back requests (arbitration, the "cache-hit" word
  optimization, or refresh interrupting a transaction) rather than a
  single isolated access — look at the access-manager `always` block's
  `old_rd`/`old_wr` edge-detection and the `last_a[]` cache-hit logic.
- A `FAIL [bank-boundary]` / `[row-boundary-*]` failure isolates the
  address-decomposition math (`{bank,a} <= addr`, `a[13:1]` row,
  `a[22:14]` column) as the culprit.
- The Micron model itself may also print its own `$display` messages
  (e.g. timing-violation warnings) if `sdram.sv` violates tRCD/tRC/tRP
  — these are worth reading even if all of this testbench's own checks
  pass, since a marginal-but-technically-working timing violation in
  simulation could still fail on real silicon with less margin.
- The model also has an internal `Debug` flag (hardcoded off in
  `mt48lc16m16a2.v`) that prints every ACT/READ/WRITE/AREF command it
  receives with bank/row/col/data — flip it to `1'b1` temporarily for
  one investigation pass if a failure needs cycle-by-cycle ground
  truth from the chip's own perspective, then flip it back off (it's
  very verbose across a full run).

## Board integration testbench (sor_board_tb.sv)

`sdram_tb.sv` proved `rtl/sdram.sv` itself correct against a realistic
concurrent-access pattern (4 tests, 20494 checks, 0 errors) — including
finding and fixing two real bugs along the way. If real hardware still
doesn't change after a confirmed-clean rebuild with a passing
`sdram_tb.sv`, the remaining bug is likely one level up: in
`sor_board.sv`'s actual ioctl-to-SDRAM write path, the `wr_pending`
latch, or the checksum scan trigger logic — none of which the
controller-only testbench exercises.

`sor_board_tb.sv` instantiates `sor_board.sv` completely unmodified,
with its real `sor_master`/`sor_slave` Z80 cores running (genuine
CPU-driven SDRAM contention once reset falls, more faithful than a
synthetic hammering loop), and drives realistic HPS `ioctl_*` signals
to load the **real Master ROM** — including toggling `ioctl_download`
low/high between each of 4 parts, matching the real MRA's 4-file
Master ROM entry and the exact pattern an earlier bug in this session
was found in.

**Before running, extract these 4 files from `offroad.zip` (MAME set
`offroad`) into `sim/`** — same filenames as the MRA uses:

```
03-22121-04.u58t
03-22122-03.u59t
03-22120-01.u57t
03-22119-02.u56t
```

The testbench reads them with `$fread` as raw binary, so no conversion
needed — just the bare files sitting next to `sor_board_tb.sv`. If a
file is missing it fails fast with an `ERROR: could not open ...`
message instead of silently running with garbage data.

Once loaded, it runs the real Master CPU boot sequence and traces
every I/O write it makes (`IOWR` lines: port, data, PC, and whether it
hit the bank register or `/MCONT`), plus a periodic `PC_SAMPLE` every
200k cycles showing the live PC, bank register, and whether
`slave_reset_n`/`cpu_cmd_wr`/`vram_we_cpu` have ever fired — this
exists because real hardware showed `slave_reset_n` never releasing
(Slave Z80 stuck in reset, no command ever sent, no VRAM ever
written) even after the SDRAM controller and ioctl-load path were
proven byte-perfect. If sim reproduces that same hang, the `IOWR`/
`PC_SAMPLE` trace should show exactly where Master boot code stalls
or loops instead of reaching its `/MCONT` write. It then waits for
`sor_board`'s own internal readback scanner to finish and checks the
result directly via hierarchical reference — the same registers
driving the on-hardware debug overlay, no video rendering needed.

Compile order matters — the tv80 Z80 core and all of `sor_board.sv`'s
submodules must be included:

```sh
vlog -suppress 2244 +incdir+. mt48lc16m16a2.v \
  ../rtl/tv80/rtl/core/tv80_alu.v \
  ../rtl/tv80/rtl/core/tv80_reg.v \
  ../rtl/tv80/rtl/core/tv80_mcode.v \
  ../rtl/tv80/rtl/core/tv80_core.v \
  ../rtl/tv80s_ce.v \
  ../rtl/sdram.sv \
  ../rtl/sor_dpram.sv \
  ../rtl/sor_vram_port.sv \
  ../rtl/sor_eeprom_93c46.sv \
  ../rtl/sor_master.sv \
  ../rtl/sor_slave.sv \
  ../rtl/sor_video.sv \
  ../rtl/sor_board.sv \
  sor_board_tb.sv

vsim -c work.sor_board_tb -do "run -all; quit"
```

This streams a full 256 KB (4 x 64 KB parts) through the real ioctl
path, so it takes noticeably longer than `sdram_tb.sv` — expect it to
run for a while in wall-clock time even though it's still far faster
than a hardware rebuild-and-flash cycle. Look for the same
`=== PASS ===` / `=== FAIL ===` summary at the end, plus the printed
`wr_chk`/`rd_chk` (and per-quarter) values and the byte-test result —
these are the exact same diagnostic registers the on-hardware debug
overlay reads, so a mismatch here should correspond directly to what
you'd see as a red block on the real screen.
