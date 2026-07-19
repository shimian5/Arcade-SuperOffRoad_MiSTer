# Super Off-Road — MiSTer FPGA Core

A from-scratch MiSTer FPGA re-implementation of Leland/Tradewest's **Super
Off-Road** (1989) — twin Z80 master/slave boards, custom tile/sprite video,
and the real Leland 80186-based sound board, all reproduced in RTL rather
than emulated at the instruction level.

## Status

The core boots, plays, and drives sound through a real 80186 CPU
implementation — this is not a sample-playback stand-in, it's the Leland
sound board's architecture (80186 core + 8253 PIT + DAC/mixer) recreated
in RTL and driven by the game's real sound ROM.

#### Known Issues - CRT/Analog video no sync. Working to resolve but for now HDMI only

| Subsystem | Status |
|---|---|
| Master/Slave Z80 (game logic, bankswitching, EEPROM) | Working |
| Tile/sprite video | Working |
| Leland 80186 sound board (real CPU, real ROM, real DAC) | Working |
| Controls (spinner, digital d-pad, analog stick, digital gas) | Working |
| DIP switches (lives, difficulty, service, free play) | Working |

A note on accuracy: this core was developed by studying MAME's Leland
drivers and the game ROMs — it has not been verified against an original
PCB, and MAME itself is an emulator rather than a hardware model. Expect
behavioral differences from real Leland hardware in edge cases.

Known rough edge: analog-stick steering sensitivity runs a little hot for
some players — tune-able in `rtl/steering_input.sv` (see Controls below).
If you hit something else, please open an issue.

## Controls

The real cabinet has a free-spinning steering wheel (not a centered
potentiometer) and a gas pedal per player. This core supports three ways
to drive, combined automatically — mix and match freely, no menu toggle
needed to pick one:

| Input | Behavior |
|---|---|
| **Spinner** | Direct 1:1 mapping, like the original cabinet |
| **Analog stick** | Deflection = turn rate (not an absolute wheel position) |
| **Digital d-pad** | Left/Right steer; OSD option **"D-Pad Steering"** picks the feel: **Velocity** (default — ramps a turn rate, coasts back to straight on release, closest to the real free-spinning wheel) or **Position** (drives a virtual spring-centered stick instead) |

Buttons (`J1,Nitro,Coin,Gas`):

| Button | Function |
|---|---|
| Button 1 | Nitro |
| Button 2 | Coin |
| Button 3 | Gas — MiSTer has no analog-trigger support, so throttle is a plain digital button (0/full) rather than the original analog pedal |

## Building

Requires **Quartus Prime 17.0.x** (Lite or Standard):

```sh
quartus_sh --flow compile SuperOffRoad
```

Produces `output_files/SuperOffRoad.rbf`.

## Installing on MiSTer

Copy to your MiSTer:

- `output_files/SuperOffRoad.rbf` → `/media/fat/_Arcade/cores/`
- `mra/SuperOffRoad.mra` → `/media/fat/_Arcade/`

You'll need the `offroad` ROM set (MAME 0.257) — this repo does not
include or distribute any ROM/PROM data, per usual MiSTer arcade-core
convention.

## Simulation

`sim/` has standalone ModelSim testbenches for the SDRAM controller and
the full board (real Master/Slave Z80 cores booting against the real
Master ROM) — see `sim/README.md` for exact commands. Useful for
iterating on RTL changes far faster than a Quartus rebuild + hardware
flash cycle.

## Credits / third-party sources

- **MAME's `leland.cpp`/`leland_m.cpp`/`leland_v.cpp`/`leland_a.cpp`** —
  studied throughout as the primary documentation of the Leland board's
  behavior (not redistributed in this repo). MAME is an emulator, not a
  hardware model, so it served as a study reference and comparison point
  rather than ground truth.
- [`tv80`](rtl/tv80) — open-source synthesizable Z80 core (MIT license).
- [`jamieiles/80x86`](rtl/s80x86/README_VENDORING.md) — vendored 8086/80186
  core powering the real sound-board CPU (GPLv3).
- [`KF8253`](rtl/KF8253/README_UPSTREAM.md) — 8253 PIT core used by the
  sound board.
- Built on the standard [MiSTer framework](https://github.com/MiSTer-devel)
  (`sys/`).

## License

GNU GPL v2 (see `LICENSE`), matching MiSTer project convention. One vendored
component (`rtl/s80x86`) is GPLv3 — mixing is consistent with how the wider
MiSTer framework already combines GPLv2/GPLv3/MIT code.
