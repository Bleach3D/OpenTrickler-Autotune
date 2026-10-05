# OpenTrickler Simulator

Runs the firmware's **real charge controller** on a Windows PC, against a
simulated trickler and a simulated A&D FX-120i scale, so you can test tuning
changes without burning powder or waiting for throws.

## What this is (and is not)

**It is** a native build of the actual control sources — `charge_mode.cpp`,
`ai_tuning.c`, `profile.c`, `common.c` — compiled unmodified against host
stubs. The control logic you are testing is the same code that runs on the
Pico, so it can be stepped through in Visual Studio or gdb with real
breakpoints and watch windows.

**It is not** a `.uf2` emulator. It does not execute your built firmware
binary, and it does not emulate the RP2350, FreeRTOS scheduling, WiFi, the
display, or the UART drivers. Bugs that live in hardware timing, driver
code, or task interaction will not show up here. Full-system emulation of
this board is a much larger project with a real risk of never working
properly; this approach was chosen because it tests the part you actually
care about — the charge control algorithm — and does so reliably.

## Building on Windows

You need CMake and a C/C++ compiler. Either works:

**MinGW / MSYS2 (verified working)**

```
cmake -S sim -B sim/build -G "MinGW Makefiles"
cmake --build sim/build
```

This is the toolchain the simulator was developed and tested against.
Reach for it first if you just want it to work.

**Visual Studio (supported, less tested)**

```
cmake -S sim -B sim/build
cmake --build sim/build --config Debug
```

Then open `sim/build/ottrickler_sim.sln` and set `ottrickler_sim` as the
startup project. Breakpoints in `charge_mode.cpp` work normally.

MSVC needs two accommodations, both handled automatically by
`sim/CMakeLists.txt`:

- `shims/msvc_compat.h` is force-included to neutralise
  `__attribute__((packed))`, which MSVC does not understand. Without it you
  get a cascade of errors in `neopixel_led.h`, `eeprom.h` and `ai_tuning.h`
  that all trace back to this one construct.
- C++20 is selected, because `charge_mode.cpp` uses designated
  initialisers.

One consequence worth knowing: dropping `packed` means MSVC may pad three
structs differently from the firmware. That is harmless for the simulator
(its EEPROM and flash are in-memory, written and read with the same
layout), but it means binary EEPROM images are **not** portable between an
MSVC simulator build and real hardware. The MinGW build does not have this
caveat.

The build defaults to `Debug` (`-O0 -g`) because the point of this tool is
to be stepped through.

## Running

```
ottrickler_sim --target 45.0 --charges 20 --kernel 0.04 --coarse-stop 0.80
```

Options:

| Option | Meaning |
|---|---|
| `--target <gn>` | Target charge weight (default 45.0) |
| `--charges <n>` | How many throws to simulate (default 20) |
| `--seed <n>` | RNG seed — same seed reproduces a run exactly |
| `--kernel <gn>` | Powder granule weight (default 0.04, ~N565) |
| `--coarse-stop <gn>` | The coarse stop threshold setting |
| `--coarse-auto` | Let AI tuning choose the handoff instead |
| `--bias <0..1>` | Accuracy/speed bias, 0 = fast, 1 = accurate |
| `--coarse-tail <gn>` | Simulated coarse tail (default 0.85) |
| `--fine-tail <gn>` | Simulated fine tail (default 0.035) |
| `--noise <frac>` | Flow noise fraction (default 0.10) |
| `-q` / `-v` / `-vv` | Summary / per-charge / full motor trace |

`-vv` prints every motor speed change with a timestamp and the pan weight,
which is the fastest way to see exactly when coarse hands off and what the
fine tube is left holding.

Output reports mean/sd/min/median/max for error, total time, fine-phase
time, and how much was left at the coarse handoff — plus the kernel-limited
best-case sd, which is the hard physical floor no controller can beat.

## What is modelled

**A&D FX-120i scale**
- 0.001 g (~0.0154 gn) readability — the resolution limit the controller
  can never see past
- ~1 s settling, modelled as a first-order lag (this is what causes the
  controller to always be reacting to slightly stale weight)
- Reading noise, slow zero drift
- ~100 ms streaming interval, and the firmware genuinely cannot get
  readings faster than this
- ST/US stable/unstable reporting, in the real wire format

**Trickler**
- Flow vs rps as a power law, separate spin-up and spin-down constants
- Tail modelled as a roughly fixed *mass* of powder in flight, which is why
  `tau = tail / flow` — a faster motor empties the same tail quicker
- Powder released in whole kernels, so a coarse extruded powder physically
  cannot be metered finer than one granule

## Calibrating it to your machine

**The default trickler numbers are plausible, not measured.** Absolute
throw times will not match your hardware until you calibrate them. To do
that, set `--coarse-tail` and `--fine-tail` from your own characterization
results, and adjust `coarse_flow_k` / `fine_flow_k` in
`src/sim_plant.c` until a simulated throw takes about as long as a real one.

Until you do that: **relative** comparisons are trustworthy (does raising
the bias slow it down? does the coarse handoff respect my setting?),
**absolute** seconds and grains are not.

## AI characterization harness (`ottrickler_ai_char`)

The adaptive controller only runs from a characterized AI model, and the
simulator starts with empty flash. `ottrickler_ai_char` closes that gap: it
runs a full AI powder characterization through the real charge-mode state
machine (the AI sample path in `charge_mode.cpp` plus `ai_tuning.c`), the
same way the web GUI's "Start characterization" does, prints every recorded
drop and the fitted model, then throws charges.

```
ottrickler_ai_char --target 43.5 --charges 5 --save-model model.bin
ottrickler_ai_char --load-model model.bin --adaptive --charges 10
```

| Option | Meaning |
|---|---|
| `--target <gn>` | Characterization / charge target (default 43.5) |
| `--charges <n>` | Charges to throw afterwards (default 5) |
| `--adaptive` | Throw them with the adaptive controller (the AI model) instead of PID |
| `--save-model <file>` / `--load-model <file>` | Keep a fitted model so charges can be re-run without the ~3 minute characterization |
| `--no-tare` | No `force_zero` on the scale (like the generic scale driver) |
| `--stall-s <s>` | Watchdog: fail if one charge-mode phase lasts longer than this (default 60) |
| `--seed`, `--kernel`, `--coarse-tail`, `--fine-tail`, `--noise`, `--coarse-k`, `--fine-k` | Plant settings, as above |
| `-q` / `-v` / `-vv` | Summary / per-drop and per-charge / full motor trace |

Exit status is 0 when characterization completes and the model is saved, 2
on a stall (for example charge mode dropping to its normal "Remove Cup" wait
because a characterization drop was rejected), and 3 if AI tuning reports an
error. Like the other targets it runs in real time, so a full
characterization takes about 3½ minutes.

## Learn-from-every-throw prototype (`ottrickler_learn`)

A candidate replacement for AI tuning, prototyped here before any firmware
change. Charges run on the normal PID loop, untouched. After each one,
`src/throw_learner.c` looks at what the charge loop saw from the scale and
adjusts the two PID stop thresholds for the next charge:

- **Fine stop** follows the observed fine tail (final weight minus the
  reading at the final stop), aiming to cover nearly every tail: the running
  mean plus two typical deviations, a quarter of the tolerance short of
  target.
- **Coarse stop** keeps what is left for the fine tube about 1.5 s after the
  coarse stop at the reserve (`--reserve`, default 0.6 gn) plus two typical
  deviations of how much coarse leaves.
- An overthrow, or coarse leaving less than half the reserve, raises the
  threshold at once. Anything else moves it gradually.

`throw_learner.c` is plain C with no simulator dependencies, so it can move
into the firmware unchanged.

```
ottrickler_learn                                   # learn from the firmware defaults
ottrickler_learn --fixed                           # same, thresholds held fixed
ottrickler_learn --coarse-kp 16 --fine-kp 20 --fine-max 6 --coarse-stop 1.0
                                                   # fast profile that overthrows when fixed
ottrickler_learn --change-at 12                    # coarse flow x1.3, tails x1.5 from charge 12
```

Each line shows the thresholds the charge used, the true error against
target, time split into coarse and fine, how much coarse left and the fine
tail, plus any immediate back-off.
