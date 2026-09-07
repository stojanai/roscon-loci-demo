# LOCI @ ROSCon 2026 — Spraying Drone Demo

Live demo for ROSCon Global 2026 (Toronto, Sep 22–24): an innocent-looking
"improvement" to ArduPilot's crop-sprayer controller that compiles clean, looks
fine in review, misbehaves in the field — and how LOCI catches it from the
compiled firmware before it ever flies.

## The story

`AC_Sprayer::update()` runs in ArduCopter's flight scheduler on the flight
controller (STM32H7 on a CubeOrange). It reads ground speed from AHRS and sets
the pump PWM proportional to speed, so the drone deposits a constant amount of
fertilizer per meter of field.

The "bug" (see `patches/speed-smoothing-bug.patch`): someone adds a
1024-sample moving-average filter to smooth the ground-speed estimate.

- **Behavior**: spray rate now lags speed changes → over-spray on
  deceleration, under-spray on acceleration. Visible in SITL.
- **Firmware cost** (what review and SITL do NOT show):
  - a `float speed_history[1024]` local → **+4 KB worst-case stack** in a
    shared-scheduler-stack context → hard-fault risk mid-flight
  - a 1024-iteration loop per call → timing + energy regression
- **LOCI**: contract bound on `AC_Sprayer::update()` timing/stack fails on the
  post-edit ELF. The sim shows the symptom; LOCI shows the cause.

## Setup (fresh clone)

`ardupilot/`, `toolchain/`, and `venv/` are git-ignored (huge / vendored). To
rebuild from a fresh clone:

```
git clone --depth 1 --branch Copter-4.6.2 https://github.com/ArduPilot/ardupilot.git
cd ardupilot && git submodule update --init --recursive --depth 1 && cd ..
python3 -m venv venv && ./venv/bin/pip install empy==3.3.4 pexpect future pymavlink MAVProxy dronecan gnureadline wxPython pillow matplotlib opencv-python
# ARM toolchain (no sudo): extract ARM GNU 15.3 arm-none-eabi into ./toolchain/
```

The pre-built ELFs in `artifacts/` let you run LOCI immediately without building.

## Layout

- `ardupilot/` — ArduPilot checkout (Copter-4.6.2), builds both artifacts:
  - `./waf configure --board CubeOrange && ./waf copter` → real STM32 ELF at
    `build/CubeOrange/bin/arducopter` (what LOCI analyzes)
  - SITL via `sim_vehicle.py` (what the audience watches)
- `patches/speed-smoothing-bug.patch` — the injected bug (apply/revert live on stage)
- `scripts/build_fw.sh` — build the CubeOrange ELF (baseline or bugged)
- `scripts/run_sitl.sh` — SITL with sprayer enabled + mission
- `scripts/sitl_sprayer.scr` — MAVProxy script: enable sprayer, arm, fly a spray pass
- `venv/` — Python env with waf/MAVProxy deps (`source venv/bin/activate`)

## Demo flow (stage script)

1. **Baseline**: show `AC_Sprayer::update()` on a slide (~40 lines of logic).
   Show LOCI contract: stack + timing bounds for `update()`. Build → LOCI passes.
2. **The edit**: apply the patch. "Speed smoothing — reviewer approved it."
3. **Rebuild + LOCI post-edit compare**: timing % diff, energy delta, stack
   jump, contract FAIL on the CubeOrange ELF.
4. **SITL**: same code flying — spray rate lags, tank drains wrong. Symptom
   confirmed, but only LOCI told you *why* and *before flight*.
5. **Punchline for ROSCon**: your ROS 2 stack (`AP_DDS`) is fine — the
   regression lives below the topic layer, in firmware, where ROS tooling
   can't see it.

## Verified facts (build of 2026-09-02, GCC 15.3, CubeOrange/STM32H7)

Pre-built ELFs are in `artifacts/`:

- `arducopter-baseline.elf` — `AC_Sprayer::update()` = 368 B code, **24 B stack frame**
- `arducopter-bugged.elf` — `update()` = 516 B code, prologue does
  `sub sp, sp, #4096` → **4 KB + 28 B stack frame** on the shared scheduler
  stack, plus `speed_history` = 4 KB `.bss` (RAM stolen from the heap), plus
  an O(1024²) insertion sort per call for timing/energy.

## Case 2 — timing bug (drift compensation)

`patches/drift-compensation-bug.patch` adds a plausible "wind-drift
compensation" model to `AC_Sprayer::update()`: a new method
`drift_compensation_factor()` that loops over 64 droplet-size bins doing
transcendental math (`sinf/cosf/expf/logf/sqrtf/atan2f`) per bin. Tiny stack,
compiles clean, flies fine in SITL — but it is a timing bomb.

LOCI exec-trace on the CubeOrange ELF (armv7e-m), throughput time
(self-time, callees excluded), measured 2026-09-02:

| function | throughput | note |
|---|---|---|
| `AC_Sprayer::update` (baseline) | **3.49 µs** | no drift call |
| `AC_Sprayer::update` (case 2)   | **3.52 µs** | +0.9% — looks harmless |
| `AC_Sprayer::drift_compensation_factor` (case 2) | **82.0 µs** | NEW callee |

The point for the talk: `update()`'s own time barely moves, so a glance at it
misses the problem — but it now calls an 82 µs function, ~23× the entire
original update cost. LOCI derived the **64-iteration loop trip count directly
from the compiled assembly** (`init r4=0; step +1; limit 64`) — no source, no
run, no board. And this 82 µs is a floor: it excludes the `sinf/cosf/expf`
callee bodies, so end-to-end is worse.

### Hot path — `drift_compensation_factor()` (loop L1 expanded ×64)

LOCI per-block timing along the execution path; each `bl` resolved to its libm
symbol. The hottest single block is the `cosf` call — 38% of the whole function.

| rank | block | call | self ns | ×iters | total | % fn |
|---|---|---|---|---|---|---|
| 1 | 0x…977e | **cosf** | 486.6 | ×64 | 31.1 µs | **38%** |
| 2 | 0x…9746 | **logf** | 264.2 | ×64 | 16.9 µs | 21% |
| 3 | 0x…97ba | **sinf** | 210.5 | ×64 | 13.5 µs | 16% |
| 4 | 0x…97da | constrain | 143.7 | ×64 | 9.2 µs | 11% |
| 5 | 0x…976a | **expf** | 80.9 | ×64 | 5.2 µs | 6% |
| 6 | 0x…97ae | **expf** | 72.5 | ×64 | 4.6 µs | 6% |
| — | entry+exit (6 blk) | — | — | ×1 | 1.4 µs | 2% |
| | **TOTAL** | | | | **82.0 µs** | 100% |

Stage line: LOCI points at one instruction address and says "this is 38% of
your timing budget" — from the binary alone. (Self-time, so libm bodies are on
top of this.)

Build it: `scripts/build_fw.sh case2` → `artifacts/arducopter-case2-drift.elf`.

## Case 1 — LOCI results (stack + honest lower-bound timing)

The median-filter bug (`patches/speed-smoothing-bug.patch`) has two defects,
both caught from the ELF:

**Stack (the headline, sound — no recursion/indirect/unknown callees):**

| | frame size | worst-case depth |
|---|---|---|
| baseline `update()` | 32 B | 136 B |
| bugged `update()` | **4136 B** | **4240 B** |

+4104 B frame (129×) from `float sorted[1024]`, on ArduCopter's shared
scheduler stack → hard-fault risk. LOCI traced the worst path
`update() → move_servo → function_assigned → update_aux_servo_function`.

**Timing (honest lower bound):** exec-trace = **≥ 4.42 µs**. LOCI found the
nested O(n²) sort (loops L1+L2, depth 2) but reports `trips ?` for both and
says why — L1 *"limit 0 — not a countable range"*, L2 *"r3 not constant on
entry"* — so it marks the total a floor rather than fabricating a count.

### Case 1 vs Case 2 — the contrast that carries the talk

| | case 2 (drift) | case 1 (median) |
|---|---|---|
| loop trip count | **derived exactly (64)** | undeterminable → ≥ bound |
| LOCI's move | pinpoints the hot instruction | flags *why* it can't, won't guess |
| primary signal | timing (82 µs) | stack (32→4136 B) |

Case 2 = LOCI's precision when code is analyzable; case 1 = its honesty when it
isn't. Both from the compiled ELF, no source, no execution.

## Running LOCI (the demo measurement)

Open a Claude Code session **inside `ardupilot/`** (LOCI activates per
project directory; the demo root is fine too but the ardupilot checkout is
what gets analyzed) and:

1. `/contract` — set stack/timing bounds on `AC_Sprayer::update`
   (see [`CONTRACT.md`](CONTRACT.md) for the exact bounds, the `/contract`
   wording, and the rationale)
2. `scripts/build_fw.sh baseline` → ask for stack-depth / exec-trace on
   `build/CubeOrange/bin/arducopter` → bounds pass
3. `scripts/build_fw.sh bugged` → re-run analysis → contract FAILS
   (timing + 4 KB stack jump)

**Contract bounds** (`ardupilot/.loci/contract.yaml`, full detail in
[`CONTRACT.md`](CONTRACT.md)):

| Function | Signal | Bound | Bug that breaks it |
|---|---|---|---|
| `AC_Sprayer::update` | timing | ≤ 50 µs | Case 2 → ~106 µs |
| `AC_Sprayer::update` | stack | ≤ 512 B | Case 1 → 4240 B |
| `ModeGuided::set_velaccel` | timing | ≤ 10 µs | Case 3 → 30 µs |

## SITL visual (audience view)

`scripts/run_sitl.sh` — MAVProxy console + map, sprayer + tank sim enabled
(`sprayer.parm`). At the MAVProxy prompt:

    mode guided
    arm throttle
    takeoff 15
    long DO_SPRAYER 1          # IMPORTANT: activates spraying (enable != on)
    graph SERVO_OUTPUT_RAW.servo10_raw   # live pump plot

then left-click a distant point on the map and `guided 15` (or `velocity 8 0 0`).
Pump parks at PWM 1100 when slow/hovering, scales with speed above
SPRAY_SPEED_MIN (1 m/s, ~1 s engage delay). Baseline: pump tracks speed within
a second. Bugged build: pump stays near minimum for minutes regardless of speed
(median of 1024 samples @ 3 Hz is still dominated by on-ground zeros).
`long DO_SPRAYER 0` turns spraying off.
# roscon-loci-demo
