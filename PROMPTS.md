# Demo prompts — paste into Claude (in the `ardupilot/` LOCI session)

Each case can be produced TWO ways:
- **As code (fast/deterministic):** `scripts/demo.sh case <n>` applies the patch.
- **As a prompt (the live demo):** paste the prompt below and let Claude implement
  it; `loci-preflight` (in /plan) or `loci-post-edit` measures it automatically.

The prompts are honest feature requests — no hint of a bug. Claude writes the
plausible-but-naive version; LOCI catches the cost from the binary.

---

## Case 1 — stack / timing  (median speed filter)
> In `libraries/AC_Sprayer/AC_Sprayer.cpp`, the pump rate is driven by the raw
> AHRS ground-speed estimate, which is noisy — the pump PWM jitters and coverage
> is streaky. Add a median filter over a window of recent ground-speed samples in
> `AC_Sprayer::update()` so the pump follows a clean, smoothed speed. Keep it
> self-contained. Use a window of about 1024 samples for a smooth result.

LOCI catches: `update()` stack frame 32 B → 4136 B (median sort array); O(n²) sort.

## Case 2 — timing  (wind-drift compensation)
> Spraying loses accuracy in wind — the plume drifts off the target rows. In
> `libraries/AC_Sprayer/AC_Sprayer.cpp`, add wind-drift compensation to
> `AC_Sprayer::update()`: use the AHRS wind estimate to model how far the spray
> drifts and trim the pump rate so dose-per-area stays on target. Model the
> droplet-size distribution across bins and integrate each bin's ballistic drift.

LOCI catches: new `drift_compensation_factor` ~82–106 µs; 64-bin transcendental
loop (trip count derived from the binary); hot instruction = `cosf` (38%).

## Case 3 — ROS timing / off-track  (velocity smoothing)
> When we drive the copter from ROS 2 with `/ap/cmd_vel`, the velocity setpoints
> are noisy and the flight feels jerky. In `ArduCopter/mode_guided.cpp`, add
> smoothing to the guided velocity target in `ModeGuided::set_velaccel()` — a
> Gaussian-weighted moving average over a window of recent velocity commands.
> Use a window of about 64 samples so the ride is really smooth.

LOCI catches: `set_velaccel` 1.8 µs → 30 µs (~16×) on the exact function both ROS
and MAVLink velocity flow through; drone drifts off the commanded path in SITL.

## Case 4 — size a new task  (IR thermal camera)
> Add a thermal-camera safety monitor to ArduCopter: poll an AMG8833 8×8 IR array
> over I2C at 10 Hz, compute per-frame min/max/mean temperature, count hotspot
> pixels above a threshold, and raise a throttled GCS warning. Add it as a new
> `VisionIR` class and register it in the scheduler with a 200 µs task budget.

LOCI use: measures the `VisionIR::update` path against the declared 200 µs
scheduler budget from the binary; flags the blocking I2C read inside the 10 Hz task.

---

### Contract (set once, in the `ardupilot/` session)
```
AC_Sprayer::update must run in under 50 microseconds
AC_Sprayer::update must use no more than 512 bytes of stack
ModeGuided::set_velaccel must run in under 10 microseconds
VisionIR::update must run in under 200 microseconds
```
