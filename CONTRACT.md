# LOCI Contract — budgets the demo is judged against

The Contract Envelope (`ardupilot/.loci/contract.yaml`) is where you state the
firmware budgets as **requirements**. Once set, every LOCI measurement is judged
against them: a run closes on **FAIL — measured X vs your stated Y**, quoting your
own sentence back, instead of a bare number. That is the beat of the talk — the
tool enforces a budget you wrote, on a cost that is invisible in the diff.

## The bounds

| Function | Signal | Bound | Baseline | The bug | Verdict |
|---|---|---|---|---|---|
| `AC_Sprayer::update` | timing (`worst_path_time`) | **≤ 50 µs** | 3.5 µs | Case 2 → ~106 µs | **FAIL ~2×** |
| `AC_Sprayer::update` | stack (`stack_depth`) | **≤ 512 B** | 136 B | Case 1 → 4240 B | **FAIL ~8×** |
| `ModeGuided::set_velaccel` | timing | **≤ 10 µs** | 1.8 µs | Case 3 → 30 µs | **FAIL ~3×** |

All `severity: fail` (hard gate). The first two ship in `.loci/contract.yaml`;
the third is only needed for the Case 3 (ROS velocity / off-track) demo.

## How to set them in LOCI

In a Claude Code session **inside `ardupilot/`**, type `/contract` and state each
requirement in plain language — the skill drafts the YAML entry and **you** approve
it (the contract is user-applied only; a hook blocks anyone else from writing it):

```
AC_Sprayer::update must run in under 50 microseconds
AC_Sprayer::update must use no more than 512 bytes of stack
ModeGuided::set_velaccel must run in under 10 microseconds
```

- View current bounds: type `/contract` (or "show the contract").
- Phrasing "must …" → `severity: fail` (❌). Phrasing "should stay under …" →
  `severity: warn` (🔶).

## Why these numbers (stage justification)

- **50 µs / `update`** — the 400 Hz main loop is a 2500 µs slice *shared* across
  the EKF, controllers and every task; 50 µs for a low-rate sprayer task is
  generous (baseline 3.5 µs = 14× margin). Catches Case 2 because live
  `loci-post-edit` measures **response time** (callees included), so `update`
  balloons past 50 µs even though its self-time barely moves.
- **512 B / `update`** — on a shared scheduler stack (a few KB per task on the
  H7), 512 B for one function is normal. `float sorted[1024]` blows it to 4240 B.
- **10 µs / `set_velaccel`** — runs on every velocity command at ROS `cmd_vel`
  rates, between ROS and the motors; baseline 1.8 µs (5× margin).

## Two things to get right

1. **Bound the stable function, not the new one.** You cannot pre-bound
   `drift_compensation_factor` — it does not exist until the feature is written.
   Bounding `AC_Sprayer::update` (always present) is what lets the contract catch
   a cost added *below* it in a callee that has not been written yet.
2. **Do not over-tighten.** Leaving the baseline comfortably green means the red
   row is obviously the *change*, not a stingy limit.
