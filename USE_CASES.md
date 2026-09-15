# LOCI use cases to show at ROSCon 2026

Every number below is measured this project (CubeOrange / STM32H7 / armv7e-m,
GCC 15.3) unless noted. Lead with the outcome (see SELLING.md), show the number.

## 1. Catch a stack overflow before it flies  (headline)
- **Skill:** stack-depth · **Case 1** (median-filter "improvement")
- **Number:** `AC_Sprayer::update` stack frame **32 B → 4136 B** (worst-case depth
  136 B → 4240 B) — an 8× overrun on a shared scheduler stack.
- **Story:** compiles clean, flies in SITL, would hard-fault over a field. LOCI
  reads it from the binary, no execution.
- **Sells to:** Safety/FuSa (evidence), Developer (confidence). ModalAI, QNX, eSOL.

## 2. Timing blowup read straight from the binary  (headline)
- **Skill:** exec-trace / loci-post-edit · **Case 2** (wind-drift compensation)
- **Number:** new `drift_compensation_factor` = **82 µs** self-time (106 µs
  response time), ~23× the whole original update — LOCI **derived the 64-trip
  loop count from the assembly** (no source, no run).
- **Sells to:** Developer, Eng Manager. eProsima, Renesas, Robotis.

## 3. Catch it BEFORE the code is written (preflight / AI physics)
- **Skill:** loci-preflight in `/plan` mode
- **Story:** you ask the coding agent for a feature; LOCI predicts the execution
  cost on the target and flags it while the design is still cheap to change.
- **Sells to:** Developer (confidence), CTO (trusted AI), AI Platform Lead
  (grounded agents). NVIDIA, eProsima CEO, ModalAI CEO.

## 4. Enforce a budget in CI (contract → hard FAIL)
- **Skill:** contract + any measurement
- **Number:** bounds `update ≤ 50 µs` / `≤ 512 B` → both cases close on **FAIL,
  quoting your own requirement** (not a bare number). See CONTRACT.md.
- **Sells to:** Eng Manager (fewer surprises), Safety (evidence), VP (velocity).

## 5. The regression lives below the ROS topic layer  (ROS hook)
- **Skill:** loci-post-edit · **Case 3** (velocity smoothing)
- **Number:** `ModeGuided::set_velaccel` **1.8 µs → 30 µs** (~16×) — the exact
  function both ROS `/ap/cmd_vel` and MAVLink velocity flow through. Symptom:
  drone drifts off the commanded path.
- **Story:** your ROS 2 graph looks healthy; the cost is in the firmware, where
  ROS tooling is blind. **The ROSCon punchline.**
- **Sells to:** all robotics buyers; the AP_DDS/ROS seam.

## 6. Analyze code you didn't write — from the binary
- **Skill:** exec-trace · **eProsima Micro-CDR on Cortex-M4** (their own OSS)
- **Number:** `ucdr_serialize_endian_array_uint32_t` — LOCI mapped the loop to
  **`array.c:179`**, per-element cost ≈ 224 ns (lower bound, callee named), on a
  runtime-varying trip count it refuses to fabricate.
- **Story:** no source assumptions, no instrumentation — point it at any compiled
  artifact. See prospect-uxrce/.
- **Sells to:** eProsima, Renesas, any platform/middleware team.

## 7. Honesty when the code isn't statically analyzable
- **Skill:** exec-trace · **Case 1 timing**
- **Number:** the O(n²) sort → **≥ 4.42 µs lower bound**; LOCI reports the floor
  and names the register that defeated the trip-count analysis — it does not
  invent a number.
- **Story:** trust. A tool that says "I can't bound this, here's why" is one you
  can put in a safety case. **Sells to:** Safety/FuSa, CTO.

## 8. Point at the hottest instruction (optimization guidance)
- **Skill:** exec-trace hot-path · **Case 2**
- **Number:** the `cosf` call at one block = **38 % of the function's budget**;
  each `bl` resolved to its libm symbol.
- **Sells to:** Developer/perf. Renesas, ModalAI, eProsima.

## 9. Size a NEW task against its declared budget (Case 4 — IR camera)
- **Skill:** exec-trace + control-flow · **Case 4** (VisionIR thermal camera)
- **What:** a new thermal-camera hotspot monitor (AMG8833 8×8 over I2C) added to
  the scheduler: `SCHED_TASK(VisionIR::update, 10 Hz, 200 µs)`. The developer
  **already declares a 200 µs WCET budget** in the code — that IS a contract.
- **Number:** LOCI reads the whole `update → read_frame → process_frame → report`
  path from the binary, derives the 64-px decode loop (trips 64 exact), confirms
  the **compute fits 200 µs** — and flags the real hazard: a **blocking I2C read
  inside a 10 Hz flight task**, a callee it cannot bound.
- **Story:** the canonical embedded workflow — "I'm adding a sensor task, does it
  fit its slot without starving the controllers?" LOCI answers from the binary.
- **Sells to:** Developer (confidence), Eng Manager (fewer surprises), Safety.
  Renesas, ModalAI, Bosch, any RTOS-task team.
- **Patch:** `patches/vision-ir-camera.patch` · ELF: `artifacts/arducopter-case4-visionir.elf`

## Also available (capability, not yet demoed here)
- **memory-report** — ROM/RAM footprint & region budgets from the ELF (for
  flash-constrained MCU teams).
- **control-flow** — annotated CFG with loop trip counts and recursion/indirect
  calls (the graph behind the numbers).
- **trends** — per-function timing/stack/memory history over a branch (the
  "are we regressing?" dashboard for managers).

## The through-line (say once, up front)
"LLMs know how to write code. LOCI knows how that code will behave on the target
— stack, timing, energy, memory — from the compiled binary, in CI, before HIL."
