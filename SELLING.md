# How to sell LOCI — persona → outcome

Same product. Different reason to care. **Adapt the message to the buyer; do not
change the product story.** (Source: LOCI_How_to_Sell_to_Different_People.)

**The rule: do NOT lead with "performance analysis." Lead with the outcome each
person wants.**

| Who | What they buy | Message (lead with this) |
|---|---|---|
| **Developer** | CONFIDENCE | Know how your code will behave while you build. LOCI works with the coding agent during `/plan` and code-writing to surface execution risks before hardware, sim, or HIL. |
| **AI / Agent Platform Lead** | GROUNDED AGENTS | Give coding agents execution reasoning. LLMs understand code; LOCI adds how that code behaves on the target CPU/GPU, so the agent adjusts its plan. |
| **Engineering Manager** | FEWER SURPRISES | Move execution problems left. Catch deadline misses, memory pressure, timing and concurrency while code is written — not during integration. |
| **VP Engineering / R&D** | VELOCITY | Reduce the cost of finding problems late. Prevent execution issues from propagating into sim, validation, HIL and field testing. |
| **CTO** | TRUSTED AI | Turn probabilistic coding into execution-aware engineering — an "AI physics" layer grounded in real CPU/GPU execution behavior. |
| **Safety / FuSa** | EVIDENCE | See execution risk before deployment: worst-case paths, deadline violations, interrupt behavior, memory pressure — evidence tied to the executable. |
| **CIO / Executive** | SAFE AI ADOPTION | Scale the productivity upside of coding agents safely — an execution-aware guardrail so AI-generated software doesn't multiply downstream risk. |

**One-liner that works for everyone:** *"LLMs know how to write code. LOCI knows
how that code will behave."*

## Mapping our ROSCon targets to a persona

| Target (person/role) | Persona to sell | Lead outcome |
|---|---|---|
| ModalAI firmware engineer | Developer | Confidence (H7 flight code) |
| ModalAI CEO (Chad Sweet) | CTO / Executive | Trusted AI / safe AI adoption |
| QNX safety PM | Safety / FuSa | Evidence (cert) |
| eSOL Autoware/eMCOS lead | Safety / FuSa | Evidence (partner) |
| eProsima CEO (Jaime Martin Losa) | CTO | Trusted AI (AI physics) |
| eProsima micro-ROS maintainer | Developer | Confidence (footprint/timing in CI) |
| Renesas RA ecosystem / FAE | Eng Manager / VP | Fewer surprises / velocity |
| Bosch rclc researcher | Developer / Safety | Evidence (static WCET) |
| Robotis firmware engineer | Developer | Confidence (servo loop) |
| PX4 / Dronecode (Ramon Roche) | VP / Executive | Velocity / safe AI adoption |
| NVIDIA platform mgr (Patzwaldt/Chen) | AI / Agent Platform Lead | Grounded agents |
| Clearpath embedded engineer | Developer / Eng Manager | Fewer surprises |

The LinkedIn messages in `roscon-2026-outreach.csv` are written to lead with each
row's outcome above, not with "timing/stack analysis."
