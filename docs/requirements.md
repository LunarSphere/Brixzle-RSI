# Requirements

The base requirements every brick family must meet. The design agent loads them into its system prompt. A person can add more at intake, and the agent asks them to resolve any conflict before the loop starts.

Each requirement is checked one of three ways:

- **Rule:** an analytic check that runs before any simulation.
- **Sim:** a simulated trial.
- **Design:** a fixed design choice.

| ID | Requirement | Checked by |
|---|---|---|
| R1 | **Generality.** Bricks must generalize to any structure (bridge, tower, arch). | Sim: tower, wall, bridge and ring builds |
| R2 | **Voxelizable.** Bricks must sit on a voxel lattice so that structures can later be planned algorithmically. | Design: lattice placements; cell size is tuned unless fixed at intake |
| R3 | **Rigidity.** Assemblies must be rigid and physically stable. | Sim: sag and lateral push tests |
| R4 | **Overhang.** An overhang must accept n bricks without falling over. One way to achieve this is angled pegs in a stepped design. | Rules: hook depth, seated play · Sim: overhang builds |
| R5 | **Biased error.** The drone will make placement errors. Instead of correcting all of them, we bias the error onto one side (e.g. always onto the right slope) and use that slope's angle as the guide. | Sim: aim bias sized from the error model · Rule: capture width |
| R6 | **Steep guides.** A guide ramp must be steeper than its friction coefficient requires (tan θ > μ, with margin). | Rule: friction-cone check on every guide face |
| R7 | **Alternating angles.** Alternating the lean between courses may improve stability. | Design: alternating parts by course |
| R8 | **Stiffness.** I-beam-like sections may be used to stop the brick from flexing. The first course sits on a stable base. | Design: rib frame, anchored base |
| R9 | **Mass.** A brick must weigh between 1 g and 25 g in PLA; lighter is preferred. | Rule: mass window |
| R10 | **Passive pickup.** A non-actuated gripper must be able to pick up the brick; the drone's payload is about 40 g. | Rules: gripper fit, hangs level, straddles the centre of mass |
| R11 | **Coarse drop.** Bricks are dropped coarsely and slide into place on the ramps. | Analytic predictor and sim: placement box score |
| R12 | **Coarse pickup.** The drone picks the brick up imprecisely, with the same kind of positioning error it has when placing. The passive gripper must still engage, and the brick must self-centre on it. | Analytic predictor and sim: pickup box score |

## Design target

Both R11 and R12 are scored the same way. Find the largest box of drone error in X, Z and θ, around the aim point, where every trial succeeds with no jams. Then scale it against three targets, sized to the drone's mean, 95th-percentile and worst measured error.

The measured error so far covers X–Y position only (Exp 3 in aerial-assembly). The Z and θ sides of the box are estimates until they are measured.
