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
| R10 | **Grippable.** Each brick offers at least one feature a non-actuated gripper could hold, such as a bore, slot or lip. The combined mass of the brick and the gripper must not be more than 35 g, the drone's payload limit. Which gripper uses the feature is decided after the brick design is settled. | Rules: at least one grip feature, brick plus gripper within the payload |
| R11 | **Coarse drop.** Bricks are dropped coarsely and slide into place on the ramps. | Analytic predictor and sim: placement box score |
| R12 | **Coarse pickup.** The drone picks the brick up imprecisely, with the same kind of positioning error it has when placing. The passive gripper must still engage, and the brick must self-centre on it. See [R12 in detail](#r12-in-detail). | Analytic predictor and sim: pickup box score |
| R13 | **Printable.** A brick must print as one piece in PLA, with walls of at least 1.2 mm. | Rules: single piece, minimum wall |

## Deferred requirements

A deferred requirement stays in the list, but the loop does not score it, and the design agent does not design for it yet. `docs/requirements.yaml` marks it `status: deferred`.

| ID | Deferred until | Why |
|---|---|---|
| R12 | The gripper is designed for the chosen brick (issue #24) | Issue #46. The loop gets placement (R11) right first. On the A/B baseline, pickup scores 0 or close to 0 on every target, so it would hide whether placement is improving. The pickup trial and the pickup predictor stay in the code with their tests, but the loop does not run them. |

## Design target

Both R11 and R12 use the same score. Find the largest box of drone error around the aim point where every trial succeeds with no jams. Then scale it against three targets, sized to the drone's mean, 95th-percentile and worst measured error.

For now the loop scores placement (R11) only, because R12 is deferred.

- **Placement (R11):** the box is in X, Z and θ, the brick's profile plane.
- **Pickup (R12):** the box is in X (across the tines), Z (vertical) and ψ (yaw).

The measured error so far covers X–Y position only (Exp 3 in aerial-assembly). The Z, θ and ψ sides of the boxes are estimates until they are measured. The boxes are listed in [Target boxes](#target-boxes).

## Target boxes

Half-widths around the aim point, from `brixzle.engine.boxes.default_boxes()`.

| Task | Statistic | X (mm) | Z (mm) | θ or ψ (deg) |
|---|---|---|---|---|
| Placement | mean | 14.8 | 3.1 | 0.2 |
| Placement | p95 | 27.8 | 6.3 | 0.3 |
| Placement | worst | 38.5 | 7.2 | 0.4 |
| Pickup | mean | 14.8 | 3.1 | 0.9 |
| Pickup | p95 | 27.8 | 6.3 | 1.4 |
| Pickup | worst | 38.5 | 7.2 | 1.7 |

The angle column is θ (pitch) for placement and ψ (yaw) for pickup.

- **X is measured.** It is the Exp 3 positioning error (session exp3-r4, 24 approaches, `src/brixzle/data/error-exp3-r4.json`), taken as a radius that may lie wholly along X.
- **Z, θ and ψ are estimates.** They are the RMS deviation from the command during the settled hovers of Exp 3 session exp3-r2 (25 approaches, `src/brixzle/data/hover-exp3-r2.json`, made by `scripts/make_hover_estimates.py`).
  - They come from a hover with no payload, not from a release or a pickup, so they are probably too small.
  - Z is the drone's own Lighthouse height against the commanded height, so it misses any error in the Lighthouse estimate itself.
  - Real measurements are aerial-assembly#27.

## R12 in detail

**Deferred (issue #46).** This section describes the pickup trial as it is built, with the fixed fork. The loop does not score it for now. It comes back, and may change, when the gripper is designed for the chosen brick (issue #24).

Until then a brick only has to be grippable (R10, issue #50): it does not need bores or openings sized for this fork.

Today's pickup (aerial-assembly `brixzle`) works like this:

- a passive fork with two 2 mm tines slides along Y into two bores in a brick waiting in the pick fixture;
- each tine has 0.3 mm of radial clearance;
- the only error correction is a tapered guide in front of each bore, with a ±5 mm mouth.

Measured drone error is far larger than ±5 mm (Exp 3, session exp3-r4: 1.5 cm mean, 3.9 cm worst approach), so R12 asks the brick to do the correcting.

**Settled in issue #12: pickup goes through large openings, not tight bores.** Every brick gives large through-Y openings for the fork's tines, like A/B's two big windows. A tight bore captures almost nothing at this error: A/B's 0.3 mm clearance bores score 0 on every pickup box (#9). An opening must be judged by what a straight tine clears through the whole depth, which is its overlap across any sheared bands. A/B's 5.5 mm gutter shear halves its windows to 3.4 mm, which scores 0.031 on the mean pickup box; with the gutter off, it scores 0.156 (predicted and simulated alike, #12).

Within the pickup box, a pickup succeeds when all four of these hold:

1. **Capture.** Every tine passes through its opening along the whole depth, guided only by passive features on the brick, with no jams. With large openings this is geometric; peg-in-hole jamming (Whitney 1982) matters only for tight fits.
2. **Self-centring.** After the lift, the brick hangs within **2 mm and 3°** of its nominal carry pose. An opening centres the brick if its ceiling slopes, steeper than friction, up to a corner the tine settles in (A/B's windows rise at 52° to an acute top corner). This is a starting value, to be revisited, slightly looser than the 1 mm, 3° seat tolerance. Any carry offset adds to placement error.
3. **Gentle contact.** Peak contact force stays within half (50 %) of the drone's thrust margin: maximum thrust (4 × 0.2942 N, datasheet, `drone_params.py` in aerial-assembly) minus the weight of drone, gripper and brick. No neighbouring brick in the fixture moves.
4. **Clean release.** After the brick seats, the gripper withdraws without moving it beyond the drift limit (2 mm). This covers the stuck-on-the-fork concern in aerial-assembly #12.

**Scope for now.** The gripper (the fork) is fixed. The loop redesigns only the brick's pickup features. Co-designing the gripper with the brick, including other passive grippers such as hooks, is future work.

**Settled in issue #9:** pickup may use 50 % of the thrust margin, and the payload limit is 35 g (R10 said about 40 g; `drone_params.py` in aerial-assembly uses 30 g).
