# Requirements

The base requirements every brick family must meet, rewritten for the error-radius goal (#206). The designer, the critic and the director read this file. Change it only in a reviewed PR.

**The goal.** Bricks that a Crazyflie 2.1 Brushless can carry with a passive magnet, lower onto a structure with 16 to 40 mm of placement error, and release by yawing once the brick is seated. The minimum worth printing is a brick that tolerates 16 mm and builds a stack of 3 and an overhang of 3; more of each is better. The gripper and the landing pad are designed by hand, outside the loop (`src/brixzle/engine/gripper_spec.py`).

Each requirement is checked one of three ways:

- **Rule:** an analytic check that runs before any simulation.
- **Sim:** a simulated trial.
- **Design:** a fixed design choice.

| ID | Requirement | Checked by |
|---|---|---|
| R1 | **Generality.** Bricks must build the structures the demo needs, a stack and a back-first overhang; walls, bridges and rings are reported for finalists. | Sim: the MVP gate builds a stack of 3 and a back-first overhang of at least 3 with held placement (#206); the running-bond wall, the bridge and the ring are reported, not required |
| R2 | **Voxelizable.** Bricks must sit on a voxel lattice so that structures can later be planned algorithmically; for now a brick spans two voxels. | Design: lattice placements, span_voxels = 2; the cell size is the designer's choice |
| R3 | **Rigidity.** Assemblies must be rigid and physically stable. | Sim: collapse and sag in the MVP gate's builds and in the noisy builds (#206) |
| R4 | **Overhang.** An overhang must accept at least 3 reaching bricks without falling over; more is better. | Sim: the MVP gate (at least 3) and the noisy corbel at the brick's tolerated error, whose length is scored (#206) |
| R5 | **Biased error.** The drone will make placement errors; a design may bias the error onto one side and use that slope as the guide, as a suggestion (design rule DR20), not a requirement. | Design: a suggestion, design rule DR20 |
| R6 | **Steep guides.** A guide ramp must be steeper than its friction coefficient requires (tan θ > μ, with margin). | Rule: friction-cone check on every guide face |
| R7 | **Alternating angles.** Alternating the lean between courses may improve stability. | Design: alternating parts by course |
| R8 | **Stiffness.** I-beam-like sections may be used to stop the brick from flexing. The first course sits on a stable base. | Design: rib frame, anchored base |
| R9 | **Mass.** A brick, with its steel insert, must weigh between 1 g and 25 g in PLA; lighter is preferred. | Rule: mass window, with the insert (engine/gripper_spec.py) |
| R10 | **Grippable.** Each brick carries a magnet target on its top face, near its centre of mass, for a passive magnet gripper that the drone releases by yawing. The combined mass of the brick and the gripper must not be more than 35 g, the drone's payload limit. | Rule: the magnet target's size and position, and the payload (engine/gripper_spec.py; its values are guesses until the bench tests) |
| R11 | **Coarse placement.** The drone places bricks with 16 to 40 mm of error, and a brick must seat by its own geometry; the score is the largest error at which every placement type seats at least 90 % of the time. | Sim: the error-radius search, r90 (#206) |
| R12 | **Coarse pickup.** The drone picks bricks up from a landing pad that absorbs its error; the brick only has to offer the magnet target (R10). | Design: the landing pad, outside the loop |
| R13 | **Printable.** A brick must print as one PLA piece plus a steel insert, with walls of at least 1.2 mm and joint gaps of about 0.2 mm or more. | Rules: single piece, minimum wall; gaps: design rule DR19 |

## Hard constraints

A design that breaks one of these is not kept (`python -m brixzle.director verify`):

1. **Mass:** brick with its insert at most 25 g; brick and gripper at most 35 g (R9, R10).
2. **Printable:** one PLA body plus a steel insert, walls at least 1.2 mm (R13). Joint gaps of about 0.2 mm or more are design rule DR19.
3. **Lattice:** a two-voxel brick on the lattice; the cell size is free (R2).
4. **Gripper interface:** a magnet target on the top face, at least 8 mm wide, within 5 mm of the centre of mass along X (R10; guesses until the bench tests).
5. **Release twist:** a seated brick must not turn under the yaw that releases it (design rule DR18; not yet simulated).
6. **Builds the minimum:** a stack of 3 and an overhang of at least 3, with ideal, held placement (R1, R4).
7. **Guides steeper than friction** (R6).

## Design target

The score is **r90**: the largest placement error, as the radius of a disc about the aim, at which every placement type seats at least 90 % of its drops (`src/brixzle/engine/radius.py`). The placement types are a full seat (stacking), each bare edge (a brick reaching one voxel past an edge) and the first reach of a back-first corbel. A jam, a wrong valley and a fall all count against it. Each brick is placed by a held release: the gripper holds its attitude and lowers it until it stops, then lets go.

Anything from 16 to 40 mm is acceptable, and more is better. Then the stack height and the overhang length that noisy builds reach at that error, brick by brick. The director's archive keeps the best design in each error band (below 16, 16-22, 22-28, 28-34, 34+ mm).

Reported, not required: the friction margin, the running-bond wall, the bridge and the ring.

## Deferred requirements

A deferred requirement stays in the list, but the loop does not score it, and the design agent does not design for it. `docs/requirements.yaml` marks it `status: deferred`.

| ID | Deferred until | Why |
|---|---|---|
| R5 | not planned | A design strategy, not a requirement: design rule DR20 (#206). |
| R7 | not planned | A design strategy, not a requirement (#206). |
| R8 | not planned | A design strategy, not a requirement (#206). |
| R12 | not planned | Pickup error is absorbed by a landing pad, outside the loop (#206). |

## Target boxes

The measured drone error behind the 16 to 40 mm range: Exp 3 in aerial-assembly (session exp3-r4, 24 approaches, `src/brixzle/data/error-exp3-r4.json`), a radial RMSE of about 16.8 mm and a 95th percentile of about 27.8 mm, with a mean offset of about 9 mm. The release heights come from the mean box's Z (`brixzle.engine.boxes.default_boxes()`), an estimate from hovers with no payload.
