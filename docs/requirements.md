# Requirements

The base requirements every brick family must meet, rewritten for the error-radius goal (#206) and scored as Z on four placements since #222. The designer, the critic and the director read this file. Change it only in a reviewed PR.

**The goal.** Bricks that a Crazyflie 2.1 Brushless can carry with a passive magnet, lower onto a structure with 16 to 40 mm of placement error, and release by yawing once the brick is seated. The minimum worth printing is a brick that seats every release within 16 mm on a stack and on each of the first three reaching bricks of an overhang (Z = 1, below), and builds a stack of 3 and an overhang of 3 with ideal placement; more is better. The gripper and the landing pad are designed by hand, outside the loop (`src/brixzle/engine/gripper_spec.py`).

Each requirement is checked one of three ways:

- **Rule:** an analytic check that runs before any simulation.
- **Sim:** a simulated trial.
- **Design:** a fixed design choice.

| ID | Requirement | Checked by |
|---|---|---|
| R1 | **Generality.** Bricks must build the structures the demo needs, a stack and a back-first overhang; walls, bridges and rings are reported for finalists. | Sim: the MVP gate builds a stack of 3 and a back-first overhang of at least 3 with ideal placement released by a free drop (#206, #218); the running-bond wall, the bridge and the ring are reported, not required |
| R2 | **Voxelizable.** Bricks must sit on a voxel lattice so that structures can later be planned algorithmically; for now a brick spans two voxels. | Design: lattice placements, span_voxels = 2; the cell size is the designer's choice |
| R3 | **Rigidity.** Assemblies must be rigid and physically stable. | Sim: collapse and sag in the MVP gate's builds, and in the noisy builds run for finalists (#206, #222) |
| R4 | **Overhang.** An overhang must accept at least 3 reaching bricks without falling over; more is better. | Sim: the MVP gate (at least 3), and three of the score's four quarters: each of the first three reaching bricks dropped with error onto an ideally built corbel (#222) |
| R5 | **Biased error.** The drone will make placement errors; a design may bias the error onto one side and use that slope as the guide, as a suggestion (design rule DR20), not a requirement. | Design: a suggestion, design rule DR20 |
| R6 | **Steep guides.** A guide ramp must be steeper than its friction coefficient requires (tan θ > μ, with margin). | Rule: friction-cone check on every guide face |
| R7 | **Alternating angles.** Alternating the lean between courses may improve stability. | Design: alternating parts by course |
| R8 | **Stiffness.** I-beam-like sections may be used to stop the brick from flexing. The first course sits on a stable base. | Design: rib frame, anchored base |
| R9 | **Mass.** A brick, with its steel insert, must weigh between 1 g and 25 g in PLA; lighter is preferred. | Rule: mass window, with the insert (engine/gripper_spec.py) |
| R10 | **Grippable.** Each brick carries a magnet target on its top face, near its centre of mass, for a passive magnet gripper that the drone releases by yawing. The combined mass of the brick and the gripper must not be more than 35 g, the drone's payload limit. | Rule: the magnet target's size and position, and the payload (engine/gripper_spec.py; its values are guesses until the bench tests) |
| R11 | **Coarse placement.** The drone places bricks with 16 to 40 mm of error, and a brick must seat by its own geometry; the score is Z = (S − J) / N within 16 mm, averaged over the stack and overhangs 1, 2 and 3. | Sim: the grid score, Z (#206, #208, #222) |
| R12 | **Coarse pickup.** The drone picks bricks up from a landing pad that absorbs its error; the brick only has to offer the magnet target (R10). | Design: the landing pad, outside the loop |
| R13 | **Printable.** A brick must print as one PLA piece plus a steel insert, with walls of at least 1.2 mm and joint gaps of about 0.2 mm or more. | Rules: single piece, minimum wall; gaps: design rule DR19 |

## Hard constraints

A design that breaks one of these is not kept (`python -m brixzle.director verify`):

1. **Mass:** brick with its insert at most 25 g; brick and gripper at most 35 g (R9, R10).
2. **Printable:** one PLA body plus a steel insert, walls at least 1.2 mm (R13). Joint gaps of about 0.2 mm or more are design rule DR19.
3. **Lattice:** a two-voxel brick on the lattice; the cell size is free (R2).
4. **Gripper interface:** a magnet target on the top face, at least 8 mm wide, within 5 mm of the centre of mass along X (R10; guesses until the bench tests).
5. **Release twist:** a seated brick must not turn under the yaw that releases it (design rule DR18; not yet simulated).
6. **Builds the minimum:** a stack of 3 and an overhang of at least 3, with ideal placement and a free drop (R1, R4).
7. **Guides steeper than friction** (R6).
8. **Passes the screen:** at least 32 of 40 random drops within 16 mm seat, 10 on each of the four placements the score uses (#222). A design that fails it is not gridded.

## Design target

The score is **Z** (#222), the same (S − J) / N as the first loop's, on the four placements a stack and an overhang need:

    Z = ¼ Z_stack + ¼ Z_overhang1 + ¼ Z_overhang2 + ¼ Z_overhang3

Each term is (S − J) / N over that placement's grid cells inside the 16 mm disc about its aim, every cell counted by its area: S seated (+1), J jammed (−1), a wrong valley or a fall 0. Z runs from −1 to 1. **Z = 1 means every release within 16 mm seats on all four**: the minimum worth printing. Jams count against Z, so a brick must seat jam-free, not just stay put.

- **Stack** (`full`): a brick across the joint of two seated bricks (running bond). If one brick stacks, many do.
- **Overhang n** (`corbel_reach`, `corbel_reach2`, `corbel_reach3`): the reaching brick of course n of a back-first corbel (`structures.corbel`: in course k a back brick at voxel k − 2, then the reaching brick at voxel k). Its tip is n voxels past the course-0 face, and it lands half on the reach below. Every brick below it, and course n's back brick, is placed ideally, so each term asks whether that one brick seats.

Each brick is released the way the drone hovers over the target and drops it (#218): a free drop from 11.2 mm, the top of its hover wobble (5 mm clearance plus the mean box's 6.2 mm height span). Hold time, yaw and descent speed are flight-time choices, so the score assumes none of them. A printed `allround` dropped onto a full seat seats almost always on the bench, and does not hold when it reaches past an edge (maintainer, 2026-11-26/27).

It is measured on an X-Y grid of release offsets out to 40 mm (`src/brixzle/engine/gridscore.py`, #208): 7.5 mm cells, each split into 2.5 mm cells wherever it touches another outcome, so every outcome change inside 16 mm is resolved to 2.5 mm (a pocket smaller than a coarse cell, inside a coarse cell whose centre and neighbours all seat, can still be missed). A failing cell that does not repeat at friction 0.2 to 0.5 with seeded shifts (fails less than 2/3 of 3 seeds at every friction) is solver noise and is listed and drawn, not hidden (maintainer, 2026-11-26, #218).

The director's archive keeps the best design in each Z band (below 0, 0–0.5, 0.5–0.8, 0.8–0.95, 0.95 and above). Within a band, Z decides; a difference under 0.02 is a tie, broken by Z over the 28 mm disc, then the 40 mm disc (the drone's p95 and worst error), then each placement's green zone weakest first. The **green zone** (`green_mm`), the largest error radius inside which every release seats, is reported per placement as a diagnostic. So are the bare edges (bridge arms) and the corbel's back brick when asked for (`--placements`), and the noisy tower and corbel builds at 16 mm of error, which run for finalists only (`run_builds`).

**The screen** (#222) comes before the grid: 40 seeded random releases inside the 16 mm disc, 10 on each of the four placements, the same free drop; it passes at 32 seated and stops at the 9th failure (`quick_check.quarter_screen`). A design that cannot seat one of the four placements fails it.

Reported, not required: the friction margin, the running-bond wall, the bridge and the ring.

## Deferred requirements

A deferred requirement stays in the list, but the loop does not score it, and the design agent does not design for it. `docs/requirements.yaml` marks it `status: deferred`.

| ID | Deferred until | Why |
|---|---|---|
| R5 | not planned | A design strategy, not a requirement: design rule DR20 (#206). |
| R7 | not planned | A design strategy, not a requirement (#206). |
| R8 | not planned | A design strategy, not a requirement (#206). |
| R12 | not planned | Pickup error is absorbed by a landing pad, outside the loop (#206); the pickup trial was removed in #227. |

## Target boxes

The measured drone error behind the 16 to 40 mm range (Exp 3 in aerial-assembly: a radial RMSE of about 16.8 mm, a 95th percentile of about 27.8 mm and a mean offset of about 9 mm). The error-radius score (#206) uses the placement mean box's release heights; the pickup rows are kept for reference, since pickup is the landing pad's job.


Half-widths around the aim point, from `brixzle.engine.boxes.default_boxes()`.

| Task | Statistic | X (mm) | Y (mm) | Z (mm) | θ or ψ (deg) |
|---|---|---|---|---|---|
| Placement | mean | 14.8 | 14.8 | 3.1 | 0.2 |
| Placement | p95 | 27.8 | 27.8 | 6.3 | 0.3 |
| Placement | worst | 38.5 | 38.5 | 7.2 | 0.4 |
| Pickup | mean | 14.8 | – | 3.1 | 0.9 |
| Pickup | p95 | 27.8 | – | 6.3 | 1.4 |
| Pickup | worst | 38.5 | – | 7.2 | 1.7 |

The angle column is θ (pitch) for placement and ψ (yaw) for pickup.

**Placement's X and Y form a disc** (issue #86). The measured error is a radius in a random X–Y direction, so a drop at (x, y) is inside a placement box when x² + y² ≤ r², with r the X half-width. A square box would ask for corners at 1.4 r, which the drone never reached.

- **X is measured.** It is the Exp 3 positioning error (session exp3-r4, 24 approaches, `src/brixzle/data/error-exp3-r4.json`), taken as a radius that may lie wholly along X.
- **Z, θ and ψ are estimates.** They are the RMS deviation from the command during the settled hovers of Exp 3 session exp3-r2 (25 approaches, `src/brixzle/data/hover-exp3-r2.json`, made by `scripts/make_hover_estimates.py`).
  - They come from a hover with no payload, not from a release or a pickup, so they are probably too small.
  - Z is the drone's own Lighthouse height against the commanded height, so it misses any error in the Lighthouse estimate itself.
  - Real measurements are aerial-assembly#27.
