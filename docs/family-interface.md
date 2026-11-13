# Brick family interface

A brick family is a Python package `src/brixzle/families/<name>/` whose `family.py` exports `FAMILY = Family(...)`. The engine reads a family only through this object. The registry finds a family automatically, so adding one doesn't mean editing a list.

Before the engine scores a family, `brixzle.families.contract.check_family(FAMILY)` must return an empty list. The code is in `src/brixzle/families/interface.py`. The A/B baseline, `ab`, is a Prismatic family. Two dummy families, `wedge` and `pyramid`, show the two forms at their simplest.

To mark a design as infeasible, for example a profile that can't be built, a family raises `ValueError`. aerial-assembly's optimizer did the same job by setting `geometry_builds = −1`.

## Conventions

- **Units:** geometry is in millimetres and degrees, and field names carry `_mm` or `_deg`. Mass is in grams. Conversion to SI happens only at the MuJoCo boundary.
- **Frame:**
  - X runs along the lattice;
  - Y runs across it (a round grip bore runs straight through Y);
  - Z points up, and z = 0 is the brick's seat.
- **Quaternions:** `[w, x, y, z]`.
- **Material:** PLA at 1.24e-3 g/mm³, solid infill.

## What a family provides

| Field | What it is |
|---|---|
| `name`, `version`, `description` | Identity. Names are unique. |
| `params_type` | A frozen dataclass subclassing `FamilyParams`, which gives `to_dict` and `from_dict`. Its defaults are the starting design. Its `__post_init__` raises `ValueError` for infeasible values. |
| `bounds` | `{name: (lo, hi)}` for the parameters the tuner may change. The defaults must lie inside. |
| `parts` | Part names, e.g. `("A",)` or `("A", "B")` for alternating courses. |
| `geometry` | `Prismatic` or `Solid3D` (see below). |
| `lattice(p)` | `Lattice(U_mm, D_mm, H0_mm, span_voxels)`: the voxel pitch in X, the depth in Y, the course pitch in Z, and the number of voxels a brick covers. |
| `part_for(course, direction)` | Which part sits at a placement. |
| `seat_pose(p, placement)` | `(pos_mm, quat_wxyz)` of a seated brick. A `Placement` is `(course, i0, direction)`. |
| `insertion_axis(p, placement)` | The unit `(x, z)` direction of the final seating motion. |
| `release_lift_mm(p)` | How far above the seat the drone releases the brick, so that nothing overlaps at release. |
| `base(p, voxels, direction)` | The anchored base that receives course 0, in a frame where x = 0 is the first voxel's left edge. It can be a `Section`, a CadQuery solid (decomposed), or a list of exact convex vertex sets. A `Section` is extruded the same way as the family's bricks (Prismatic, e.g. the A/B gutter shear), or straight through D (Solid3D). |
| `pickup(p, part)` | The part's **grip features** (R10 grippable, issue #50): a tuple of `GripFeature(kind, x_mm, z_mm, size_mm, outline=None)`, with at least one. A grip feature is anything a non-actuated gripper could hold. The gripper is designed after the brick (issue #24), so a feature says only what it is, where its centre is in the section frame, and its extent in X and Z. See [Grip features](#grip-features). |
| `extra_rules(p, part, bodies)` | Optional. Family-specific margins `{"R<n>.<name>": value}`, where a value ≥ 0 means satisfied. |
| `ring` | Optional, but a design without it is rejected: a `RingKit`, how the family turns a corner and closes a ring (issue #89). See [Ring kit](#ring-kit). |

## Where a brick seats in the checks and the sim

The contract and the drop trial (`engine/sim/trials.py`, `rig_layout`) use the same placements (issues #70 and #82):

- **Course 0** is `Placement(0, 1)`: start voxel 1, so a two-voxel brick is centred at x = 2U. It sits on a base covering voxels 0 to 3 (`base(p, 4, 1)`, x = 0 at voxel 0's left edge).
- **Course 1 is a running bond.** Two course-0 bricks are seated at `Placement(0, 1)` and `Placement(0, 1 + span)`, and the brick is dropped at `Placement(1, 1 + span // 2)`, across the joint between them, as in a wall, a bridge or an overhang. Its seat is judged against the first brick below. The base covers both bricks plus one voxel each side (`base(p, 2 + 2 span, 1)`). A one-voxel brick (`span_voxels = 1`) keeps the stack drop: one brick at `Placement(0, 1)` and the drop at `Placement(1, 1)`.
- **The contract** also checks that a course-0 brick does not overlap its neighbour at `Placement(0, 1 + span)`. A course-1 brick that overlaps either brick below is a violation: the brick cannot lay a running bond.

The course-0 drop has no neighbour. Issue #82 proposed one at `Placement(0, 1 + span)`, but on A/B that brick sits on the side the stroke comes from, which a real build keeps open (the assembly order fills each course from the other side), and A/B then jammed at its own aim.

So the base must receive a brick centred at x = 2U. A base whose pattern repeats every two voxels has to put its seat at 2U, 4U and so on. The dummy `wedge` used to put it one voxel over, and never seated.

## Grip features

R10 asks for at least two large through-Y openings, above and either side of the centre of mass, so the brick hangs level from two points (issue #87, approved by the maintainer on 2026-11-12). Each must fit a 5 mm circle: a bore at least 5 mm across, or an opening (a section hole given with `opening(...)`) whose largest inscribed circle is at least 5 mm across. Smaller features may still be listed, and count for `R10.grippable`. R10 does not say which gripper, so don't size an opening for a fork or any other tool.

- **`kind`** is free text, such as `"bore"`, `"opening"`, `"slot"` or `"lip"`.
- **A `"bore"` is the one kind the builders act on.** It is a round hole straight through Y, of diameter `size_mm[0]`, and the builders cut it into the solid. Make one with `interface.bore(x_mm, z_mm, d_mm)`.
- **Every other kind is already part of your geometry.** If it has an XZ outline, give it: `interface.opening(polygon)` wraps a section hole and takes its centre and size from the polygon's bounding box (pass `kind="slot"` or another name if "opening" doesn't fit). A feature with no outline, such as a lip on the brick's edge, is just `GripFeature("lip", x_mm, z_mm, (width, height))`.

A/B gives four: its two 2.6 mm bores and its two big lightening windows.

## Ring kit

Issue #89. A design must close a ring: a square of n × n cells, course by course, as in R1. The family says how it turns a corner and how it closes a course; the engine (`engine/sim/ring_build.py`) builds what it plans, with ideal placement, and stops at the first brick that does not seat.

**`RingKit(parts, solid, plan, base, collision=None)`:**

- `parts`: the parts used only in a ring, for example the turn piece (`("L3A", "L3B", …)`).
- `solid(p, part)`: the CadQuery solid of each, and optionally `collision(p, part)` as exact convex vertex sets (otherwise it is convex-decomposed, as for `Solid3D`).
- `plan(p, n, courses)`: the ring's bricks in assembly order, each a `RingBrick(kind, part, course, pos_mm, quat, cells, name)`. The part may be one of the family's ordinary parts or one of the kit's; `cells` are the (i, j) cells it fills, which tell the build which brick it sits on.
- `base(p, n)`: the anchored base under course 0, as convex vertex sets.

**The simplest kit is `engine/ring_cells.kit_from_cell(family, cell, stroke_mm=None)`** (Prismatic families). It builds every ring part from one **cell**: one voxel of a part's profile, centred on x = 0, swept like the brick.

- `cell(p, part, hooks=True, exposed_lo=False, exposed_hi=False, cut_hi=0.0) -> Section`. `hooks=False` leaves out any feature that seats along a slanted stroke: the turn piece's side arm must nest by its keel only. `exposed_lo` and `exposed_hi` say which ends face no neighbour in the same piece, to trim them as the straight brick's ends are. `cut_hi` cuts that much more off the +x end.
- The parts: **L3**, the turn piece on even courses (a corner cell, one cell along +x and one along +y); **I3**, three straight cells that own the corner on odd courses, so the corner bond alternates; **K2**, the **closer**, the last brick of each course, its upstream end cut back by the stroke's sideways travel. The straight brick is the family's own part.
- `stroke_mm(p)` is the slanted seating stroke's length for the closer's cut (default: `release_lift_mm`). A/B passes its tooth stroke, `tooth_L + slot_extra`, since its release lift also holds the sawtooth's drop.
- A/B's kit (`families/ab/ring.py`) is `kit_from_cell` with a port of aerial-assembly's `cell_profile`.

## Two ways to describe the geometry

**Prismatic by default** (maintainer, 2026-11-06, on #18). The design agent writes `Prismatic` families. A `Solid3D` family is allowed only when a needed feature cannot be prismatic, and the agent must say which feature and why (`solid3d_reason` in its `design_note.json`, which the design log records). The reason: the analytic predictor steps one XZ slice at y = 0, so it is only sound when the contact geometry is uniform along Y. Mirror symmetry is not enough.

**`Prismatic(section, extrusion)`** is for 2.5D bricks: an XZ profile swept through Y.
- `section(p, part)` returns a `Section(outer, holes, guide_faces)`, built from shapely polygons in XZ. The holes are lightening holes, not the round grip bores.
- `extrusion(p)` returns an `Extrusion(depth_mm, bands)`. With no bands it's a straight extrusion. Bands of the form `(y0, y1, z_shift_y0, z_shift_y1)` shear the profile, for example to make a V gutter across Y.
- The builders derive the solid, exact convex collision pieces and the predictor section.

**`Solid3D(solid, collision=None, section=None)`** is for fully 3D bricks.
- `solid(p, part)` returns a CadQuery solid without the round grip bores.
- `collision` is optional. If given, it returns exact convex vertex sets: use it when the brick is built from convex primitives. Otherwise the solid is convex-decomposed with CoACD (seed 0, preprocessing off). That takes about 2 s per part for `pyramid`.
- `section` is optional. Otherwise the 2D predictor uses the solid's slice on the XZ plane at y = 0. Outline edges between 5° and 85° from horizontal then count as guide faces.
- The 2D predictor is only approximate for a genuinely 3D brick. The sim verifier is the reference, and the scorer does not run the predictor for a `Solid3D` family: its report is sim only (issue #51).

## Shared rules

`brixzle.families.rules` computes these margins for every family. Each key starts with a requirement id from `docs/requirements.yaml`.

| Key | Margin |
|---|---|
| `R6.guide_slides` | The shallowest guide face's angle minus (atan μ_max + 8°), with μ_max = 0.4 |
| `R9.mass_min`, `R9.mass_max` | Mass − 1 g, and 25 g − mass |
| `R10.grippable` | Number of grip features − 1 |
| `R10.two_openings` | Number of carrying openings − 2. A carrying opening is a through-Y grip feature a 5 mm circle fits in: a bore at least 5 mm across, or an opening whose outline's largest inscribed circle is at least 5 mm across (issue #87) |
| `R10.hangs_below` | Over the two outermost carrying openings in X, the lower of their top points (an outline's highest z, or a bore's centre + d/2), − centre-of-mass height − 1 mm. −1 with fewer than two carrying openings |
| `R10.straddles_com` | How far the centre of mass lies inside those two openings' centres in X, − 1 mm. −1 with fewer than two carrying openings |
| `R10.within_payload` | 35 g − (mass + 5 g for the gripper). The 35 g payload carries the brick and the gripper together. The gripper isn't designed yet (issue #24), so 5 g is an allowance, not a measured mass. |
| `R13.bore_wall` | Thinnest material round any bore, to the outline or a lightening hole, measured in the section, − 1.2 mm. Only if the part has a bore. |

The R9 formulas and the constants come from aerial-assembly `rules.py` @ a649663. A family's `extra_rules` may add margins but not redefine these.

**Not in the loop's rule set: the fork checks.** `rules.fork_margins(family, p, part, bodies, tine_d_mm=2.0)` gives the two checks against today's fixed fork, for a part with bores. Nothing in the loop calls it. It is kept, with its tests, for the gripper step (issue #24).

| Key | Margin |
|---|---|
| `R10.hangs_stable` | Lowest tine top (bore height + tine diameter / 2) − centre-of-mass height − 1 mm |
| `R10.tines_straddle_com` | How far the centre of mass lies inside the outermost bores in X, minus 1 mm |

These formulas come from the same `rules.py`. aerial-assembly used the first bore where these use the lowest; for A/B the first bore is the lowest.

## What the contract checks

`check_family(family, samples=3, seed=0)` runs on the defaults and on seeded random points inside the bounds. At a sampled point, raising `ValueError` marks the point as infeasible and is allowed. Any other exception is a violation, and so is any exception at the defaults.

1. **params:** bounds are finite with lo < hi and name real fields. The defaults are finite and inside the bounds. `to_dict`/`from_dict` round-trips, and `from_dict` rejects unknown keys.
2. **section:** `outer` is a valid polygon with area. Holes lie inside it, and guide faces lie on the outline.
3. **solid:** the solid is valid and gives one watertight piece. For a straight `Prismatic` extrusion, its volume matches section area × depth within 1%.
4. **sim_bodies:**
   - every collision piece is a convex vertex set with volume;
   - the collision is right at the key contact points. These are points a quarter, half and three quarters along every outline edge, 0.5 mm inside and 0.5 mm outside it (half the 1 mm seat tolerance), at three depths across Y (Prismatic) or in the y = 0 slice (Solid3D). Every inside point must be in the collision (no missing material at a face), and no outside point may be (no phantom ledge). Sharp corners are left out, because a convex decomposition rounds them off slightly without changing how a brick seats;
   - mass, centre of mass and inertia are finite, and inertia is symmetric positive-definite;
   - MuJoCo compiles the bodies.
5. **lattice:**
   - quaternions have unit norm and insertion axes are unit vectors;
   - a seated brick doesn't overlap its neighbour in the course or the base, and a course-1 brick laid across the joint between two course-0 bricks (running bond, issue #82) doesn't overlap either of them. This is checked at key points, a quarter of the way from each collision corner to its piece's centre; contact closer than 0.5 mm counts as touching;
   - `part_for` returns a listed part.
6. **rules:** every margin is finite and its key names a requirement that exists.
7. **pickup (R10 grippable):** there's at least one grip feature, and each has a kind, a finite position and a size above zero. None breaks out of the brick:
   - a bore lies inside the section's outline and doesn't break into a hole. The wall thickness round it is the `R13.bore_wall` margin, not a pass/fail check;
   - a feature with an outline lies inside the section's outline;
   - a feature with only a position and a size lies inside the brick's extent in X and Z.

   Nothing here checks a gripper. Whether a straight tine clears an opening through the whole depth is the pickup predictor's job (`engine/predictor/pickup.py`), for the gripper step.

Each violation says which check failed, where (for example `defaults, part A`) and what to fix.

## Not covered yet

- **The gripper:** which gripper holds which grip feature is decided after the brick is settled (issue #24). Until then nothing checks that a feature can really be held; R10 only asks that one exists.
- **The fixed fork:** the pickup trial (`engine/sim/pickup.py`) and its predictor still assume today's fork. They read the bores and openings among a family's grip features, and they don't apply to a family that has neither. The loop doesn't run them (issue #46).
- **Soft contact:** `SimBodies.soft_collision` is not built or simulated yet, so A/B's snap barbs (`barb_h > 0`) are rejected.
- **Agent-written families** live outside `src/`, in the loop's run directory, as a package `<root>/<module>/` imported by path (`agents/check_family.load_family`). The contract and the rules run in a separate process with a time limit before the loop imports a family itself, so a family that hangs or crashes the interpreter cannot stop the loop. The code still runs with the loop's own permissions: it is not sandboxed.
