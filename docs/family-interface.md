# Brick family interface

A brick family is a Python package `src/brixzle/families/<name>/` whose `family.py` exports `FAMILY = Family(...)`. The engine reads a family only through this object. The registry finds a family automatically, so adding one doesn't mean editing a list.

Before the engine scores a family, `brixzle.families.contract.check_family(FAMILY)` must return an empty list. The code is in `src/brixzle/families/interface.py`. The A/B baseline, `ab`, is a Prismatic family. Two dummy families, `wedge` and `pyramid`, show the two forms at their simplest.

To mark a design as infeasible, for example a profile that can't be built, a family raises `ValueError`. aerial-assembly's optimizer did the same job by setting `geometry_builds = −1`.

## Conventions

- **Units:** geometry is in millimetres and degrees, and field names carry `_mm` or `_deg`. Mass is in grams. Conversion to SI happens only at the MuJoCo boundary.
- **Frame:**
  - X runs along the lattice;
  - Y runs across it (the fork's tines slide along Y);
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
| `pickup(p, part)` | `PickupFeatures(openings=(Polygon, ...), bores=(Bore(x_mm, z_mm, d_mm, clearance_mm), ...))`. **`openings`** are large through-Y openings for the fork's tines: XZ polygons in the section frame that sweep with the extrusion like the section, usually some of its holes. The fork is picked up through these (#12); give one per tine. `bores` are tight straight through-Y bores, kept for the A/B baseline's CAD and R10/R13 rules; new families give openings instead. The builders cut bores into the solid; openings are already section holes. |
| `extra_rules(p, part, bodies)` | Optional. Family-specific margins `{"R<n>.<name>": value}`, where a value ≥ 0 means satisfied. |

## Two ways to describe the geometry

**`Prismatic(section, extrusion)`** is for 2.5D bricks: an XZ profile swept through Y.
- `section(p, part)` returns a `Section(outer, holes, guide_faces)`, built from shapely polygons in XZ. The holes are lightening holes, not the fork bores.
- `extrusion(p)` returns an `Extrusion(depth_mm, bands)`. With no bands it's a straight extrusion. Bands of the form `(y0, y1, z_shift_y0, z_shift_y1)` shear the profile, for example to make a V gutter across Y.
- The builders derive the solid, exact convex collision pieces and the predictor section.

**`Solid3D(solid, collision=None, section=None)`** is for fully 3D bricks.
- `solid(p, part)` returns a CadQuery solid without the fork bores.
- `collision` is optional. If given, it returns exact convex vertex sets: use it when the brick is built from convex primitives. Otherwise the solid is convex-decomposed with CoACD (seed 0, preprocessing off). That takes about 2 s per part for `pyramid`.
- `section` is optional. Otherwise the 2D predictor uses the solid's slice on the XZ plane at y = 0. Outline edges between 5° and 85° from horizontal then count as guide faces.
- The 2D predictor is only approximate for a genuinely 3D brick. The sim verifier is the reference.

## Shared rules

`brixzle.families.rules` computes these margins for every family. Each key starts with a requirement id from `docs/requirements.yaml`.

| Key | Margin |
|---|---|
| `R6.guide_slides` | The shallowest guide face's angle minus (atan μ_max + 8°), with μ_max = 0.4 |
| `R9.mass_min`, `R9.mass_max` | Mass − 1 g, and 25 g − mass |
| `R10.hangs_stable` | Lowest tine top (bore height + tine diameter / 2) − centre-of-mass height − 1 mm |
| `R10.tines_straddle_com` | How far the centre of mass lies inside the outermost bores in X, minus 1 mm |
| `R13.bore_wall` | Thinnest material round any bore, to the outline or a lightening hole, measured in the section, − 1.2 mm |

The R9 and R10 formulas and all the constants come from aerial-assembly `rules.py` @ a649663. aerial-assembly used the first bore for R10 where these use the lowest; for A/B the first bore is the lowest. A family's `extra_rules` may add margins but not redefine these.

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
   - a seated brick doesn't overlap the brick above it, its neighbour in the course or the base. This is checked at key points, a quarter of the way from each collision corner to its piece's centre; contact closer than 0.5 mm counts as touching;
   - `part_for` returns a listed part.
6. **rules:** every margin is finite and its key names a requirement that exists.
7. **pickup:** there's at least one opening or bore. Every opening lies inside the section's outline, and a straight 2 mm tine clears it through the whole depth. No bore breaks out of the outline or into a hole. The wall thickness round a bore is the `R13.bore_wall` margin, not a pass/fail check.

Each violation says which check failed, where (for example `defaults, part A`) and what to fix.

## Not covered yet

- **Fork spacing:** checking the bores against the fixed fork's tine spacing comes with the pickup trial (#9).
- **Pickup openings:** openings are checked for capture and self-centring by translation only (`engine/predictor/pickup.py`); the hanging brick's rotation is checked only in the sim. R10's rules still use the bores, so a family without bores has no R10 pickup margins yet.
- **Soft contact:** `SimBodies.soft_collision` is not built or simulated yet, so A/B's snap barbs (`barb_h > 0`) are rejected.
- **Running agent-written families safely** belongs to the design agent (#18).
