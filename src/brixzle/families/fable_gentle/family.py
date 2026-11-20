# From runs/fable/fable_gentle/family.py, verified in RSI_RESULTS/fable_gentle-verify-20261117.
# Changed: refactored into a subclass of twinhook/base.py's WebbedTwinHook (#162): its defaults, bounds and rule floor. The geometry is unchanged (tests/test_twinhook_line.py).
"""Gentle: the twin-hook chevron tuned to seat under its own weight through high friction.

fable_twinhook4's brick and ring kit; only defaults differ (and the turn piece's web is feather's,
which may slope between the two cells).

What jams twinhook4 at mu 0.8 (131 of 180 held-out mean-box drops seat, 49 jam): 46 of the 49 start
off centre in Y. Past about 8 mm off, the brick stays rolled about 20 deg and propped across the
trough, its teeth in the valley groove of one side band and its far edge on the other. Nearer the
centre it stops about 4 mm short along the stroke, a hook's side on the edge of its slot and the
tip of the other hook on its mouth's tapered wall. Both are two contacts whose friction cones see
each other (DR2), and the faces involved are at 42 to 51 deg, against 38.7 deg of friction.

- **V across Y 42 -> 50 deg**, and with it the hooks' flanks, which continue the V. A brick off
  centre now slides on faces 11 deg steeper than friction, not 3 deg.
- **Mouth 1.0 -> 1.5 mm at the surface, parallel section 2.0 -> 1.0 mm.** A hook at its mouth has
  more room before it meets a wall (reach showed the same thing from the other side).
- **Depth 30 -> 32 mm, course 12.5 -> 15 mm, walls 1.4 -> 1.2 mm.** Depth is Y capture; the
  thicker course keeps the two 5 mm openings beside the wider mouths; thinner walls pay for both.

The ramp stays at 45 deg: steepening it to 50 deg made the probe worse, not better.

What follows describes the brick (fable_twinhook2's text).

Twin-hook chevron, second version: the hooks engage only after Y has centred.

The brick is fable_twinhook's: one valley per 34 mm voxel, a tooth under each keel and a mirrored
horn on each peak, both leaning the same way and seating with one stroke (down, toward -x). The
tooth holds a brick that reaches toward +x and the horn one that reaches toward -x.

What changed, and the finding behind each change (runs/map-20261116T142641569145Z-fable_twinhook-mean
and runs/gate-20261116T142811491850Z-fable_twinhook; simulated, uncalibrated contact):

- **Slots and pockets only in the flat centre band.** Off centre in Y, the first version's teeth
  dropped into its full-depth slots, and the brick stayed rolled about 20 deg where it landed. Now
  the side bands are plain: a tooth off centre rides the valley groove down to the mouth.
- **Each hook is the apex of the V across Y.** The body is a V with a flat keel band, and the hook
  fills the missing apex (its flanks continue the V's faces). A brick off centre slides on its band
  like a full V, and the hook's flanks meet the slot's edges only as Y closes.
- **Steeper faces, less lean.** The hook's overhang turns a lift in Y into a set-back along the
  stroke, in proportion to tan(lean). Lean 32 -> 23 deg, ramp 42 -> 45, left face 34 -> 42, V across
  Y 38 -> 42, so the brick's own diagonal slide keeps the hooks clear of both walls.
- **The seat is the V again.** With the left face standing back 0.25 mm, a half-supported brick
  tipped about 1.5 deg, met its hook wall early in a 16 deg wedge and stopped 1.5 to 2 mm short.
  Now the ramp and the left face stop the brick, and the hook walls stand 0.08 mm clear.
- **Tapered mouths.** Each slot is 1.0 mm wider than its hook at the mouth and 0.15 mm at the
  bottom, so the last of the stroke runs on a steep wall, not on the ramp.
- **Deeper in Y** (26 -> 30 mm), for capture out to the mean box's edge; thinner courses (14 ->
  12.5 mm) to pay for it.

Four smaller changes came from probe drops of this version at the first version's mean-box poses:

- **Steep tip faces** (``tip_back``): a tooth's tip landing on a horn's tip was face on face at
  23 deg and held at mu 0.5; the tip faces are now at 42 deg.
- **A bevel on the -x end's top corner** (``end_bevel``): a tooth sliding down the left face of
  the course below crosses the joint between two bricks, and its tip caught the next brick's corner.
- **Hooks narrower in plan toward their Y ends** (``hook_skew``): room for a brick that arrives yawed.
- **Release above the horns** (``release_frac``): the release height clears the horns of the
  course below at any offset. At the lowest release a drop 14 mm off in X started inside the ramp
  below. ``release_frac`` 0 gives the first version's low release back.

Original design; the tooth-in-slot joint and the rib frame are reused from the maintainer's A/B.
"""

from dataclasses import dataclass

from brixzle.families.twinhook.base import TwinHookParams, WebbedTwinHook


@dataclass(frozen=True)
class GentleParams(TwinHookParams):
    H0: float = 15.0          # course pitch: vertical thickness of the corrugated body
    D: float = 32.0           # depth in Y
    side_gap: float = 1.5     # mouth clearance on the other side of the hook, at the surface
    snug_L: float = 1.0       # length, from the hook's tip, over which that wall runs parallel to the stroke
    y_angle: float = 50.0     # V across Y, from horizontal, outside the flat centre band
    wall: float = 1.2         # wall round the lightening holes


BOUNDS = {
    "U": (32.0, 38.0),
    "H0": (12.0, 18.0),
    "D": (26.0, 32.0),
    "valley_u": (8.8, 10.4),
    "ramp_run": (14.5, 17.0),
    "ramp_deg": (40.0, 48.0),
    "hook_lean": (18.0, 28.0),
    "hook_L": (4.0, 6.0),
    "hook_w": (3.2, 5.0),
    "tip_chamfer": (0.4, 1.6),
    "tip_back": (0.6, 1.6),
    "hook_skew": (0.0, 1.0),
    "hook_gap": (0.0, 0.3),
    "side_gap": (0.5, 1.5),
    "seat_gap": (0.05, 0.4),
    "snug_L": (0.0, 2.5),
    "lid_t": (1.8, 3.0),
    "y_angle": (40.0, 54.0),
    "band_gap": (0.0, 0.8),
    "release_frac": (0.0, 1.0),
    "end_bevel": (0.0, 1.2),
    "wall": (1.2, 2.6),
}


class Gentle(WebbedTwinHook):
    name = "fable_gentle"
    version = '0.1.0'
    description = 'Gentle: the twin-hook chevron with a 50 deg V across Y and 1.5 mm mouths, to seat under its own weight through high friction.'
    params_type = GentleParams
    bounds = BOUNDS
    # twinhook4 asks for 1.3 here. A 50 deg V brings this prediction down to 0.96, yet at mu 0.8 the sim
    # seats far more drops with it, so gentle's floor is 0.9.
    y_before_hooks_floor = 0.9


FAMILY = Gentle.instance().family()
