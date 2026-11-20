# From runs/fable/fable_feather/family.py, verified in RSI_RESULTS/fable_feather-verify-20261117.
# Changed: refactored into a subclass of twinhook/base.py's WebbedTwinHook (#162): its defaults and bounds. The geometry is unchanged (tests/test_twinhook_line.py).
"""Feather: the twin-hook chevron made as light as it will go (the capture-for-mass experiment).

fable_twinhook4's brick and ring kit, with five things given up for mass. Each one is a change of
default; the geometry code is twinhook4's.

- **Depth 30 -> 22 mm.** The biggest saving, and the capture it costs is in Y: a brick more than
  about D / 2 off centre is past the course below.
- **Blind side pockets on** (``pocket_wall`` 1.2): each side band is hollow behind a skin. On
  twinhook4 this left Z unchanged and cost about 1.5 standard errors of held-out Z.
- **Course 12.5 -> 10.5 mm.** As thin as the slot and the two 5 mm carrying openings (R10) allow.
- **Ramp 45 -> 42 deg** (left face 41.6 -> 38.7). The ramp segment is then thick enough, square to
  its faces, for the 5 mm opening in the thinner course. Shallower guide faces are friction margin
  given up.
- **Walls and skins 1.4 -> 1.2 mm**, the R13 floor.

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
class FeatherParams(TwinHookParams):
    H0: float = 10.5          # course pitch: vertical thickness of the corrugated body
    D: float = 22.0           # depth in Y
    ramp_deg: float = 42.0    # ramp angle from horizontal; the left face takes the rest of the voxel
    pocket_wall: float = 1.2  # skin left round the blind side pockets in the left-face segment (0: no pockets)
    wall: float = 1.2         # wall round the lightening holes


BOUNDS = {
    "U": (32.0, 38.0),
    "H0": (10.0, 18.0),
    "D": (20.0, 32.0),
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
    "y_angle": (36.0, 46.0),
    "band_gap": (0.0, 0.8),
    "release_frac": (0.0, 1.0),
    "end_bevel": (0.0, 1.2),
    "wall": (1.2, 2.6),
}


class Feather(WebbedTwinHook):
    name = "fable_feather"
    version = '0.1.0'
    description = 'Feather: the twin-hook chevron at 22 mm depth and 10.5 mm courses, with blind side pockets and 1.2 mm walls; under 10 g.'
    params_type = FeatherParams
    bounds = BOUNDS


FAMILY = Feather.instance().family()
