# From runs/fable/fable_reach/family.py, verified in RSI_RESULTS/fable_reach-verify-20261117.
# Changed: refactored into a subclass of twinhook/base.py's TwinHook (#162): its defaults, bounds and rule floor. The geometry is unchanged (tests/test_twinhook_line.py).
"""Reach: the twin-hook chevron tuned for the longest overhang (the capture-for-reach experiment).

fable_twinhook4's brick and ring kit; only defaults differ.

- **Hook lean 23 -> 36 deg.** A hook wall holds a brick down with the vertical part of its normal,
  which is sin(lean): 0.39 at 23 deg, 0.59 at 36 deg. A back brick pulled straight up also meets
  its hook wall sooner (gap / sin(lean)). 36 deg leaves 1 deg of the 8 deg asked between the stroke
  and the 45 deg ramp.
- **Seated gaps at the 0.1 mm floor**: ``hook_gap`` 0.08 -> 0.10 (it was under the floor) and
  ``seat_gap`` 0.15 -> 0.10; the parallel section is 2.5 mm, not 2.0.
- **A narrow mouth**: ``side_gap`` 1.0 -> 0.45 mm. A hook leaning 36 deg leaves the mouth's tapered
  wall no more room before it is shallower than the ramp.

What it gives up, on paper: the hook's overhang now turns a lift in Y into three times the set-back
along the stroke (``y_coupling`` ratio 0.46; twinhook4 has 1.67 and asked for 1.3), and the mouth
takes a hook pitched 6 deg with 0.03 mm to spare, where twinhook4 had 0.56 mm.

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

from brixzle.families.twinhook.base import TwinHook, TwinHookParams


@dataclass(frozen=True)
class ReachParams(TwinHookParams):
    hook_lean: float = 36.0   # lean of both hooks from vertical: the stroke is down and toward -x
    hook_gap: float = 0.1     # seated gap at the hook walls (the printable floor)
    side_gap: float = 0.45    # mouth clearance on the other side of the hook, at the surface
    seat_gap: float = 0.1     # the same clearance at the hook's tip, seated (the printable floor)
    snug_L: float = 2.5       # length, from the hook's tip, over which that wall runs parallel to the stroke


BOUNDS = {
    "U": (32.0, 38.0),
    "H0": (12.0, 18.0),
    "D": (26.0, 32.0),
    "valley_u": (8.8, 10.4),
    "ramp_run": (14.5, 17.0),
    "ramp_deg": (40.0, 48.0),
    "hook_lean": (28.0, 37.0),
    "hook_L": (4.0, 6.0),
    "hook_w": (3.2, 5.0),
    "tip_chamfer": (0.4, 1.6),
    "tip_back": (0.6, 1.6),
    "hook_skew": (0.0, 1.0),
    "hook_gap": (0.0, 0.3),
    "side_gap": (0.3, 1.5),
    "seat_gap": (0.05, 0.4),
    "snug_L": (0.0, 3.0),
    "lid_t": (1.8, 3.0),
    "y_angle": (36.0, 46.0),
    "band_gap": (0.0, 0.8),
    "release_frac": (0.0, 1.0),
    "end_bevel": (0.0, 1.2),
    "wall": (1.2, 2.6),
}


class Reach(TwinHook):
    name = "fable_reach"
    version = '0.1.0'
    description = 'Reach: the twin-hook chevron with hooks leaning 36 deg and every seated gap at 0.1 mm, for the longest overhang.'
    params_type = ReachParams
    bounds = BOUNDS
    # Reach gives this up: twinhook4 asks for 1.3 and has 1.67; here the floor is 0.4.
    y_before_hooks_floor = 0.4


FAMILY = Reach.instance().family()
