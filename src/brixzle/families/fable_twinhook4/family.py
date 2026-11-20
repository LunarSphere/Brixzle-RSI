# From runs/fable/fable_twinhook4/family.py, verified in RSI_RESULTS/fable-twinhook4-verify-20261117.
# Changed: the module is now twinhook/base.py's TwinHook, the base class of the line (#162). The geometry is unchanged (tests/test_twinhook_line.py).
"""Twin-hook chevron, fourth version: fable_twinhook3 with a printable turn piece, a stiffer joint
under load, and less mass.

What changed from fable_twinhook3, and the finding behind each change (simulated, uncalibrated
contact; runs/gate-20261116T214241210733Z-fable_twinhook3):

- **The turn piece is one solid.** Its side arm stood 5.5 mm off its corner cell (CadQuery counted
  two solids). A web now joins them (``l3_web``), rooted in both cells' side bands; the ring base is
  cut back under the corner cells so the web clears it; and every ring part must come out as one
  solid, or the kit raises.
- **A parallel section at the bottom of each mouth** (``snug_L``). The overhang collapsed at 8
  steps when the back brick b2_0 left its seat. It was not lifted by its own joint: it rode on the
  first reaching brick b1_1, which carried nearly the whole moment on one tooth (9.4 N on its hook
  wall) and had slid 0.7 mm out along its stroke. The couple that holds a reaching brick is the hook
  wall on one hook and the mouth's other wall on the tip of the other hook. That wall tapered all
  the way down, 10 deg off the stroke, and the ramp is 22 deg off it, so both pushed the brick out
  along its stroke. Over the last 2 mm the wall now runs parallel to the stroke, so the couple has
  no component along it. The clearances are fable_twinhook3's.
- **Lids, not plugs** (``lid_t``). Outside the centre band each mouth is closed by a 2 mm skin
  along the surface; its inside stays hollow.
- **Blind side pockets, off by default on the brick** (``pocket_wall``, ``brick_solid``). The
  inside of each left-face segment can be cut out of the two side bands, behind a 1.4 mm skin, open
  at the brick's side face; the centre band, where the hooks and mouths are, stays solid. It is a
  cut on the printed solid only (the collision pieces are the same with and without it), and it
  takes the brick from 19.7 g to 15.9 g. Z on the three boxes and the gate did not change
  (RSI_RESULTS/fable-twinhook4-20261117-final2), but held-out Z over four pose seeds fell from
  +0.947 to +0.917 at mu 0.2 and from +0.857 to +0.836 at mu 0.5, about 1.5 standard errors. So the
  brick ships without them. The ring's three-cell parts always have them (23.5 to 24.7 g, not 30
  to 31 g): their capture is not scored, and with a gripper they were over the 35 g payload.

The ring kit's two fixes from fable_twinhook3 are kept: the side arm's pocket is open through the
depth, and the corner cell's side band is cut back (``ring_side_trim``).

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

from brixzle.families.twinhook.base import TwinHook


FAMILY = TwinHook.instance().family()
