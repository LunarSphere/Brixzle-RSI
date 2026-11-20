# From runs/fable/fable_wide/family.py, verified in RSI_RESULTS/fable_wide-verify-20261117.
# Changed: refactored into a subclass of twinhook/base.py's WingedTwinHook (#162): its defaults, bounds and wings. The geometry is unchanged (tests/test_twinhook_line.py).
"""Wide: the twin-hook chevron stretched to catch the drone's worst miss (the max box: an X-Y disc
of radius 38.5 mm). It breaks the mass limit on purpose: this brick needs a bigger drone than ours.

fable_twinhook4's joint and ring kit, on feather's thin body, with two changes of shape:

- **Pitch 34 -> 81 mm.** The profile must repeat every voxel, or a running bond does not nest, so
  the next valley is one pitch away and catches every drop that lands past the peak. In twinhook4's
  map that is 405 of the max box's 1100 drops. With an 81 mm pitch the valley's two faces reach
  from about -37 to +44 mm round the seat.
- **Wings.** Past about D / 2 in Y a brick's centre of mass is no longer over the course below
  (332 of the 1100 fell). Widening the whole brick to 84 mm would pass 100 g, so only its top
  surface is widened: on each side a thin plate continues the top face outward and upward at
  ``wing_deg``, carrying the same valleys and peaks, out to 42 mm from the centre line. A brick
  that lands on a wing slides down it onto the body's own V. The wings of stacked courses nest one
  course apart, like stacked trays. The ring's parts have no wings (they would cross at corners).

The guide faces are shallower than twinhook4's (ramp 34 deg, left face 33.4 deg, wings 35 deg): an
81 mm voxel with 45 deg faces would stand 40 mm tall. That is friction margin given up.

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
import math

from shapely import affinity
from shapely.geometry import Polygon

from brixzle.families import builders as B
from brixzle.families.interface import Extrusion
from brixzle.families.twinhook.base import MIN_GUIDE_DEG, TwinHookParams, WingedTwinHook, _rise


@dataclass(frozen=True)
class WideParams(TwinHookParams):
    U: float = 81.0           # voxel pitch in X (the brick is 2U long)
    H0: float = 10.5          # course pitch: vertical thickness of the corrugated body
    D: float = 22.0           # depth in Y
    valley_u: float = 20.0    # valley position from the voxel's left edge
    ramp_run: float = 40.0    # horizontal run of the ramp, valley to peak (the stroke side)
    ramp_deg: float = 34.0    # ramp angle from horizontal; the left face takes the rest of the voxel
    pocket_wall: float = 1.2  # skin left round the blind side pockets in the left-face segment (0: no pockets)
    wall: float = 1.2         # wall round the lightening holes
    wing: float = 31.0        # how far each wing reaches past the body's side (0: no wings)
    wing_deg: float = 35.0    # the wing's slope across Y, from horizontal
    wing_t: float = 1.8       # the wing's thickness, measured vertically

    def __post_init__(self):
        super().__post_init__()
        if self.wing < 0 or (self.wing > 0 and (self.wing_deg < MIN_GUIDE_DEG or wing_wall(self) < 1.2)):
            raise ValueError("a wing must slope at least 30 deg and be at least 1.2 mm thick square to its face")


BOUNDS = {
    "U": (70.0, 90.0),
    "H0": (10.0, 18.0),
    "D": (20.0, 32.0),
    "valley_u": (10.0, 30.0),
    "ramp_run": (35.0, 45.0),
    "ramp_deg": (31.0, 40.0),
    "wing": (20.0, 36.0),
    "wing_deg": (31.0, 42.0),
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


def wing_wall(p):
    """The wing's thickness square to its face (it slopes with the ramp in X and at wing_deg in Y)."""
    slope_x = math.tan(math.radians(p.ramp_deg))
    return p.wing_t / math.sqrt(1.0 + slope_x ** 2 + math.tan(math.radians(p.wing_deg)) ** 2)


def wing_rise(p):
    """How far a wing's outer edge stands above the body's top at its side."""
    return p.wing * math.tan(math.radians(p.wing_deg))


def _wings(p, core):
    """The two wings as convex pieces: the top ``wing_t`` of the uncut body ``core`` (so they carry
    its valleys, peaks and end bevel), swept outward from each side of the body and upward."""
    if p.wing <= 0:
        return []
    skin = core.difference(affinity.translate(core, 0.0, -p.wing_t))
    r0, r1, h = _rise(p), _rise(p) + wing_rise(p), p.D / 2
    ext = Extrusion(depth_mm=p.D + 2 * p.wing, bands=((-h - p.wing, -h, r1, r0), (h, h + p.wing, r0, r1)))
    out = []
    for poly in getattr(skin, "geoms", [skin]):
        if isinstance(poly, Polygon) and poly.area > 0.05:
            out += B._pieces(poly, ext)
    return out


class Wide(WingedTwinHook):
    name = "fable_wide"
    version = '0.1.0'
    description = 'Wide: the twin-hook chevron at an 81 mm pitch with thin wings out to 42 mm each side, to catch a 38.5 mm miss; over the mass limit on purpose.'
    params_type = WideParams
    bounds = BOUNDS

    def wings(self, p, core):
        return _wings(p, core)

    def wing_rise(self, p):
        return wing_rise(p)

    def wing_wall(self, p):
        return wing_wall(p)


FAMILY = Wide.instance().family()
