# From runs/fable/fable_allround/family.py, verified in RSI_RESULTS/fable_allround-20261117-opus.
# Changed: refactored into a subclass of twinhook/base.py's WingedTwinHook (#162): its defaults, bounds, two-part ribbed wings, the mouths' lead-in chamfer and a watertight check. The geometry is unchanged (tests/test_twinhook_line.py).
"""Allround: the twin-hook chevron sized for the p95 box at a weight our drone can carry, with
gentle's grip on friction.

fable_twinhook4's joint and ring kit on a thin body, with wide's two changes of shape cut down to
what 25 g allows, and gentle's two changes for friction:

- **Pitch 34 -> 58 mm.** The next valley is one pitch away, and the catchment is about half a
  pitch each way: 58 mm reaches about 29 mm, the p95 box's radius (27.8 mm) and the 3 mm aim bias.
  The faces stay steep (ramp 45 deg, left face 43 deg), so the profile stands 28 mm tall.
- **Ribbed wings to 28 mm.** On each side a plate continues the top face outward and upward: the
  first 6 mm at the body's own 50 deg, so the whole mean box lands on 50 deg faces, then 12 mm at
  40 deg. Under each wing, at every keel, peak and voxel boundary, where the body has a rib of its
  own, a 1.2 mm gusset carries the plate. The ring's parts have no wings.
- **V across Y 50 deg, mouths 1.5 mm wide with a 1 mm parallel section** (gentle's findings).
- **Body 20 mm deep, 11 mm courses, 1.2 mm walls, side pockets on** (feather's thinning), to pay
  for the wings.

Finished by Opus on 2026-11-17, after Fable's session hit its spend limit mid-edit (simulated,
uncalibrated contact; evidence in RSI_RESULTS/fable_allround-20261117-opus):

- **A 0.6 mm lead-in (``lip_chamfer``) on the hook-side lip of each mouth**, Fable's last idea,
  rebuilt as one clean polygon in the mouth's (s, t) frame. Fable's version unioned a buffered
  triangle; its rounded micro-edges reached the side-band lids and the solid would not mesh
  watertight. On the brick's own map cells the lead-in takes the mean box from 6 jams in 88 to 2,
  the X-Y jams where a pocket's lip caught a horn while the far tooth was already in its slot. At
  twinhook4's finer poses it changes the mean box from 3 jams in 180 to 2 and leaves p95 alone.
- **Tried and not kept:** moving the valley so the two crests sit evenly about the aim (ramp run 30
  to 32 mm). It traded p95 jams for wrong valleys and falls with no net gain inside the noise band.
- **What is left at p95** (all on the X-Y slice): a false V where a horn's root rubs the inner edge
  of a side-band lid (0.25 mm Y clearance) while the far tooth rests on its slot's tapered wall;
  drops near the +x crest (25 mm from the aim on that side, 33 mm on the other); and the wing tips
  at |y| ~ 27 mm. Fable found that more Y clearance costs friction margin.

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

import numpy as np
from shapely import affinity
from shapely.geometry import Polygon

from brixzle.families import builders as B
from brixzle.families.interface import Extrusion
from brixzle.families.twinhook.base import (
    MIN_GUIDE_DEG, TwinHookParams, WingedTwinHook, _bottom_z, _rise, geom,
)


@dataclass(frozen=True)
class AllroundParams(TwinHookParams):
    U: float = 58.0           # voxel pitch in X (the brick is 2U long)
    H0: float = 11.0          # course pitch: vertical thickness of the corrugated body
    D: float = 20.0           # depth in Y (the body; the wings reach further)
    valley_u: float = 22.0    # valley position from the voxel's left edge
    ramp_run: float = 28.0    # horizontal run of the ramp, valley to peak (the stroke side)
    side_gap: float = 1.5     # mouth clearance on the other side of the hook, at the surface
    snug_L: float = 1.0       # length, from the hook's tip, over which that wall runs parallel to the stroke
    pocket_wall: float = 1.2  # skin left round the blind side pockets in the left-face segment (0: no pockets)
    y_angle: float = 50.0     # V across Y, from horizontal, outside the flat centre band
    wall: float = 1.2         # wall round the lightening holes
    lip_chamfer: float = 0.6    # lead-in cut on the hook-side corner of each mouth (0: none)
    hook_y_clear: float = 0.25  # gap in Y between a seated hook's root and its slot's end, each side
    wing: float = 18.0        # how far each wing reaches past the body's side (0: no wings)
    wing_in: float = 6.0      # its first part continues the body's V at y_angle, this far
    wing_deg: float = 40.0    # the slope across Y of the rest, from horizontal
    wing_t: float = 2.0       # thickness of the outer part, measured vertically
    wing_t_in: float = 2.25   # thickness of the first part, measured vertically
    rib_d: float = 5.0        # depth of the gussets under the wings, down the body's side (0: none)

    def __post_init__(self):
        super().__post_init__()
        if self.wing < 0 or not 0.0 <= self.wing_in <= self.wing or self.rib_d < 0:
            raise ValueError("the wing's first part must lie within the wing, and rib_d must not be negative")
        if self.wing > 0 and (self.wing_deg < MIN_GUIDE_DEG or wing_wall(self) < 1.2):
            raise ValueError("a wing must slope at least 30 deg and be at least 1.2 mm thick square to its face")

    def hook_y_clear_mm(self):
        return self.hook_y_clear

    def mouth_lip_mm(self):
        return self.lip_chamfer


BOUNDS = {
    "U": (50.0, 64.0),
    "H0": (10.0, 16.0),
    "D": (19.0, 30.0),
    "valley_u": (12.0, 24.0),
    "ramp_run": (24.0, 32.0),
    "ramp_deg": (40.0, 48.0),
    "wing": (12.0, 24.0),
    "wing_deg": (32.0, 46.0),
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


def _wing_parts(p):
    """The wing's parts from the body's side outward: (y0, y1, rise at y0, rise at y1, vertical
    thickness, slope across Y in deg). Rises are above the body's top at the centre band."""
    h, r0 = p.D / 2, _rise(p)
    out = []
    if p.wing_in > 0:
        out.append((h, h + p.wing_in, r0, r0 + p.wing_in * geom(p)["k"], p.wing_t_in, p.y_angle))
    if p.wing > p.wing_in:
        y0, z0 = (out[-1][1], out[-1][3]) if out else (h, r0)
        out.append((y0, h + p.wing, z0, z0 + (p.wing - p.wing_in) * math.tan(math.radians(p.wing_deg)), p.wing_t, p.wing_deg))
    return out


def wing_wall(p):
    """The wing's thickness square to its face, at its thinnest (it slopes with the ramp in X too)."""
    slope_x = math.tan(math.radians(p.ramp_deg))
    return min((t / math.sqrt(1.0 + slope_x ** 2 + math.tan(math.radians(deg)) ** 2) for *_, t, deg in _wing_parts(p)),
               default=99.0)


def wing_rise(p):
    """How far a wing's outer edge stands above the body's top at its side."""
    parts = _wing_parts(p)
    return parts[-1][3] - _rise(p) if parts else 0.0


def _wings(p, core):
    """The two wings as convex pieces: the top of the uncut body ``core`` (so they carry its valleys,
    peaks and end bevel), swept outward from each side of the body and upward, part by part."""
    out = []
    for y0, y1, z0, z1, t, _ in _wing_parts(p):
        skin = core.difference(affinity.translate(core, 0.0, -t))
        ext = Extrusion(depth_mm=2 * y1, bands=((-y1, -y0, z1, z0), (y0, y1, z0, z1)))
        for poly in getattr(skin, "geoms", [skin]):
            if isinstance(poly, Polygon) and poly.area > 0.05:
                out += B._pieces(poly, ext)
    return out


RIB_T_MM = 1.2      # thickness of a wing gusset, along X
RIB_REACH = 0.75    # how far along the wing it runs
RIB_BITE_MM = 0.5   # how far it runs into the plate and into the body's side


def _wing_ribs(p, lay):
    """Gussets under the wings, as convex pieces: one each side at every keel, peak and voxel
    boundary, where the body has a rib of its own to root in. Each is a 1.2 mm plate in the Y-Z
    plane under the wing's underside, from ``rib_d`` down the body's side out to 3/4 of the wing.
    Stacked wings stand one course apart, so a gusset this deep stays clear of the wing below."""
    parts = _wing_parts(p)
    if not parts or p.rib_d <= 0:
        return []
    x0, xa, xb = lay["x0"], lay["xa"], lay["xb"]
    stations = [x0 + k * p.U for k in range(lay["nvox"] + 1)] + [x for x, _ in lay["keels"] + lay["peaks"]]
    h, end = p.D / 2, p.D / 2 + RIB_REACH * p.wing

    def under(y):  # the wing's underside above the body's centre-band top, at y
        for y0, y1, z0, z1, t, _ in parts:
            if y <= y1 + 1e-9:
                return z0 + (z1 - z0) * (y - y0) / (y1 - y0) - t
        return parts[-1][3] - parts[-1][4]

    ys = [h - RIB_BITE_MM] + [q[1] for q in parts if q[1] < end - 0.5] + [end]
    out = []
    for xs in sorted(set(round(min(max(x, xa + RIB_T_MM / 2), xb - RIB_T_MM / 2), 6) for x in stations)):
        top = p.H0 + _bottom_z(p, xs - x0)
        yz = [(y, top + under(max(y, h)) + RIB_BITE_MM) for y in ys] + [(h - RIB_BITE_MM, top + under(h) - p.rib_d)]
        for side in (1.0, -1.0):
            out.append(np.array([(x, side * y, z) for x in (xs - RIB_T_MM / 2, xs + RIB_T_MM / 2) for y, z in yz], dtype=float))
    return out


class Allround(WingedTwinHook):
    name = "fable_allround"
    version = '0.1.1'
    description = 'Allround: the twin-hook chevron at a 58 mm pitch with ribbed wings to 28 mm, a 50 deg V and 1.5 mm mouths: the p95 box at a weight our drone can carry.'
    params_type = AllroundParams
    bounds = BOUNDS
    # gentle's floor: twinhook4 asks for 1.3; a 50 deg V predicts 0.96 and seats better at mu 0.8.
    y_before_hooks_floor = 0.9

    def wings(self, p, core):
        return _wings(p, core)

    def wing_ribs(self, p, lay):
        return _wing_ribs(p, lay)

    def wing_rise(self, p):
        return wing_rise(p)

    def wing_wall(self, p):
        return wing_wall(p)

    def check_solid(self, shape):
        if not B._mesh(shape).is_watertight:  # wings, gussets and pockets meet at many faces; some sizes do not fuse cleanly
            raise ValueError("the brick's solid does not mesh watertight for these parameters")


FAMILY = Allround.instance().family()
