# From runs/fable/fable_allround/family.py (local and untracked; the two-agent design workflow of #143/#144), registered for #160.
# Changed: nothing; this header added.
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

import cadquery as cq
import numpy as np
import shapely
from shapely import affinity
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

from brixzle.engine import ring_cells as RC
from brixzle.families import builders as B
from brixzle.families import capture
from brixzle.families.interface import (
    Extrusion, Family, FamilyParams, Lattice, Placement, RingKit, Section, Solid3D, opening,
)

GUIDE_TOL_DEG = 0.1
OPENING_MIN_D_MM = 5.0
HOOK_TIP_HALF_Y = 0.4   # half-width in Y of a hook's tip
HOOK_Y_CLEAR = 0.25     # gap in Y between a seated hook's root and its slot's end
HOOK_ROOT_MM = 1.5      # how far a hook's root reaches into the body
MOUTH_BACK_MM = 8.0     # how far a mouth is cut back out of the body, along the stroke
END_BEVEL_DEG = 65.0     # angle of the bevel on the -x end's top corner
MIN_GUIDE_DEG = 30.0    # R6: atan(0.4) + 8 deg is about 29.8


@dataclass(frozen=True)
class TwinHook2Params(FamilyParams):
    U: float = 58.0           # voxel pitch in X (the brick is 2U long)
    H0: float = 11.0          # course pitch: vertical thickness of the corrugated body
    D: float = 20.0           # depth in Y (the body; the wings reach further)
    valley_u: float = 22.0    # valley position from the voxel's left edge
    ramp_run: float = 28.0    # horizontal run of the ramp, valley to peak (the stroke side)
    ramp_deg: float = 45.0    # ramp angle from horizontal; the left face takes the rest of the voxel
    hook_lean: float = 23.0   # lean of both hooks from vertical: the stroke is down and toward -x
    hook_L: float = 5.0       # hook length along its axis
    hook_w: float = 4.0       # hook thickness across its axis
    tip_chamfer: float = 0.8  # chamfer on the hook-side corner of each tip
    tip_back: float = 1.1     # how far the tip face's hook-side end is set back along the hook
    lip_chamfer: float = 0.6    # lead-in cut on the hook-side corner of each mouth (0: none)
    hook_y_clear: float = 0.25  # gap in Y between a seated hook's root and its slot's end, each side
    hook_skew: float = 0.5    # how far each flank of a hook is drawn in at its Y ends (yaw room)
    hook_gap: float = 0.08    # seated gap at the hook walls (the ramp and the left face are the stops)
    side_gap: float = 1.5     # mouth clearance on the other side of the hook, at the surface
    seat_gap: float = 0.15    # the same clearance at the hook's tip, seated
    snug_L: float = 1.0       # length, from the hook's tip, over which that wall runs parallel to the stroke
    lid_t: float = 2.0        # the mouths are closed outside the centre band by a lid this thick (0: solid plugs)
    pocket_wall: float = 1.2  # skin left round the blind side pockets in the left-face segment (0: no pockets)
    slot_extra: float = 1.0   # slot and pocket depth beyond the hook
    y_angle: float = 50.0     # V across Y, from horizontal, outside the flat centre band
    band_gap: float = 0.3     # how far a hook's tip stays inside the V's apex (sets the flat band)
    release_frac: float = 1.0 # release height: 0 just clears the seat below, 1 clears its horns at any offset
    wall: float = 1.2         # wall round the lightening holes
    wing: float = 18.0        # how far each wing reaches past the body's side (0: no wings)
    wing_in: float = 6.0      # its first part continues the body's V at y_angle, this far
    wing_deg: float = 40.0    # the slope across Y of the rest, from horizontal
    wing_t: float = 2.0       # thickness of the outer part, measured vertically
    wing_t_in: float = 2.25   # thickness of the first part, measured vertically
    rib_d: float = 5.0        # depth of the gussets under the wings, down the body's side (0: none)
    end_gap: float = 0.3      # gap between neighbours in a course
    end_bevel: float = 0.6    # height taken off the top corner of the -x end (the joint's lead-in)
    base_depth: float = 14.0  # base plate thickness below the valleys

    def __post_init__(self):
        super().__post_init__()
        g = geom(self)
        if g["left_run"] <= 4.0 or self.ramp_run <= 4.0:
            raise ValueError("the voxel needs both a ramp and a left face")
        if min(self.ramp_deg, g["left_deg"], self.y_angle) < MIN_GUIDE_DEG:
            raise ValueError(f"guide faces too shallow (ramp {self.ramp_deg:.1f}, left face {g['left_deg']:.1f}, "
                             f"Y {self.y_angle:.1f} deg)")
        if not 15.0 <= self.hook_lean <= 90.0 - self.ramp_deg - 8.0:
            raise ValueError("the hook lean must be at least 15 deg and leave 8 deg to the ramp")
        if g["flare_deg"] < self.ramp_deg:
            raise ValueError("the mouth's tapered wall would be shallower than the ramp")
        reach = (self.hook_L + self.slot_extra) * g["sin"] + (self.hook_w + self.hook_gap) * g["cos"]
        if self.valley_u - reach < 1.5 or self.valley_u + self.ramp_run + reach > self.U - 1.5:
            raise ValueError("a slot or a pocket would break through the brick's end")
        if not 0.0 <= self.hook_skew <= self.hook_w / 2 - 0.8:
            raise ValueError("hook_skew must leave 1.6 mm of hook at its Y ends")
        if self.hook_w - self.tip_chamfer < 1.2 or self.hook_L <= self.tip_chamfer + self.tip_back + 1.0:
            raise ValueError("hook tip too thin")
        if g["tip_deg"] < MIN_GUIDE_DEG:
            raise ValueError(f"the hook's tip face is too shallow ({g['tip_deg']:.1f} deg): raise tip_back")
        if self.H0 < (self.hook_L + self.slot_extra) * g["cos"] + 2 * self.wall + 2.0:
            raise ValueError("body too thin for the slot")
        if g["flat"] - self.hook_y_clear < 1.5 or self.hook_y_clear < 0 or g["flat"] > self.D / 2 - 5.0:
            raise ValueError("the flat band must hold the hook and leave 5 mm of V each side")
        if min(self.hook_gap, self.band_gap) < 0 or self.seat_gap < 0.05 or self.side_gap < self.seat_gap:
            raise ValueError("gaps must not be negative, and the mouth not narrower than the seat")
        if not 0.0 <= self.snug_L <= self.hook_L - 2.0:
            raise ValueError("snug_L must leave 2 mm of taper")
        if self.lid_t != 0.0 and not 1.7 <= self.lid_t <= 4.0:
            raise ValueError("lid_t must be 0 (solid plugs) or 1.7 to 4 mm (a 1.2 mm wall, measured square to a 45 deg face)")
        if self.pocket_wall != 0.0 and not 1.2 <= self.pocket_wall <= 4.0:
            raise ValueError("pocket_wall must be 0 (no side pockets) or at least 1.2 mm (R13)")
        if self.pocket_wall > 0 and self.lid_t == 0.0:
            raise ValueError("side pockets need lids (lid_t > 0): a solid plug would stand alone inside them")
        if self.wing < 0 or not 0.0 <= self.wing_in <= self.wing or self.rib_d < 0:
            raise ValueError("the wing's first part must lie within the wing, and rib_d must not be negative")
        if self.wing > 0 and (self.wing_deg < MIN_GUIDE_DEG or wing_wall(self) < 1.2):
            raise ValueError("a wing must slope at least 30 deg and be at least 1.2 mm thick square to its face")
        if not 0.0 <= self.end_bevel <= 1.5:
            raise ValueError("end_bevel must lie in 0 to 1.5 mm")
        if not 0.0 <= self.release_frac <= 1.0:
            raise ValueError("release_frac must lie in 0 to 1")
        if self.D > self.U - 2.0:
            raise ValueError("the depth must stay under the pitch (the ring's corners)")


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


def geom(p):
    """Derived lengths and the hook frame: ``a`` is the hook axis (down, toward -x) and ``n`` points
    across the tooth, toward its hook side (up, toward -x). ``flat`` is the half-width of the flat
    centre band: the V's faces, continued to the centre, meet ``band_gap`` below a hook's tip."""
    s, c = math.sin(math.radians(p.hook_lean)), math.cos(math.radians(p.hook_lean))
    h = p.ramp_run * math.tan(math.radians(p.ramp_deg))
    left_run = p.U - p.ramp_run
    left_deg = math.degrees(math.atan2(h, left_run)) if left_run > 0 else 0.0
    k = math.tan(math.radians(p.y_angle))
    grow = (p.side_gap - p.seat_gap) / max(p.hook_L - p.snug_L, 1e-9)
    tip_deg = p.hook_lean + math.degrees(math.atan2(p.tip_back, max(p.hook_w - p.tip_chamfer, 1e-9)))
    return dict(tip_deg=tip_deg, h=h, left_run=left_run, left_deg=left_deg, sin=s, cos=c, a=np.array([-s, -c]), n=np.array([-c, s]),
                k=k, flat=(p.hook_L * c + p.band_gap) / k, grow=grow,
                flare_deg=90.0 - p.hook_lean - math.degrees(math.atan(grow)))


def _voxels(x0, x1, U):
    return range(math.floor(x0 / U) - 1, math.ceil(x1 / U) + 1)


def _profile_pts(p, x0, x1, z=0.0):
    """The chevron (valleys at ``z``) over whole voxels covering [x0, x1], left to right."""
    g = geom(p)
    pts = []
    for k in _voxels(x0, x1, p.U):
        pts += [(k * p.U + p.valley_u, z), (k * p.U + p.valley_u + p.ramp_run, z + g["h"])]
    return pts


def _strip(p, xa, xb, bevel=True):
    """The corrugated body between x = xa and xb (brick frame: keels at z = 0), uncut. ``bevel``
    takes ``end_bevel`` off the top corner of the -x end: a tooth sliding down the left face of the
    course below crosses the joint between two bricks there, and the two tops are in line, so its
    tip caught the next brick's corner (a course-1 drop 11 mm off rested 7 mm short, at every mu)."""
    top = _profile_pts(p, xa, xb, p.H0)
    bot = _profile_pts(p, xa, xb, 0.0)
    body = _largest(Polygon(top + bot[::-1]).buffer(0).intersection(box(xa, -200, xb, 300)))
    if bevel and p.end_bevel > 0:
        g = geom(p)
        u = (xa - p.valley_u) % p.U                      # where the end cuts the voxel's profile
        if u > p.ramp_run + 1.0:                         # on the left face (it always is, for a whole voxel)
            slope = g["h"] / g["left_run"]
            zt = p.H0 + g["h"] - (u - p.ramp_run) * slope
            e = p.end_bevel / (math.tan(math.radians(END_BEVEL_DEG)) - slope)
            body = _largest(body.difference(Polygon([(xa - 1.0, zt + 1.0), (xa - 1.0, zt - p.end_bevel),
                                                     (xa, zt - p.end_bevel), (xa + e, zt - e * slope),
                                                     (xa + e, zt + 1.0)])))
    return body


def _hook(p, at, up):
    """A hook's XZ outline: the tooth hanging from keel ``at`` (``up`` False), or the horn standing
    on peak ``at`` (``up`` True), which is the tooth turned half a turn."""
    g = geom(p)
    f = -1.0 if up else 1.0
    a, n = f * g["a"], f * g["n"]
    o = np.asarray(at, dtype=float)
    L, w, ch, r, tb = p.hook_L, p.hook_w, p.tip_chamfer, HOOK_ROOT_MM, p.tip_back
    # The tip face is set back on its hook side, so it is steep enough to slide off whatever it lands
    # on: a tooth's tip on a horn's tip is face on face (first cut: 23 deg, which held at mu 0.5).
    pts = [o - r * a, o + L * a, o + (L - tb) * a + (w - ch) * n, o + (L - tb - ch) * a + w * n, o + w * n - r * a]
    return Polygon([tuple(q) for q in pts])


def _mouth(p, at, up):
    """Where a hook goes: the slot under valley ``at`` (``up`` False) or the pocket above apex ``at``
    (``up`` True). The hook wall is parallel to the stroke, ``hook_gap`` from the seated hook. The
    other wall tapers: ``seat_gap`` at the hook's tip, ``side_gap`` at the surface."""
    g = geom(p)
    f = -1.0 if up else 1.0
    a, n = f * g["a"], f * g["n"]
    o = np.asarray(at, dtype=float)
    L, back = p.hook_L, MOUTH_BACK_MM / g["cos"]
    t1 = p.hook_w + p.hook_gap
    st = [(-back, -(p.side_gap + g["grow"] * back)), (L - p.snug_L, -p.seat_gap), (L + p.slot_extra, -p.seat_gap),
          (L + p.slot_extra, t1), (-back, t1)]
    if p.lip_chamfer > 0:
        # A lead-in on the hook side: the corner where the hook wall meets the left face is cut back, so
        # a hook that arrives a little to the hook side of its mouth (a tall profile turns a small pitch
        # into that) meets a slope, not a ledge. The hook wall loses only its part above the valley.
        # (Opus, 2026-11-17: built in the mouth's own (s, t) frame, along the stroke and across it, as
        # one clean polygon. Fable's first cut unioned a buffered triangle, whose rounded micro-edges
        # carried into the side-band lids and broke the solid's mesh.)
        e = f * np.array([-math.cos(math.radians(g["left_deg"])), math.sin(math.radians(g["left_deg"]))])  # along the left face
        q, sl = np.linalg.solve(np.column_stack([e, -a]), t1 * n)        # o + q e = o + t1 n + sl a: the lip corner
        ea, en = float(e @ a), float(e @ n)                              # the left face in the (s, t) frame
        c = p.lip_chamfer
        if en <= 0 or not -back < sl < sl + c < L - p.snug_L:
            raise ValueError("the lip chamfer does not fit between the surface and the hook's seat")
        st = st[:4] + [(sl + c, t1), (sl + c * ea, t1 + c * en), (sl, t1)] + st[4:]
    return Polygon([tuple(o + s * a + t * n) for s, t in st])


def _vertical_mouth(p, at):
    """A pocket a horn can enter straight up (the ring's side arm seats without the slanted stroke)."""
    horn = _hook(p, at, up=True)
    return unary_union([horn, affinity.translate(horn, 0.0, -30.0)]).convex_hull.buffer(p.side_gap, join_style=2)


def _largest(g):
    if isinstance(g, Polygon):
        return g
    return max(getattr(g, "geoms", []), key=lambda q: q.area)


def _clean(g):
    return _largest(g.buffer(-0.01, join_style=2).buffer(0.01, join_style=2))


def _layout(p, x0, nvox, xa, xb, hooks=True, bevel=True):
    """The uncut body of ``nvox`` voxels from x0 cut to [xa, xb], and where its hooks and mouths are.
    ``hooks`` False (the ring's side arm): pockets a horn can enter straight up."""
    g = geom(p)
    keels = [(x0 + k * p.U + p.valley_u, 0.0) for k in range(nvox)]
    peaks = [(x0 + k * p.U + p.valley_u + p.ramp_run, g["h"]) for k in range(nvox)]
    slots = [_mouth(p, (x, z + p.H0), up=False) for x, z in keels]
    pockets = [_mouth(p, q, up=True) if hooks else _vertical_mouth(p, q) for q in peaks]
    return dict(core=_strip(p, xa, xb, bevel), keels=keels, peaks=peaks, slots=slots, pockets=pockets, x0=x0, nvox=nvox,
                xa=xa, xb=xb)


def _outline(p, lay, with_hooks=False):
    """The cut outline; ``with_hooks`` adds the teeth and horns in XZ (for the release lift)."""
    shape = lay["core"]
    if with_hooks:
        shape = unary_union([shape] + [_hook(p, q, up=False) for q in lay["keels"]]
                            + [_hook(p, (x, z + p.H0), up=True) for x, z in lay["peaks"]])
    return _clean(shape.difference(unary_union(lay["slots"] + lay["pockets"])))


def _holes(p, lay, outer):
    """Lightening: inside the body, clear of the slots, the pockets, the hook roots and the ribs."""
    x0, xa, xb = lay["x0"], lay["xa"], lay["xb"]
    keep = [c.buffer(p.wall, join_style=2) for c in lay["slots"] + lay["pockets"]]
    keep += [_hook(p, q, up=False).buffer(p.wall, join_style=2) for q in lay["keels"]]
    keep += [_hook(p, (x, z + p.H0), up=True).buffer(p.wall, join_style=2) for x, z in lay["peaks"]]
    keep += [box(x0 + k * p.U - p.wall / 2, -200, x0 + k * p.U + p.wall / 2, 300) for k in range(lay["nvox"] + 1)]
    keep += [box(x - p.wall / 2, -200, x + p.wall / 2, 300) for x, _ in lay["keels"] + lay["peaks"]]
    keep += [box(xa - 50, -200, xa + p.wall, 300), box(xb - p.wall, -200, xb + 50, 300)]
    inner = lay["core"].buffer(-p.wall, join_style=2).difference(unary_union(keep))
    inner = inner.buffer(-0.4, join_style=2).buffer(0.4, join_style=2)
    holes = []
    for q in getattr(inner, "geoms", [inner]):
        if isinstance(q, Polygon) and q.area > 30.0:
            q = q.simplify(0.05)
            if q.is_valid and q.within(outer):
                holes.append(q)
    return holes


def _lid_skins(p, core):
    """The layers of the uncut body a lid may occupy: (under the top, over the bottom)."""
    if p.lid_t <= 0:
        return core, core
    return (core.difference(affinity.translate(core, 0.0, -p.lid_t)),
            core.difference(affinity.translate(core, 0.0, p.lid_t)))


def _side_pockets(p, lay, wall=None):
    """Blind lightening pockets, brick only (XZ polygons, cut in the two side bands): the inside of
    each left-face segment, which the slot and the pocket leave no room to cut a through hole in.
    The centre band, where the hooks and their mouths are, stays solid; each side band is hollow
    behind a ``pocket_wall`` skin, open at the brick's side face, and runs into the mouths' hollow
    insides under the lids. The lids, the ribs at the keels, peaks and voxel boundaries, and the
    outside of the brick are untouched, so the collision geometry is too. ``wall`` overrides the
    brick's ``pocket_wall`` (the ring parts always have them)."""
    w = p.pocket_wall if wall is None else wall
    if w <= 0:
        return []
    x0, U = lay["x0"], p.U
    spans, ribs = [], []
    for k in range(-1, lay["nvox"] + 1):
        peak = x0 + k * U + p.valley_u + p.ramp_run
        spans.append(box(peak + w / 2, -200, peak + geom(p)["left_run"] - w / 2, 300))
        ribs.append(box(x0 + k * U - w / 2, -200, x0 + k * U + w / 2, 300))
    top, bottom = _lid_skins(p, lay["core"])
    lids = [c.intersection(top) for c in lay["slots"]] + [c.intersection(bottom) for c in lay["pockets"]]
    region = lay["core"].buffer(-w, join_style=2).intersection(unary_union(spans))
    region = region.difference(unary_union(ribs + [q.buffer(0.01, join_style=2) for q in lids]))
    region = region.buffer(-0.4, join_style=2).buffer(0.4, join_style=2)
    out = []
    for q in getattr(region, "geoms", [region]):
        if isinstance(q, Polygon) and q.area > 8.0:
            q = q.simplify(0.02)
            if q.is_valid and not q.is_empty:
                out.append(q)
    return out


def _brick_layout(p):
    return _layout(p, -p.U, 2, -p.U + p.end_gap / 2, p.U - p.end_gap / 2)


def section(p, part):
    """The interlock: the sheared corrugated body with its slots and pockets cut through the depth.
    The features plug them outside the centre band and add the hooks."""
    try:
        lay = _brick_layout(p)
        outer = _outline(p, lay)
        holes = _holes(p, lay, outer)
    except ValueError:
        raise
    except Exception as e:  # geometry that cannot be built
        raise ValueError(f"profile cannot be built: {e}") from e
    if not outer.is_valid or outer.area <= 0:
        raise ValueError("invalid outline")
    g = geom(p)
    angles = (p.ramp_deg, g["left_deg"], 90.0 - p.hook_lean, g["flare_deg"])
    guides = tuple(f for f in B.inclined_edges(outer)
                   if min(abs(f.angle_deg - t) for t in angles) < GUIDE_TOL_DEG
                   and math.hypot(f.a[0] - f.b[0], f.a[1] - f.b[1]) > 1.0)
    return Section(outer=outer, holes=tuple(holes), guide_faces=guides)


def _rise(p):
    g = geom(p)
    return (p.D / 2 - g["flat"]) * g["k"]


def extrusion(p):
    f, hy = geom(p)["flat"], _rise(p)
    return Extrusion(depth_mm=p.D, bands=((-p.D / 2, -f, hy, 0.0), (-f, f, 0.0, 0.0), (f, p.D / 2, 0.0, hy)))


def side_extrusion(p):
    """The two sheared side bands only: where the mouths are plugged."""
    f, hy = geom(p)["flat"], _rise(p)
    return Extrusion(depth_mm=p.D, bands=((-p.D / 2, -f, hy, 0.0), (f, p.D / 2, 0.0, hy)))


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


def _hook_set(p, at, up, xa=None, xb=None):
    """A hook as a convex vertex set: its XZ outline, and across Y the apex of the V (flanks at the
    V's angle, ``HOOK_Y_CLEAR`` inside the flat band at the root, a narrow flat at the tip)."""
    g = geom(p)
    poly = _hook(p, at, up)
    if xa is not None:
        poly = poly.intersection(box(xa, -200, xb, 300))
        if not isinstance(poly, Polygon) or poly.area < 0.5:
            return None
    xz = np.asarray(poly.exterior.coords)[:-1]
    out = (xz[:, 1] - at[1]) if up else (at[1] - xz[:, 1])   # how far past the root, toward the tip
    root = g["flat"] - p.hook_y_clear
    hw = np.maximum(HOOK_TIP_HALF_Y, root - np.clip(out, 0.0, None) / g["k"])
    # In plan the hook narrows toward its Y ends (both flanks drawn in by up to ``hook_skew``), so
    # a brick that arrives yawed does not bind its wide hooks across the mouth.
    f = -1.0 if up else 1.0
    a, n = f * g["a"], f * g["n"]
    o = np.asarray(at, dtype=float)
    s_ax, t_ax = (xz - o) @ a, (xz - o) @ n
    mid = p.hook_w / 2
    t_end = mid + (t_ax - mid) * (1.0 - 2.0 * p.hook_skew * (hw / root) / p.hook_w)
    ends = o + np.outer(s_ax, a) + np.outer(t_end, n)
    return np.vstack([np.column_stack([xz[:, 0], np.zeros(len(xz)), xz[:, 1]])]
                     + [np.column_stack([ends[:, 0], s * hw, ends[:, 1]]) for s in (-1.0, 1.0)])


def _plugs(p, core, slots, pockets=(), side=None, lid=None):
    """The mouths closed again outside the centre band, as convex pieces sheared like the body. With
    ``lid`` > 0 only a skin that thick is put back along the surface each mouth opens on (the top for
    a slot, the bottom for a pocket): the outside is the same, and the mouth's inside stays hollow.
    ``lid`` 0 fills the mouth solid. Default: the brick's ``lid_t``."""
    side = side_extrusion(p) if side is None else side
    lid = p.lid_t if lid is None else lid
    out = []
    for cuts, dz in ((slots, -lid), (pockets, lid)):
        skin = core.difference(affinity.translate(core, 0.0, dz)) if lid > 0 else core
        for c in cuts:  # (the same layers as _lid_skins, for this lid thickness)
            q = c.intersection(skin)
            for poly in getattr(q, "geoms", [q]):
                if isinstance(poly, Polygon) and poly.area > 0.05:
                    out += B._pieces(poly, side)
    return out


def _extras(p, lay, hooks=True):
    """Everything that is not the swept interlock: the lids, and (``hooks``) the teeth and horns."""
    out = _plugs(p, lay["core"], lay["slots"], lay["pockets"])
    if hooks:
        sets = [_hook_set(p, q, False, lay["xa"], lay["xb"]) for q in lay["keels"]]
        sets += [_hook_set(p, (x, z + p.H0), True, lay["xa"], lay["xb"]) for x, z in lay["peaks"]]
        out += [s for s in sets if s is not None]
    return out


def features(p, part):
    """Per voxel: the tooth, the horn, and the lids that close its slot and pocket in the side bands;
    and the two wings."""
    lay = _brick_layout(p)
    return _extras(p, lay) + _wings(p, lay["core"]) + _wing_ribs(p, lay)


_CAP = capture.CaptureGeometry(section, extrusion, features)


def brick_solid(p, part):
    """The printed brick: the interlock with its hooks and lids (``_CAP``), with the blind side
    pockets cut out of its two side bands. One solid, or ValueError (R13)."""
    shape = _CAP.solid(p, part)
    side = side_extrusion(p)
    for q in _side_pockets(p, _brick_layout(p)):
        shape = shape.cut(B._swept(q, [], side))
    shape = shape.clean()
    count = len(shape.Solids())
    if count != 1:
        raise ValueError(f"the brick would be {count} separate solids, not one")
    if not B._mesh(shape).is_watertight:  # wings, gussets and pockets meet at many faces; some sizes do not fuse cleanly
        raise ValueError("the brick's solid does not mesh watertight for these parameters")
    return shape


def pickup(p, part):
    """The carrying openings (R10): the through-Y lightening holes a 5 mm circle fits in. The blind
    side pockets are not through, so they do not count."""
    lay = _brick_layout(p)
    through = _holes(p, lay, _outline(p, lay))
    wide = [h for h in through if 2 * shapely.maximum_inscribed_circle(h).length >= OPENING_MIN_D_MM]
    # The long ramp segment has several lightening holes, and the wings lift the centre of mass:
    # the carrying openings are the highest wide hole on each side of the brick's middle.
    sides = [[h for h in wide if (h.centroid.x < 0) == left] for left in (True, False)]
    if not all(sides):
        raise ValueError("fewer than two openings a 5 mm circle fits in, one each side of the middle")
    return tuple(opening(max(hs, key=lambda h: h.bounds[3])) for hs in sides)


def lattice(p):
    return Lattice(U_mm=p.U, D_mm=p.D, H0_mm=p.H0, span_voxels=2)


def part_for(course, direction):
    return "A"


def seat_pose(p, b: Placement):
    pos = np.array([(b.i0 + 1) * p.U, 0.0, b.course * p.H0])
    quat = np.array([1.0, 0, 0, 0]) if b.direction > 0 else np.array([0.0, 0, 0, 1])
    return pos, quat


def insertion_axis(p, b: Placement):
    a = geom(p)["a"]
    return a if b.direction > 0 else a * np.array([-1.0, 1.0])


_LIFT_CACHE = {}


def release_lift_mm(p):
    """Release height above the seat. At least the smallest lift (plus 1 mm) at which the brick,
    straight above its seat, clears the course below; with ``release_frac`` 1, high enough for its
    teeth to clear the horns below wherever it is released (a drop 14 mm off in X otherwise starts
    inside the ramp of the course below)."""
    if p in _LIFT_CACHE:
        return _LIFT_CACHE[p]
    g = geom(p)
    over_horns = g["h"] + 2 * p.hook_L * g["cos"] + 1.0 + (_rise(p) + wing_rise(p) if p.wing > 0 else 0.0)
    upper = _outline(p, _brick_layout(p), with_hooks=True)
    lower = _outline(p, _layout(p, -3 * p.U, 6, -3 * p.U, 3 * p.U), with_hooks=True)
    lower = affinity.translate(lower, 0.0, -p.H0)
    lo, hi = 0.0, 60.0
    for _ in range(30):
        mid = (lo + hi) / 2
        if affinity.translate(upper, 0.0, mid).intersection(lower).area > 1e-6:
            lo = mid
        else:
            hi = mid
    nest = hi + 1.0
    _LIFT_CACHE[p] = nest + p.release_frac * max(0.0, over_horns - nest)
    return _LIFT_CACHE[p]


def stroke_mm(p):
    """The slanted final stroke: the hook and its slot clearance."""
    return p.hook_L + p.slot_extra


def _mirror(pieces, X):
    return [np.column_stack([X - v[:, 0], v[:, 1], v[:, 2]]) for v in pieces]


def base(p, voxels, direction):
    """The anchored base: a course's top (valleys at z = 0), with slots in the centre band and a horn
    on each peak, as exact convex pieces sheared like the bricks."""
    pieces = _base_pieces(p, voxels, extrusion(p), side_extrusion(p), wings=True)
    return _mirror(pieces, voxels * p.U) if direction < 0 else pieces


def _base_pieces(p, voxels, ext, side, wings=False):
    g = geom(p)
    X = voxels * p.U
    top = _profile_pts(p, 0.0, X, 0.0)
    plate = _largest(Polygon(top + [(top[-1][0], -p.base_depth), (top[0][0], -p.base_depth)]).buffer(0)
                     .intersection(box(0.0, -p.base_depth, X, 300)))
    slots = [_mouth(p, (k * p.U + p.valley_u, 0.0), up=False) for k in range(voxels)]
    peaks = [(k * p.U + p.valley_u + p.ramp_run, g["h"]) for k in range(voxels)]
    outer = _clean(plate.difference(unary_union(slots)))
    pieces = B._pieces(outer, ext) + _plugs(p, plate, slots, side=side, lid=0.0)
    pieces += [s for s in (_hook_set(p, q, True, 0.0, X) for q in peaks) if s is not None]
    if wings:
        pieces += _wings(p, plate)
    return pieces


# --- the ring kit: engine/ring_cells' cells, with this brick's centre-band hooks and plugs ---


def _cell_layout(p, hooks, exposed_lo, exposed_hi, cut_hi):
    xa = p.end_gap / 2 if exposed_lo else 0.0
    xb = (p.U - p.end_gap / 2 if exposed_hi else p.U) - cut_hi
    return _layout(p, 0.0, 1, xa, xb, hooks=hooks, bevel=exposed_lo)


def cell(p, part="A", hooks=True, exposed_lo=False, exposed_hi=False, cut_hi=0.0):
    """One voxel (x in [0, U]) centred on x = 0: the interlock's section (no hooks, no plugs)."""
    lay = _cell_layout(p, hooks, exposed_lo, exposed_hi, cut_hi)
    outer = _outline(p, lay)
    sh = lambda q: affinity.translate(q, -p.U / 2, 0.0)  # noqa: E731
    return Section(outer=sh(outer), holes=tuple(sh(h) for h in _holes(p, lay, outer)))


RING_APPROACH_MM = 5.5  # room a straight brick's upstream end needs beside a corner: the 3 mm aim bias, and margin


def ring_side_trim(p):
    """How far the corner cell's side band is cut back on the side the last wall arrives from. That
    wall's straight bricks are released 3 mm upstream (the aim bias), with their end over the
    corner cell of the course below; the cells stand (U - D) / 2 apart, which is 2 mm here."""
    return max(0.0, RING_APPROACH_MM - (p.U - p.D) / 2)


def _ring_bands(p, trim):
    """The cell's sweep with its +y side band cut back by ``trim``: (all three bands, side bands only)."""
    f, k = geom(p)["flat"], geom(p)["k"]
    hi = p.D / 2 - trim
    if hi < f + 2.0:
        raise ValueError("the ring's corner cell would lose its side band")
    lo_band, flat, hi_band = (-p.D / 2, -f, _rise(p), 0.0), (-f, f, 0.0, 0.0), (f, hi, 0.0, (hi - f) * k)
    return Extrusion(depth_mm=p.D, bands=(lo_band, flat, hi_band)), Extrusion(depth_mm=p.D, bands=(lo_band, hi_band))


def cell_extras(p, part="A", hooks=True, exposed_lo=False, exposed_hi=False, cut_hi=0.0, trim=0.0):
    """The same voxel's plugs and (``hooks``) its tooth and horn, centred on x = 0. ``trim`` cuts
    the +y side band back (``_ring_bands``).

    ``hooks`` False (the turn piece's side arm): the slot is plugged as on the brick, but the pocket
    is left open through the whole depth. The arm runs along the piece's Y, so the piece's slanted
    stroke carries the arm sideways, along its own depth, over the horn below. A pocket plugged
    outside the centre band leaves that horn 0.25 mm of room each way, and the arm hung on it."""
    lay = _cell_layout(p, hooks, exposed_lo, exposed_hi, cut_hi)
    # Solid plugs, not lids: the turn piece's web is rooted in the corner cell's side band.
    out = _plugs(p, lay["core"], lay["slots"], lay["pockets"] if hooks else (), side=_ring_bands(p, trim)[1], lid=0.0)
    if hooks:
        sets = [_hook_set(p, q, False, lay["xa"], lay["xb"]) for q in lay["keels"]]
        sets += [_hook_set(p, (x, z + p.H0), True, lay["xa"], lay["xb"]) for x, z in lay["peaks"]]
        out += [v for v in sets if v is not None]
    return [v - np.array([p.U / 2, 0.0, 0.0]) for v in out]


def _bottom_z(p, vx):
    """Height of the chevron (keels at 0) at voxel coordinate ``vx``."""
    g = geom(p)
    u = (vx - p.valley_u) % p.U
    return u * g["h"] / p.ramp_run if u <= p.ramp_run else g["h"] - (u - p.ramp_run) * g["h"] / g["left_run"]


RING_POCKET_WALL_MM = 1.2  # the ring parts' side pockets: three cells in one piece must stay under the payload
WEB_W_MM = 3.5      # the turn piece's web: width along the piece's x
WEB_ROOT_MM = 1.5   # how far it runs into the corner cell and into the arm
WEB_MIN_H_MM = 3.0
WEB_MAX_H_MM = 5.5


def _web_place(p):
    """Where the turn piece's web goes, in the piece's frame: (x0, x1, y0, edge, (zlo, zhi) in the
    corner cell, U / 2, y1, (zlo, zhi) in the arm), or ValueError. It is tried on both sides of the
    centre band (the corner cell is built without its through hole, so it is solid there). The two
    cells need not be solid at the same height, so the web may slope between them; the place where
    it slopes least wins."""
    g = geom(p)
    f, k, trim = g["flat"], g["k"], ring_side_trim(p)
    edge = p.D / 2 - trim
    y0, y1 = edge - WEB_ROOT_MM, p.U / 2 + WEB_ROOT_MM
    arm_b = [_bottom_z(p, 1.5 * p.U - y) for y in (p.U / 2, y1)]
    peak = p.valley_u + p.ramp_run
    best = None
    for sign in (1.0, -1.0):
        a = f + 1.0
        while a + WEB_W_MM <= p.D / 2 - 0.25:
            xs = [sign * (a + t * WEB_W_MM / 8) for t in range(9)]
            vs = [x + p.U / 2 for x in xs]
            if True:  # the turn piece's corner cell has no through hole here (see _build), so any place will do
                cb = [_bottom_z(p, v) for v in vs]
                corner = (max(cb) + k * (edge - f) + 0.4, min(cb) + k * (y0 - f) + p.H0 - 0.4)
                arm = (max(arm_b) + k * (a + WEB_W_MM - f) + 0.4, min(arm_b) + k * (a - f) + p.H0 - 1.5)
                if min(corner[1] - corner[0], arm[1] - arm[0]) >= WEB_MIN_H_MM:
                    # Take up to WEB_MAX_H_MM of each window, as near the other's height as it allows.
                    hc, ha = min(WEB_MAX_H_MM, corner[1] - corner[0]), min(WEB_MAX_H_MM, arm[1] - arm[0])
                    mid = 0.5 * (0.5 * (corner[0] + corner[1]) + 0.5 * (arm[0] + arm[1]))
                    c0 = min(max(mid - hc / 2, corner[0]), corner[1] - hc)
                    a0 = min(max(mid - ha / 2, arm[0]), arm[1] - ha)
                    step = abs((c0 + hc / 2) - (a0 + ha / 2))
                    if best is None or step < best[0]:
                        best = (step, (min(xs), max(xs), y0, edge, (c0, c0 + hc), p.U / 2, y1, (a0, a0 + ha)))
            a += 0.25
    if best is None:
        raise ValueError("no room for the turn piece's web: the corner cell and the arm are not both solid "
                         f"over {WEB_MIN_H_MM} mm of height anywhere beside the centre band")
    return best[1]


def _web_outline(p):
    """(x0, x1, the web's convex Y-Z outline as an (n, 2) array, counter-clockwise)."""
    from scipy.spatial import ConvexHull

    x0, x1, y0, edge, (c0, c1), ya, y1, (a0, a1) = _web_place(p)
    yz = np.array([(y0, c0), (y0, c1), (edge, c0), (edge, c1), (ya, a0), (ya, a1), (y1, a0), (y1, a1)], dtype=float)
    return x0, x1, yz[ConvexHull(yz).vertices]


def l3_web(p):
    """The web that joins the turn piece's side arm to its corner cell, as a convex vertex set in the
    piece's frame (corner cell centred on the origin, side arm along +y): a block rooted in each
    cell's side band, and the span between them. The cells are U long and D deep, so without it the
    arm stands off the corner cell and the piece is two solids. Its top stays 1.5 mm under a
    straight brick that is released over the arm with its end over the gap, and its roots lie
    inside the two cells."""
    x0, x1, yz = _web_outline(p)
    return np.array([(x, y, z) for x in (x0, x1) for y, z in yz], dtype=float)


def l3_web_solid(p):
    """The same web as a CadQuery solid: its Y-Z outline pushed along X."""
    x0, x1, yz = _web_outline(p)
    wire = cq.Wire.makePolygon([cq.Vector(float(x0), float(y), float(z)) for y, z in yz], close=True)
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), cq.Vector(float(x1 - x0), 0.0, 0.0))


class HookCellKit(RC.CellKit):
    """engine/ring_cells.CellKit whose cells also carry ``cell_extras``: kit_from_cell sweeps a cell's
    section through the whole depth, and this brick's hooks and plugs do not span it. Each cell's
    printed solid has the brick's blind side pockets. The corner cell
    of the turn piece (L3) and of the corner straight (I3) has its side band cut back on the side
    the last wall arrives from (``ring_side_trim``)."""

    def _build(self, p, name):
        key = (p, name)
        if key in self._cache:
            return self._cache[key]
        if name not in self.names:
            raise ValueError(f"no ring part {name!r}; the kit has {self.names}")
        kind, part = name[:2], name[2:]
        U = self.family.lattice(p).U_mm
        cells = RC.part_cells(kind, self.travel(p))
        pieces, shape = [], None
        for c in cells:
            lo, hi = RC._exposed(c, cells)
            # The corner cell (local 0, 0): the last wall's final cell lies on its local +y side.
            trim = ring_side_trim(p) if kind in ("L3", "I3") and (c.i, c.j) == (0, 0) else 0.0
            ext = _ring_bands(p, trim)[0]
            sec = cell(p, part, hooks=c.hooks, exposed_lo=lo, exposed_hi=hi, cut_hi=c.cut_hi)
            extras = cell_extras(p, part, hooks=c.hooks, exposed_lo=lo, exposed_hi=hi, cut_hi=c.cut_hi, trim=trim)
            R = RC._yaw_matrix(c.yaw)
            t = np.array([c.i * U, c.j * U, 0.0])
            pieces += [v @ R.T + t for v in B._pieces(sec.outer, ext) + extras]
            # At this pitch the web meets the corner cell on its ramp segment, where the lightening hole
            # would be: the turn piece's corner cell is built without it.
            holes = [] if kind == "L3" and (c.i, c.j) == (0, 0) else list(sec.holes)
            solid = B._swept(sec.outer, holes, ext)
            for v in extras:
                solid = solid.fuse(capture.hull_solid(v))
            # The brick's blind side pockets, on the printed solid only (the collision keeps its solid
            # plugs). The turn piece's corner cell and side arm keep their +y band whole: the web roots there.
            bands = _ring_bands(p, trim)[1].bands
            if kind == "L3" and (c.i, c.j) == (0, 0):
                bands = bands[:1]            # the corner cell: the web is in its +y band
            elif kind == "L3" and (c.i, c.j) == (0, 1):
                bands = bands[:1] if _web_place(p)[0] > 0 else bands[1:]   # the arm: its band on the web's side
            lay = _cell_layout(p, c.hooks, lo, hi, c.cut_hi)
            for q in _side_pockets(p, lay, wall=RING_POCKET_WALL_MM):
                solid = solid.cut(B._swept(affinity.translate(q, -U / 2, 0.0), [], Extrusion(depth_mm=p.D, bands=bands)))
            solid = solid.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), 90 * (c.yaw % 4)).translate(cq.Vector(*t))
            shape = solid if shape is None else shape.fuse(solid)
        if kind == "L3":
            pieces.append(l3_web(p))
            shape = shape.fuse(l3_web_solid(p))
        shape = shape.clean()
        count = len(shape.Solids())
        if count != 1:  # R13: a ring part prints as one piece
            raise ValueError(f"ring part {name} is {count} separate solids, not one")
        self._cache[key] = (shape, pieces)
        return self._cache[key]

    def base(self, p, n):
        """The ring's base, a base voxel under every course-0 cell. Under a corner cell its side band
        is cut back like the cell's, so the turn piece's web clears it."""
        U = self.family.lattice(p).U_mm
        corners = {(0, 0), (n - 1, 0), (n - 1, n - 1), (0, n - 1)}
        out = []
        for (ci, cj), yaw in RC.course0_yaws(RC.ring(n, 1)).items():
            ext, side = _ring_bands(p, ring_side_trim(p) if (ci, cj) in corners else 0.0)
            R = RC._yaw_matrix(yaw)
            t = np.array([(ci + .5) * U, (cj + .5) * U, 0.])
            out += [(v - np.array([U / 2, 0., 0.])) @ R.T + t for v in _base_pieces(p, 1, ext, side)]
        return out


# --- rules ---


def hook_levers(p):
    """Predicted, quasi-static 2D. The lever (mm) of each hook wall's normal about the pivot of a
    half-supported brick: the horn about the supporter's -x end (a brick reaching toward -x) and the
    tooth about its +x end (a brick reaching toward +x). Positive means the hook holds the brick down."""
    g = geom(p)
    e = np.array([g["sin"], g["cos"]])           # across the hook normal: lever = (C - P) . e
    mid = 0.5 * (p.hook_L - p.tip_chamfer - p.tip_back)
    left_z = g["h"] * p.valley_u / g["left_run"]                   # the left face's height at the voxel's left edge
    right_z = g["h"] * (1.0 - (p.U - p.valley_u - p.ramp_run) / g["left_run"])  # and at its right edge
    tooth = np.array([p.valley_u, 0.0]) + mid * g["a"] + p.hook_w * g["n"]
    horn = np.array([p.valley_u + p.ramp_run, g["h"]]) - mid * g["a"] - p.hook_w * g["n"]
    return {"horn_about_left_end": float((horn - np.array([0.0, left_z])) @ e),
            "tooth_about_right_end": float((np.array([p.U, right_z]) - tooth) @ e)}


def y_coupling(p):
    """Predicted, quasi-static. A brick off centre by dy rides k dy higher on the V across Y, and the
    hook's overhang then needs it set back up the ramp by ``need`` mm per mm of dy. Sliding down the
    ramp and the V together, it stays ``slide`` mm up the ramp per mm of dy. slide / need > 1 means
    the hooks stay clear of their hook walls until Y has closed."""
    g = geom(p)
    t = math.radians(p.ramp_deg)
    need = g["k"] * g["sin"] / math.cos(t + math.radians(p.hook_lean))
    slide = math.sin(t) / (g["k"] * math.cos(t) ** 2)
    return {"need": need, "slide": slide, "ratio": slide / need}


def extra_rules(p, part, bodies):
    g = geom(p)
    lv = hook_levers(p)
    reach = (p.hook_L + p.slot_extra) * g["sin"] + (p.hook_w + p.hook_gap) * g["cos"]
    return {
        # R4: each hook's normal passes inside the pivot by at least 8 mm (predicted, quasi-static 2D).
        "R4.horn_holds_minus_x": lv["horn_about_left_end"] - 8.0,
        "R4.tooth_holds_plus_x": lv["tooth_about_right_end"] - 8.0,
        # R4: a straight lift is blocked by the undercut of the hook wall.
        "R4.hook_depth": (p.hook_L - p.tip_chamfer - p.tip_back) * g["sin"] - 1.0,
        # R6: a hook's tip face is a guide face too (it can land on a tip of the course below).
        "R6.tip_slides": g["tip_deg"] - (math.degrees(math.atan(0.4)) + 8.0),
        # R4: seated play of a half-supported brick before a hook bears, under 0.5 deg.
        "R4.seated_play": 0.5 - math.degrees(math.atan2(p.hook_gap, min(lv.values()))),
        # R11: the stroke lies inside the wedge between the ramp and the hook walls.
        "R11.nest_cone": 90.0 - p.ramp_deg - p.hook_lean - 8.0,
        # R11: the brick's own slide keeps the hooks off their walls until Y has closed (predicted).
        # (gentle's floor: twinhook4 asks for 1.3; a 50 deg V predicts 0.96 and seats better at mu 0.8.)
        "R11.y_before_hooks": y_coupling(p)["ratio"] - 0.9,
        # R11: the mouth takes a hook pitched 6 deg (A/B's rule).
        "R11.mouth_takes_tilt": p.side_gap + p.hook_gap - p.hook_L * math.sin(math.radians(6.0)),
        # R13: printable hook tips and walls at the brick's ends.
        "R13.hook_tip": p.hook_w - p.tip_chamfer - 1.2,
        # R6 and R13: the wings are guide faces, and walls.
        "R6.wing_slides": (p.wing_deg - (math.degrees(math.atan(0.4)) + 8.0)) if p.wing > 0 else 1.0,
        "R13.wing_wall": (wing_wall(p) - 1.2) if p.wing > 0 else 1.0,
        "R13.end_wall": min(p.valley_u - reach, p.U - p.valley_u - p.ramp_run - reach) - 1.2,
    }


FAMILY = Family(
    name="fable_allround",
    version="0.1.1",
    description="Allround: the twin-hook chevron at a 58 mm pitch with ribbed wings to 28 mm, a 50 deg V and "
                "1.5 mm mouths: the p95 box at a weight our drone can carry.",
    params_type=TwinHook2Params,
    bounds=BOUNDS,
    parts=("A",),
    geometry=Solid3D(solid=brick_solid, collision=_CAP.collision, section=_CAP.section),
    lattice=lattice,
    part_for=part_for,
    seat_pose=seat_pose,
    insertion_axis=insertion_axis,
    release_lift_mm=release_lift_mm,
    base=base,
    pickup=pickup,
    extra_rules=extra_rules,
)


def _with_ring(family):
    from dataclasses import replace

    k = HookCellKit(family, cell, stroke_mm, None)
    kit = RingKit(parts=k.names, solid=RC._Call(k, "solid"), plan=RC._Call(k, "plan"), base=RC._Call(k, "base"),
                  collision=RC._Call(k, "collision"))
    return replace(family, ring=kit)


FAMILY = _with_ring(FAMILY)
