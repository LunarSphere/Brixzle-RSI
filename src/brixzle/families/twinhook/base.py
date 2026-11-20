# From runs/fable/fable_twinhook4/family.py, verified in RSI_RESULTS/fable-twinhook4-verify-20261117; the
# web from runs/fable/fable_gentle/family.py and the wings' base, kit and pickup from runs/fable/fable_wide/family.py.
# Changed: the twinhook line refactored into a class hierarchy (#162). The joint's geometry is these module
# functions, copied verbatim, except that the mouth's lead-in chamfer and the hook's Y clearance (fable_allround's)
# are read from the parameters and are off by default. Every branch's geometry is unchanged
# (tests/test_twinhook_line.py compares each with the module it came from, exactly).
"""The twin-hook chevron line: the joint every branch shares, and the families as classes (#162).

The brick is ``fable_twinhook4``'s (its module docstring tells the design history): a chevron with one
valley per voxel, a tooth under each keel and a mirrored horn on each peak, both leaning the same
way and seated with one slanted stroke; centre-band mouths with a parallel seat section; lids, blind
side pockets, and a ring kit built from single cells.

- **The shared joint** is the module functions below: the voxel profile, the hooks, the mouths, the
  lids and pockets, the layout of a brick or a ring cell, and the rules' predictions.
- **A branch** is a subclass of ``TwinHook`` with its own parameter defaults (a ``TwinHookParams``
  subclass) and bounds, overriding only what it changes:

  - ``TwinHook``: twinhook4 itself, and ``fable_reach`` (a 36 deg hook lean; its own rule floor).
  - ``WebbedTwinHook``: a turn piece whose web is placed by search and may slope (``fable_gentle``,
    ``fable_feather``).
  - ``WingedTwinHook``: wings on the brick and the base, the highest opening each side as the
    carrying openings, a release that clears the wings, and a solid turn-piece corner cell
    (``fable_wide``, ``fable_allround``).

A family is ``SomeBranch.instance().family()``. The instance is one per class and pickles by
reference, so the family can be sent to the sim's worker processes.
"""



from dataclasses import dataclass, replace
import importlib
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
class TwinHookParams(FamilyParams):
    """twinhook4's parameters. A branch subclasses them to change defaults or add its own."""

    U: float = 34.0           # voxel pitch in X (the brick is 2U long)
    H0: float = 12.5          # course pitch: vertical thickness of the corrugated body
    D: float = 30.0           # depth in Y
    valley_u: float = 9.6     # valley position from the voxel's left edge
    ramp_run: float = 16.0    # horizontal run of the ramp, valley to peak (the stroke side)
    ramp_deg: float = 45.0    # ramp angle from horizontal; the left face takes the rest of the voxel
    hook_lean: float = 23.0   # lean of both hooks from vertical: the stroke is down and toward -x
    hook_L: float = 5.0       # hook length along its axis
    hook_w: float = 4.0       # hook thickness across its axis
    tip_chamfer: float = 0.8  # chamfer on the hook-side corner of each tip
    tip_back: float = 1.1     # how far the tip face's hook-side end is set back along the hook
    hook_skew: float = 0.5    # how far each flank of a hook is drawn in at its Y ends (yaw room)
    hook_gap: float = 0.08    # seated gap at the hook walls (the ramp and the left face are the stops)
    side_gap: float = 1.0     # mouth clearance on the other side of the hook, at the surface
    seat_gap: float = 0.15    # the same clearance at the hook's tip, seated
    snug_L: float = 2.0       # length, from the hook's tip, over which that wall runs parallel to the stroke
    lid_t: float = 2.0        # the mouths are closed outside the centre band by a lid this thick (0: solid plugs)
    pocket_wall: float = 0.0  # skin left round blind side pockets in the left-face segment: 0, none (the
                              # default); 1.4 gives a 15.9 g brick, at a small cost in held-out Z (see above)
    slot_extra: float = 1.0   # slot and pocket depth beyond the hook
    y_angle: float = 42.0     # V across Y, from horizontal, outside the flat centre band
    band_gap: float = 0.3     # how far a hook's tip stays inside the V's apex (sets the flat band)
    release_frac: float = 1.0 # release height: 0 just clears the seat below, 1 clears its horns at any offset
    wall: float = 1.4         # wall round the lightening holes
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
        if g["flat"] - self.hook_y_clear_mm() < 1.5 or self.hook_y_clear_mm() < 0 or g["flat"] > self.D / 2 - 5.0:
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
        if not 0.0 <= self.end_bevel <= 1.5:
            raise ValueError("end_bevel must lie in 0 to 1.5 mm")
        if not 0.0 <= self.release_frac <= 1.0:
            raise ValueError("release_frac must lie in 0 to 1")
        if self.D > self.U - 2.0:
            raise ValueError("the depth must stay under the pitch (the ring's corners)")

    def hook_y_clear_mm(self):
        """The gap in Y between a seated hook's root and its slot's end, each side."""
        return HOOK_Y_CLEAR

    def mouth_lip_mm(self):
        """The lead-in chamfer on the hook-side corner of each mouth (0: none)."""
        return 0.0


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
    "y_angle": (36.0, 46.0),
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
    if p.mouth_lip_mm() > 0:
        # A lead-in on the hook side: the corner where the hook wall meets the left face is cut back, so
        # a hook that arrives a little to the hook side of its mouth (a tall profile turns a small pitch
        # into that) meets a slope, not a ledge. The hook wall loses only its part above the valley.
        # (Opus, 2026-11-17: built in the mouth's own (s, t) frame, along the stroke and across it, as
        # one clean polygon. Fable's first cut unioned a buffered triangle, whose rounded micro-edges
        # carried into the side-band lids and broke the solid's mesh.)
        e = f * np.array([-math.cos(math.radians(g["left_deg"])), math.sin(math.radians(g["left_deg"]))])  # along the left face
        q, sl = np.linalg.solve(np.column_stack([e, -a]), t1 * n)        # o + q e = o + t1 n + sl a: the lip corner
        ea, en = float(e @ a), float(e @ n)                              # the left face in the (s, t) frame
        c = p.mouth_lip_mm()
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
    root = g["flat"] - p.hook_y_clear_mm()
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


def stroke_mm(p):
    """The slanted final stroke: the hook and its slot clearance."""
    return p.hook_L + p.slot_extra


def _mirror(pieces, X):
    return [np.column_stack([X - v[:, 0], v[:, 1], v[:, 2]]) for v in pieces]


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


WEB_W_MM = 3.5      # the turn piece's web: width along the piece's x
WEB_ROOT_MM = 1.5   # how far it runs into the corner cell and into the arm
WEB_MIN_H_MM = 3.0
WEB_MAX_H_MM = 5.5


# --- rules (predicted, quasi-static 2D) ---


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


# --- the families ---


def _singleton(module, qualname):
    """Unpickle a branch as its class's one instance."""
    obj = importlib.import_module(module)
    for name in qualname.split("."):
        obj = getattr(obj, name)
    return obj.instance()


class TwinHook:
    """twinhook4 as a family. A branch subclasses it, sets its own name, parameters and bounds, and
    overrides only the methods it changes. Geometry every branch shares is the module's functions."""

    name = "fable_twinhook4"
    version = "0.4.0"
    description = ("Twin-hook chevron v4: tooth and mirrored horn that are the apex of the V across Y, in "
                   "centre-band mouths with a parallel seat section; lids and blind side pockets; 34 mm pitch.")
    params_type = TwinHookParams
    bounds = BOUNDS
    ring_pocket_wall_mm = 1.4    # the ring parts' side pockets: three cells in one piece must stay under the payload
    y_before_hooks_floor = 1.3   # R11.y_before_hooks: the predicted slide / need ratio the brick must keep

    _instances = {}

    @classmethod
    def instance(cls):
        inst = TwinHook._instances.get(cls)
        if inst is None:
            inst = TwinHook._instances[cls] = cls()
        return inst

    def __init__(self):
        self._lift_cache = {}
        self.cap = capture.CaptureGeometry(section, extrusion, self.features)

    def __reduce__(self):
        return (_singleton, (type(self).__module__, type(self).__qualname__))

    # --- the brick ---

    def wings(self, p, core):
        """Convex pieces added to each side of the uncut body ``core`` (none here)."""
        return []

    def wing_ribs(self, p, lay):
        """Gussets under the wings (none here)."""
        return []

    def features(self, p, part):
        """Per voxel: the tooth, the horn, and the lids that close its slot and pocket in the side bands;
        and any wings."""
        lay = _brick_layout(p)
        return _extras(p, lay) + self.wings(p, lay["core"]) + self.wing_ribs(p, lay)

    def check_solid(self, shape):
        """A branch's own check on the printed solid; raise ValueError to refuse it."""

    def brick_solid(self, p, part):
        """The printed brick: the interlock with its hooks and lids (``cap``), with the blind side
        pockets cut out of its two side bands. One solid, or ValueError (R13)."""
        shape = self.cap.solid(p, part)
        side = side_extrusion(p)
        for q in _side_pockets(p, _brick_layout(p)):
            shape = shape.cut(B._swept(q, [], side))
        shape = shape.clean()
        count = len(shape.Solids())
        if count != 1:
            raise ValueError(f"the brick would be {count} separate solids, not one")
        self.check_solid(shape)
        return shape

    def pickup(self, p, part):
        """The carrying openings (R10): the through-Y lightening holes a 5 mm circle fits in. The blind
        side pockets are not through, so they do not count."""
        lay = _brick_layout(p)
        through = _holes(p, lay, _outline(p, lay))
        wide = [h for h in through if 2 * shapely.maximum_inscribed_circle(h).length >= OPENING_MIN_D_MM]
        if len(wide) < 2:
            raise ValueError("fewer than two openings a 5 mm circle fits in")
        return tuple(opening(h) for h in sorted(wide, key=lambda h: h.centroid.x))

    def wing_clearance(self, p):
        """Extra release height to clear anything standing above the horns below (nothing here)."""
        return 0.0

    def release_lift_mm(self, p):
        """Release height above the seat. At least the smallest lift (plus 1 mm) at which the brick,
        straight above its seat, clears the course below; with ``release_frac`` 1, high enough for its
        teeth to clear the horns below wherever it is released (a drop 14 mm off in X otherwise starts
        inside the ramp of the course below)."""
        if p in self._lift_cache:
            return self._lift_cache[p]
        g = geom(p)
        over_horns = g["h"] + 2 * p.hook_L * g["cos"] + 1.0 + self.wing_clearance(p)
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
        self._lift_cache[p] = nest + p.release_frac * max(0.0, over_horns - nest)
        return self._lift_cache[p]

    def base(self, p, voxels, direction):
        """The anchored base: a course's top (valleys at z = 0), with slots in the centre band and a horn
        on each peak, as exact convex pieces sheared like the bricks; and the brick's wings, if any."""
        pieces = self.base_pieces(p, voxels, extrusion(p), side_extrusion(p), wings=True)
        return _mirror(pieces, voxels * p.U) if direction < 0 else pieces

    def base_pieces(self, p, voxels, ext, side, wings=False):
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
            pieces += self.wings(p, plate)
        return pieces

    # --- the ring kit ---

    def l3_web(self, p):
        """The web that joins the turn piece's side arm to its corner cell, as a box's vertex set in the
        piece's frame (corner cell centred on the origin, side arm along +y). The cells are U long and D
        deep, and the corner cell's side band is cut back, so without it the arm stands off the corner
        cell and the piece is two solids. It sits in both cells' side bands, where they are solid, near
        the corner cell's +x end; its top stays 1.5 mm under a straight brick that is released over the
        arm with its end over the gap, and its bottom above both cells' undersides."""
        g = geom(p)
        f, k, trim = g["flat"], g["k"], ring_side_trim(p)
        x1 = min(p.U / 2 - 2.25, p.D / 2 - 0.25)
        x0 = x1 - WEB_W_MM
        y0, y1 = p.D / 2 - trim - WEB_ROOT_MM, p.U / 2 + WEB_ROOT_MM
        if x0 <= f + 1.0:
            raise ValueError("no room for the turn piece's web beside the centre band")
        xs = (x0, x1)
        corner_bot = max(_bottom_z(p, x + p.U / 2) for x in xs) + k * (p.D / 2 - trim - f)
        arm_bot = max(_bottom_z(p, 1.5 * p.U - y) for y in (p.U / 2, y1)) + k * (x1 - f)
        corner_top = min(_bottom_z(p, x + p.U / 2) for x in xs) + k * (y0 - f) + p.H0
        arm_top = min(_bottom_z(p, 1.5 * p.U - y) for y in (p.U / 2, y1)) + k * (x0 - f) + p.H0
        zlo, zhi = max(corner_bot, arm_bot) + 0.4, min(corner_top, arm_top - 1.5)
        if zhi - zlo < WEB_MIN_H_MM:
            raise ValueError(f"the turn piece's web would be only {zhi - zlo:.1f} mm high")
        return np.array([(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (zlo, zhi)], dtype=float)

    def l3_web_solid(self, p):
        return capture.hull_solid(self.l3_web(p))

    def pocket_bands(self, p, kind, c, bands):
        """The side bands a ring cell's blind pockets are cut in. The turn piece's corner cell and side
        arm keep their +y band whole: the web roots there."""
        return bands[:1] if kind == "L3" and (c.i, c.j) != (1, 0) else bands

    def cell_holes(self, kind, c, sec):
        """The through holes a ring cell's printed solid keeps."""
        return list(sec.holes)

    # --- rules ---

    def extra_rules(self, p, part, bodies):
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
            "R11.y_before_hooks": y_coupling(p)["ratio"] - self.y_before_hooks_floor,
            # R11: the mouth takes a hook pitched 6 deg (A/B's rule).
            "R11.mouth_takes_tilt": p.side_gap + p.hook_gap - p.hook_L * math.sin(math.radians(6.0)),
            # R13: printable hook tips and walls at the brick's ends.
            "R13.hook_tip": p.hook_w - p.tip_chamfer - 1.2,
            "R13.end_wall": min(p.valley_u - reach, p.U - p.valley_u - p.ramp_run - reach) - 1.2,
        }

    # --- the family ---

    def family(self) -> Family:
        fam = Family(
            name=self.name,
            version=self.version,
            description=self.description,
            params_type=self.params_type,
            bounds=self.bounds,
            parts=("A",),
            geometry=Solid3D(solid=self.brick_solid, collision=self.cap.collision, section=self.cap.section),
            lattice=lattice,
            part_for=part_for,
            seat_pose=seat_pose,
            insertion_axis=insertion_axis,
            release_lift_mm=self.release_lift_mm,
            base=self.base,
            pickup=self.pickup,
            extra_rules=self.extra_rules,
        )
        k = HookCellKit(fam, self)
        kit = RingKit(parts=k.names, solid=RC._Call(k, "solid"), plan=RC._Call(k, "plan"), base=RC._Call(k, "base"),
                      collision=RC._Call(k, "collision"))
        return replace(fam, ring=kit)


class HookCellKit(RC.CellKit):
    """engine/ring_cells.CellKit whose cells also carry ``cell_extras``: kit_from_cell sweeps a cell's
    section through the whole depth, and this brick's hooks and plugs do not span it. Each cell's
    printed solid has the brick's blind side pockets. The corner cell of the turn piece (L3) and of the
    corner straight (I3) has its side band cut back on the side the last wall arrives from
    (``ring_side_trim``). What differs between branches (the web, the pockets' bands, the corner cell's
    holes) is asked of the branch, ``th``."""

    def __init__(self, family, th):
        super().__init__(family, cell, stroke_mm, None)
        self.th = th

    def _build(self, p, name):
        key = (p, name)
        if key in self._cache:
            return self._cache[key]
        if name not in self.names:
            raise ValueError(f"no ring part {name!r}; the kit has {self.names}")
        th = self.th
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
            solid = B._swept(sec.outer, th.cell_holes(kind, c, sec), ext)
            for v in extras:
                solid = solid.fuse(capture.hull_solid(v))
            # The brick's blind side pockets, on the printed solid only (the collision keeps its solid plugs).
            bands = th.pocket_bands(p, kind, c, _ring_bands(p, trim)[1].bands)
            lay = _cell_layout(p, c.hooks, lo, hi, c.cut_hi)
            for q in _side_pockets(p, lay, wall=th.ring_pocket_wall_mm):
                solid = solid.cut(B._swept(affinity.translate(q, -U / 2, 0.0), [], Extrusion(depth_mm=p.D, bands=bands)))
            solid = solid.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), 90 * (c.yaw % 4)).translate(cq.Vector(*t))
            shape = solid if shape is None else shape.fuse(solid)
        if kind == "L3":
            pieces.append(th.l3_web(p))
            shape = shape.fuse(th.l3_web_solid(p))
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
            out += [(v - np.array([U / 2, 0., 0.])) @ R.T + t for v in self.th.base_pieces(p, 1, ext, side)]
        return out


class WebbedTwinHook(TwinHook):
    """A turn piece whose web is placed by search: on either side of the centre band, over the corner
    cell's left-face segments, as near level as the two cells allow (it may slope between them).
    Thinner ring-part pocket walls. From fable_feather, shared by gentle, wide and allround."""

    ring_pocket_wall_mm = 1.2
    web_anywhere = False  # True: the corner cell is built solid (no through hole), so any place will do

    def web_place(self, p):
        """Where the turn piece's web goes, in the piece's frame: (x0, x1, y0, edge, (zlo, zhi) in the
        corner cell, U / 2, y1, (zlo, zhi) in the arm), or ValueError. It is tried on both sides of the
        centre band, over the corner cell's left-face segments (solid there: no through hole). The two
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
                if self.web_anywhere or all(v >= peak + 1.0 or v <= p.valley_u + 0.5 for v in vs):
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

    def web_outline(self, p):
        """(x0, x1, the web's convex Y-Z outline as an (n, 2) array, counter-clockwise)."""
        from scipy.spatial import ConvexHull

        x0, x1, y0, edge, (c0, c1), ya, y1, (a0, a1) = self.web_place(p)
        yz = np.array([(y0, c0), (y0, c1), (edge, c0), (edge, c1), (ya, a0), (ya, a1), (y1, a0), (y1, a1)], dtype=float)
        return x0, x1, yz[ConvexHull(yz).vertices]

    def l3_web(self, p):
        """The web that joins the turn piece's side arm to its corner cell, as a convex vertex set in the
        piece's frame (corner cell centred on the origin, side arm along +y): a block rooted in each
        cell's side band, and the span between them. The cells are U long and D deep, so without it the
        arm stands off the corner cell and the piece is two solids. Its top stays 1.5 mm under a
        straight brick that is released over the arm with its end over the gap, and its roots lie
        inside the two cells."""
        x0, x1, yz = self.web_outline(p)
        return np.array([(x, y, z) for x in (x0, x1) for y, z in yz], dtype=float)

    def l3_web_solid(self, p):
        """The same web as a CadQuery solid: its Y-Z outline pushed along X."""
        x0, x1, yz = self.web_outline(p)
        wire = cq.Wire.makePolygon([cq.Vector(float(x0), float(y), float(z)) for y, z in yz], close=True)
        return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), cq.Vector(float(x1 - x0), 0.0, 0.0))

    def pocket_bands(self, p, kind, c, bands):
        if kind == "L3" and (c.i, c.j) == (0, 0):
            return bands[:1]            # the corner cell: the web is in its +y band
        if kind == "L3" and (c.i, c.j) == (0, 1):
            return bands[:1] if self.web_place(p)[0] > 0 else bands[1:]   # the arm: its band on the web's side
        return bands


class WingedTwinHook(WebbedTwinHook):
    """Wings on each side of the body (a subclass gives ``wings``, ``wing_rise`` and ``wing_wall``):
    the base carries them too, the release clears them, the carrying openings are the highest wide
    hole each side of the middle, and the turn piece's corner cell is solid, so the web may go
    anywhere beside the centre band. From fable_wide, shared by allround."""

    web_anywhere = True

    def wing_rise(self, p):
        raise NotImplementedError

    def wing_wall(self, p):
        raise NotImplementedError

    def wing_clearance(self, p):
        return _rise(p) + self.wing_rise(p) if p.wing > 0 else 0.0

    def pickup(self, p, part):
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

    def cell_holes(self, kind, c, sec):
        # At this pitch the web meets the corner cell on its ramp segment, where the lightening hole
        # would be: the turn piece's corner cell is built without it.
        return [] if kind == "L3" and (c.i, c.j) == (0, 0) else list(sec.holes)

    def extra_rules(self, p, part, bodies):
        return {
            **super().extra_rules(p, part, bodies),
            # R6 and R13: the wings are guide faces, and walls.
            "R6.wing_slides": (p.wing_deg - (math.degrees(math.atan(0.4)) + 8.0)) if p.wing > 0 else 1.0,
            "R13.wing_wall": (self.wing_wall(p) - 1.2) if p.wing > 0 else 1.0,
        }
