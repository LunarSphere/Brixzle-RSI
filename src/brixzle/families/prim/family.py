"""prim: a brick family made of simple primitives, for script-driven search (#166).

Every shape is a few convex polygons set by a short parameter vector, so a search can make and check
thousands of candidates without writing CAD:

- **The voxel profile:** one chevron per voxel. A valley ("keel" underneath) at ``valley_frac`` of the
  voxel, a ramp rising at ``ramp_deg`` over ``ramp_frac`` of the voxel, and a left face down to the
  next valley. The body is that profile thickened by the course pitch ``H0``.
- **The hook:** under each keel, a convex quadrilateral tooth of length ``hook_L`` and width ``hook_w``,
  leaning ``lean_deg`` from vertical. Above each valley of the top, a slot that takes the tooth of the
  course above: ``hook_gap`` clear on the hook wall, tapering from ``side_gap`` at the surface to
  ``seat_gap`` at the tip on the other side. ``hook_L`` 0 gives a plain chevron.
- **Across Y:** a flat centre band of half-width ``flat`` and two side bands sheared up at ``y_deg``,
  a V that centres the brick in Y. ``y_deg`` 0 gives a straight extrusion.
- **Grip and carrying:** the side faces are full-height planes (the arm's jaws close on them, #153).
  There are two round through-Y bores of 5.2 mm (R10), one on each voxel's ramp.

Prismatic (2.5D), so the analytic predictor and the jam analysis apply. The ring kit is
engine/ring_cells.kit_from_cell from one cell (``cell``).

Original design. The tooth-in-slot joint and the rib-free chevron are generic. The parameter region
holds shapes like A/B (``hook_L`` small) and like the twinhook line (a leaning hook and a V across
Y), without copying either.
"""
from dataclasses import dataclass
import math

import numpy as np
from shapely import affinity
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

from brixzle.families import builders as B
from brixzle.families.interface import (
    Extrusion, Family, FamilyParams, Lattice, Placement, Prismatic, Section, bore,
)

MIN_GUIDE_DEG = 30.0      # R6: atan(0.4) + 8 deg is about 29.8
BORE_D_MM = 5.2           # a 5 mm circle fits (R10), with 0.1 mm clearance
SLOT_EXTRA_MM = 0.8       # slot depth beyond the tooth's tip
BASE_DEPTH_MM = 8.0
GUIDE_TOL_DEG = 0.1


@dataclass(frozen=True)
class PrimParams(FamilyParams):
    U: float = 30.0           # voxel pitch (the brick is 2U long)
    D: float = 26.0           # depth in Y
    H0: float = 13.0          # course pitch
    valley_frac: float = 0.3  # valley position in the voxel, from its left edge
    ramp_frac: float = 0.45   # the ramp's horizontal run, as a fraction of U
    ramp_deg: float = 45.0    # the ramp's angle from horizontal
    lean_deg: float = 20.0    # the tooth's lean from vertical (the stroke is down and toward -x)
    hook_L: float = 4.0       # tooth length along its axis (0: no hook)
    hook_w: float = 4.0       # tooth width across its axis
    hook_gap: float = 0.1     # seated gap at the slot's hook wall
    side_gap: float = 1.0     # slot clearance on the other side, at the surface
    seat_gap: float = 0.15    # the same at the tooth's tip, seated
    y_deg: float = 40.0       # the V across Y, from horizontal (0: straight)
    flat: float = 5.0         # half-width of the flat centre band
    end_gap: float = 0.3      # gap between neighbours in a course
    bore_x: float = 0.7       # bore position along each voxel's ramp, from its valley (fraction of the run)
    bore_z: float = 0.55      # bore height in the body, as a fraction of H0

    def __post_init__(self):
        super().__post_init__()
        g = geom(self)
        if min(self.U, self.D, self.H0) <= 0 or not 0 < self.valley_frac < 1 or not 0 < self.ramp_frac < 1:
            raise ValueError("U, D and H0 must be > 0, and the fractions inside (0, 1)")
        if min(self.ramp_deg, g["left_deg"]) < MIN_GUIDE_DEG:
            raise ValueError(f"guide faces too shallow (ramp {self.ramp_deg:.1f}, left {g['left_deg']:.1f} deg)")
        if self.hook_L < 0 or (self.hook_L > 0 and (self.hook_w < 1.2 or not 0 <= self.lean_deg < 90 - self.ramp_deg)):
            raise ValueError("the hook needs a width of at least 1.2 mm and a lean inside the ramp's cone")
        if min(self.hook_gap, self.seat_gap) < 0 or self.side_gap < self.seat_gap:
            raise ValueError("gaps must not be negative, and the mouth not narrower than the seat")
        if self.hook_L > 0 and self.H0 < (self.hook_L + SLOT_EXTRA_MM) * g["cos"] + 2.4 + 1.2:
            raise ValueError("body too thin for the slot")
        if not 0 <= self.y_deg < 60 or not 0 < self.flat <= self.D / 2:
            raise ValueError("y_deg must lie in [0, 60) and flat in (0, D/2]")
        if self.y_deg > 0 and self.y_deg < MIN_GUIDE_DEG:
            raise ValueError("the V across Y must be 0 or at least 30 deg (R6)")


BOUNDS = {
    "U": (22.0, 40.0), "D": (20.0, 32.0), "H0": (10.0, 18.0), "valley_frac": (0.15, 0.45),
    "ramp_frac": (0.3, 0.65), "ramp_deg": (32.0, 60.0), "lean_deg": (0.0, 35.0), "hook_L": (0.0, 6.0),
    "hook_w": (2.5, 5.0), "hook_gap": (0.05, 0.3), "side_gap": (0.3, 1.5), "seat_gap": (0.05, 0.4),
    "y_deg": (0.0, 55.0), "flat": (2.0, 9.0), "bore_x": (0.4, 0.95), "bore_z": (0.35, 0.75),
}


def geom(p):
    s, c = math.sin(math.radians(p.lean_deg)), math.cos(math.radians(p.lean_deg))
    vu, run = p.valley_frac * p.U, p.ramp_frac * p.U
    h = run * math.tan(math.radians(p.ramp_deg))
    left = p.U - run
    return {"vu": vu, "run": run, "h": h, "left_run": left, "left_deg": math.degrees(math.atan2(h, left)),
            "sin": s, "cos": c, "a": np.array([-s, -c]), "n": np.array([-c, s])}


def profile(p, x0, x1, z=0.0):
    """The chevron (valleys at height ``z``) over whole voxels covering [x0, x1], left to right."""
    g = geom(p)
    pts = []
    for k in range(math.floor(x0 / p.U) - 1, math.ceil(x1 / p.U) + 1):
        pts += [(k * p.U + g["vu"], z), (k * p.U + g["vu"] + g["run"], z + g["h"])]
    return pts


def _largest(geo):
    return geo if isinstance(geo, Polygon) else max(getattr(geo, "geoms", []), key=lambda q: q.area)


def tooth(p, at):
    """The tooth hanging from keel ``at``: a convex quadrilateral along the stroke."""
    g = geom(p)
    o, a, n = np.asarray(at, float), g["a"], g["n"]
    pts = [o - 1.0 * a, o + p.hook_L * a, o + p.hook_L * a + p.hook_w * n, o + p.hook_w * n - 1.0 * a]
    return Polygon([tuple(q) for q in pts])


def slot(p, at):
    """Where the tooth of the course above goes, at the top valley ``at``."""
    g = geom(p)
    o, a, n = np.asarray(at, float), g["a"], g["n"]
    back = 6.0
    grow = (p.side_gap - p.seat_gap) / max(p.hook_L, 1e-9)
    t1 = p.hook_w + p.hook_gap
    st = [(-back, -(p.side_gap + grow * back)), (p.hook_L, -p.seat_gap), (p.hook_L + SLOT_EXTRA_MM, -p.seat_gap),
          (p.hook_L + SLOT_EXTRA_MM, t1), (-back, t1)]
    return Polygon([tuple(o + s * a + t * n) for s, t in st])


def body(p, x0, nvox, xa, xb, hooks=True):
    """The section of ``nvox`` voxels from x0, cut to [xa, xb]: the thickened chevron, its teeth and slots."""
    g = geom(p)
    top, bot = profile(p, xa, xb, p.H0), profile(p, xa, xb, 0.0)
    core = _largest(Polygon(top + bot[::-1]).buffer(0).intersection(box(xa, -200, xb, 300)))
    if p.hook_L <= 0:
        return core
    keels = [(x0 + k * p.U + g["vu"], 0.0) for k in range(nvox)]
    shape = unary_union([core] + [tooth(p, q).intersection(box(xa, -200, xb, 300)) for q in keels] if hooks else [core])
    cut = unary_union([slot(p, (x, p.H0)) for x, _ in keels])
    return _largest(shape.difference(cut).buffer(-0.01, join_style=2).buffer(0.01, join_style=2))


def section(p, part):
    outer = body(p, -p.U, 2, -p.U + p.end_gap / 2, p.U - p.end_gap / 2)
    if not outer.is_valid or outer.area <= 0:
        raise ValueError("invalid outline")
    g = geom(p)
    angles = (p.ramp_deg, g["left_deg"], 90.0 - p.lean_deg)
    guides = tuple(f for f in B.inclined_edges(outer)
                   if min(abs(f.angle_deg - t) for t in angles) < GUIDE_TOL_DEG
                   and math.hypot(f.a[0] - f.b[0], f.a[1] - f.b[1]) > 1.0)
    return Section(outer=outer, guide_faces=guides)


def extrusion(p):
    if p.y_deg <= 0:
        return Extrusion(depth_mm=p.D)
    rise = (p.D / 2 - p.flat) * math.tan(math.radians(p.y_deg))
    return Extrusion(depth_mm=p.D, bands=((-p.D / 2, -p.flat, rise, 0.0), (-p.flat, p.flat, 0.0, 0.0),
                                          (p.flat, p.D / 2, 0.0, rise)))


def _bottom_z(p, x):
    g = geom(p)
    u = (x - g["vu"]) % p.U
    return u * g["h"] / g["run"] if u <= g["run"] else g["h"] - (u - g["run"]) * g["h"] / g["left_run"]


def pickup(p, part):
    g = geom(p)
    out = []
    for k in (-1, 0):  # one on each voxel's ramp, clear of the slot at its valley
        x = k * p.U + g["vu"] + p.bore_x * g["run"]
        out.append(bore(x, _bottom_z(p, x) + p.bore_z * p.H0, BORE_D_MM))
    return tuple(out)


def lattice(p):
    return Lattice(U_mm=p.U, D_mm=p.D, H0_mm=p.H0, span_voxels=2)


def part_for(course, direction):
    return "A"


def seat_pose(p, b: Placement):
    pos = np.array([(b.i0 + 1) * p.U, 0.0, b.course * p.H0])
    quat = np.array([1.0, 0, 0, 0]) if b.direction > 0 else np.array([0.0, 0, 0, 1])
    return pos, quat


def insertion_axis(p, b: Placement):
    a = geom(p)["a"] if p.hook_L > 0 else np.array([0.0, -1.0])
    return a if b.direction > 0 else a * np.array([-1.0, 1.0])


_LIFT = {}


def release_lift_mm(p):
    """The smallest lift (plus 1 mm) at which the brick, straight above its seat, clears the course below."""
    if p in _LIFT:
        return _LIFT[p]
    upper = body(p, -p.U, 2, -p.U, p.U)
    lower = affinity.translate(body(p, -3 * p.U, 6, -3 * p.U, 3 * p.U), 0.0, -p.H0)
    lo, hi = 0.0, 60.0
    for _ in range(30):
        mid = (lo + hi) / 2
        if affinity.translate(upper, 0.0, mid).intersection(lower).area > 1e-6:
            lo = mid
        else:
            hi = mid
    _LIFT[p] = hi + 1.0
    return _LIFT[p]


def base(p, voxels, direction):
    """A plate whose top is a course's top (valleys at z = 0), with the slots."""
    g = geom(p)
    X = voxels * p.U
    top = profile(p, 0.0, X, 0.0)
    plate = _largest(Polygon(top + [(top[-1][0], -BASE_DEPTH_MM), (top[0][0], -BASE_DEPTH_MM)]).buffer(0)
                     .intersection(box(0.0, -BASE_DEPTH_MM, X, 300)))
    if p.hook_L > 0:
        plate = _largest(plate.difference(unary_union([slot(p, (k * p.U + g["vu"], 0.0)) for k in range(voxels)])))
    if direction < 0:
        plate = affinity.scale(plate, xfact=-1, origin=(X / 2, 0))
    return Section(outer=plate)


def cell(p, part="A", hooks=True, exposed_lo=False, exposed_hi=False, cut_hi=0.0):
    """One voxel (x in [0, U]) centred on x = 0, for the ring kit (engine/ring_cells.kit_from_cell).
    ``hooks`` False: no tooth (the turn piece's side arm nests by its keel only)."""
    xa = p.end_gap / 2 if exposed_lo else 0.0
    xb = (p.U - p.end_gap / 2 if exposed_hi else p.U) - cut_hi
    outer = body(p, 0.0, 1, xa, xb, hooks=hooks)
    return Section(outer=affinity.translate(outer, -p.U / 2, 0.0))


FAMILY = Family(
    name="prim",
    version="0.1.0",
    description="Primitive chevron for script-driven search (#166): one chevron per voxel from plane angles, a "
                "convex leaning tooth in a tapered slot, a V across Y from sheared bands, two bores.",
    params_type=PrimParams,
    bounds=BOUNDS,
    parts=("A",),
    geometry=Prismatic(section=section, extrusion=extrusion),
    lattice=lattice,
    part_for=part_for,
    seat_pose=seat_pose,
    insertion_axis=insertion_axis,
    release_lift_mm=release_lift_mm,
    base=base,
    pickup=pickup,
)


def _with_ring(family):
    from dataclasses import replace as _replace

    from brixzle.engine import ring_cells as RC
    return _replace(family, ring=RC.kit_from_cell(family, cell))


FAMILY = _with_ring(FAMILY)
