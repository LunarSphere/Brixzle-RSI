"""yfunnel: a block with a V across Y outside its core, kept as a test fixture (issue #105). Not a registered family.

The interlock is prismatic: a flat-banded block 2U long, with a V ridge under each voxel's centre and
the matching V groove on top, so it lays a running bond (any one-voxel offset nests). On each side
of the core (depth D) a wing reaches ``wing_w`` further out in Y: a copy of the interlock's section
sheared up by ``wing_h`` over that width, so the ridges and grooves run across the wing too. A
brick's wings nest in the wings of the one below as a V across Y, and a brick dropped up to about
``wing_w`` off in Y slides back down it. The base carries the same V. It is built as a ``Solid3D``
(``capture.with_capture_features``), which is what this fixture tests.

The same shape is also a ``Prismatic`` family with three ``Extrusion`` bands (A/B's gutter is one):
a V across Y that keeps the X interlock is a shear of the section. ``Solid3D`` is needed for what a
shear cannot make. Three such versions jammed in the sim (simulated, uncalibrated contact), which
is why the fixture is this one:

- skirts hung beside the course below, flared inside: the guide met the brick 12 mm under its core,
  far off the line through its centre of mass, so it rolled 13 degrees and jammed from 6 mm;
- rails on top with a flat bottom: its corner wedged on the slope, rolled 8.6 degrees, from 4 mm;
- wings and a V with no X ridges on them: a brick off in Y landed its ridges on the smooth wing, so
  X could not correct until Y had, and then the ridge sat on its flank with the wings loaded. Every
  drop off in Y ended 3 mm off in X (the aim bias).

It shows the machinery, not a design to build. Its map (issue #105,
runs/map-20261114T013741540525Z-yfunnel-mean, 48.0 s): Y capture +-10 mm on both courses against
A/B's +-4 mm, Z mean -0.169 (its X capture is poor). Its ring, with the interlock-only ring base,
seated 7 of 16 and collapsed at the closer (runs/ring-20261114T013819974881Z-yfunnel).
"""

from dataclasses import dataclass

import numpy as np
from shapely.geometry import Polygon

from brixzle.engine import ring_cells
from brixzle.families.capture import prism_x, with_capture_features
from brixzle.families.interface import (
    Extrusion, Family, FamilyParams, Lattice, Placement, Section, bore,
)

BASE_THICKNESS_MM = 12.0
RELEASE_CLEARANCE_MM = 1.0
END_GAP_MM = 0.3   # gap between neighbours in a course


@dataclass(frozen=True)
class YFunnelParams(FamilyParams):
    U: float = 25.0             # voxel pitch in X (the brick spans two voxels)
    D: float = 10.0             # depth of the core in Y
    H0: float = 14.0            # course pitch
    ridge_h: float = 5.0        # V ridge under each voxel centre (and the groove on top)
    ridge_w: float = 6.0        # half-width of the ridge at its root
    wing_w: float = 12.0        # how far each wing reaches out in Y beyond the core
    wing_h: float = 10.0        # how far its underside and top rise over that width
    bore_d: float = 5.0         # two bores: carrying openings (R10)

    def __post_init__(self):
        super().__post_init__()
        if min(self.U, self.D, self.H0, self.ridge_h, self.ridge_w, self.wing_w, self.wing_h, self.bore_d) <= 0:
            raise ValueError("every length must be > 0")
        if not 2 * self.ridge_w < self.U / 2:
            raise ValueError("the ridges must fit inside their voxels")
        if self.bore_d + 2.4 > self.H0 - self.ridge_h:
            raise ValueError("the bores need 1.2 mm of wall above and below")


BOUNDS = {
    "H0": (13.0, 16.0),
    "wing_w": (8.0, 14.0),
    "wing_h": (6.0, 12.0),
}


def _row(p, x0, x1, centres, z_flat, z_tip):
    """Points along a flat at z_flat from x0 to x1, with a V (tip at z_tip) at each centre."""
    pts = [(x0, z_flat)]
    for c in centres:
        pts += [(c - p.ridge_w, z_flat), (c, z_tip), (c + p.ridge_w, z_flat)]
    return pts + [(x1, z_flat)]


def _profile(p, x0, x1, centres):
    bottom = _row(p, x0, x1, centres, p.ridge_h, 0.0)
    top = _row(p, x0, x1, centres, p.H0 + p.ridge_h, p.H0)
    return Polygon(bottom + top[::-1])


def section(p: YFunnelParams, part):
    e = END_GAP_MM / 2
    return Section(outer=_profile(p, -p.U + e, p.U - e, (-p.U / 2, p.U / 2)))


def extrusion(p: YFunnelParams):
    return Extrusion(depth_mm=p.D)


def _wings(p, outer):
    """The two wings: ``outer`` (an XZ polygon) swept from each side of the core out by wing_w,
    sheared up by wing_h, as exact convex pieces."""
    from brixzle.families import builders as B

    h = p.D / 2
    bands = ((h, h + p.wing_w, 0.0, p.wing_h), (-h, -h - p.wing_w, 0.0, p.wing_h))
    return B._pieces(outer, Extrusion(depth_mm=p.D + 2 * p.wing_w, bands=bands))


def features(p: YFunnelParams, part):
    return _wings(p, section(p, part).outer)


def pickup(p: YFunnelParams, part):
    z = (p.ridge_h + p.H0) / 2
    return tuple(bore(s * p.U / 2, z, p.bore_d) for s in (-1, 1))


def lattice(p: YFunnelParams):
    return Lattice(U_mm=p.U, D_mm=p.D, H0_mm=p.H0, span_voxels=2)


def part_for(course, direction):
    return "A"


def seat_pose(p: YFunnelParams, b: Placement):
    pos = np.array([(b.i0 + 1) * p.U, 0.0, b.course * p.H0])
    quat = np.array([1.0, 0, 0, 0]) if b.direction > 0 else np.array([0.0, 0, 0, 1])
    return pos, quat


def insertion_axis(p: YFunnelParams, b: Placement):
    return np.array([0.0, -1.0])


def release_lift_mm(p: YFunnelParams):
    return p.ridge_h + RELEASE_CLEARANCE_MM


def _slab(p, voxels):
    top = _row(p, 0.0, voxels * p.U, [(k + 0.5) * p.U for k in range(voxels)], p.ridge_h, 0.0)
    return Polygon(top + [(voxels * p.U, -BASE_THICKNESS_MM), (0.0, -BASE_THICKNESS_MM)])


def ring_base(p: YFunnelParams, voxels, direction):
    """The ring's base: the interlock's slab only. The V's wings reach 12 mm past the core and would
    stand in the way of the turn piece's side arm at each corner (the ring jammed at its first brick)."""
    return Section(outer=_slab(p, voxels))


def base(p: YFunnelParams, voxels, direction):
    """Exact convex pieces: a slab whose top is the brick's top (a flat with a V groove under every
    voxel centre, and the V across Y), base frame, x = 0 at voxel 0."""
    from brixzle.families import builders as B

    slab = _slab(p, voxels)
    return B._pieces(slab, Extrusion(depth_mm=p.D)) + _wings(p, slab)


def cell(p: YFunnelParams, part, hooks=True, exposed_lo=False, exposed_hi=False, cut_hi=0.0):
    """One voxel of the interlock, centred on x = 0 (ring_cells.kit_from_cell)."""
    e = END_GAP_MM / 2
    x0 = -p.U / 2 + (e if exposed_lo else 0.0)
    x1 = p.U / 2 - (e if exposed_hi else 0.0) - cut_hi
    return Section(outer=_profile(p, x0, x1, (0.0,)))


FAMILY = Family(
    name="yfunnel",
    version="0.1.0",
    description="Block with a V across Y outside its core, on a prismatic interlock: a test fixture for issue #105.",
    params_type=YFunnelParams,
    bounds=BOUNDS,
    parts=("A",),
    geometry=with_capture_features(section, extrusion, features),
    lattice=lattice,
    part_for=part_for,
    seat_pose=seat_pose,
    insertion_axis=insertion_axis,
    release_lift_mm=release_lift_mm,
    base=base,
    pickup=pickup,
)


def _with_ring(family):
    from dataclasses import replace

    return replace(family, ring=ring_cells.kit_from_cell(family, cell, base=ring_base))


FAMILY = _with_ring(FAMILY)
