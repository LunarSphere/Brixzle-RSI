"""wedge: a dummy Prismatic family (2.5D) used to test the family contract and shared builders.

A W of constant vertical thickness H0 spanning two voxels: one V-groove per voxel on top at
``groove_deg`` from horizontal, with its bottom at the voxel's centre, and the matching V-ridges
underneath. The pattern repeats every voxel, so a brick nests straight on one brick (stack bond)
and across the joint between two (running bond, issue #82), sliding to the nearest groove in X.
Straight extrusion through D, two Y bores as grip features. Its area (2 U H0) and centroid can be
worked out by hand. Not a design proposal.

(Before #82 it was a single chevron over two voxels, which cannot lay a running bond; that shape is
kept as the test fixture tests/fixtures/stackv.)
"""

from dataclasses import dataclass
import math

import numpy as np
from shapely.geometry import Polygon

from brixzle.families.interface import (
    Extrusion, Family, FamilyParams, GuideFace, Lattice, Placement, Prismatic, Section, bore,
)

BASE_THICKNESS_MM = 6.0
RELEASE_CLEARANCE_MM = 1.0


@dataclass(frozen=True)
class WedgeParams(FamilyParams):
    U: float = 25.0             # voxel pitch in X (the brick spans two voxels)
    D: float = 25.0             # depth in Y
    H0: float = 15.0            # course pitch (vertical thickness)
    groove_deg: float = 35.0    # groove and ridge flank angle from horizontal (one V per voxel)
    bore_d: float = 2.0         # bore diameter before clearance
    bore_clearance: float = 0.3  # extra radius on each bore
    bore_x_frac: float = 0.6    # bore |x| as a fraction of U
    bore_z_frac: float = 0.55   # bore height above the ridge flank, as a fraction of H0

    def __post_init__(self):
        super().__post_init__()
        if min(self.U, self.D, self.H0, self.bore_d) <= 0 or self.bore_clearance < 0:
            raise ValueError("U, D, H0 and bore_d must be > 0 and bore_clearance >= 0")
        if not 0 < self.groove_deg < 90:
            raise ValueError("groove_deg must be between 0 and 90")
        if not (0 < self.bore_x_frac < 1 and 0 < self.bore_z_frac < 1):
            raise ValueError("bore fractions must be between 0 and 1")


BOUNDS = {
    "U": (18.0, 32.0),
    "D": (18.0, 32.0),
    "H0": (12.0, 20.0),
    "groove_deg": (30.0, 45.0),
    "bore_x_frac": (0.4, 0.7),
    "bore_z_frac": (0.4, 0.6),
}


def _rise(p):
    """Height of each V: half a voxel at the flank angle."""
    return p.U / 2 * math.tan(math.radians(p.groove_deg))


def underside(p: WedgeParams, x):
    """Height of the underside at ``x`` (brick frame): 0 at the ridges (x = +-U/2), the rise at the
    ends and the centre."""
    return abs(abs(x) - p.U / 2) * math.tan(math.radians(p.groove_deg))


def section(p: WedgeParams, part):
    h = _rise(p)
    xs = (-p.U, -p.U / 2, 0.0, p.U / 2, p.U)
    bottom = [(x, h if i % 2 == 0 else 0.0) for i, x in enumerate(xs)]
    top = [(x, z + p.H0) for x, z in reversed(bottom)]
    outer = Polygon(bottom + top)
    guides = tuple(GuideFace(a, b) for a, b in zip(top, top[1:]))
    return Section(outer=outer, guide_faces=guides)


def extrusion(p: WedgeParams):
    return Extrusion(depth_mm=p.D)


def pickup(p: WedgeParams, part):
    x = p.bore_x_frac * p.U
    z = underside(p, x) + p.bore_z_frac * p.H0
    return tuple(bore(s * x, z, p.bore_d + 2 * p.bore_clearance) for s in (-1, 1))


def lattice(p: WedgeParams):
    return Lattice(U_mm=p.U, D_mm=p.D, H0_mm=p.H0, span_voxels=2)


def part_for(course, direction):
    return "A"


def seat_pose(p: WedgeParams, b: Placement):
    pos = np.array([(b.i0 + 1) * p.U, 0.0, b.course * p.H0])
    quat = np.array([1.0, 0, 0, 0]) if b.direction > 0 else np.array([0.0, 0, 0, 1])
    return pos, quat


def insertion_axis(p: WedgeParams, b: Placement):
    return np.array([0.0, -1.0])  # straight down into the groove


def release_lift_mm(p: WedgeParams):
    return _rise(p) + RELEASE_CLEARANCE_MM


def base(p: WedgeParams, voxels, direction):
    """A slab whose top is a row of V-grooves, one per voxel, in the base frame (x = 0 at voxel 0).

    A seated brick at Placement(0, i0) is centred at x = (i0 + 1) U (``seat_pose``), so its ridges
    sit at voxel centres: the grooves' bottoms are at (k + 1/2) U and their crests at k U, for every
    voxel k. Any number of voxels works."""
    h = _rise(p)
    top = [(0.0, h)]
    for k in range(voxels):
        top += [((k + 0.5) * p.U, 0.0), ((k + 1) * p.U, h)]
    outer = Polygon(top + [(voxels * p.U, -BASE_THICKNESS_MM), (0.0, -BASE_THICKNESS_MM)])
    return Section(outer=outer)


FAMILY = Family(
    name="wedge",
    version="0.2.0",
    description="Dummy W brick (one V-groove over one V-ridge per voxel) for contract tests; not a design.",
    params_type=WedgeParams,
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
