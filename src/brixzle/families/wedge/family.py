"""wedge: a dummy Prismatic family (2.5D) used to test the family contract and shared builders.

A chevron of constant vertical thickness H0 spanning two voxels: a V-groove on top at
``groove_deg`` from horizontal and the matching V-ridge underneath, so a brick dropped on another
slides to the groove's centre in X. Straight extrusion through D, two Y bores as grip features. Its
area (2 U H0) and centroid can be worked out by hand. Not a design proposal.
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
    groove_deg: float = 35.0    # groove and ridge flank angle from horizontal
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
    return p.U * math.tan(math.radians(p.groove_deg))


def section(p: WedgeParams, part):
    h = _rise(p)
    left_top, top_valley, right_top = (-p.U, h + p.H0), (0.0, p.H0), (p.U, h + p.H0)
    outer = Polygon([(-p.U, h), (0.0, 0.0), (p.U, h), right_top, top_valley, left_top])
    return Section(outer=outer, guide_faces=(GuideFace(left_top, top_valley), GuideFace(top_valley, right_top)))


def extrusion(p: WedgeParams):
    return Extrusion(depth_mm=p.D)


def pickup(p: WedgeParams, part):
    x = p.bore_x_frac * p.U
    z = x * math.tan(math.radians(p.groove_deg)) + p.bore_z_frac * p.H0
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
    """A slab whose top is the ridge profile repeated per brick, in the base frame (x = 0 at voxel 0)."""
    if voxels % 2:
        raise ValueError("a wedge base must cover an even number of voxels")
    h = _rise(p)
    top = [(0.0, h)]
    for k in range(voxels // 2):
        x0 = 2 * k * p.U
        top += [(x0 + p.U, 0.0), (x0 + 2 * p.U, h)]
    outer = Polygon(top + [(voxels * p.U, -BASE_THICKNESS_MM), (0.0, -BASE_THICKNESS_MM)])
    return Section(outer=outer)


FAMILY = Family(
    name="wedge",
    version="0.1.0",
    description="Dummy chevron brick (V-groove over V-ridge) for contract tests; not a design.",
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
