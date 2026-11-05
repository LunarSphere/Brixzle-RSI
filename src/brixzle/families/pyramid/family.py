"""pyramid: a dummy Solid3D family (full 3D) used to test the family contract and shared builders.

A U x U x H0 block with a square-frustum peg on top and the same frustum as a pocket underneath,
so a brick dropped on another self-centres in both X and Y. It gives neither ``collision`` nor
``section``, so the builders' CoACD decomposition and y = 0 slice are exercised. Its volume equals
the block's (peg and pocket cancel), which can be checked by hand. Not a design proposal.
"""

from dataclasses import dataclass

import cadquery as cq
import numpy as np

from brixzle.families.interface import (
    Bore, Family, FamilyParams, Lattice, Placement, PickupFeatures, Solid3D,
)

BASE_THICKNESS_MM = 6.0
RELEASE_CLEARANCE_MM = 1.0


@dataclass(frozen=True)
class PyramidParams(FamilyParams):
    U: float = 25.0              # voxel pitch in X and depth in Y (square footprint)
    H0: float = 12.0             # course pitch (block height)
    peg_base_half: float = 8.0   # frustum half-width at its base
    peg_top_half: float = 1.0    # frustum half-width at its top
    peg_h: float = 6.0           # frustum height
    bore_d: float = 2.0          # fork tine diameter (fixed fork)
    bore_clearance: float = 0.3  # radial clearance around each tine
    bore_x_frac: float = 0.6     # bore |x| as a fraction of U/2
    bore_z_frac: float = 0.75    # bore height as a fraction of H0

    def __post_init__(self):
        super().__post_init__()
        if min(self.U, self.H0, self.peg_h, self.peg_top_half, self.bore_d) <= 0 or self.bore_clearance < 0:
            raise ValueError("lengths must be > 0 and bore_clearance >= 0")
        if not self.peg_top_half < self.peg_base_half < self.U / 2:
            raise ValueError("need peg_top_half < peg_base_half < U/2")
        if self.peg_h >= self.H0:
            raise ValueError("the pocket (peg_h) must be shallower than the block (H0)")
        if not (0 < self.bore_x_frac < 1 and 0 < self.bore_z_frac < 1):
            raise ValueError("bore fractions must be between 0 and 1")


BOUNDS = {
    "U": (20.0, 30.0),
    "H0": (11.0, 16.0),
    "peg_base_half": (6.0, 9.0),
    "peg_h": (4.0, 7.0),
    "bore_x_frac": (0.5, 0.7),
    "bore_z_frac": (0.7, 0.8),
}


def _square(half, z):
    return cq.Wire.makePolygon([cq.Vector(x, y, z) for x, y in
                                ((-half, -half), (half, -half), (half, half), (-half, half))], close=True)


def _frustum(p, z0):
    return cq.Solid.makeLoft([_square(p.peg_base_half, z0), _square(p.peg_top_half, z0 + p.peg_h)], True)


def solid(p: PyramidParams, part):
    block = cq.Solid.makeBox(p.U, p.U, p.H0, cq.Vector(-p.U / 2, -p.U / 2, 0))
    return block.fuse(_frustum(p, p.H0)).cut(_frustum(p, 0.0)).clean()


def _frustum_vertices(p, x, z0):
    lo = [(x + sx * p.peg_base_half, sy * p.peg_base_half, z0) for sx in (-1, 1) for sy in (-1, 1)]
    hi = [(x + sx * p.peg_top_half, sy * p.peg_top_half, z0 + p.peg_h) for sx in (-1, 1) for sy in (-1, 1)]
    return np.array(lo + hi)


def pickup(p: PyramidParams, part):
    x = p.bore_x_frac * p.U / 2
    return PickupFeatures(bores=tuple(Bore(x_mm=s * x, z_mm=p.bore_z_frac * p.H0, d_mm=p.bore_d,
                                           clearance_mm=p.bore_clearance) for s in (-1, 1)))


def lattice(p: PyramidParams):
    return Lattice(U_mm=p.U, D_mm=p.U, H0_mm=p.H0, span_voxels=1)


def part_for(course, direction):
    return "A"


def seat_pose(p: PyramidParams, b: Placement):
    pos = np.array([(b.i0 + 0.5) * p.U, 0.0, b.course * p.H0])
    quat = np.array([1.0, 0, 0, 0]) if b.direction > 0 else np.array([0.0, 0, 0, 1])
    return pos, quat


def insertion_axis(p: PyramidParams, b: Placement):
    return np.array([0.0, -1.0])  # straight down onto the peg


def release_lift_mm(p: PyramidParams):
    return p.peg_h + RELEASE_CLEARANCE_MM


def base(p: PyramidParams, voxels, direction):
    """Exact convex pieces: a slab and one peg per voxel (base frame, x = 0 at voxel 0)."""
    w = p.U / 2
    slab = np.array([(x, y, z) for x in (0.0, voxels * p.U) for y in (-w, w) for z in (-BASE_THICKNESS_MM, 0.0)])
    return [slab] + [_frustum_vertices(p, (k + 0.5) * p.U, 0.0) for k in range(voxels)]


FAMILY = Family(
    name="pyramid",
    version="0.1.0",
    description="Dummy block with a frustum peg and pocket for contract tests; not a design.",
    params_type=PyramidParams,
    bounds=BOUNDS,
    parts=("A",),
    geometry=Solid3D(solid=solid),
    lattice=lattice,
    part_for=part_for,
    seat_pose=seat_pose,
    insertion_axis=insertion_axis,
    release_lift_mm=release_lift_mm,
    base=base,
    pickup=pickup,
)
