# From LunarSphere/aerial-assembly brixzle/src/brixzle/lattice.py @ a649663
# Changed: only seat poses kept, and Placement comes from the family interface; Structure, connectivity, assembly order, tiling and footprints dropped (no assembly trials yet), so networkx and shapely imports go too.
"""Voxel lattice, brick placements and their seat poses.

Lattice: voxel i spans x in [iU, (i+1)U]; course k sits at z = k H0. A brick
at (course k, start voxel i0) covers voxels i0 and i0+1 with its centre at
x = (i0+1) U. Two distinct parts alternate (A on even, B on odd courses), both
unrotated, so every tooth trails toward -x; a placement with direction=-1
(building toward -x) uses the other part rotated 180 degrees ("orientation B"),
which flips the lean to +x.
Bases are anchored plates covering a voxel range and receive course 0.
"""
import numpy as np

from brixzle.families.interface import Placement

from . import profile as pr
from .params import BrickParams


def orientation(p: BrickParams, b: Placement):
    """'A' (unrotated) or 'B' (rotated 180 degrees about Z) frame for the placement."""
    return 'B' if b.direction < 0 else 'A'


def part(p: BrickParams, b: Placement):
    """Which physical part ('A' or 'B') sits at the placement."""
    even = b.course % 2 == 0
    return ('A' if even else 'B') if b.direction > 0 else ('B' if even else 'A')


def pose(p: BrickParams, b: Placement):
    """Seated pose (mm, quaternion wxyz) of a placement."""
    pos = np.array([(b.i0 + 1)*p.U, 0., b.course*p.H0])
    quat = np.array([1., 0, 0, 0]) if orientation(p, b) == 'A' else np.array([0., 0, 0, 1])
    return pos, quat


def insertion_axis(p: BrickParams, b: Placement):
    """Unit (x, z) direction the brick moves along during final seating."""
    # Both parts' teeth trail toward -x; the 180-degree rotation flips it.
    _, d, _, _ = pr.interface(p, mirrored=False)
    return d if orientation(p, b) == 'A' else d*np.array([-1., 1.])
