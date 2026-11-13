# From LunarSphere/aerial-assembly brixzle/src/brixzle/structures.py @ fc4f724
# Changed: Structure (from lattice.py) holds families.interface.Placement bricks; every builder takes the family's span and raises ValueError for any span but two voxels; structure_for builds a catalog entry for a family; brick names and voxels are helpers here, since the interface's Placement has neither.
"""Trial structures on the lattice (issue #81).

Anchored bases extend one voxel past the first course on every side that is
not deliberately cantilevered, so edge bricks get the same hopper catchment
as interior ones. Bases are ``(v0, v1[, direction])`` voxel ranges, half-open,
as ``engine/sim/scene.build_scene`` takes them.

Every structure here assumes a two-voxel brick (``Lattice.span_voxels == 2``):
the overhang steps one voxel per course, half a brick, and the wall's running
bond shifts odd courses by one voxel.
"""
from dataclasses import dataclass, field

from brixzle.families.interface import Placement

SPAN = 2  # the structures are laid out for a two-voxel brick


@dataclass
class Structure:
    name: str
    bricks: list
    bases: list = field(default_factory=list)  # [(v0, v1[, direction])] voxel ranges, half-open

    def course(self, k):
        return [b for b in self.bricks if b.course == k]


def brick_name(b: Placement):
    return f'b{b.course}_{b.i0}' + ('m' if b.direction < 0 else '')


def voxels(b: Placement, span=SPAN):
    return tuple(range(b.i0, b.i0 + span))


def _two_voxels(span, what):
    if span != SPAN:
        raise ValueError(f'the {what} is laid out for a two-voxel brick; not supported for span_voxels = {span}')


def tower(height, span=SPAN):
    """Stack bond column: every brick fully seated on the one below."""
    _two_voxels(span, 'tower')
    return Structure(f'tower{height}', [Placement(k, 0) for k in range(height)], bases=[(-1, 3)])


def overhang(n, direction=1, span=SPAN):
    """Single-brick staircase stepping one voxel per course past a 2-voxel base.

    Each brick after the first engages its supporter over one voxel only; its
    centre of mass sits over the supporter's edge, so only the interlock holds it.
    """
    _two_voxels(span, 'overhang')
    if direction > 0:
        bricks, base = [Placement(k, k) for k in range(n)], (-1, 2)
    else:
        bricks, base = [Placement(k, -k, -1) for k in range(n)], (0, 3, -1)
    return Structure(f'overhang{"+" if direction > 0 else "-"}{n}', bricks, bases=[base])


def wall(width_bricks, height, span=SPAN):
    """Running-bond wall; odd courses shift by one voxel."""
    _two_voxels(span, 'wall')
    bricks = []
    for k in range(height):
        shift = k % 2
        count = width_bricks - shift
        bricks += [Placement(k, shift + 2*j) for j in range(count)]
    return Structure(f'wall{width_bricks}x{height}', bricks, bases=[(-1, 2*width_bricks + 1)])


def bridge(arm_courses, span=SPAN):
    """Two corbelled arms on separate anchored bases closed by one key brick."""
    _two_voxels(span, 'bridge')
    m = 2*arm_courses
    bricks = []
    for k in range(arm_courses):
        bricks.append(Placement(k, k))
        bricks.append(Placement(k, m - k))
    bricks.append(Placement(arm_courses, arm_courses))
    return Structure(f'bridge{arm_courses}', bricks, bases=[(-1, 2), (m, m + 3)])


CATALOG = {
    'tower': lambda span=SPAN: tower(6, span),
    'overhang+': lambda span=SPAN: overhang(12, +1, span),
    'overhang-': lambda span=SPAN: overhang(12, -1, span),
    'overhang+20': lambda span=SPAN: overhang(20, +1, span),
    'overhang-20': lambda span=SPAN: overhang(20, -1, span),
    'wall': lambda span=SPAN: wall(3, 4, span),
    'bridge': lambda span=SPAN: bridge(3, span),
}


def structure_for(family, p, name):
    """A catalog structure for ``family`` (ValueError if its span is not two voxels)."""
    if name not in CATALOG:
        raise KeyError(f'no structure {name!r}; known: {sorted(CATALOG)}')
    return CATALOG[name](family.lattice(p).span_voxels)
