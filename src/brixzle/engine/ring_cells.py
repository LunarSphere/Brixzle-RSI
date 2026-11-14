# From LunarSphere/aerial-assembly brixzle/src/brixzle/ring.py @ fc4f724
# Changed: the cell comes from the family (cell(p, part, hooks, exposed_lo, exposed_hi, cut_hi) -> Section) instead of A/B's cell_profile, and kit_from_cell turns it into a families.interface.RingKit (parts, solid, collision, plan, base). U, H0 and the extrusion come from the family; the stroke's sideways travel is the stroke length times the insertion axis's X component (A/B passes its slanted stroke, tooth_L + slot_extra); the course's part is family.part_for. Collision is builders._pieces of each cell (the canonical cut, #98). The simulation (assemble, _release, _expected) moved to engine/sim/ring_build.py.
"""Ring kits from one cell (issue #89): the turn piece, the corner straight and the closer.

Everything is built from *cells*: one voxel of a part's profile, centred on x = 0, swept through
the depth. A cell is placed with a yaw in quarter turns. The straight brick is two cells; a
Y-running wall is the same brick turned 90 degrees. Courses alternate parts as in the straight
lattice, so a cell nests on the cell below when it has the same yaw: every column keeps one yaw.

Every brick seats with a stroke along its local -x, the same way round the ring on every course.

Parts, one per kind and course part (e.g. ``L3A``):
  L3  turn, even courses: corner + 1 cell along local +x + 1 cell along local +y. Hooks only on
      the corner and x-arm cells: a rigid brick seats along one stroke, so hooks leaning along x
      and along y would fight. The y-arm cell has yaw 270 and nests by its keel only.
  I3  three straight cells along local +x that own the corner on odd courses, so the corner
      alternates between the two walls (corner bond) and straights shift one cell per course.
  K2  closer: a straight whose upstream (+x) end is cut back by the stroke's sideways travel.
      Every stroke points back into its neighbour all the way round, so on every course one
      brick must go in last next to an already placed upstream neighbour.
The straight (S2) is the family's own part.

The ring has n x n cells (n even, >= 6). Even course: L3 at each corner, S2 fill. Odd course: I3
owns each corner, S2 fill shifted by one. The last straight of the last wall is the K2.
"""

from dataclasses import dataclass
import math

import cadquery as cq
import numpy as np
from shapely import affinity

from brixzle.families import builders as B
from brixzle.families.interface import Placement, Prismatic, RingBrick, RingKit, Section

DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]
KINDS = ("L3", "I3", "K2")
CLOSER_MARGIN_MM = 0.5  # the closer is cut back by the stroke's travel plus this (the original's)


def stroke_travel(family, p, stroke_mm=None):
    """Sideways travel of the slanted final seating stroke: the stroke length (default the release
    lift) times the X component of the insertion axis."""
    axis = np.asarray(family.insertion_axis(p, Placement(0, 0)), dtype=float)
    length = family.release_lift_mm(p) if stroke_mm is None else float(stroke_mm)
    return length * abs(float(axis[0])) / float(np.linalg.norm(axis))


def _yaw_matrix(q):
    c, s = DIRS[q % 4]
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


@dataclass(frozen=True)
class Cell:
    i: int          # local cell coordinates (corner cell at 0, 0)
    j: int
    yaw: int        # quarter turns of the cell profile
    hooks: bool = True
    cut_hi: float = 0.   # extra cut at the local +x (upstream) end (closer)


def _exposed(cell, cells):
    """Which local profile ends (lo = -x, hi = +x in the cell's own frame) face no sibling."""
    occupied = {(c.i, c.j) for c in cells}
    out = []
    for sign in (-1, 1):
        dx, dy = DIRS[cell.yaw % 4]
        out.append((cell.i + sign * dx, cell.j + sign * dy) not in occupied)
    return out


def part_cells(kind, travel):
    if kind == "L3":
        return [Cell(0, 0, 0), Cell(1, 0, 0), Cell(0, 1, 3, hooks=False)]
    if kind == "I3":
        return [Cell(0, 0, 0), Cell(1, 0, 0), Cell(2, 0, 0)]
    if kind == "K2":
        return [Cell(0, 0, 0), Cell(1, 0, 0, cut_hi=travel + CLOSER_MARGIN_MM)]
    raise ValueError(f"no ring part kind {kind!r}")


# ---------------------------------------------------------------- placements


@dataclass(frozen=True)
class Brick:
    kind: str       # 'S2', 'L3', 'I3', 'K2'
    course: int
    x: float        # frame position in units of U (cell centre grid; S2 sits on a cell boundary)
    y: float
    yaw: int

    @property
    def name(self):
        return f"{self.kind}_{self.course}_{self.x:g}_{self.y:g}"


LOCAL_CELLS = {"L3": [(0, 0), (1, 0), (0, 1)], "I3": [(0, 0), (1, 0), (2, 0)], "S2": [(-0.5, 0), (0.5, 0)],
               "K2": [(0, 0), (1, 0)]}


def cells_of(b: Brick, local_cells=None):
    R = _yaw_matrix(b.yaw)[:2, :2]
    out = []
    for c in (LOCAL_CELLS[b.kind] if local_cells is None else local_cells):
        w = R @ np.asarray(c, dtype=float) + np.array([b.x, b.y])
        out.append((int(round(w[0])), int(round(w[1]))))
    return out


def pose(U, H0, b: Brick):
    """(pos mm, quat wxyz) of a brick's frame."""
    pos = np.array([(b.x + .5) * U, (b.y + .5) * U, b.course * H0])
    half = math.pi / 4 * (b.yaw % 4)
    return pos, np.array([math.cos(half), 0., 0., math.sin(half)])


def stroke(b: Brick):
    """World grid direction the brick moves along while seating (its local -x)."""
    dx, dy = DIRS[b.yaw % 4]
    return (-dx, -dy)


def ring(n=6, courses=6):
    """Square ring of n x n cells (n even >= 6), as a list of bricks."""
    if n % 2 or n < 6:
        raise ValueError("ring needs an even side of at least 6 cells")
    corners = [(0, 0), (n - 1, 0), (n - 1, n - 1), (0, n - 1)]
    bricks = []
    for k in range(courses):
        odd = k % 2
        for r, c in enumerate(corners):
            e = np.array(DIRS[r])
            bricks.append(Brick("I3" if odd else "L3", k, c[0], c[1], r))
            ts = range(3, n - 1, 2) if odd else range(2, n - 2, 2)
            for t in ts:
                if r == 3 and t == ts[-1]:
                    # Closer: K2's frame sits on its first cell; its +x (upstream) end is cut back.
                    first = np.array(c) + t * e
                    bricks.append(Brick("K2", k, float(first[0]), float(first[1]), r))
                    continue
                mid = np.array(c) + (t + 0.5) * e
                bricks.append(Brick("S2", k, float(mid[0]), float(mid[1]), r))
    return bricks


def _cell_yaw(b, local):
    return (b.yaw + 3) % 4 if b.kind == "L3" and tuple(local) == (0, 1) else b.yaw % 4


def course0_yaws(bricks):
    """Cell -> yaw of the course-0 cell above it (the base must match it)."""
    yaws = {}
    for b in bricks:
        if b.course == 0:
            for lc, wc in zip(LOCAL_CELLS[b.kind], cells_of(b)):
                yaws[wc] = _cell_yaw(b, lc)
    return yaws


def check_columns(bricks):
    """Every cell is used once per course and every column keeps its yaw (parts alternate by course)."""
    grid = {}
    for b in bricks:
        for lc, wc in zip(LOCAL_CELLS[b.kind], cells_of(b)):
            key = (b.course, wc)
            if key in grid:
                raise ValueError(f"cell {key} used twice")
            grid[key] = _cell_yaw(b, lc)
    for (k, c), yaw in grid.items():
        below = grid.get((k - 1, c))
        if k > 0 and (below is None or yaw != below):
            raise ValueError(f"cell {c} course {k}: yaw {yaw} over {below}")
    return grid


def _blocked(b, placed_cells):
    if b.kind == "K2":
        return False   # its upstream end is cut back by the stroke travel
    sx, sy = stroke(b)
    mine = set(cells_of(b))
    return any((c[0] - sx, c[1] - sy) in placed_cells and (c[0] - sx, c[1] - sy) not in mine for c in mine)


def assembly_order(bricks):
    """Course by course; a brick waits while a placed sibling occupies the cell upstream of any of
    its cells along its seating stroke (vertical end faces would collide during the slanted final
    stroke). Depth-first per course."""
    order = []
    for k in sorted({b.course for b in bricks}):
        todo = [b for b in bricks if b.course == k]

        def dfs(remaining, placed, seq):
            if not remaining:
                return seq
            for b in sorted(remaining, key=lambda x: x.kind == "K2"):
                if _blocked(b, placed):
                    continue
                out = dfs([x for x in remaining if x is not b], placed | set(cells_of(b)), seq + [b])
                if out is not None:
                    return out
            return None
        seq = dfs(todo, frozenset(), [])
        if seq is None:
            raise ValueError(f"no insertion order for course {k}")
        order += seq
    return order


# ---------------------------------------------------------------- the kit


class CellKit:
    """The RingKit's callables for one family and cell (kit_from_cell). A class, not closures, so a
    family with a kit can be pickled into worker processes; the cache of built parts is not pickled."""

    def __init__(self, family, cell, stroke_mm=None, base=None):
        self.family, self.cell, self.stroke_mm, self.base_fn = family, cell, stroke_mm, base
        self.names = tuple(f"{kind}{part}" for kind in KINDS for part in family.parts)
        self._cache = {}

    def __getstate__(self):
        return {k: v for k, v in self.__dict__.items() if k != "_cache"}

    def __setstate__(self, state):
        self.__dict__.update(state)
        self._cache = {}

    def __eq__(self, other):
        # Equal kits after a round trip to a worker: same family, cell and stroke.
        return (isinstance(other, CellKit) and (self.family, self.cell, self.stroke_mm, self.base_fn)
                == (other.family, other.cell, other.stroke_mm, other.base_fn))

    def __hash__(self):
        return hash((self.family.name, self.cell, self.stroke_mm, self.base_fn))

    def travel(self, p):
        return stroke_travel(self.family, p, None if self.stroke_mm is None else self.stroke_mm(p))

    def _build(self, p, name):
        key = (p, name)
        if key in self._cache:
            return self._cache[key]
        if name not in self.names:
            raise ValueError(f"no ring part {name!r}; the kit has {self.names}")
        kind, part = name[:2], name[2:]
        fam = self.family
        U = fam.lattice(p).U_mm
        ext = interlock_extrusion(fam)(p)
        cells = part_cells(kind, self.travel(p))
        pieces, shape = [], None
        for c in cells:
            lo, hi = _exposed(c, cells)
            sec = self.cell(p, part, hooks=c.hooks, exposed_lo=lo, exposed_hi=hi, cut_hi=c.cut_hi)
            R = _yaw_matrix(c.yaw)
            t = np.array([c.i * U, c.j * U, 0.])
            pieces += [v @ R.T + t for v in B._pieces(sec.outer, ext)]
            solid = B._swept(sec.outer, list(sec.holes), ext)
            solid = solid.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), 90 * (c.yaw % 4)).translate(cq.Vector(*t))
            shape = solid if shape is None else shape.fuse(solid)
        self._cache[key] = (shape.clean(), pieces)
        return self._cache[key]

    def solid(self, p, part):
        return self._build(p, part)[0]

    def collision(self, p, part):
        return self._build(p, part)[1]

    def plan(self, p, n, courses):
        lat = self.family.lattice(p)
        out = []
        for b in assembly_order(ring(n, courses)):
            part = self.family.part_for(b.course, 1)
            pos, quat = pose(lat.U_mm, lat.H0_mm, b)
            out.append(RingBrick(kind=b.kind, part=part if b.kind == "S2" else f"{b.kind}{part}", course=b.course,
                                 pos_mm=tuple(float(v) for v in pos), quat=tuple(float(v) for v in quat),
                                 cells=tuple(cells_of(b)), name=b.name))
        return out

    def base(self, p, n):
        lat = self.family.lattice(p)
        sec = (self.base_fn or self.family.base)(p, 1, 1)
        if isinstance(sec, Section):
            outer = affinity.translate(sec.outer, -lat.U_mm / 2, 0.)
            piece = B._pieces(outer, interlock_extrusion(self.family)(p))
        else:  # exact convex pieces (base frame, x = 0 at the voxel's left edge): centre the voxel on x = 0
            piece = [np.asarray(v, dtype=float) - np.array([lat.U_mm / 2, 0., 0.]) for v in sec]
        out = []
        for (ci, cj), yaw in course0_yaws(ring(n, 1)).items():
            R = _yaw_matrix(yaw)
            t = np.array([(ci + .5) * lat.U_mm, (cj + .5) * lat.U_mm, 0.])
            out += [v @ R.T + t for v in piece]
        return out


class _Call:
    """A CellKit method as a value: picklable, and equal after a round trip (bound methods compare
    their object by identity)."""

    def __init__(self, kit, name):
        self.kit, self.name = kit, name

    def __call__(self, *args):
        return getattr(self.kit, self.name)(*args)

    def __eq__(self, other):
        return isinstance(other, _Call) and (self.kit, self.name) == (other.kit, other.name)

    def __hash__(self):
        return hash((self.kit, self.name))


def interlock_extrusion(family):
    """The extrusion function of the family's prismatic interlock: a Prismatic family's own, or the
    core of a Solid3D made with families.capture.with_capture_features (issue #105); else None."""
    from brixzle.families import capture

    if isinstance(family.geometry, Prismatic):
        return family.geometry.extrusion
    return capture.core_extrusion(family.geometry)


def kit_from_cell(family, cell, stroke_mm=None, base=None):
    """A RingKit from one cell, for a family with a prismatic interlock: a Prismatic family, or a
    Solid3D made with families.capture.with_capture_features (issue #105), whose ring parts are its
    interlock only, without the capture features.

    ``cell(p, part, hooks=True, exposed_lo=False, exposed_hi=False, cut_hi=0.0) -> Section`` is one
    voxel of ``part``'s profile centred on x = 0: ``hooks=False`` leaves out any feature that seats
    along a slanted stroke (the turn piece's side arm nests by its keel only); ``exposed_lo`` and
    ``exposed_hi`` say which ends face no sibling in the same part (trim them as the straight
    brick's ends are); ``cut_hi`` cuts that much more off the +x end (the closer). ``stroke_mm(p)``
    is the slanted stroke's length for the closer's cut (default: the release lift). ``base(p,
    voxels, direction)`` is the ring base's voxel (default: the family's base); a family whose base
    carries capture features that reach past its depth gives its interlock's base here, since the
    ring's corners would run into them (issue #105). ``cell``, ``stroke_mm`` and ``base`` must be
    module-level functions, so the family can be pickled."""
    if interlock_extrusion(family) is None:
        raise ValueError("kit_from_cell needs a family with a prismatic interlock: a Prismatic family, or a Solid3D "
                         "made with families.capture.with_capture_features")
    k = CellKit(family, cell, stroke_mm, base)
    return RingKit(parts=k.names, solid=_Call(k, "solid"), plan=_Call(k, "plan"), base=_Call(k, "base"),
                   collision=_Call(k, "collision"))
