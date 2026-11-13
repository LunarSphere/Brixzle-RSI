"""The brick family contract: what every family module exports, and the data types it uses.

A family module ``brixzle/families/<name>/family.py`` exports ``FAMILY = Family(...)``. The
engine only ever talks to a family through this object, and ``contract.check_family`` checks it.
See docs/family-interface.md for the prose version the design agent reads.

Units: geometry in mm and degrees (fields carry ``_mm``/``_deg``), mass in g; conversion to SI
happens at the MuJoCo boundary. Quaternions are [w, x, y, z]. The brick frame has X along the
lattice, Y across it and Z up; z = 0 is the seat.
"""

from dataclasses import asdict, dataclass, field, fields
import math
from typing import Any, Callable

import numpy as np
from shapely.geometry import Polygon


@dataclass(frozen=True)
class FamilyParams:
    """Base for a family's parameter dataclass (subclass it with ``@dataclass(frozen=True)``)."""

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, values: dict):
        names = {f.name for f in fields(cls)}
        unknown = set(values) - names
        if unknown:
            raise ValueError(f"unknown {cls.__name__} parameters: {sorted(unknown)}")
        return cls(**values)

    def __post_init__(self):
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"{f.name} must be finite, got {value}")


@dataclass(frozen=True)
class Placement:
    """A brick on the lattice: its course (layer), first voxel, and build direction (+1 or -1)."""

    course: int
    i0: int
    direction: int = 1


@dataclass(frozen=True)
class GuideFace:
    """A segment of the section outline that a falling brick slides along (R6)."""

    a: tuple[float, float]  # (x, z) mm
    b: tuple[float, float]

    @property
    def angle_deg(self) -> float:
        """Angle from horizontal, 0 to 90 degrees."""
        dx, dz = self.b[0] - self.a[0], self.b[1] - self.a[1]
        return math.degrees(math.atan2(abs(dz), abs(dx)))


@dataclass(frozen=True)
class Section:
    """The brick's 2D profile in the XZ plane, used by the predictor and the pickup check."""

    outer: Polygon
    holes: tuple[Polygon, ...] = ()        # through-Y lightening holes (not the round grip bores)
    guide_faces: tuple[GuideFace, ...] = ()


@dataclass(frozen=True)
class Extrusion:
    """How a Prismatic section is swept through Y: bands [(y0, y1, z_shift_at_y0, z_shift_at_y1)].

    No bands means a straight extrusion through the depth, centred on y = 0. Sheared bands give a
    V gutter across Y, as in the A/B brick.
    """

    depth_mm: float
    bands: tuple[tuple[float, float, float, float], ...] = ()

    def resolved_bands(self):
        return self.bands or ((-self.depth_mm / 2, self.depth_mm / 2, 0.0, 0.0),)


BORE = "bore"        # the one kind the builders act on: a round hole straight through Y, cut into the solid
OPENING = "opening"  # a through-Y opening that is already one of the section's holes


@dataclass(frozen=True)
class GripFeature:
    """Something on the brick that a non-actuated gripper could hold (R10 grippable, issue #50).

    The gripper is designed after the brick (issue #24), so a feature only says what it is, where
    it is and how big: no tine diameter, no clearance, no fork. ``kind`` is free text ('bore',
    'opening', 'slot', 'lip', ...) with one reserved meaning: a 'bore' is a round hole straight
    through Y, of diameter ``size_mm[0]``, which the builders cut into the solid. Every other
    kind is already part of the family's own geometry; give its XZ ``outline`` (section frame) if
    it has one, as for an opening that is one of the section's holes.
    """

    kind: str
    x_mm: float                     # centre, in the XZ section frame
    z_mm: float
    size_mm: tuple[float, float]    # extent in X and in Z (a bore: its diameter, twice)
    outline: Polygon | None = None  # the feature's XZ outline, if it has one

    @property
    def radius_mm(self) -> float:
        """Half the X extent: a bore's radius."""
        return self.size_mm[0] / 2


def bore(x_mm, z_mm, d_mm) -> GripFeature:
    """A round through-Y bore of diameter ``d_mm``, which the builders cut."""
    return GripFeature(kind=BORE, x_mm=float(x_mm), z_mm=float(z_mm), size_mm=(float(d_mm), float(d_mm)))


def opening(outline: Polygon, kind=OPENING) -> GripFeature:
    """A feature with an XZ outline that the geometry already has (a section hole, a slot);
    its centre and size are the outline's bounding box."""
    x0, z0, x1, z1 = outline.bounds
    return GripFeature(kind=kind, x_mm=(x0 + x1) / 2, z_mm=(z0 + z1) / 2, size_mm=(x1 - x0, z1 - z0),
                       outline=outline)


def grip_bores(features) -> tuple[GripFeature, ...]:
    """The round through-Y bores among a part's grip features."""
    return tuple(f for f in features if f.kind == BORE)


def grip_openings(features) -> tuple[Polygon, ...]:
    """The outlines of the through-Y openings among a part's grip features. Today's fixed fork is
    picked up through these (issue #12); the pickup trial and its predictor read them."""
    return tuple(f.outline for f in features if f.kind == OPENING and f.outline is not None)


@dataclass(frozen=True)
class Lattice:
    """Voxel pitch in X (U), depth in Y (D), course pitch in Z (H0), and voxels per brick."""

    U_mm: float
    D_mm: float
    H0_mm: float
    span_voxels: int


@dataclass(frozen=True)
class Prismatic:
    """2.5D geometry: an XZ section swept through Y. Builders derive solid and exact collision."""

    section: Callable[[Any, str], Section]
    extrusion: Callable[[Any], Extrusion]


@dataclass(frozen=True)
class Solid3D:
    """Full 3D geometry: a CadQuery solid per part, without the grip bores (builders cut them).

    ``collision`` optionally gives exact convex vertex sets (mm, brick frame); otherwise the solid
    is convex-decomposed. ``section`` optionally gives the predictor's 2D section; otherwise the
    solid is sliced on the XZ plane at y = 0.
    """

    solid: Callable[[Any, str], Any]
    collision: Callable[[Any, str], list[np.ndarray]] | None = None
    section: Callable[[Any, str], Section] | None = None


@dataclass(frozen=True)
class SimBodies:
    """What the MuJoCo scene needs for one part. Keys match the ported scene's brick bundle."""

    collision: list[np.ndarray]  # convex vertex sets, mm, brick frame
    collision_source: str        # 'exact' or 'decomposed'
    mass_g: float
    volume_mm3: float
    com_mm: list[float]
    inertia_g_mm2: list[list[float]]  # about the centre of mass
    mesh: Any                    # trimesh.Trimesh of the printed (bored) solid
    soft_collision: list[np.ndarray] = field(default_factory=list)


@dataclass(frozen=True)
class RingBrick:
    """One brick of a ring (issue #89): its part (one of the family's parts or of its ring kit), its
    course, its seat pose (mm, quaternion [w, x, y, z]) and the cells (i, j) of the course it fills."""

    kind: str                 # e.g. 'S2' (the straight brick), 'L3' (a turn), 'I3', 'K2' (the closer)
    part: str
    course: int
    pos_mm: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    cells: tuple[tuple[int, int], ...]
    name: str


@dataclass(frozen=True)
class RingKit:
    """What a family needs to close a ring (issue #89). ``parts`` are the parts used only in a ring;
    ``solid(p, part)`` is the CadQuery solid of each and ``collision(p, part)`` optionally its exact
    convex vertex sets (otherwise it is convex-decomposed); ``plan(p, n, courses)`` the ring's
    bricks in assembly order (RingBrick); ``base(p, n)`` the anchored base under course 0, as convex
    vertex sets (mm, world frame). engine/ring_cells.kit_from_cell builds one from a single cell."""

    parts: tuple[str, ...]
    solid: Callable[[Any, str], Any]
    plan: Callable[[Any, int, int], list]
    base: Callable[[Any, int], list]
    collision: Callable[[Any, str], list] | None = None


@dataclass(frozen=True)
class Family:
    """The contract. Every callable takes the family's params object ``p`` first."""

    name: str
    version: str
    description: str
    params_type: type
    bounds: dict[str, tuple[float, float]]  # tunable parameters only
    parts: tuple[str, ...]                  # e.g. ('A', 'B') for alternating courses
    geometry: Prismatic | Solid3D
    lattice: Callable[[Any], Lattice]
    part_for: Callable[[int, int], str]                      # (course, direction) -> part
    seat_pose: Callable[[Any, Placement], tuple]             # -> (pos_mm (3,), quat_wxyz (4,))
    insertion_axis: Callable[[Any, Placement], np.ndarray]   # unit (x, z) of final seating motion
    release_lift_mm: Callable[[Any], float]                  # release height above the seat
    base: Callable[[Any, int, int], Any]  # (p, voxels, direction) -> Section | cq shape | pieces
    pickup: Callable[[Any, str], tuple[GripFeature, ...]]    # (p, part) -> the part's grip features (R10)
    extra_rules: Callable[[Any, str, SimBodies], dict[str, float]] | None = None  # (p, part, bodies)
    ring: RingKit | None = None  # how the family turns a corner and closes a ring (issue #89)

    @property
    def defaults(self):
        return self.params_type()
