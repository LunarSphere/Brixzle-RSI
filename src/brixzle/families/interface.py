"""The brick family contract: what every family module exports, and the data types it uses.

A family module ``brixzle/families/<name>/family.py`` exports ``FAMILY = Family(...)``. The
engine only ever talks to a family through this object, and ``contract.check_family`` checks it.
See docs/family-interface.md for the prose version the design agent reads.

Units: geometry in mm and degrees (fields carry ``_mm``/``_deg``), mass in g; conversion to SI
happens at the MuJoCo boundary. Quaternions are [w, x, y, z]. The brick frame has X along the
lattice, Y across it (the fork's tines slide along Y) and Z up; z = 0 is the seat.
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
    holes: tuple[Polygon, ...] = ()        # through-Y lightening holes (not the fork bores)
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


@dataclass(frozen=True)
class Bore:
    """A straight through-Y bore for one tine of the fixed fork, with an optional lead-in chamfer
    at its mouth (the near face, where the tine enters).

    The chamfer follows Whitney 1982 (Fig. 3): ``chamfer_w_mm`` is its radial width, so the mouth
    radius is ``radius_mm + chamfer_w_mm``; ``chamfer_deg`` is the face's angle from the bore's end
    face, so 90 is a straight wall and lower is flatter. A tine slides down it only if
    tan(chamfer_deg) > mu. No chamfer (width 0) is a plain bore. The pickup predictor
    (engine/predictor/pickup.py) uses the chamfer; the sim's bored collision does not cut it yet.
    """

    x_mm: float
    z_mm: float
    d_mm: float          # tine diameter
    clearance_mm: float  # radial clearance around the tine
    chamfer_w_mm: float = 0.0   # lead-in width at the mouth (radial)
    chamfer_deg: float = 45.0   # lead-in face angle from the bore's end face; used when the width > 0

    def __post_init__(self):
        if not (math.isfinite(self.chamfer_w_mm) and self.chamfer_w_mm >= 0):
            raise ValueError("chamfer_w_mm must be finite and >= 0")
        if not (math.isfinite(self.chamfer_deg) and 0 < self.chamfer_deg < 90):
            raise ValueError("chamfer_deg must be between 0 and 90 (exclusive)")

    @property
    def radius_mm(self) -> float:
        return self.d_mm / 2 + self.clearance_mm


@dataclass(frozen=True)
class PickupFeatures:
    bores: tuple[Bore, ...]


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
    """Full 3D geometry: a CadQuery solid per part, without the fork bores (builders cut them).

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
    pickup: Callable[[Any, str], PickupFeatures]
    extra_rules: Callable[[Any, str, SimBodies], dict[str, float]] | None = None  # (p, part, bodies)

    @property
    def defaults(self):
        return self.params_type()
