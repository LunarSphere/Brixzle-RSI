# From LunarSphere/aerial-assembly brixzle/src/brixzle/gripper.py @ a649663
# Changed: only the fork's collision and mass are built (capsule tines as in build_gripper, a plate box, and a bracket box for its mass, plus optional printed guide boxes); no CadQuery solid, pressed pins, COM-balancing shift or gripper rules. Tines come from the bores among family.pickup's grip features; the frame is the carried brick's (tine tops on the bore tops), and the drone sits on the vertical through the brick's centre of mass, clear_guard + 15.3 mm above the brick top as in carried_pose.
"""The fixed passive fork (R12): two straight Y tines, a plate behind them, the drone above.

Frame (mm): the carried brick's own frame at its nominal carry pose. The tines run along +Y from
the plate face (y_face, behind the brick's near face) to y_tip (beyond its far face); each tine's
top touches its bore's top, so the brick hangs on the tines. The drone is a point mass with the
Crazyflie's inertia, above the brick's centre of mass.
"""

from dataclasses import asdict, dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

from brixzle.engine.sim.drone import DroneConfig, HoverGains
from brixzle.engine.sim.guards import MIN_CLEARANCE_M
from brixzle.engine.sim.scene import MM
from brixzle.families import builders as B
from brixzle.families.interface import grip_bores

PLA_DENSITY = B.PLA_DENSITY
GUARD_BOTTOM = -15.3     # mm, prop-guard bottom below the drone origin (aerial-assembly gripper.py)
HEADER_BOTTOM = -2.4     # mm, bottom header pins: the bracket hangs here
HEADER_HALF = (9.4, 11.4)


@dataclass(frozen=True)
class ForkParams:
    tine_d: float = 2.0          # rod diameter
    protrude: float = 4.0        # tine length beyond the brick's far face
    gap: float = 8.0             # plate to the brick's near face
    clear_guard: float = 8.0     # brick top below the prop-guard bottom
    plate_t: float = 1.6         # plate thickness (y)
    plate_margin: float = 3.0    # plate beyond the outer tines (x)
    bracket_t: float = 2.0       # mount bracket thickness (z)
    wall: float = 1.2            # plate below the lowest tine

    def __post_init__(self):
        if not math.isfinite(self.clear_guard) or self.clear_guard < MIN_CLEARANCE_M/MM:
            raise ValueError("guard clearance must be finite and at least 1 mm")

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class ForkGuide:
    """Solid rectangular guide in the fork carry frame (SI, full side lengths)."""
    material: str
    size_m: tuple
    centre_m: tuple
    quat_wxyz: tuple = (1., 0., 0., 0.)

    def __post_init__(self):
        if self.material not in ("PLA", "TPU"):
            raise ValueError("fork guide material must be PLA or TPU")
        for name, length in (("size_m", 3), ("centre_m", 3), ("quat_wxyz", 4)):
            values = getattr(self, name)
            if (not isinstance(values, (list, tuple)) or len(values) != length
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
                raise ValueError(f"fork guide {name} requires {length} finite numbers")
            object.__setattr__(self, name, tuple(values))
        if min(self.size_m) <= 0 or not math.isclose(math.hypot(*self.quat_wxyz), 1., abs_tol=1e-6):
            raise ValueError("fork guide needs positive dimensions and a unit quaternion")

    @property
    def density_kg_m3(self):
        # As in #167, use the PLA density for solid TPU until weighed.
        return PLA_DENSITY*1e6


def _box_inertia(m, size):
    a, b, c = size
    return m/12*np.diag([b*b + c*c, a*a + c*c, a*a + b*b])


def _combine(parts):
    """(mass, com, inertia about com) of [(mass, com, inertia about own com)]."""
    M = sum(m for m, _, _ in parts)
    com = sum(m*np.asarray(c, float) for m, c, _ in parts)/M
    I = np.zeros((3, 3))
    for m, c, Ic in parts:
        d = np.asarray(c, float) - com
        I += Ic + m*(d @ d*np.eye(3) - np.outer(d, d))
    return M, com, I


def build_fork(family, p, part, fp: ForkParams = ForkParams(), drone: DroneConfig = DroneConfig(),
               gains: HoverGains = HoverGains(), tines=None, *, y_tip_mm=None, with_drone=True, tine_dx_dy=0., guides=()):
    """Fork geometry and the rigid drone+fork body's mass properties, in the carry frame (mm, g).

    ``tines`` are the tine centres (x, z) in the carry frame; by default each sits at the top of
    its bore (tine top on bore top). With oblique tines, these centres are at y = 0 and
    ``tine_dx_dy`` is their fixed slope in plan. Through openings they are the seats (issue #12). A short
    fork can end inside the window at ``y_tip_mm``. Set ``with_drone=False`` for a separate wrist.
    Optional printed guides add their solid mass and inertia in the same frame."""
    bodies = B.sim_bodies(family, p, part)
    D = family.lattice(p).D_mm
    r = fp.tine_d/2
    if tines is None:
        tines = [(float(b.x_mm), float(b.z_mm + b.radius_mm - r)) for b in grip_bores(family.pickup(p, part))]
    tines = [(float(x), float(z)) for x, z in tines]
    y_face, y_tip = -(D/2 + fp.gap), D/2 + fp.protrude
    if y_tip_mm is not None:
        if not math.isfinite(y_tip_mm) or y_tip_mm - r <= y_face:
            raise ValueError("fork tip must be finite and extend past the plate by more than its radius")
        y_tip = y_tip_mm
    if not math.isfinite(tine_dx_dy):
        raise ValueError("tine slope must be finite")
    top = max(float(np.max(c[:, 2])) for c in bodies.collision)
    drone_pos = np.array([bodies.com_mm[0], 0.0, top + fp.clear_guard - GUARD_BOTTOM])
    xs, zs = [t[0] + y_face*tine_dx_dy for t in tines], [t[1] for t in tines]
    plate_lo = np.array([min(xs) - r - fp.plate_margin, y_face - fp.plate_t, min(zs) - r - fp.wall])
    plate_hi = np.array([max(xs) + r + fp.plate_margin, y_face, drone_pos[2] + HEADER_BOTTOM])
    b_size = np.array([2*(HEADER_HALF[0] + 2), 2*(HEADER_HALF[1] + 2), fp.bracket_t])
    b_com = drone_pos + [0, 0, HEADER_BOTTOM - fp.bracket_t/2]
    parts = []
    m = float(np.prod(plate_hi - plate_lo))*PLA_DENSITY
    parts.append((m, (plate_lo + plate_hi)/2, _box_inertia(m, plate_hi - plate_lo)))
    m = float(np.prod(b_size))*PLA_DENSITY
    parts.append((m, b_com, _box_inertia(m, b_size)))
    axis = np.array([tine_dx_dy, 1., 0.])
    scale = np.linalg.norm(axis)
    axis /= scale
    L = (y_tip - y_face)*scale
    y_mid = (y_face + y_tip)/2
    for x, z in tines:
        m = math.pi*r*r*L*PLA_DENSITY
        I = m*(L*L/12*np.eye(3) + (r*r/2-L*L/12)*np.outer(axis, axis))
        parts.append((m, (x+tine_dx_dy*y_mid, y_mid, z), I))
    for guide in guides:
        m = float(np.prod(guide.size_m)) * guide.density_kg_m3 * 1000
        R = Rotation.from_quat(guide.quat_wxyz, scalar_first=True).as_matrix()
        inertia = R @ _box_inertia(m, np.asarray(guide.size_m)*1000) @ R.T
        parts.append((m, np.asarray(guide.centre_m)*1000, inertia))
    gripper_g = sum(m for m, _, _ in parts)
    if with_drone:
        J = np.diag(gains.J)*1e9  # kg m^2 -> g mm^2
        parts.append((drone.mass_g, drone_pos, J))
    mass, com, inertia = _combine(parts)
    return {"tine_dx_dy": tine_dx_dy, "tines": tines, "tine_r": r, "y_face": y_face, "y_tip": y_tip,
            "guides": [asdict(g) for g in guides], "plate": (plate_lo.tolist(), plate_hi.tolist()),
            "drone_pos_mm": drone_pos.tolist(),
            "gripper_mass_g": gripper_g, "body_mass_kg": mass*1e-3, "com_mm": com.tolist(),
            "inertia_g_mm2": inertia.tolist(), "params": fp.to_dict()}
