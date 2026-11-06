# From LunarSphere/aerial-assembly brixzle/src/brixzle/gripper.py @ a649663
# Changed: only the fork's collision and mass are built (capsule tines as in build_gripper, a plate box, and a bracket box for its mass); no CadQuery solid, pressed pins, COM-balancing shift or gripper rules. Tines come from family.pickup bores; the frame is the carried brick's (tine tops on the bore tops), and the drone sits on the vertical through the brick's centre of mass, clear_guard + 15.3 mm above the brick top as in carried_pose.
"""The fixed passive fork (R12): two straight Y tines, a plate behind them, the drone above.

Frame (mm): the carried brick's own frame at its nominal carry pose. The tines run along +Y from
the plate face (y_face, behind the brick's near face) to y_tip (beyond its far face); each tine's
top touches its bore's top, so the brick hangs on the tines. The drone is a point mass with the
Crazyflie's inertia, above the brick's centre of mass.
"""

from dataclasses import asdict, dataclass
import math

import numpy as np

from brixzle.engine.sim.drone import DroneConfig, HoverGains
from brixzle.families import builders as B

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

    def to_dict(self):
        return asdict(self)


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
               gains: HoverGains = HoverGains()):
    """Fork geometry and the rigid drone+fork body's mass properties, in the carry frame (mm, g)."""
    bores = family.pickup(p, part).bores
    bodies = B.sim_bodies(family, p, part)
    D = family.lattice(p).D_mm
    r = fp.tine_d/2
    tines = [(float(b.x_mm), float(b.z_mm + b.radius_mm - r)) for b in bores]
    y_face, y_tip = -(D/2 + fp.gap), D/2 + fp.protrude
    top = max(float(np.max(c[:, 2])) for c in bodies.collision)
    drone_pos = np.array([bodies.com_mm[0], 0.0, top + fp.clear_guard - GUARD_BOTTOM])
    xs, zs = [t[0] for t in tines], [t[1] for t in tines]
    plate_lo = np.array([min(xs) - r - fp.plate_margin, y_face - fp.plate_t, min(zs) - r - fp.wall])
    plate_hi = np.array([max(xs) + r + fp.plate_margin, y_face, drone_pos[2] + HEADER_BOTTOM])
    b_size = np.array([2*(HEADER_HALF[0] + 2), 2*(HEADER_HALF[1] + 2), fp.bracket_t])
    b_com = drone_pos + [0, 0, HEADER_BOTTOM - fp.bracket_t/2]
    parts = []
    m = float(np.prod(plate_hi - plate_lo))*PLA_DENSITY
    parts.append((m, (plate_lo + plate_hi)/2, _box_inertia(m, plate_hi - plate_lo)))
    m = float(np.prod(b_size))*PLA_DENSITY
    parts.append((m, b_com, _box_inertia(m, b_size)))
    L = y_tip - y_face
    for x, z in tines:
        m = math.pi*r*r*L*PLA_DENSITY
        I = np.diag([m*L*L/12, m*r*r/2, m*L*L/12])
        parts.append((m, (x, (y_face + y_tip)/2, z), I))
    gripper_g = sum(m for m, _, _ in parts)
    J = np.diag(gains.J)*1e9  # kg m^2 -> g mm^2
    parts.append((drone.mass_g, drone_pos, J))
    mass, com, inertia = _combine(parts)
    return {"tines": tines, "tine_r": r, "y_face": y_face, "y_tip": y_tip,
            "plate": (plate_lo.tolist(), plate_hi.tolist()), "drone_pos_mm": drone_pos.tolist(),
            "gripper_mass_g": gripper_g, "body_mass_kg": mass*1e-3, "com_mm": com.tolist(),
            "inertia_g_mm2": inertia.tolist(), "params": fp.to_dict()}
