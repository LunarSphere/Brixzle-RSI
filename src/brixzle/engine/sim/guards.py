# From LunarSphere/aerial-assembly brixzle/src/brixzle/drone.py @ fc4f724
# Changed: only the motor-plane collision solids; the held mission supplies the body inertia.
# From LunarSphere/aerial-assembly brixzle/src/brixzle/cf/params.py @ fc4f724
# Changed: copy only the motor sites, in their original order and SI units.
# From LunarSphere/aerial-assembly brixzle/src/brixzle/gripper.py @ fc4f724
# Changed: express the clears_guards minimum separation in metres.
"""Crazyflie guard geometry and nominal vertical access for held placements."""
import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from shapely.geometry import MultiPoint, Point

from brixzle.engine.sim.drone import DroneConfig
from brixzle.engine.sim.scene import MM, _numbers
from brixzle.engine.structures import brick_name

# fixture.py at the same revision uses 139.6 mm for slot planning. The
# collision cylinders below span 2*(35.36 + 34) = 138.72 mm along body X/Y.
GUARD_RADIUS = .034
GUARD_HALF_H = .0153
FRAME_HALF = (.041, .041, .0087)
FRAME_Z = .0075
MOTOR_POS = ((.03536, -.03536, 0.), (-.03536, -.03536, 0.),
             (-.03536, .03536, 0.), (.03536, .03536, 0.))
MIN_CLEARANCE_M = .001


def guarded_drone():
    # crazyflow d28ec70 cf21B_500 does not specify the guard configuration.
    # Assume 43.4 g with legs, add the datasheet's guard-minus-leg increment
    # (aerial-assembly brixzle/configs/drone-cf21B.json @ fc4f724).
    return DroneConfig(body_mass_g=DroneConfig.body_mass_g + (37. - 34.))


def add_airframe(body):
    contact = dict(group="3", contype="1", conaffinity="1", mass="0")
    ET.SubElement(body, "geom", name="cf_frame", type="box", size=_numbers(FRAME_HALF),
                  pos=f"0 0 {FRAME_Z}", **contact)
    for i, pos in enumerate(MOTOR_POS):
        ET.SubElement(body, "geom", name=f"cf_guard{i}", type="cylinder", pos=_numbers(pos),
                      size=f"{GUARD_RADIUS} {GUARD_HALF_H}", **contact)


def _blocks_descent(pieces, bottom, motors):
    """Project each convex piece above the final guard bottom onto the motor plane."""
    for piece in pieces:
        upper = piece[piece[:, 2] >= bottom]
        if not len(upper):
            continue
        lower = piece[piece[:, 2] < bottom]
        xy = upper[:, :2]
        if len(lower):
            # All upper/lower chords include every cut edge; interior points do
            # not change the convex hull. Work only on pieces crossing this plane.
            delta = upper[:, None] - lower
            cut = lower[:, :2] + delta[:, :, :2]*((bottom-lower[:, 2])/delta[:, :, 2])[:, :, None]
            xy = np.vstack((xy, cut.reshape(-1, 2)))
        outline = MultiPoint(xy).convex_hull
        if any(outline.distance(motor) <= GUARD_RADIUS for motor in motors):
            return True
    return False


def placement_clearances(family, p, order, parts):
    """Sweep the four upright disks down to each nominal seat against earlier brick hulls."""
    prior, rows = [], []
    for b in order:
        fork, collision = parts[family.part_for(b.course, b.direction)]
        xyz, quat = family.seat_pose(p, b)
        rot = Rotation.from_quat(quat, scalar_first=True)
        xyz = np.asarray(xyz)*MM
        mount = xyz + rot.apply(np.asarray(fork["drone_pos_mm"])*MM)
        carrier = rot * Rotation.from_euler("z", -math.atan(fork["tine_dx_dy"]))
        motors = [Point(x, y) for x, y, _ in mount + carrier.apply(MOTOR_POS)]
        bottom = mount[2] - GUARD_HALF_H
        blocked = [name for name, top, pieces in prior
                   if top >= bottom and _blocks_descent(pieces, bottom, motors)]
        name = brick_name(b)
        rows.append(dict(brick=name, blocked_by=blocked))
        pieces = [xyz + rot.apply(piece*MM) for piece in collision]
        prior.append((name, max(piece[:, 2].max() for piece in pieces), pieces))
    return rows
