"""Crazyflie fork mission (#191), simulated, uncalibrated contact. SI except drop classifier outputs."""
from collections import deque
from dataclasses import asdict, dataclass, field, replace
from functools import cache
import math
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import shapely
from shapely.geometry import Point, box
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from brixzle.engine.predictor.pickup import _seat, _fork_region, capture_region
from brixzle.engine.structures import brick_name
from brixzle.engine.sim import trials as T
from brixzle.engine.sim import approach_error
from brixzle.engine.sim.config import read_json
from brixzle.engine.sim.drone import CF21B, DroneConfig, G, HoverGains
from brixzle.engine.sim.fork import ForkParams, build_fork
from brixzle.engine.sim.pickup import (
    PickConfig, _R, _wxyz, add_fork_parts, bored_collision, withdrawal_stroke_mm,
    fixture_pieces, hover_wrench, minimum_jerk, vertical_sweep, tine_segment, SEAT,
)
from brixzle.engine.sim.scene import MM, _numbers, activate, body_pose, build_scene
from brixzle.families import builders as B
from brixzle.families.capture import capture_geometry
from brixzle.families.interface import Extrusion, Placement, Prismatic, grip_bores, grip_openings


@dataclass(frozen=True)
class HeldSample:
    """World translation in metres and pitch/yaw in radians."""
    dx_m: float = 0.
    dy_m: float = 0.
    dyaw_rad: float = 0.
    dz_m: float = 0.
    dpitch_rad: float = 0.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError("held placement errors must be finite")


@cache
def _measured_errors():
    data = approach_error.DATA_DIR
    xy = approach_error.error_model(approach_error.load(data / "error-exp3-r4.json"), "empirical")
    return xy, read_json(data / "hover-exp3-r2.json")["approaches"]


def mission_samples(seed=0):
    """Independent station and target approaches; preserve the RMS triplet in each hover row."""
    rng = np.random.default_rng(seed)
    model, hover = _measured_errors()

    def draw():
        dx, dy = model._xy(rng)
        row = rng.choice(hover)
        dz, pitch, yaw = rng.choice([-1, 1], size=3)
        return HeldSample(dx*MM, dy*MM, yaw*math.radians(row["yaw_rms_deg"]),
                          dz*row["z_rms_mm"]*MM, pitch*math.radians(row["pitch_rms_deg"]))

    return draw(), draw()


@dataclass(frozen=True)
class WristConfig:
    """Guess (#193). Triples are X/Y/Z, i.e. roll/pitch/yaw for the hinges.

    Linear k and c in N/m and N s/m, angular in N m/rad and N m s/rad.
    centre_m is expressed in the carried brick frame. rigid omits all six
    joints, the infinite-stiffness limit without non-finite configuration.
    """
    linear_k: tuple = (80., 80., 240.)
    linear_c: tuple = (1.5, 1.5, 2.)
    angular_k: tuple = (0.12, 0.12, 0.08)
    angular_c: tuple = (0.002, 0.002, 0.001)
    centre_m: tuple = (0., 0., 0.)
    rigid: bool = False

    def __post_init__(self):
        if not isinstance(self.rigid, bool):
            raise ValueError("rigid must be a boolean")
        for name in ("linear_k", "linear_c", "angular_k", "angular_c", "centre_m"):
            v = getattr(self, name)
            if len(v) != 3 or not all(math.isfinite(x) for x in v):
                raise ValueError(f"{name} needs three finite components")
            if name != "centre_m" and min(v) < 0:
                raise ValueError(f"{name} must be nonnegative")


@dataclass(frozen=True)
class HeldConfig:
    wrist: WristConfig = field(default_factory=WristConfig)
    physics: T.Physics = field(default_factory=lambda: T.TrialConfig().physics)
    drone: DroneConfig = field(default_factory=DroneConfig)
    transport_m_s: float = 0.08
    descent_m_s: float = PickConfig.v_lift
    press_depth_m: float = 0.003        # maximum commanded distance below the seat
    press_force_N: float = 0.45         # optional lower ceiling on the R12 force budget
    hold_s: float = PickConfig.hold_s
    release_m_s: float = PickConfig.v_release
    wobble_m: float = 0.
    wobble_hz: float = 2.
    tine_d_m: float = ForkParams.tine_d * MM
    tip_engagement_m: float = 0.004    # past mid-depth for an oblique fork
    oblique_yaw_rad: float = math.atan(0.55)  # fixed candidate mount angle, guess
    release_at_contact: bool = False

    def __post_init__(self):
        if not isinstance(self.release_at_contact, bool):
            raise ValueError("release_at_contact must be a boolean")
        if self.drone.mass_g <= 0:
            raise ValueError("drone needs positive body mass")
        if not math.isfinite(self.oblique_yaw_rad) or not 0 <= self.oblique_yaw_rad < math.pi/2:
            raise ValueError("oblique fork yaw must be in [0, pi/2)")
        positive = (self.transport_m_s, self.descent_m_s, self.press_force_N, self.release_m_s, self.tine_d_m,
                    self.tip_engagement_m)
        nonnegative = (self.press_depth_m, self.hold_s, self.wobble_m, self.wobble_hz)
        if any(not math.isfinite(v) or v <= 0 for v in positive):
            raise ValueError("speeds, force limit and fork dimensions must be finite and positive")
        if any(not math.isfinite(v) or v < 0 for v in nonnegative):
            raise ValueError("depth, hold and wobble must be finite and nonnegative")


def _ordered_bands(core, p):
    """Extrusions may list their bands and their endpoints in either Y direction."""
    bands = core.geometry.extrusion(p).resolved_bands()
    return sorted((a, b, za, zb) if a < b else (b, a, zb, za) for a, b, za, zb in bands)


class ForkFitError(ValueError):
    """The candidate fork cannot enter the carrying features."""


def _fork_fit(family, p, part, cfg):
    cap = capture_geometry(family.geometry)
    core = replace(family, geometry=Prismatic(cap.core_section, cap.core_extrusion)) if cap else family
    if not isinstance(core.geometry, Prismatic):
        raise ForkFitError("held fork needs a swept section with known through-Y geometry")
    features = core.pickup(p, part)
    windows = grip_openings(features)
    r = cfg.tine_d_m / (2*MM)
    depth = family.lattice(p).D_mm
    bands = _ordered_bands(core, p)
    slope = 0.
    tip = None
    if windows:
        if len(windows) < 2:
            raise ForkFitError("held fork needs two carrying windows")
        regions = [capture_region(w, bands, r) for w in windows]
        if any(q.is_empty for q in regions):
            tip = cfg.tip_engagement_m/MM
            if not -depth/2 < tip < depth/2:
                raise ForkFitError("short fork tips must end inside the window")
            clipped = [(a, min(b, tip), za, za+(zb-za)*(min(b, tip)-a)/(b-a))
                       for a, b, za, zb in bands if a < tip]
            # Try a fixed fork yaw in either direction so a level shaft can reach past mid-depth.
            for slope in (math.tan(cfg.oblique_yaw_rad), -math.tan(cfg.oblique_yaw_rad)):
                regions = [capture_region(w, clipped, r*math.hypot(1, slope), -math.atan(slope)) for w in windows]
                if all(not q.is_empty for q in regions):
                    break
        if any(q.is_empty for q in regions):
            raise ForkFitError("single-sided fork does not fit the windows")
        seats = [_seat(q) for q in regions]
        region = _fork_region(regions, seats)
        if region.is_empty:
            raise ForkFitError("no common fork offset fits the carrying windows")
        aim = shapely.maximum_inscribed_circle(region).coords[0]
    else:
        bores = [b for b in grip_bores(features) if b.size_mm[0] >= 16 and b.radius_mm > r]
        if len(bores) < 2:
            raise ForkFitError("held fork needs through-Y windows or two fitting bores at least 16 mm across")
        seats = [(b.x_mm, b.z_mm + b.radius_mm - r) for b in bores]
        aim = (0., -min(b.radius_mm-r for b in bores))
    fork = build_fork(family, p, part, ForkParams(tine_d=2*r), tines=seats, y_tip_mm=tip, with_drone=False, tine_dx_dy=slope)
    collision = bored_collision(core, p, part)
    if cap:
        collision.extend(cap.feature_sets(p, part))
    return fork, collision, core, np.array([aim[0], 0, aim[1]])*MM


def _station(core, p, part):
    """Two stationary saddles at the outer bands, cut by the same vertical sweep as PickRig."""
    outer = core.geometry.section(p, part).outer
    x0, z0, x1, z1 = outer.bounds
    blank = box(x0, z0-2, x1, z0+(z1-z0)/3)
    pocket = vertical_sweep(outer, z1-z0+2)
    shape = blank.difference(pocket)
    bands = _ordered_bands(core, p)
    rails = []
    for a, b, za, zb in (bands[0], bands[-1]):
        width = min(2., (b-a)/4)
        start, end = (a, a+width) if not rails else (b-width, b)
        slope = (zb-za)/(b-a)
        rails.append((start, end, za+(start-a)*slope, za+(end-a)*slope))
    ext = Extrusion(core.lattice(p).D_mm, tuple(rails))
    return [piece for poly in getattr(shape, "geoms", [shape]) for piece in B._pieces(poly, ext)]


def _carrier(cfg, fork):
    wrist = cfg.wrist

    # The printed tines run along drone-local +Y. The brick hangs at a known yaw on oblique tines.
    brick_in_drone = Rotation.from_euler("z", math.atan(fork["tine_dx_dy"]))

    def add(asset, world):
        base = ET.SubElement(world, "body", name="carrier", pos="1 -1 1")
        ET.SubElement(base, "freejoint", name="carrier_free")
        ET.SubElement(base, "inertial", mass=str(cfg.drone.mass_g*1e-3), pos="0 0 0",
                      diaginertia=_numbers(HoverGains().J))
        body = ET.SubElement(base, "body", name="wrist", quat=_numbers(_wxyz(brick_in_drone)),
                             pos=_numbers(brick_in_drone.apply(-MM*np.array(fork["drone_pos_mm"]))))
        # Lumped TPU carriage mass/inertia, guesses until #193 (SI).
        ET.SubElement(body, "inertial", mass="0.001", diaginertia="2e-7 2e-7 2e-7", pos="0 0 0")
        if not wrist.rigid:
            for kind, ks, cs in (("slide", wrist.linear_k, wrist.linear_c),
                                 ("hinge", wrist.angular_k, wrist.angular_c)):
                for axis, k, c in zip(np.eye(3), ks, cs):
                    ET.SubElement(body, "joint", type=kind, axis=_numbers(axis), pos=_numbers(wrist.centre_m),
                                  stiffness=str(k), damping=str(c))
        jaw = ET.SubElement(body, "body", name="fork")
        add_fork_parts(jaw, fork)
    return add


@dataclass(frozen=True)
class _ContactLoads:
    supported: bool
    on_floor: bool
    reaction_N: float       # magnitude of the net external force on the brick and fork
    normals_N: float        # sum before opposing contact loads cancel
    fork_brick_N: float
    fork_strike_N: float
    fork_force_N: np.ndarray  # net world-frame contact force on the fork alone


class _ForceWindow:
    """Mean contact vector over the last 50 ms, with zero contact before the phase."""
    def __init__(self, timestep):
        self.steps = max(1, round(0.050 / timestep))
        self.samples = deque()
        self.total = np.zeros(3)

    def mean(self, force):
        if len(self.samples) == self.steps:
            self.total -= self.samples.popleft()
        self.samples.append(force)
        self.total += force
        return self.total / self.steps


class HeldRig:
    """Compile once per family, placement and gripper. Each trial resets all moving bodies."""
    def __init__(self, family, p, placement, gripper_cfg=HeldConfig(), *, structure=None):
        self.family, self.p, self.cfg = family, p, gripper_cfg
        lat = family.lattice(p)
        self.refs = None
        if structure is not None:
            from brixzle.engine.sim.assemble import connectivity

            self.target = placement
            self.placed = structure.bricks[:structure.bricks.index(placement)]
            base, = structure.bases
            graph = connectivity(structure, lat.span_voxels)
            prior = {}
            self.refs = []
            for i, b in enumerate([*self.placed, placement]):
                parents = graph.predecessors(brick_name(b))
                self.refs.append(next((prior[name] for name in parents if name in prior), None))
                prior[brick_name(b)] = (i, b)
            self.ref = self.refs[-1]
        elif isinstance(placement, Placement):
            if not isinstance(placement.i0, int) or placement.direction not in (-1, 1):
                raise ValueError("placement requires an integer voxel and direction +1 or -1")
            # Translate a full-seat layout to the requested voxel.
            placed, target, base = T.rig_layout(lat, placement.course)
            shift = placement.i0 - target.i0
            self.placed = [replace(b, i0=b.i0+shift, direction=placement.direction) for b in placed]
            self.target = placement
            base = (base[0]+shift, base[1]+shift, placement.direction)
        else:
            layout = T.edge_layout if placement in T.EDGE_CASES else T.rig_layout
            self.placed, self.target, base = layout(lat, placement)
        self.j = len(self.placed)
        parts = [family.part_for(b.course, b.direction) for b in (*self.placed, self.target)]
        part = parts[-1]
        self.fork, collision, core, self.aim = _fork_fit(family, p, part, self.cfg)
        features = core.pickup(p, part)
        self.windows = grip_openings(features)
        self.bores = [b for b in grip_bores(features) if b.size_mm[0] >= 16 and b.radius_mm > self.fork["tine_r"]]
        bands = _ordered_bands(core, p)
        self.mid_shift = next(za-a*(zb-za)/(b-a) for a, b, za, zb in bands if a <= 0 <= b)
        bodies = {k: B.sim_bodies(family, p, k) for k in set(parts)}
        bodies["held_target"] = replace(bodies[part], collision=collision)
        if structure is not None:
            # Previously placed pieces retain the same physical windows as the carried piece.
            for key in set(parts[:-1]):
                previous = collision if key == part else _fork_fit(family, p, key, self.cfg)[1]
                bodies[key] = replace(bodies[key], collision=previous)
        pos, q = family.seat_pose(p, self.target)
        self.seat_R = _R(q)
        self.station_R = self.seat_R if structure is None else Rotation.identity()
        station_x = pos[0]*MM if structure is None else 0.
        self.station = np.array([station_x, 0., 0.]) + self.station_R.apply([0., -0.10, 0.])
        if isinstance(family.geometry, Prismatic) and part == family.part_for(0, 1):
            station_origin, _ = family.seat_pose(p, SEAT)
            pieces = [v-station_origin for v in fixture_pieces(family, p, part)]
        else:
            pieces = _station(core, p, part)
        fixture = [self.station_R.apply(piece) + self.station/MM for piece in pieces]
        self.model, _, self.info = build_scene(family, p, bodies, [base], len(parts), self.cfg.physics,
                                               body_parts=[*parts[:-1], "held_target"], visual=False,
                                               extra=_carrier(self.cfg, self.fork), static=fixture)
        m = self.model
        self.data = d = mujoco.MjData(m)
        self.owner = T._brick_geoms(m, self.info)
        self.target_body = self.info["bodies"][self.j]
        self.gripper_kg = float(m.body_subtreemass[m.body("wrist").id])
        self.jaw = m.body("fork").id
        self.carrier = m.body("carrier").id
        self.fa = m.joint("carrier_free").qposadr[0]
        self.fv = m.joint("carrier_free").dofadr[0]
        self.drone_in_brick = Rotation.from_euler("z", -math.atan(self.fork["tine_dx_dy"]))
        self.fork_axis = np.array([self.fork["tine_dx_dy"], 1., 0.])
        self.mount = np.array(self.fork["drone_pos_mm"])*MM
        self.fork_geoms = m.geom_bodyid == self.jaw
        self.target_geoms = m.geom_bodyid == self.target_body
        self.assembly_geoms = self.target_geoms | self.fork_geoms
        self.payload = self.gripper_kg + float(m.body_mass[self.target_body])
        self.force_cap = max(0., min(self.cfg.press_force_N,
                                     PickConfig.force_share*self.cfg.drone.thrust_margin_N(self.payload*1000)))
        gains = HoverGains()
        omega = 2*math.pi*gains.att_hz
        self.gains = (np.array(gains.kp), np.array(gains.kd), np.array(gains.J)*omega**2,
                      2*gains.att_damping*np.array(gains.J)*omega)
        if structure is None:
            self.ref = (0, self.placed[0]) if self.placed else None
        self.tol = T.Tolerance()
        self.stroke_m = withdrawal_stroke_mm(lat.D_mm, self.fork["y_tip"]) * MM
        # The convex envelope closes the windows: a tine inside a hole is still
        # engaged even if it momentarily has no contact with the brick.
        all_pieces = collision + bodies[part].soft_collision
        self.envelope = ConvexHull(np.vstack(all_pieces)*MM).equations
        parked = [self.carrier, m.body("wrist").id, self.jaw]
        m.body_gravcomp[parked] = 1.
        for i, b in enumerate(self.placed):
            activate(m, d, self.info, i, *family.seat_pose(p, b))
        if self.placed:
            T.run_until_settled(m, d, self.info, 0, replace(self.tol, max_time=0.3),
                                self.owner, watch=range(1, self.j))
        activate(m, d, self.info, self.j, self.station/MM, _wxyz(self.station_R))
        T.run_until_settled(m, d, self.info, self.j, self.tol, self.owner)
        m.body_gravcomp[parked] = 0.
        self.start = mujoco.MjData(m)
        mujoco.mj_copyData(self.start, m, d)

    def _contacts(self):
        """Separate external support loads, fork/brick contact and any direct fork strike."""
        d = self.data
        pair = d.contact.geom
        assembly = self.assembly_geoms[pair]
        active = np.flatnonzero(assembly.any(axis=1) & (d.contact.efc_address >= 0))
        supported = floor = False
        normals = grip = strike = 0.
        resultant = np.zeros(3)
        fork_force = np.zeros(3)
        force = np.empty(6)
        for i in active:
            mujoco.mj_contactForce(self.model, d, i, force)
            normal = float(force[0])
            # Contact frame rows are world vectors. Force is on geom2, opposite on geom1.
            world_force = force[:3] @ d.contact[i].frame.reshape(3, 3)
            fork1, fork2 = self.fork_geoms[pair[i]]
            if fork1 != fork2:
                fork_force += world_force if fork2 else -world_force
            if assembly[i].all():
                if self.target_geoms[pair[i]].any() and self.fork_geoms[pair[i]].any():
                    grip += normal
                continue
            normals += normal
            side = int(assembly[i, 1])
            resultant += world_force if side else -world_force
            if self.target_geoms[pair[i, side]]:
                supported = True
                floor |= pair[i, 1-side] == self.info["floor"]
            else:
                strike += normal
        return _ContactLoads(supported, bool(floor), float(np.linalg.norm(resultant)), normals, grip, strike,
                             fork_force)

    def _inserted(self):
        """Check the actual depth and where each straight shaft crosses the brick mid-plane."""
        pos, q = body_pose(self.data, self.info, self.j)
        inv = _R(q).inv()
        rot = self.data.xmat[self.jaw].reshape(3, 3)
        origin = self.data.xpos[self.jaw]/MM - pos
        depth = self.fork["y_tip"] - PickConfig.insert_tol_mm
        inserted = []
        for i, (x, z) in enumerate(self.fork["tines"]):
            segment = tine_segment(self.fork, x, z)
            a, b = inv.apply(segment @ rot.T + origin)
            if not a[1] < 0 < b[1] or b[1] + self.fork["tine_r"] < depth:
                inserted.append(False)
                continue
            xmid, _, zmid = a - a[1]*(b-a)/(b[1]-a[1])
            if self.windows:
                inside = self.windows[i].covers(Point(xmid, zmid-self.mid_shift))
            else:
                bore = self.bores[i]
                inside = math.hypot(xmid-bore.x_mm, zmid-bore.z_mm) <= bore.radius_mm
            inserted.append(bool(inside))
        return inserted

    def _outcome(self, floor):
        err, angle = T.pose_error(self.family, self.p, self.data, self.info, self.j, self.target, self.ref)
        return T.classify(self.family.lattice(self.p), err, angle, self.tol, floor), err.tolist(), angle

    def _withdrawn(self):
        d = self.data
        pos, quat = body_pose(d, self.info, self.j)
        inv = _R(quat).inv()
        normals, offsets = self.envelope[:, :3], self.envelope[:, 3]
        if np.any(self.fork_geoms[d.contact.geom].any(axis=1)
                  & self.target_geoms[d.contact.geom].any(axis=1)):
            return False
        jaw, fork = self.jaw, self.fork
        rot = d.xmat[jaw].reshape(3, 3)
        for x, z in fork["tines"]:
            r = fork["tine_r"]
            ends = tine_segment(fork, x, z)*MM
            local = inv.apply(ends @ rot.T + d.xpos[jaw] - pos*MM)
            # Both capsule ends clear the same hull face by more than the radius.
            if not np.any(np.min(local @ normals.T + offsets, axis=0) > r*MM):
                return False
        return True

    def _control(self, pos, vel, acc, orientation, loaded, pressing):
        d, v, drone = self.data, self.fv, self.cfg.drone
        mass = drone.mass_g*1e-3 + (self.payload if loaded else self.gripper_kg)
        force, torque = hover_wrench(mass, *self.gains, d.qpos[self.fa:self.fa+3],
                                     _R(d.qpos[self.fa+3:self.fa+7]), d.qvel[v:v+6],
                                     pos, vel, acc, orientation*self.drone_in_brick)
        # No reverse thrust. Limit the commanded downward load as well as total thrust.
        lower = max(0., (drone.mass_g*1e-3 + self.payload)*G - self.force_cap) if pressing else 0.
        force[2] = np.clip(force[2], lower, drone.thrust_max_N())
        lateral_max = math.sqrt(max(0., drone.thrust_max_N()**2-force[2]**2))
        lateral = math.hypot(*force[:2])
        if lateral > lateral_max:
            force[:2] *= lateral_max/lateral
        d.xfrc_applied[self.carrier, :3] = force
        d.xfrc_applied[self.carrier, 3:] = torque
        self.peak_thrust = max(self.peak_thrust, float(np.linalg.norm(force)))

    def _move(self, end, orientation, speed, *, loaded=False, pressing=False, observe=None,
              hold=0., wobble=False):
        """Drive the drone COM along a smooth line. An observer can stop a segment on contact."""
        m, d = self.model, self.data
        dt = m.opt.timestep
        origin = self.command
        rotation = self.orientation
        turn = (orientation*rotation.inv()).as_rotvec()
        turning = bool(np.any(turn))
        delta = end-origin
        duration = max(dt, float(np.linalg.norm(delta))/speed)
        steps = math.ceil((duration+hold)/dt)
        for i in range(1, steps+1):
            t = i*dt
            s, ds, dds = minimum_jerk(t, duration)
            pos, vel, acc = origin+s*delta, ds*delta, dds*delta
            if wobble and self.cfg.wobble_m:
                omega = 2*math.pi*self.cfg.wobble_hz
                pos[0] += self.cfg.wobble_m*math.sin(omega*t)
                vel[0] += self.cfg.wobble_m*omega*math.cos(omega*t)
                acc[0] -= self.cfg.wobble_m*omega**2*math.sin(omega*t)
            self.command = pos
            self.orientation = Rotation.from_rotvec(s*turn)*rotation if turning else orientation
            self._control(pos, vel, acc, self.orientation, loaded, pressing)
            mujoco.mj_step(m, d)
            if observe and observe():
                break
        mujoco.mj_forward(m, d)

    def _carry(self):
        """Brick relative to the physical fork, plus absolute tilt from level (R12)."""
        d = self.data
        pos, q = body_pose(d, self.info, self.j)
        fork_R = Rotation.from_matrix(d.xmat[self.jaw].reshape(3, 3))
        offset = fork_R.inv().apply(pos*MM-d.xpos[self.jaw])
        relative = (fork_R.inv()*_R(q)).magnitude()
        tilt = math.acos(np.clip(_R(q).as_matrix()[2, 2], -1., 1.))
        return offset, max(relative, tilt)

    @staticmethod
    def _centred(offset, angle):
        return bool(math.hypot(offset[0], offset[2]) <= PickConfig.carry_xz_mm*MM
                    and abs(offset[1]) <= PickConfig.carry_y_mm*MM
                    and angle <= math.radians(PickConfig.carry_angle_deg))

    def _approach(self, pick, station):
        cfg, m, d = self.cfg, self.model, self.data
        rot = self.station_R
        pick_R = Rotation.from_euler("yz", [pick.dpitch_rad, pick.dyaw_rad])*rot
        approach = station + rot.apply(self.mount+self.aim) + [pick.dx_m, pick.dy_m, pick.dz_m]
        back = self.stroke_m + PickConfig.standoff_mm*MM
        self.command = station + rot.apply(self.mount + self.aim - 2*back*self.fork_axis)
        self.orientation = rot
        d.qpos[self.fa:self.fa+3] = self.command
        d.qpos[self.fa+3:self.fa+7] = _wxyz(rot*self.drone_in_brick)
        d.qvel[self.fv:self.fv+6] = 0.
        mujoco.mj_forward(m, d)
        standoff = approach + pick_R.apply(-back*self.fork_axis)
        self._move(standoff, pick_R, cfg.transport_m_s, hold=cfg.hold_s)
        tracking = np.linalg.norm(d.xpos[self.carrier]-standoff)
        reached = tracking <= PickConfig.insert_tol_mm*MM
        return approach, dict(passed=bool(reached), outcome="at_station" if reached else "tracking_error",
                              tracking_error_m=float(tracking))

    def _target(self, sample):
        d = self.data
        seat, q = (T.expected_seat(self.family, self.p, d, self.info, self.target, self.ref)
                   if self.ref else self.family.seat_pose(self.p, self.target))
        goal_R = Rotation.from_euler("yz", [sample.dpitch_rad, sample.dyaw_rad])*_R(q)
        goal = np.asarray(seat)*MM + [sample.dx_m, sample.dy_m, sample.dz_m] + _R(q).apply(self.mount)
        lift = (self.family.release_lift_mm(self.p) + T.ErrorModel().clearance[0])*MM
        return goal, goal_R, lift

    def _grip(self, engage, station, above, lift):
        cfg, d = self.cfg, self.data
        peak_engage = sustained_engage = 0.
        window = _ForceWindow(self.model.opt.timestep)
        brick_weight = (self.payload - self.gripper_kg) * G

        def engage_contact():
            nonlocal peak_engage, sustained_engage
            loads = self._contacts()
            peak_engage = max(peak_engage, loads.fork_brick_N + loads.fork_strike_N)
            mean = window.mean(loads.fork_force_N)
            # Between fork resting on brick and brick hanging on fork, gravity
            # alone accounts for this range of vertical contact loads.
            mean[2] -= np.clip(mean[2], -brick_weight, self.gripper_kg * G)
            sustained_engage = max(sustained_engage, float(np.linalg.norm(mean)))

        self._move(engage, self.orientation, PickConfig.v_engage, observe=engage_contact, hold=cfg.hold_s)
        inserted = self._inserted()
        approach = self.command
        raised = approach.copy()
        raised[2] = max(above[2], approach[2]+lift)
        self._move(raised, self.orientation, PickConfig.v_lift, loaded=True, hold=PickConfig.hold_s)
        rise = body_pose(d, self.info, self.j)[0][2]*MM-station[2]
        offset, angle = self._carry()
        loads = self._contacts()
        after_lift = self._inserted()
        captured = (not loads.supported and all(inserted) and rise > (raised[2]-approach[2])/2
                    and loads.fork_brick_N > 0 and not self._withdrawn())
        centred = self._centred(offset, angle)
        gentle = sustained_engage <= self.force_cap
        grip = captured and all(after_lift) and centred and gentle
        outcome = "picked"
        if not captured:
            outcome = "missed"
        elif not all(after_lift):
            outcome = "loose"
        elif not centred:
            outcome = "off_centre"
        elif not grip:
            outcome = "forceful"
        return dict(passed=bool(grip), outcome=outcome, captured=bool(captured), tines_inserted=inserted,
                    tines_after_lift=after_lift, rise_m=float(rise), peak_engage_N=peak_engage,
                    sustained_engage_N=sustained_engage, within_force_limit=gentle,
                    offset_m=offset.tolist(), angle_rad=angle)

    def _transport(self, above, orientation):
        cfg = self.cfg
        secure = self._centred(*self._carry())
        travel = float(np.linalg.norm(above-self.command))
        peak_offset, peak_angle = 0., 0.

        def watch_carry():
            nonlocal secure, peak_offset, peak_angle
            offset, angle = self._carry()
            peak_offset = max(peak_offset, float(np.linalg.norm(offset)))
            peak_angle = max(peak_angle, angle)
            secure &= self._centred(offset, angle)

        self._move(above, orientation, cfg.transport_m_s, loaded=True, observe=watch_carry, hold=PickConfig.hold_s)
        tw = cfg.drone.t_w(self.payload*1000)
        budget = self.payload*1000 <= min(cfg.drone.payload_max_g, CF21B.payload_max_g) and tw >= cfg.drone.tw_min
        loads = self._contacts()
        after_transport = self._inserted()
        attached = not loads.supported and loads.fork_brick_N > 0 and not self._withdrawn()
        transport = attached and all(after_transport) and secure and budget
        outcome = "carried" if transport else "loose" if budget else "over_budget"
        return dict(passed=bool(transport), outcome=outcome, attached=bool(attached), carry_force_N=loads.fork_brick_N,
                    tw=tw, tines_after_transport=after_transport, peak_offset_m=peak_offset,
                    peak_angle_rad=peak_angle, travel_m=travel)

    def brick_states(self):
        """Free-body states for the next mission in a build (mm, wxyz, SI velocities)."""
        states = []
        for i, v in enumerate(self.info["qvel"]):
            pos, quat = body_pose(self.data, self.info, i)
            states.append((pos, quat, self.data.qvel[v:v+3].copy(), self.data.qvel[v+3:v+6].copy()))
        return states

    def off_seat(self):
        """Previously placed bricks that left their supporter or touched the floor at rest."""
        if self.refs is None:
            raise ValueError("off_seat requires a structure build")
        off = []
        pairs = self.data.contact.geom[self.data.contact.efc_address >= 0]
        floor_bodies = {self.owner.get(int(g)) for pair in pairs if self.info["floor"] in pair for g in pair}
        for i, b in enumerate(self.placed):
            err, angle = T.pose_error(self.family, self.p, self.data, self.info, i, b, self.refs[i])
            if T.classify(self.family.lattice(self.p), err, angle, self.tol, i in floor_bodies) != "seated":
                off.append(brick_name(b))
        return off

    def trial(self, sample=None, *, pickup_sample=None, seed=0, support_states=None):
        """Fly one mission. With explicit target error the default pickup is aligned."""
        trial_start = time.perf_counter()
        measured = sample is None
        if measured:
            pick, sample = mission_samples(seed)
        else:
            pick = HeldSample()
        pick = pickup_sample or pick
        sources = dict(pickup="explicit" if pickup_sample is not None else "exp3" if measured else "aligned",
                       placement="exp3" if measured else "explicit", seed=int(seed) if measured else None)
        mujoco.mj_copyData(self.data, self.model, self.start)
        if support_states is not None:
            if len(support_states) != self.j:
                raise ValueError("one state is required per previous brick")
            for i, state in enumerate(support_states):
                activate(self.model, self.data, self.info, i, *state)
        self.peak_thrust = 0.
        start = time.perf_counter()
        empty_tw = self.cfg.drone.t_w(self.gripper_kg*1000)
        takeoff = empty_tw >= self.cfg.drone.tw_min
        phases = {"takeoff": dict(passed=takeoff, outcome="available" if takeoff else "insufficient_thrust",
                                  tw=empty_tw, wall_s=time.perf_counter()-start)}
        for name in ("approach", "grip", "transport", "release"):
            phases[name] = dict(passed=False, outcome="skipped", wall_s=0.)
        result = dict(outcome=None, post_departure_outcome=None, err_mm=None, angle_deg=None, settled=False,
                      phases=phases, label="simulated, uncalibrated contact",
                      wrist_defaults="guess", gripper_mass_kg=self.gripper_kg, payload_kg=self.payload,
                      force_cap_N=self.force_cap, fork_layout="oblique" if self.fork["tine_dx_dy"] else "through",
                      fork_yaw_rad=-math.atan(self.fork["tine_dx_dy"]), sample_source=sources,
                      pickup_sample=asdict(pick), placement_sample=asdict(sample))

        def record(name, metrics):
            nonlocal start
            now = time.perf_counter()
            phases[name] = dict(metrics, wall_s=now-start)
            start = now

        def finish():
            return dict(result, passed=all(p["passed"] for p in phases.values()), peak_thrust_N=self.peak_thrust,
                        trial_wall_s=time.perf_counter()-trial_start)

        if not takeoff:
            return finish()
        start = time.perf_counter()
        station = body_pose(self.data, self.info, self.j)[0]*MM
        engage, approach = self._approach(pick, station)
        record("approach", approach)
        goal, orientation, lift = self._target(sample)
        above = goal + [0., 0., lift]
        grip = self._grip(engage, station, above, lift)
        record("grip", grip)
        if not grip["captured"]:
            return finish()
        transport = self._transport(above, orientation)
        record("transport", transport)
        if not transport["attached"]:
            return finish()
        release = self._release(goal, orientation, lift)
        result.update(release, carry_force_N=transport["carry_force_N"])
        clean = (release["cleared_along_y"] and release["stayed_seated"]
                 and release["release_drift_m"] <= PickConfig.drift_mm*MM)
        gentle = release["sustained_push_N"] <= self.force_cap
        success = clean and gentle and release["climbed"] and not self.cfg.release_at_contact
        failure = "fork_retained" if not release["withdrawn"] else "failed_climb"
        record("release", dict(passed=success, outcome=release["post_departure_outcome"] or failure,
                               climbed=release["climbed"], within_force_limit=gentle, clean_withdrawal=clean))
        return finish()

    def _release(self, goal, orientation, lift):
        cfg, m, d = self.cfg, self.model, self.data
        t0 = d.time
        first = None
        floor = False
        peak = peak_normals = sustained_push = 0.
        window = _ForceWindow(m.opt.timestep)
        stop = "depth"

        def sense():
            nonlocal first, peak, peak_normals, sustained_push, floor
            loads = self._contacts()
            floor |= loads.on_floor
            peak = max(peak, loads.reaction_N)
            peak_normals = max(peak_normals, loads.normals_N)
            # On the fork side the brick's own weight never enters as an
            # upward load. Discount the weight of the gripper resting on it.
            push = max(0., float(window.mean(loads.fork_force_N)[2]) - self.gripper_kg * G)
            sustained_push = max(sustained_push, push)
            if loads.supported and first is None:
                first = d.time-t0
            return loads.reaction_N

        def descent_stop():
            nonlocal stop
            reaction = sense()
            if cfg.release_at_contact and first is not None:
                stop = "first_contact"
                return True
            # Arrest the descent promptly on contact. This controller trigger
            # does not determine whether the sustained push passes R12.
            if reaction >= self.force_cap:
                stop = "force"
                return True
            return False

        self._move(goal-[0., 0., cfg.press_depth_m], orientation, cfg.descent_m_s, loaded=True,
                   pressing=True, observe=descent_stop, wobble=True)

        def hold_contact():
            nonlocal stop
            sense()
            if cfg.release_at_contact and first is not None:
                stop = "first_contact"
                return True

        if stop != "first_contact":
            self._move(self.command, orientation, cfg.descent_m_s, pressing=True, hold=cfg.hold_s,
                       observe=hold_contact)
        release_time = d.time-t0
        before = self._outcome(floor)[0]
        stayed = before == "seated"
        start = body_pose(d, self.info, self.j)[0]*MM
        drift = 0.

        def watch_release():
            nonlocal drift, stayed
            sense()
            drift = max(drift, float(np.linalg.norm(body_pose(d, self.info, self.j)[0]*MM-start)))
            if stayed:
                stayed = self._outcome(floor)[0] == "seated"

        if cfg.release_at_contact:
            # Diagnostic only: preserve the brick velocity and remove the carrier at first contact.
            self.command = self.command + [0., 0., 1.]
            d.qpos[self.fa:self.fa+3] = self.command
            d.qvel[self.fv:self.fv+6] = 0.
            mujoco.mj_forward(m, d)
        else:
            self._move(self.command+orientation.apply(-self.stroke_m*self.fork_axis), orientation,
                       cfg.release_m_s, observe=watch_release, hold=cfg.hold_s)
        cleared_along_y = self._withdrawn()
        z_before = float(d.xpos[self.carrier, 2])
        self._move(self.command+[0., 0., lift], orientation, cfg.release_m_s, observe=watch_release,
                   hold=cfg.hold_s)
        climbed = self._withdrawn() and d.xpos[self.carrier, 2]-z_before >= lift/2
        settled = False
        outcome = None
        if climbed:
            zero = np.zeros(3)

            def hover():
                self._control(self.command, zero, zero, orientation, False, False)

            metrics = T.run_until_settled(m, d, self.info, self.j, self.tol, self.owner,
                                          watch=range(self.j), before_step=hover)
            mujoco.mj_forward(m, d)
            if self._withdrawn():
                settled = metrics["settled"]
                outcome = self._outcome(floor or metrics["floor"])[0]
                if outcome == "seated" and not settled:
                    outcome = "jammed"
        err, angle = T.pose_error(self.family, self.p, d, self.info, self.j, self.target, self.ref)
        return dict(outcome=outcome or "jammed", post_departure_outcome=outcome,
                    err_mm=err.tolist(), angle_deg=angle, settled=settled,
                    before_release=before, stayed_seated=stayed and outcome == "seated", release_drift_m=drift,
                    first_contact_s=first, release_s=release_time, descent_stop=stop, peak_support_N=peak,
                    peak_contact_normals_N=peak_normals, sustained_push_N=sustained_push,
                    withdrawn=self._withdrawn(), cleared_along_y=cleared_along_y, climbed=bool(climbed))


def held_trial(family, p, placement, sample=None, gripper_cfg=HeldConfig(), *, pickup_sample=None, seed=0):
    """One mission including scene construction. Reuse HeldRig to amortise CAD and compilation."""
    start = time.perf_counter()
    row = HeldRig(family, p, placement, gripper_cfg).trial(sample, pickup_sample=pickup_sample, seed=seed)
    return {**row, "wall_s": time.perf_counter()-start}
