"""Pickup trial (R12): the fixed fork picks a brick out of the pick fixture, then a release check.

Scene: one brick waiting in the pick fixture, and the fork as one rigid body with the drone's
mass (fork.py). The drone holds its setpoint with the Mellinger position PD and a stiff attitude
spring (drone.HoverGains); its weight, and the brick's once the payload is switched on, is
compensated as the firmware does. The setpoint carries the approach error: the drone believes it
is on the bore axis but is off by (dx, dz, psi), and contact with the brick is all that can
correct it. The motion follows aerial-assembly planner.fork_pick (ALIGN, ENGAGE along +y, payload
on, LIFT), with a shorter lift: just clear of the fixture.

The fixture is the family's course-0 anchored base with the brick's vertical sweep cut out (as
aerial-assembly fixture.slot_outer with hooked=False), so the brick is located but lifts straight
out. It has no tine funnel: R12 asks the brick's own features to do the guiding. Neighbouring
slots are not modelled (the fork model has no prop guards, so nothing can reach them).

The four R12 checks, per pick:
1. capture: after ENGAGE every tine is in its bore ('jammed' if blocked short of it, 'missed' if
   it went past the bore, or the brick did not come up with the fork);
2. self-centring: after LIFT the brick hangs within carry_xz_mm / carry_angle_deg of the nominal
   carry pose ('off_centre');
3. gentle contact: the peak fork contact force while engaging stays within force_share of the
   thrust margin ('forceful');
4. clean release: ``release_check``, once per design: from the nominal carry pose on the course-0
   base, the fork lowers by the bore clearance and withdraws along -y; the brick may move at most
   drift_mm.

Units: mm and degrees at the interface, SI inside MuJoCo. Results are simulated, uncalibrated contact.
"""

from dataclasses import asdict, dataclass, field, replace
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import split, unary_union

from brixzle.engine.sim.config import Physics
from brixzle.engine.sim.drone import G, DroneConfig, HoverGains
from brixzle.engine.sim.fork import ForkParams, build_fork
from brixzle.engine.sim.scene import MM, _numbers, activate, body_pose, build_scene
from brixzle.engine.sim.trials import TrialConfig, _brick_geoms, _contact_name, _trace_rows
from brixzle.families import builders as B
from brixzle.families.convex import convex_parts
from brixzle.families.interface import Placement, Prismatic, grip_bores, grip_openings

SEAT = Placement(0, 1)   # the waiting brick: course 0 at voxel 1, as in the drop trial
BASE_VOXELS = 4
PARK = np.array([0.0, -0.5, 0.5])  # m, where the fork waits while the brick settles


@dataclass(frozen=True)
class PickConfig:
    physics: Physics = field(default_factory=lambda: TrialConfig().physics)
    standoff_mm: float = 10.0        # planner.PlannerConfig: tine tips this far short of the near face
    v_engage: float = 0.05           # m/s, planner
    v_lift: float = 0.03             # m/s, planner
    v_release: float = 0.04          # m/s, planner
    settle_s: float = 0.2            # brick settles in the fixture before the approach
    hold_s: float = 0.3              # after ENGAGE and after LIFT (planner holds 0.5 s)
    lift_clear_mm: float = 2.0       # LIFT ends this far above the fixture top
    insert_tol_mm: float = 2.0       # a tine counts as engaged within this of full depth
    carry_xz_mm: float = 2.0         # R12 self-centring (starting value)
    carry_y_mm: float = 2.5          # along the tines (Tolerance.seat_y)
    carry_angle_deg: float = 3.0     # R12 self-centring
    force_share: float = 0.5         # R12 gentle contact: share of the thrust margin (issue #9)
    drift_mm: float = 2.0            # R12 clean release
    pocket_clearance_mm: float = 0.4  # fixture.FixtureParams.pocket_clearance
    sub_bands: int = 3               # bored collision: slices per sheared extrusion band
    fork: ForkParams = field(default_factory=ForkParams)
    drone: DroneConfig = field(default_factory=DroneConfig)
    gains: HoverGains = field(default_factory=HoverGains)

    def to_dict(self):
        return asdict(self)


# --- geometry: fixture and a bored brick ---


def _require_prismatic(family):
    if not isinstance(family.geometry, Prismatic):
        raise NotImplementedError("the pickup fixture and bored collision need a Prismatic family for now")


def seated_outline(family, p, part):
    """The waiting brick's section outline in the base frame (x = 0 at voxel 0), mm."""
    _require_prismatic(family)
    pos, quat = family.seat_pose(p, SEAT)
    if not np.allclose(quat, [1, 0, 0, 0]):
        raise NotImplementedError("pickup fixture assumes an unrotated course-0 seat")
    return affinity.translate(family.geometry.section(p, part).outer, pos[0], pos[2])


def vertical_sweep(poly, height):
    """Everything poly passes through moving straight up by height (Minkowski sum with a segment)."""
    c = list(poly.exterior.coords)
    quads = [Polygon([a, b, (b[0], b[1] + height), (a[0], a[1] + height)]) for a, b in zip(c[:-1], c[1:])]
    quads = [q.buffer(0) for q in quads if q.area > 1e-12]
    return unary_union([poly, affinity.translate(poly, 0, height), *quads])


def fixture_section(family, p, part, clearance=0.4):
    """Pick-fixture profile: the course-0 base minus the brick's upward sweep (with clearance)."""
    base = family.base(p, BASE_VOXELS, 1)
    brick = seated_outline(family, p, part)
    # Buffered and raised by the clearance: sideways room, but the brick still sits at its seat.
    grown = affinity.translate(brick.buffer(clearance, join_style=2), 0, clearance)
    h = base.outer.bounds[3] - brick.bounds[1] + 10.0
    return base.outer.difference(vertical_sweep(grown, h))


def fixture_pieces(family, p, part, clearance=0.4):
    sec = fixture_section(family, p, part, clearance)
    ext = family.geometry.extrusion(p)
    polys = list(sec.geoms) if hasattr(sec, "geoms") else [sec]
    return [piece for poly in polys if poly.area > 1e-6 for piece in B._pieces(poly, ext)]


def _bore_hole(b, z_a, z_b):
    # Octagons whose inscribed circle is the bore, swept between the two section heights.
    R = b.radius_mm/math.cos(math.pi/8)
    return unary_union([Point(b.x_mm, z).buffer(R, quad_segs=2) for z in (z_a, z_b)]).convex_hull


def bored_collision(family, p, part, sub_bands=3):
    """Convex pieces (mm, brick frame) of the brick with its straight bores and its through-Y
    openings (the grip features of those kinds; openings sweep with the section) open.

    The engine's usual collision covers the unbored outline (builders). A sheared band moves the
    section in z along y while the bore stays straight, so each sheared band is cut into
    ``sub_bands`` slices and each slice's hole covers the bore's whole travel through it (a little
    loose in z, exact in x). Unsheared bands are exact.
    """
    _require_prismatic(family)
    outer = family.geometry.section(p, part).outer
    feats = family.pickup(p, part)
    bores, openings = grip_bores(feats), grip_openings(feats)
    # Openings are part of the section, so they shear with each band's prism: cut them as they are.
    outer = outer.difference(unary_union(openings)) if openings else outer
    x0, _, x1, _ = outer.bounds
    pieces = []
    for y0, y1, z0, z1 in family.geometry.extrusion(p).resolved_bands():
        n = 1 if abs(z1 - z0) < 1e-9 else sub_bands
        for k in range(n):
            ya, yb = y0 + (y1 - y0)*k/n, y0 + (y1 - y0)*(k + 1)/n
            za, zb = z0 + (z1 - z0)*k/n, z0 + (z1 - z0)*(k + 1)/n
            holes = unary_union([_bore_hole(b, b.z_mm - za, b.z_mm - zb) for b in bores])
            polys = [outer.difference(holes)]
            cuts = [b.z_mm - (za + zb)/2 for b in bores] + [o.centroid.y for o in openings]
            for zc in cuts:  # cut through each hole so every piece is simple
                line = LineString([(x0 - 1, zc), (x1 + 1, zc)])
                polys = [q for poly in polys for q in split(poly, line).geoms]
            for poly in polys:
                if poly.area < 1e-6:
                    continue
                if poly.interiors:
                    raise ValueError("bored section still has a hole")
                coords = np.asarray(poly.simplify(1e-4).exterior.coords)[:-1]
                for q in convex_parts(coords):
                    near = np.column_stack([q[:, 0], np.full(len(q), ya), q[:, 1] + za])
                    far = np.column_stack([q[:, 0], np.full(len(q), yb), q[:, 1] + zb])
                    pieces.append(np.vstack([near, far]))
    return pieces


# --- the rig ---


def _fork_mjcf(fork):
    def add(asset, world):
        body = ET.SubElement(world, "body", name="fork", pos=_numbers(PARK))
        ET.SubElement(body, "freejoint", name="fork_free")
        I = np.asarray(fork["inertia_g_mm2"])*1e-9
        ET.SubElement(body, "inertial", mass=str(fork["body_mass_kg"]), pos=_numbers(np.asarray(fork["com_mm"])*MM),
                      fullinertia=_numbers([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]))
        r = fork["tine_r"]
        for k, (x, z) in enumerate(fork["tines"]):
            ET.SubElement(body, "geom", name=f"fork_tine{k}", type="capsule", size=str(r*MM),
                          fromto=_numbers(np.array([x, fork["y_face"], z, x, fork["y_tip"] - r, z])*MM),
                          rgba=".8 .8 .8 1", contype="1", conaffinity="1")
        lo, hi = (np.asarray(v) for v in fork["plate"])
        ET.SubElement(body, "geom", name="fork_plate", type="box", size=_numbers((hi - lo)/2*MM),
                      pos=_numbers((hi + lo)/2*MM), rgba=".8 .8 .8 1", contype="1", conaffinity="1")
    return add


def _R(q):
    return Rotation.from_quat([q[1], q[2], q[3], q[0]])


def _wxyz(R):
    q = R.as_quat()
    return np.array([q[3], q[0], q[1], q[2]])


class PickRig:
    """One compiled scene for repeated picks of one brick (``mode='pick'``, from the fixture), or
    for the release check (``mode='release'``, on the course-0 base)."""

    def __init__(self, family, p, cfg: PickConfig = None, part=None, bodies=None, mode="pick"):
        if mode not in ("pick", "release"):
            raise ValueError("mode must be 'pick' or 'release'")
        self.family, self.p, self.cfg, self.mode = family, p, cfg or PickConfig(), mode
        cfg = self.cfg
        self.part = part = part or family.part_for(0, 1)
        plain = (bodies or {}).get(part) or B.sim_bodies(family, p, part)
        self.brick = replace(plain, collision=bored_collision(family, p, part, cfg.sub_bands))
        feats = family.pickup(p, part)
        self.openings = grip_openings(feats)
        if self.openings:  # through the openings (issue #12): the fork fitted to their seats
            from brixzle.engine.predictor import pickup as PP  # not at module level: PP imports box_search

            self.geo = PP.geometry(family, p, cfg.fork)
            self.fork = build_fork(family, p, part, cfg.fork, cfg.drone, cfg.gains,
                                   tines=[tuple(s) for s in self.geo["seats"]])
        else:
            self.geo = None
            self.fork = build_fork(family, p, part, cfg.fork, cfg.drone, cfg.gains)
        static = fixture_pieces(family, p, part, cfg.pocket_clearance_mm) if mode == "pick" else None
        bases = [] if mode == "pick" else [(0, BASE_VOXELS)]
        self.model, self.xml, self.info = build_scene(
            family, p, {part: self.brick}, bases, 1, cfg.physics, visual=False, body_parts=[part],
            static=static, extra=_fork_mjcf(self.fork))
        m = self.model
        self.data = mujoco.MjData(m)
        self.owner = _brick_geoms(m, self.info)
        self.names = [m.geom(g).name for g in range(m.ngeom)]
        self.fork_body = m.body("fork").id
        jid = m.joint("fork_free").id
        self.fa, self.fv = m.jnt_qposadr[jid], m.jnt_dofadr[jid]
        self.fork_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] == self.fork_body}
        self.seat_pos, self.seat_quat = family.seat_pose(p, SEAT)
        bottom = min(float(np.min(c[:, 2])) for c in self.brick.collision)
        top = max(float(np.max(c[:, 2])) for c in static) if static else self.seat_pos[2]
        self.lift_mm = top - (self.seat_pos[2] + bottom) + cfg.lift_clear_mm
        self.brick_kg = self.brick.mass_g*1e-3
        self.body_kg = self.fork["body_mass_kg"]
        self.force_cap = cfg.force_share*cfg.drone.thrust_margin_N(self.fork["gripper_mass_g"] + self.brick.mass_g)
        g = cfg.gains
        I = np.diag(np.asarray(self.fork["inertia_g_mm2"]))*1e-9
        w = 2*math.pi*g.att_hz
        self.k_att, self.c_att = I*w*w, 2*g.att_damping*I*w
        self.kp, self.kd = np.asarray(g.kp), np.asarray(g.kd)
        self.bores = grip_bores(feats)
        if self.openings:  # aim the fork at the centre of the region where every tine clears
            self.aim_dx, self.aim_low = float(self.geo["aim"][0]), -float(self.geo["aim"][1])
        else:
            self.aim_dx = 0.0
            self.aim_low = float(np.mean([b.radius_mm - self.fork["tine_r"] for b in self.bores]))  # tine centred

    # -- low level --

    def _set_fork(self, pos_m, R, vel_m=(0, 0, 0)):
        d, a, v = self.data, self.fa, self.fv
        d.qpos[a:a+3] = pos_m
        d.qpos[a+3:a+7] = _wxyz(R)
        d.qvel[v:v+3] = vel_m
        d.qvel[v+3:v+6] = 0.
        mujoco.mj_forward(self.model, d)

    def _fork_pose(self):
        d, a = self.data, self.fa
        return d.qpos[a:a+3].copy(), _R(d.qpos[a+3:a+7])

    def _control(self, sp_pos, sp_vel, sp_acc, sp_R, loaded):
        d, v = self.data, self.fv
        pos, R = self._fork_pose()
        # Mellinger (crazyflow state2attitude): m (a_des + g) + kp e + kd de, with m the controller's
        # mass (the payload is switched on for LIFT).
        mass = self.body_kg + (self.brick_kg if loaded else 0.)
        f = mass*(np.asarray(sp_acc) + [0., 0., G]) + self.kp*(sp_pos - pos) + self.kd*(sp_vel - d.qvel[v:v+3])
        w = R.apply(d.qvel[v+3:v+6])  # free-joint angular velocity is in the body frame
        tau = self.k_att*(sp_R*R.inv()).as_rotvec() - self.c_att*w
        d.xfrc_applied[self.fork_body, :3] = f
        d.xfrc_applied[self.fork_body, 3:] = tau

    def _run(self, duration, setpoint, loaded, contacts, t0):
        """Step for ``duration`` s tracking setpoint(t) -> (pos_m, vel_m, acc_m, R); returns the peak fork force (N)."""
        m, d = self.model, self.data
        dt = m.opt.timestep
        force = np.zeros(6)
        peak = 0.
        for i in range(max(1, round(duration/dt))):
            self._control(*setpoint(i*dt), loaded)
            mujoco.mj_step(m, d)
            total, active = 0., set()
            for k in range(d.ncon):
                c = d.contact[k]
                if c.geom1 in self.fork_geoms:
                    mine, other = c.geom1, c.geom2
                elif c.geom2 in self.fork_geoms:
                    mine, other = c.geom2, c.geom1
                else:
                    continue
                mujoco.mj_contactForce(m, d, k, force)
                fn = abs(float(force[0]))
                total += fn
                if contacts is None:
                    continue
                key, now = (mine, other), d.time - t0
                e = contacts.get(key)
                if e is None:
                    e = contacts[key] = {
                        "piece": self.names[mine].split("_", 1)[1], "other": _contact_name(self.names, self.owner, other),
                        "other_geom": self.names[other], "start_s": now, "end_s": now, "touches": 0, "peak_N": 0.,
                        "first_pos_mm": (np.asarray(c.pos)/MM).tolist(), "active_at_end": False, "_live": False}
                if not e["_live"]:
                    e["touches"] += 1
                e["end_s"], e["peak_N"] = now, max(e["peak_N"], fn)
                active.add(key)
            if contacts is not None:
                for key, e in contacts.items():
                    e["_live"] = key in active
            peak = max(peak, total)
        return peak

    def _hold_park(self, duration):
        R0 = Rotation.identity()
        self._run(duration, lambda t: (PARK, np.zeros(3), np.zeros(3), R0), False, None, 0.)

    def _brick_pose(self):
        pos, quat = body_pose(self.data, self.info, 0)
        return pos, _R(quat)

    def _path(self, aim_pos, aim_R, err, y0, y1, z0, z1, v):
        """Setpoint along a straight fork move in the aim (brick) frame, origin offset by the error.

        A minimum-jerk profile at average speed v, with velocity and acceleration feedforward
        (the planner's 'line' segments are smooth setpoints, not a step in velocity)."""
        dx, dz, psi = err
        Rpsi = Rotation.from_euler("z", psi, degrees=True)
        dp = np.asarray(self.fork["drone_pos_mm"])
        dp[2] = 0.
        yaw_off = Rpsi.apply(-dp) + dp   # yaw about the drone's vertical axis
        R = aim_R*Rpsi
        delta = np.array([0., y1 - y0, z1 - z0])
        T = max(np.linalg.norm(delta)*MM/v, 1e-9)
        step = aim_R.apply(delta)*MM

        def sp(t):
            u = min(max(t/T, 0.), 1.)
            s = 10*u**3 - 15*u**4 + 6*u**5
            ds = (30*u**2 - 60*u**3 + 30*u**4)/T if t < T else 0.
            dds = (60*u - 180*u**2 + 120*u**3)/T**2 if t < T else 0.
            local = yaw_off + [dx, y0 + s*(y1 - y0), dz + z0 + s*(z1 - z0)]
            return (aim_pos + aim_R.apply(local))*MM, step*ds, step*dds, R
        return sp, T

    def _in_frame(self, R_ref, p_ref, pts_world_mm):
        return R_ref.inv().apply(np.asarray(pts_world_mm) - p_ref)

    def _tine_points(self, y):
        pos, R = self._fork_pose()
        return R.apply(np.array([[x, y, z] for x, z in self.fork["tines"]])*MM)/MM + pos/MM

    def _carry_error(self):
        fpos, fR = self._fork_pose()
        bpos, bR = self._brick_pose()
        err = fR.inv().apply(bpos - fpos/MM)
        return err, math.degrees((fR.inv()*bR).magnitude())

    # -- trials --

    def pick(self, sample, trace=True):
        """One pick at error sample {dx, dz (mm), psi (deg)}; returns a JSON-safe row."""
        cfg, m, d = self.cfg, self.model, self.data
        mujoco.mj_resetData(m, d)
        activate(m, d, self.info, 0, self.seat_pos, self.seat_quat)
        self._set_fork(PARK, Rotation.identity())
        self._hold_park(cfg.settle_s)
        aim_pos, aim_R = self._brick_pose()   # the camera-measured pose of the waiting brick
        start_z = aim_pos[2]
        err = (float(sample["dx"]) + self.aim_dx, float(sample["dz"]), float(sample["psi"]))
        D = self.family.lattice(self.p).D_mm
        y_back = -(D + cfg.fork.protrude + cfg.standoff_mm)
        contacts = {} if trace else None
        t0 = d.time
        engage, T = self._path(aim_pos, aim_R, err, y_back, 0., -self.aim_low, -self.aim_low, cfg.v_engage)
        p0, v0, _, R0 = engage(0.)
        self._set_fork(p0, R0, v0)
        peak_engage = self._run(T + cfg.hold_s, engage, False, contacts, t0)
        # Capture: every tine's mid point in its bore, at full depth.
        bpos, bR = self._brick_pose()
        mids = self._in_frame(bR, bpos, self._tine_points(0.))
        if self.openings:  # each tine's mid point inside its opening (centre band: no shear at y = 0)
            off = [float(o.exterior.distance(Point(q[0], q[2]))) * (1 if o.contains(Point(q[0], q[2])) else -1)
                   for q, o in zip(mids, self.openings)]
            in_bore = all(v >= 0 for v in off)
        else:
            off = [math.hypot(q[0] - b.x_mm, q[2] - b.z_mm) for q, b in zip(mids, self.bores)]
            in_bore = all(o <= b.radius_mm + 0.5 for o, b in zip(off, self.bores))
        depth_ok = all(abs(q[1]) <= cfg.insert_tol_mm for q in mids)
        blocked = any(q[1] < -cfg.insert_tol_mm for q in mids)
        lift, Tl = self._path(aim_pos, aim_R, err, 0., 0., -self.aim_low, -self.aim_low + self.lift_mm, cfg.v_lift)
        peak_lift = self._run(Tl + cfg.hold_s, lift, True, contacts, t0)
        rise = self._brick_pose()[0][2] - start_z
        carry_err, carry_angle = self._carry_error()
        if not (in_bore and depth_ok):
            outcome = "jammed" if blocked else "missed"
        elif rise < self.lift_mm/2:
            outcome = "missed"
        elif (math.hypot(carry_err[0], carry_err[2]) > cfg.carry_xz_mm or abs(carry_err[1]) > cfg.carry_y_mm
              or carry_angle > cfg.carry_angle_deg):
            outcome = "off_centre"
        elif peak_engage > self.force_cap:
            outcome = "forceful"
        else:
            outcome = "picked"
        row = {"dx": float(sample["dx"]), "dz": err[1], "psi": err[2], "outcome": outcome,
               "captured": bool(in_bore and depth_ok), "tine_offset_mm": [float(o) for o in off],
               "rise_mm": float(rise), "lift_mm": float(self.lift_mm),
               "carry_err_mm": [float(v) for v in carry_err], "carry_angle_deg": float(carry_angle),
               "angle_deg": float(carry_angle), "peak_engage_N": float(peak_engage), "peak_lift_N": float(peak_lift),
               "force_cap_N": float(self.force_cap), "time": float(d.time - t0)}
        if trace:
            for e in contacts.values():
                e["active_at_end"] = e.pop("_live")
            row["contacts"] = _trace_rows(contacts)
        return row

    def release(self, trace=True):
        """Clean release from the nominal carry pose on the course-0 base; returns a JSON-safe dict."""
        cfg, m, d = self.cfg, self.model, self.data
        if self.mode != "release":
            raise ValueError("release needs a PickRig built with mode='release'")
        mujoco.mj_resetData(m, d)
        activate(m, d, self.info, 0, self.seat_pos, self.seat_quat)
        seat_R = _R(self.seat_quat)
        contacts = {} if trace else None
        hold, _ = self._path(self.seat_pos, seat_R, (0., 0., 0.), 0., 0., 0., 0., cfg.v_release)
        p0, _, _, R0 = hold(0.)
        self._set_fork(p0, R0)
        t0 = d.time
        self._run(cfg.settle_s, hold, False, contacts, t0)
        start = self._brick_pose()[0]
        unload, Tu = self._path(self.seat_pos, seat_R, (0., 0., 0.), 0., 0., 0., -self.aim_low, cfg.v_lift)
        peak = self._run(Tu, unload, False, contacts, t0)
        D = self.family.lattice(self.p).D_mm
        back = -(D + cfg.fork.protrude + 5.0)
        out, To = self._path(self.seat_pos, seat_R, (0., 0., 0.), 0., back, -self.aim_low, -self.aim_low, cfg.v_release)
        peak = max(peak, self._run(To + cfg.hold_s, out, False, contacts, t0))
        drift = float(np.linalg.norm(self._brick_pose()[0] - start))
        tips = self._in_frame(*self._brick_pose()[::-1], self._tine_points(self.fork["y_tip"]))
        clear = bool(all(q[1] < -D/2 for q in tips))
        result = {"passed": bool(drift <= cfg.drift_mm and clear), "drift_mm": drift, "withdrawn": clear,
                  "peak_N": float(peak), "label": "simulated, uncalibrated contact"}
        if trace:
            for e in contacts.values():
                e["active_at_end"] = e.pop("_live")
            result["contacts"] = _trace_rows(contacts)
        return result


def release_check(family, p, cfg: PickConfig = None, part=None, bodies=None):
    """R12 clean release, once per design (see PickRig.release)."""
    return PickRig(family, p, cfg, part=part, bodies=bodies, mode="release").release()
