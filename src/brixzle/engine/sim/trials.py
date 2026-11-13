# From LunarSphere/aerial-assembly brixzle/src/brixzle/trials.py @ a649663
# Changed: the drop rig lays course 1 across two seated bricks (running bond) (issue #82; rig_layout); classify's wrong valley uses the voxel pitch U, not the brick length 2U; run_until_settled can record a contact trace, and the drop loop is a reusable DropRig (scene built once), which can also return the contacts at rest (rest_contacts, issue #49); trials take a family and its params instead of BrickParams and cad bundles (seat pose, insertion axis, release lift and lattice pitch come from the family; drop_trials takes a course instead of a part); sequential assembly and push tests (assemble, push_test, expected_seat) dropped, leaving the drop trial; DropRig's recording rate is settable (record_fps, issue #77).
"""MuJoCo trials: coarse-placement drops.

No drone dynamics: a brick is released as a free body at a sampled pose error
above its lattice seat. Metrics are simulated values under uncalibrated
contact parameters, not hardware success rates.
"""
from dataclasses import asdict, dataclass, field
import math

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from brixzle.engine.sim.config import Physics
from brixzle.engine.sim.scene import MM, activate, body_pose, build_scene
from brixzle.families import builders as B
from brixzle.families.interface import Family, Lattice, Placement


RADIAL_MODES = ('gauss', 'ring', 'bootstrap')


@dataclass(frozen=True)
class ErrorModel:
    """Coarse drone placement: aim at the cavity, rely on geometry."""
    sigma_xy: float = 4.0        # mm
    clip_xy: float = 12.0        # mm
    aim_bias: float = 3.0        # mm toward the side the tooth stroke comes from
    sigma_yaw: float = 5.0       # deg
    sigma_tilt: float = 3.0      # deg (roll, pitch)
    clearance: tuple = (5., 30.)  # mm above the highest-reach release height
    sigma_vel: float = 0.03      # m/s residual velocity
    ideal: bool = False
    # Aim at the seat implied by the supporter's *measured* pose (global camera)
    # instead of the ideal lattice pose; matters once a cantilever sags.
    track_supporter: bool = False
    exact_courses: int = 0       # courses below this are placed without error (anchored start)
    # X-Y error shape: 'gauss' (sigma_xy, clip_xy; the hand-set default), or measured Exp 3
    # magnitudes in a uniformly random direction: 'ring' (fixed radius_mm) or 'bootstrap'
    # (resampled radii_mm). See approach_error.py.
    radial_mode: str = 'gauss'
    radius_mm: float = 0.
    radii_mm: tuple = ()
    source: dict = field(default_factory=dict)  # provenance, written with every result

    def __post_init__(self):
        if self.radial_mode not in RADIAL_MODES:
            raise ValueError(f'radial_mode must be one of {RADIAL_MODES}')
        if self.radial_mode == 'ring' and not (math.isfinite(self.radius_mm) and self.radius_mm >= 0):
            raise ValueError('ring mode needs a finite radius_mm >= 0')
        if self.radial_mode == 'bootstrap' and (not self.radii_mm or
                                                not all(math.isfinite(r) and r >= 0 for r in self.radii_mm)):
            raise ValueError('bootstrap mode needs finite radii_mm >= 0')

    def _xy(self, rng):
        if self.radial_mode == 'gauss':
            return np.clip(rng.normal(0, self.sigma_xy, 2), -self.clip_xy, self.clip_xy)
        r = self.radius_mm if self.radial_mode == 'ring' else float(rng.choice(self.radii_mm))
        a = rng.uniform(0, 2*math.pi)
        return r*math.cos(a), r*math.sin(a)

    def sample(self, rng):
        if self.ideal:
            return dict(dx=self.aim_bias, dy=0., yaw=0., roll=0., pitch=0., clear=self.clearance[0],
                        vel=[0., 0., 0.], aim=self.aim_bias)
        dx, dy = self._xy(rng)
        return dict(dx=float(dx + self.aim_bias), dy=float(dy), aim=self.aim_bias,
                    yaw=float(rng.normal(0, self.sigma_yaw)),
                    roll=float(rng.normal(0, self.sigma_tilt)), pitch=float(rng.normal(0, self.sigma_tilt)),
                    clear=float(rng.uniform(*self.clearance)),
                    vel=rng.normal(0, self.sigma_vel, 3).tolist())


@dataclass(frozen=True)
class Tolerance:
    seat_xz: float = 1.0       # mm
    seat_y: float = 2.5        # mm
    seat_angle: float = 3.0    # deg
    drift: float = 2.0         # mm moved by a placed brick during one stage => collapse
    lin_speed: float = 0.002   # m/s settle threshold
    ang_speed: float = 0.05    # rad/s
    settle_hold: float = 0.08  # s
    max_time: float = 1.5      # s per brick


@dataclass
class TrialConfig:
    physics: Physics = field(default_factory=lambda: Physics(timestep=0.001, friction=0.35,
                                                             contact_timeconst=0.004, iterations=50))
    error: ErrorModel = field(default_factory=ErrorModel)
    tolerance: Tolerance = field(default_factory=Tolerance)


def release_pose(family: Family, p, placement, sample, seat_pose=None):
    """Seat pose displaced by the sampled error, lifted so nothing overlaps at release.

    ``seat_pose`` (pos, quat) overrides the ideal lattice seat, e.g. the seat
    implied by a sagging supporter's measured pose.
    """
    pos, quat = seat_pose if seat_pose is not None else family.seat_pose(p, placement)
    d = family.insertion_axis(p, placement)
    side = -np.sign(d[0]) or 1.0  # the stroke arrives from this x side (+x for a vertical insertion)
    lift = family.release_lift_mm(p) + sample['clear']
    rel = pos + np.array([side*sample['dx'], sample['dy'], lift])
    seat = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])
    err = Rotation.from_euler('xyz', [sample['roll'], sample['pitch'], sample['yaw']], degrees=True)
    q = (err*seat).as_quat()
    return rel, np.array([q[3], q[0], q[1], q[2]])


def _rot(q):
    return Rotation.from_quat([q[1], q[2], q[3], q[0]])


def pose_error(family: Family, p, data, info, j, placement, ref=None):
    """Position (mm) and angle (deg) error of brick j against its lattice seat.

    With ``ref = (k, ref_placement)`` the seat is taken relative to brick k's
    actual pose, so a brick that seats correctly on a sagging supporter counts
    as seated; sag is reported separately.
    """
    pos, quat = body_pose(data, info, j)
    tpos, tquat = family.seat_pose(p, placement)
    target_r = _rot(tquat)
    if ref is not None:
        k, ref_placement = ref
        rpos, rquat = body_pose(data, info, k)
        ipos, iquat = family.seat_pose(p, ref_placement)
        delta = _rot(rquat)*_rot(iquat).inv()
        tpos = rpos + delta.apply(tpos - ipos)
        target_r = delta*target_r
    r = _rot(quat)
    angle = math.degrees((target_r.inv()*r).magnitude())
    return pos - tpos, angle


def classify(lat: Lattice, err, angle, tol: Tolerance, on_floor):
    """'fell', 'seated', 'wrong_valley' (seated cleanly a whole number of voxels off in X) or 'jammed'.

    Bricks sit on the voxel lattice (R2), so a clean seat off by a multiple of the voxel pitch U is
    a wrong valley. (aerial-assembly used a whole brick length, 2U, and so called one-voxel-off
    seats jammed.)
    """
    U = pitch = lat.U_mm
    dx, dy, dz = err
    if on_floor:
        return 'fell'
    if abs(dx) <= tol.seat_xz and abs(dz) <= tol.seat_xz and abs(dy) <= tol.seat_y and angle <= tol.seat_angle:
        return 'seated'
    if abs(abs(dx) - pitch*round(abs(dx)/pitch)) <= 2*tol.seat_xz and abs(dx) > U/2 and angle <= tol.seat_angle:
        return 'wrong_valley'
    return 'jammed'


def _brick_geoms(model, info):
    owner = {}
    for j, b in enumerate(info['bodies']):
        for g in range(model.ngeom):
            if model.geom_bodyid[g] == b:
                owner[g] = j
    return owner


def _contact_name(names, owner, g):
    """Body-level name of the thing brick geoms touch: 'brick<k>', 'base', 'floor' or 'static'."""
    if g in owner:
        return f'brick{owner[g]}'
    name = names[g]
    return 'base' if name.startswith('base') else name.split('_')[0]


def _trace_rows(contacts):
    rows = sorted(contacts.values(), key=lambda c: (c['start_s'], c['piece'], c['other_geom']))
    return [{**c, 'first_pos_mm': [float(v) for v in c['first_pos_mm']]} for c in rows]


def run_until_settled(model, data, info, j, tol: Tolerance, owner, watch=(), frames=None, fps=60,
                      contacts=None, names=None):
    """Step until brick j (and watched bricks) are still; return impact metrics.

    ``contacts`` (a dict, with ``names`` = geom names) collects brick j's contact sequence: one
    entry per (own collision piece, touched geom) with first and last contact time, the number
    of separate touches, peak normal force and the first contact point. It only reads contact
    data, so the simulation is unchanged.
    """
    force = np.zeros(6)
    t0 = data.time
    still_since = None
    f_max, onsets, in_contact, floor_hit, pen_max = 0., 0, False, False, 0.
    moving = [j, *watch]
    every = max(1, round(1/(fps*model.opt.timestep)))
    step = 0
    while data.time - t0 < tol.max_time:
        mujoco.mj_step(model, data)
        if frames is not None and step % every == 0:
            frames.append(data.qpos.copy())
        step += 1
        total, touching = 0., False
        active = set()
        for i in range(data.ncon):
            c = data.contact[i]
            if owner.get(c.geom1) != j and owner.get(c.geom2) != j:
                continue
            touching = True
            mujoco.mj_contactForce(model, data, i, force)
            total += abs(force[0])
            pen_max = max(pen_max, -c.dist)
            if info['floor'] in (c.geom1, c.geom2):
                floor_hit = True
            if contacts is not None:
                mine, other = (c.geom1, c.geom2) if owner.get(c.geom1) == j else (c.geom2, c.geom1)
                key = (mine, other)
                now = data.time - t0
                entry = contacts.get(key)
                if entry is None:
                    entry = contacts[key] = {
                        'piece': int(names[mine].rsplit('_c', 1)[1]), 'other': _contact_name(names, owner, other),
                        'other_geom': names[other], 'start_s': now, 'end_s': now, 'touches': 0, 'peak_N': 0.,
                        'first_pos_mm': (np.asarray(c.pos)/MM).tolist(), 'active_at_end': False, '_live': False}
                if not entry['_live']:
                    entry['touches'] += 1
                entry['end_s'] = now
                entry['peak_N'] = max(entry['peak_N'], float(abs(force[0])))
                active.add(key)
        if contacts is not None:
            for key, entry in contacts.items():
                entry['_live'] = key in active
        f_max = max(f_max, total)
        if touching and not in_contact:
            onsets += 1
        in_contact = touching
        speeds = [np.linalg.norm(data.qvel[info['qvel'][m]:info['qvel'][m]+3]) for m in moving]
        spins = [np.linalg.norm(data.qvel[info['qvel'][m]+3:info['qvel'][m]+6]) for m in moving]
        if max(speeds) < tol.lin_speed and max(spins) < tol.ang_speed and in_contact:
            still_since = still_since if still_since is not None else data.time
            if data.time - still_since >= tol.settle_hold:
                break
        else:
            still_since = None
    settled = still_since is not None and data.time - still_since >= tol.settle_hold
    if contacts is not None:
        for entry in contacts.values():
            entry['active_at_end'] = entry.pop('_live')
    return {'f_max': f_max, 'onsets': onsets, 'floor': floor_hit, 'penetration_mm': pen_max/MM,
            'time': (still_since if settled else data.time) - t0, 'settled': settled}


def rest_contacts(model, data, info, owner, names, j):
    """Brick j's contacts in the current (settled) state, against the bricks and base it rests on
    (never the floor): one row per MuJoCo contact point, in the world frame.

    ``normal`` points into brick j and ``force_N`` is the contact force on brick j (normal plus
    friction); ``normal_N`` is its normal part. MuJoCo's split of the load between redundant
    contacts is one of many valid ones (issue #49), so treat the forces as indicative."""
    mujoco.mj_forward(model, data)
    force = np.zeros(6)
    rows = []
    for i in range(data.ncon):
        c = data.contact[i]
        mine2 = owner.get(c.geom2) == j
        if not mine2 and owner.get(c.geom1) != j:
            continue
        mine, other = (c.geom2, c.geom1) if mine2 else (c.geom1, c.geom2)
        if other == info['floor']:
            continue
        mujoco.mj_contactForce(model, data, i, force)
        frame = np.asarray(c.frame).reshape(3, 3)  # rows: the normal (geom1 -> geom2) and two tangents
        sign = 1. if mine2 else -1.
        rows.append({'pos_mm': (np.asarray(c.pos)/MM).tolist(), 'normal': (sign*frame[0]).tolist(),
                     'force_N': (sign*(frame.T @ force[:3])).tolist(), 'normal_N': float(force[0]),
                     'dist_mm': float(c.dist/MM), 'piece': int(names[mine].rsplit('_c', 1)[1]),
                     'other': _contact_name(names, owner, other), 'other_geom': names[other]})
    return rows


COURSE0_BASE = (0, 4)  # the course-0 drop's base: voxels 0 to 3, as before #82


def rig_layout(lat: Lattice, course, neighbour=False):
    """(placed, target, base) of the drop rig (issue #82), shared with the family contract.

    - Course 0: the target is Placement(0, 1) on the base, alone, as before #82. ``neighbour`` adds
      a seated brick at Placement(0, 1 + span) (the issue's proposal; not used by the loop, see
      docs/family-interface.md).
    - Course 1: two seated course-0 bricks, Placement(0, 1) and Placement(0, 1 + span), and the
      target Placement(1, 1 + span // 2) across their joint (running bond), on a base covering both
      plus one voxel each side. A one-voxel brick keeps the stack drop: one supporter,
      Placement(0, 1), and the target Placement(1, 1).

    The seat of a course-1 brick is judged against the first placed brick."""
    span = lat.span_voxels
    if course not in (0, 1):
        raise ValueError('course must be 0 or 1')
    if course == 0:
        placed = [Placement(0, 1 + span)] if neighbour else []
        return placed, Placement(0, 1), (0, max(COURSE0_BASE[1], 2 + 2*span) if neighbour else COURSE0_BASE[1])
    if span == 1:
        return [Placement(0, 1)], Placement(1, 1), COURSE0_BASE
    return [Placement(0, 1), Placement(0, 1 + span)], Placement(1, 1 + span//2), (0, 2 + 2*span)


class DropRig:
    """One compiled scene for repeated drops of one brick (course 0 onto the base, or course 1 across
    two ideally seated course-0 bricks; ``rig_layout``), so many poses don't rebuild the model."""

    def __init__(self, family: Family, p, cfg: TrialConfig, course=0, record=False, bodies=None, record_fps=60):
        """``bodies`` ({part: SimBodies}) skips rebuilding the bricks, e.g. in a worker process.
        ``record_fps``: frames per second of sim time kept when recording."""
        self.record_fps = record_fps
        if course not in (0, 1):
            raise ValueError('course must be 0 or 1')
        self.family, self.p, self.cfg, self.course, self.record = family, p, cfg, course, record
        bodies = bodies or {k: B.sim_bodies(family, p, k) for k in family.parts}
        self.placed, self.target, self.base = rig_layout(family.lattice(p), course)
        # The seat of a course-1 brick is judged against the first brick below it.
        self.support = self.placed[0] if course == 1 else None
        body_parts = [family.part_for(b.course, b.direction) for b in (*self.placed, self.target)]
        self.j = len(self.placed)
        self.model, self.xml, self.info = build_scene(family, p, bodies, [self.base], len(body_parts), cfg.physics,
                                                      visual=record, body_parts=body_parts)
        self.data = mujoco.MjData(self.model)
        self._settled = None  # the state after the bricks below have settled (first drop)
        self.owner = _brick_geoms(self.model, self.info)
        self.names = [self.model.geom(g).name for g in range(self.model.ngeom)]

    def drop(self, sample, trace=True, rest=False):
        """Drop at a sample pose (ErrorModel.sample keys) and judge the seat; returns one row.

        ``rest`` adds the settled state under 'rest': the brick's contacts (rest_contacts), its
        pose and weight, and the pose of the brick it landed on (None on the base)."""
        model, data, info, family, p = self.model, self.data, self.info, self.family, self.p
        if self._settled is None:
            mujoco.mj_resetData(model, data)
            for k, b in enumerate(self.placed):
                spos, squat = family.seat_pose(p, b)
                activate(model, data, info, k, spos, squat)
            if self.placed:
                ideal = Tolerance(max_time=0.3)
                run_until_settled(model, data, info, 0, ideal, self.owner, watch=range(1, self.j))
            # Settling the bricks below is deterministic, so it is done once and its whole state
            # kept; each drop starts from an exact copy of it (issue #82: two bricks below).
            self._settled = mujoco.MjData(model)
            mujoco.mj_copyData(self._settled, model, data)
        else:
            mujoco.mj_copyData(data, model, self._settled)
        pos, quat = release_pose(family, p, self.target, sample)
        activate(model, data, info, self.j, pos, quat, linvel=sample['vel'])
        frames = [] if self.record else None
        contacts = {} if trace else None
        m = run_until_settled(model, data, info, self.j, self.cfg.tolerance, self.owner, frames=frames, fps=self.record_fps,
                              watch=range(self.j), contacts=contacts, names=self.names)
        ref = (0, self.support) if self.support is not None else None
        err, angle = pose_error(family, p, data, info, self.j, self.target, ref)
        outcome = classify(family.lattice(p), err, angle, self.cfg.tolerance, m['floor'])
        row = {**sample, **m, 'outcome': outcome, 'err_mm': err.tolist(), 'angle_deg': angle}
        if trace:
            row['contacts'] = _trace_rows(contacts)
        if rest:
            pos, quat = body_pose(data, info, self.j)
            support = None
            if self.support is not None:
                spos, squat = body_pose(data, info, 0)
                support = {'pos_mm': spos.tolist(), 'quat': squat.tolist()}
            placed = []
            for k, b in enumerate(self.placed):
                bpos, bquat = body_pose(data, info, k)
                placed.append({'brick': f'brick{k}', 'part': family.part_for(b.course, b.direction),
                               'pos_mm': bpos.tolist(), 'quat': bquat.tolist()})
            row['rest'] = {'contacts': rest_contacts(model, data, info, self.owner, self.names, self.j),
                           'pos_mm': pos.tolist(), 'quat': quat.tolist(), 'support': support, 'placed': placed,
                           'weight_N': float(model.body_mass[info['bodies'][self.j]]*9.81)}
        if self.record:
            row['frames'] = frames
        return row


def drop_trials(family: Family, p, cfg: TrialConfig, samples=32, seed=0, offsets=None, record=False,
                course=0, trace=False):
    """Drop one brick and judge its seat; Monte Carlo over the error model.

    course 0: a brick onto an anchored base at voxel 1. course 1: a brick
    across two ideally seated course-0 bricks (running bond; for A/B, a B onto two As), or onto one
    for a one-voxel brick (``rig_layout``, issue #82).
    ``offsets`` instead runs a deterministic capture sweep over X offsets (mm).
    ``trace`` adds each drop's contact sequence (``DropRig.drop``).
    """
    rig = DropRig(family, p, cfg, course, record)
    rng = np.random.default_rng(seed)
    rows = []
    runs = offsets if offsets is not None else range(samples)
    for item in runs:
        sample = cfg.error.sample(rng)
        if offsets is not None:
            sample = dict(sample, dx=float(item), dy=0., yaw=0., roll=0., pitch=0., vel=[0, 0, 0],
                          clear=cfg.error.clearance[0])
        rows.append(rig.drop(sample, trace=trace))
    return rows, rig.xml


def summarize_drops(rows):
    n = len(rows)
    seated = [r for r in rows if r['outcome'] == 'seated']
    outcomes = {}
    for r in rows:
        outcomes[r['outcome']] = outcomes.get(r['outcome'], 0) + 1
    return {
        'n': n, 'p_success': len(seated)/n if n else 0.,
        'outcomes': outcomes,
        'f_max_N': float(np.percentile([r['f_max'] for r in rows], 95)) if rows else 0.,
        't_settle_s': float(np.mean([r['time'] for r in rows])) if rows else 0.,
        'collisions': float(np.mean([max(0, r['onsets'] - 1) + (r['penetration_mm'] > 1.0) for r in rows])),
        'max_penetration_mm': float(max(r['penetration_mm'] for r in rows)) if rows else 0.,
    }


def capture_radius(rows, level=0.9, bins=None):
    """Largest |offset from aim| band whose seated fraction stays >= level."""
    offs = np.array([abs(r['dx'] - r.get('aim', 0)) for r in rows])
    ok = np.array([r['outcome'] == 'seated' for r in rows])
    bins = bins if bins is not None else np.arange(0, offs.max() + 1.0, 1.0)
    radius = 0.
    for hi in bins[1:]:
        sel = offs <= hi
        if sel.sum() and ok[sel].mean() >= level:
            radius = float(hi)
        else:
            break
    return radius


def capture_margin(family: Family, p, tolerance, level=0.9, step=1.0, course=0, cfg=None):
    """X capture radius at ``level`` minus the required tolerance (mm); >= 0 satisfied.

    A deterministic sweep of offsets up to ``tolerance + step`` on both sides of the aim
    (ideal pose otherwise), so a design must seat across the measured approach error.
    """
    cfg = cfg or TrialConfig(error=ErrorModel(ideal=True))
    d = np.arange(0., tolerance + step + 1e-9, step)
    offsets = cfg.error.aim_bias + np.concatenate([-d[:0:-1], d])
    rows, _ = drop_trials(family, p, cfg, offsets=offsets.tolist(), course=course)
    return capture_radius(rows, level) - tolerance


def as_dict(cfg: TrialConfig):
    return {'physics': asdict(cfg.physics), 'error': asdict(cfg.error), 'tolerance': asdict(cfg.tolerance)}
