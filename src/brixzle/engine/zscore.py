"""Objective Z (issue #61): (seated - jammed) / drops inside each target box.

For a target box (the drone's mean, p95 or worst error; engine/boxes.py), take every drop of the
full sim map (engine/sim_map.py) whose pose lies inside that box, across all slices and both
courses:

    S = seated, J = jammed, N = every drop in the box;   Z = (S - J) / N

A seated drop counts +1, a jammed one -1, and a fall, a wrong valley or any other outcome 0, so
Z runs from -1 to 1. Z = 1 means every drop in the box seated: that is the pass mark.

Z is counted over the target box, not over the green zone. The box is fixed by the drone's
measured error, so a design cannot raise Z by shrinking its green zone. Zoom drops (an override
at a finer step) are left out, so a caller cannot weight Z by zooming.

A drop depends only on its pose, so the map's Z can also be had by dropping just the map's
cells inside the boxes (``box_cells``, ``ZScore``): the same poses, about a third of the drops,
and no grid fitting. The end tune (#14) and the loop's acceptance rule (#21) use that. The
held-out check shifts each of those poses by a seeded amount, keeps it inside the box, and drops
it at another friction coefficient (``held_out_poses``, ``HeldOutZ``).

**The X-Y disc (issue #86).** A drop is inside a placement box when x^2 + y^2 <= r^2 (r the box's
X half-width) and its release height and pitch are inside the box. Only the ``xy`` slice moves Y;
the other slices have y = 0, so their membership is as before. ``x_only`` counts the X slices'
drops alone: Z as it was before #86 (``z_x_only`` in the report), so earlier runs stay comparable.

Results are simulated, uncalibrated contact. Placement only for now (issue #46).
"""

from dataclasses import dataclass, field, replace
import math

import numpy as np

from brixzle.engine import box_search as BS
from brixzle.engine import sim_map as SM
from brixzle.engine.sim import trials as T

LABEL = BS.LABEL
SEATED, JAMMED = "seated", "jammed"
NAMED = ("fell", "wrong_valley")  # count 0; anything else (for example 'unstable') is 'other', also 0
HELD_OUT_FRICTION = (0.2, 0.5)    # the ends of the friction range (architecture.md)
DEFINITION = ("Z = (S - J) / N over the full sim map's drops inside the target box, all slices and both "
              "courses: S seated (+1), J jammed (-1), fell and wrong valley 0; Z = 1 passes")
EPS = 1e-9


def counts(outcomes):
    """Z and the counts behind it, from the outcomes of the drops inside one box."""
    outcomes = list(outcomes)
    if not outcomes:
        raise ValueError("no drops inside the box: Z is undefined (the map must cover the target box)")
    n = len(outcomes)
    s, j = outcomes.count(SEATED), outcomes.count(JAMMED)
    named = {k: outcomes.count(k) for k in NAMED}
    return {"Z": (s - j) / n, "S": s, "J": j, "N": n, **named, "other": n - s - j - sum(named.values()),
            "passed": s == n}


def z_band(c):
    """The noise band of Z (#110): one standard error of Z for the counts ``c`` ({'Z', 'S', 'J', 'N'}).
    Each drop scores +1, 0 or -1, so its variance is (S + J) / N - Z^2, and the band is
    sqrt(variance / N). A change in Z smaller than the band is not a change."""
    n = c.get("N") or 0
    if n <= 0:
        raise ValueError("no drops: the band of Z is undefined")
    var = (c["S"] + c["J"]) / n - c["Z"] ** 2
    return math.sqrt(max(var, 0.0) / n)


def within_band(z_new, z_ref, band):
    """True if ``z_new`` differs from ``z_ref`` by no more than ``band`` (#110)."""
    return abs(z_new - z_ref) <= band + EPS


def inside(pose, box):
    """Whether a pose {x, y, z, t} (mm, mm, mm above the minimum clearance, deg; y defaults to 0)
    lies inside a placement box: the X-Y disc of a disc box (#86), and the box's height and pitch."""
    r = SM.box_ranges(box)
    x, y = pose["x"], pose.get("y", 0.0)
    if getattr(box, "xy_shape", "box") == "disc":
        rad = box.half_widths["x_mm"]
        xy_in = x * x + y * y <= rad * rad + EPS
    else:
        xy_in = r["x"][0] - EPS <= x <= r["x"][1] + EPS and r["y"][0] - EPS <= y <= r["y"][1] + EPS
    return xy_in and all(r[a][0] - EPS <= pose[a] <= r[a][1] + EPS for a in ("z", "t"))


def drop_pose(d, clearance0):
    """The pose of a map drop, about the aim: X and Y offset, release height above the minimum, pitch."""
    return {"x": float(d["dx_mm"]), "y": float(d.get("dy_mm", 0.0)), "z": float(d["clear_mm"]) - clearance0,
            "t": float(d["pitch_deg"])}


def z_from_drops(drops, boxes, clearance0, x_only=False):
    """{statistic: counts} for every box in ``boxes`` ({statistic: TargetBox}), over the drops whose
    pose lies inside it. Zoom drops are left out. ``x_only``: Z as it was before #86, from the X
    slices alone (no ``xy`` drop, not even its y = 0 row, which repeats the ``xt`` slice's poses at
    coarser X, and no drop with a Y offset)."""
    drops = [d for d in drops if not d.get("zoom")
             and not (x_only and (d.get("dy_mm", 0.0) != 0.0 or d.get("slice") == "xy"))]
    poses = [drop_pose(d, clearance0) for d in drops]
    return {stat: counts([d["outcome"] for d, q in zip(drops, poses) if inside(q, box)])
            for stat, box in boxes.items()}


def union_ranges(boxes):
    """(lo, hi) per axis covering every box."""
    rs = [SM.box_ranges(b) for b in boxes]
    return {a: (min(r[a][0] for r in rs), max(r[a][1] for r in rs)) for a in SM.AXES}


# --- the map's cells inside the boxes, dropped directly ---


def box_cells(aim_box, boxes, U, config=None, courses=(0, 1), slices=tuple(SM.SLICES)):
    """The map's cells, on slices through ``aim_box``'s aim, whose pose lies inside at least one box.

    Each is {slice, course, cell, pose}. These are exactly the cells of a map run for ``aim_box``
    with ``cover`` = the boxes, so their drops give that map's Z."""
    config = config or SM.MapConfig()
    boxes = list(boxes.values()) if isinstance(boxes, dict) else list(boxes)
    aim = SM.aim_point(aim_box)
    ranges = union_ranges(boxes)
    out = []
    for s in slices:
        steps = config.slice_steps(float(U), s)
        a, b = SM.SLICES[s]
        (i0, i1), (j0, j1) = SM.index_extent(ranges, steps, (a, b))
        for course in courses:
            for j in range(j0, j1 + 1):
                for i in range(i0, i1 + 1):
                    pose = dict(aim)
                    pose[a], pose[b] = i * steps[a], j * steps[b]
                    if any(inside(pose, box) for box in boxes):
                        out.append({"slice": s, "course": course, "cell": [i, j], "pose": pose})
    return out


def _sample(cfg, pose):
    bias = cfg.error.aim_bias
    return {"dx": bias + pose["x"], "dy": pose.get("y", 0.0) + 0.0, "aim": bias, "yaw": 0.0, "roll": 0.0,
            "pitch": pose["t"] + 0.0, "clear": cfg.error.clearance[0] + pose["z"], "vel": [0.0, 0.0, 0.0]}


def held_out_poses(cells, box, U, seed, config=None, y=True):
    """Each cell's pose shifted by a seeded amount and kept inside ``box``.

    On each axis the shift is uniform up to half a map cell, or the box's half-width if that is
    smaller (pitch: the box is +-0.2 deg against 2 deg cells), and the pose is then clipped to the
    box. So every held-out pose is inside the box and most are off the map's grid.

    ``y`` (issue #86): Y is shifted too, by up to half a Y cell, and (x, y) is then pulled back into
    the box's X-Y disc along its radius. ``y=False`` shifts X, Z and pitch only, as before #86."""
    config = config or SM.MapConfig()
    steps, r = config.steps(float(U)), SM.box_ranges(box)
    half = {a: (r[a][1] - r[a][0]) / 2 for a in r}
    rng = np.random.default_rng(seed)
    out = []
    for c in cells:
        u = rng.uniform(-1.0, 1.0, size=3)
        pose = {}
        for k, a in enumerate(("x", "z", "t")):
            v = c["pose"][a] + u[k] * min(steps[a] / 2, half[a])
            pose[a] = float(min(max(v, r[a][0]), r[a][1]))
        if y:
            # A separate draw, after the first three, so the X, Z and pitch shifts are as before #86.
            yv = c["pose"].get("y", 0.0) + rng.uniform(-1.0, 1.0) * min(steps["y"] / 2, half["y"])
            pose["y"] = float(min(max(yv, r["y"][0]), r["y"][1]))
            if getattr(box, "xy_shape", "box") == "disc":
                rad, d = box.half_widths["x_mm"], math.hypot(pose["x"], pose["y"])
                if d > rad:
                    pose["x"], pose["y"] = pose["x"] * rad / d, pose["y"] * rad / d
        out.append(pose)
    return out


def _drop_poses(family, p, cfg, jobs, workers, friction=None):
    opts = {"trace": False} if friction is None else {"trace": False, "friction": float(friction)}
    with BS.DropPool(family, p, cfg, workers=min(workers or SM.DEFAULT_WORKERS, max(1, len(jobs)))) as pool:
        return [row["outcome"] for row in pool.imap([(course, _sample(cfg, pose), opts) for course, pose in jobs])]


def _ideal():
    return T.TrialConfig(error=T.ErrorModel(ideal=True))


def slices_for(y_error=True):
    """The map's slices: all of them, or without the X-Y slice when Y error is not scored (#86)."""
    return tuple(s for s in SM.SLICES if y_error or s != "xy")


@dataclass(frozen=True)
class ZScore:
    """A score function for the end tune: p -> Z on ``aim_box`` and on every box in ``boxes``,
    from the map's cells inside them (``box_cells``). An object, so it can be sent to another
    process. Returns {'score': Z on aim_box, 'targets': {statistic: counts}, 'drop_count'}."""

    family: object
    aim_box: object
    boxes: dict
    cfg: T.TrialConfig = field(default_factory=_ideal)
    config: SM.MapConfig | None = None
    workers: int | None = None
    y_error: bool = True  # score the X-Y disc (#86); False: X offsets only, as before

    def __call__(self, p):
        U = float(self.family.lattice(p).U_mm)
        cells = box_cells(self.aim_box, self.boxes, U, self.config, slices=slices_for(self.y_error))
        outcomes = _drop_poses(self.family, p, self.cfg, [(c["course"], c["pose"]) for c in cells], self.workers)
        targets = {stat: counts([o for c, o in zip(cells, outcomes) if inside(c["pose"], box)])
                   for stat, box in self.boxes.items()}
        return {"score": targets[self.aim_box.statistic]["Z"], "targets": targets, "drop_count": len(cells)}


@dataclass(frozen=True)
class HeldOutZ:
    """Z on one box from the held-out poses (``held_out_poses``) at friction ``mu``.
    Returns {'score': Z, 'counts': counts, 'seed', 'mu'}."""

    family: object
    aim_box: object
    box: object
    seed: int
    mu: float
    cfg: T.TrialConfig = field(default_factory=_ideal)
    config: SM.MapConfig | None = None
    workers: int | None = None
    y_error: bool = True  # shift Y too and score the X-Y disc (#86); False: as before

    def __call__(self, p):
        U = float(self.family.lattice(p).U_mm)
        cells = box_cells(self.aim_box, {self.box.statistic: self.box}, U, self.config,
                          slices=slices_for(self.y_error))
        poses = held_out_poses(cells, self.box, U, self.seed, self.config, y=self.y_error)
        outcomes = _drop_poses(self.family, p, self.cfg, [(c["course"], q) for c, q in zip(cells, poses)],
                               self.workers, friction=self.mu)
        c = counts(outcomes)
        return {"score": c["Z"], "counts": c, "seed": self.seed, "mu": self.mu}


def held_out_scores(family, box, seed, aim_box=None, cfg=None, config=None, workers=None, mus=HELD_OUT_FRICTION,
                    y_error=True):
    """{name: HeldOutZ} for ``box``: the next seed's shifted poses at each end of the friction range."""
    aim_box = aim_box or box
    cfg = cfg or _ideal()
    return {f"seed {seed}, friction {mu}": HeldOutZ(family, aim_box, box, seed, mu, cfg, config, workers, y_error)
            for mu in mus}


def with_workers(fn, workers):
    """A ZScore or HeldOutZ with another worker count (any other function unchanged)."""
    return replace(fn, workers=workers) if isinstance(fn, (ZScore, HeldOutZ)) else fn
