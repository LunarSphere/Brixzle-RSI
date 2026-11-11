"""Jam analysis inside the green zone (issue #49): which failures are real, and what holds the brick.

The goal is a green zone with no jams inside it. The fitted map (engine/sim_map.py) lists the
failing cells inside its green zone; this module tells the critic (#19) which of them repeat and
why they happen:

1. **Repeatable or one-off.** Each cell is re-run with several seeds (a small seeded shift of the
   pose) at each friction coefficient (0.2, 0.3, 0.4, 0.5). A cell that fails most seeds at some
   friction is real; a lone cell that does not come back is solver noise.
2. **Bands.** Repeatable cells that touch (diagonals count) on one slice and course form a band.
3. **Contacts.** The drop nearest each band's centre is run again, and its contacts at rest are
   reduced, from the sim, not chosen by an agent:
   a. keep the contacts between the brick and what it rests on that still touch;
   b. flatten to the X-Z plane and merge points that lie together with similar normals into
      sites; a flat face-on-face patch becomes its two end points, the rule the predictor uses
      (planar._line_extremes). A site must carry more than 5% of the brick's weight;
   c. if the brick rests out of plane, or a large share of the sites' force is along Y, the band
      is a "3D effect" and the reduction stops: the 2D analysis cannot explain it;
   d. otherwise find the smallest set of sites whose friction can balance gravity, with the
      predictor's weak-stability test (contact_modes, after Pang & Trinkle 2000) at the sim's
      friction. MuJoCo's own split of the load is one of many valid ones, so it is not trusted;
   e. report each contact of that set: position, normal, force, and which face of the falling
      brick touches which face below, with the predictor's stick margin and the Whitney (1982)
      wedging test.

The failure type (jammed, wrong valley, fell) comes from the sim, and the mechanism from the
reduction. docs/predictor-validation.md is why: the predictor is reliable on seated-or-not and
on stability, but it often swaps jammed and wrong valley.

One contact means stuck on a face too shallow for its friction (R6). Two contacts are the usual
jam: both sticking (Balkcom & Trinkle 2002), and wedged if they push from opposite sides with
the line between them inside both friction cones (Whitney 1982). Three or more are reported as
found.

jam_analysis.json is written beside the map. Sim results are simulated, uncalibrated contact;
predictor results are predicted, quasi-static 2D. Placement only for now (issue #46).

    uv run python -m brixzle.engine.jam_analysis --map runs/map-<stamp>-ab-mean
"""

import argparse
from dataclasses import asdict, dataclass
import itertools
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
from shapely.geometry.polygon import orient

from brixzle.engine import box_search as BS
from brixzle.engine import sim_map as SM
from brixzle.engine.predictor import predict as PR
from brixzle.engine.predictor.contact_modes import Contact, ContactSystem
from brixzle.engine.sim import trials as T
from brixzle.families import builders as B
from brixzle.families.interface import Prismatic, Section

LABELS = {"sim": BS.LABEL, "pred": PR.LABEL}
SEATED = BS.SUCCESS["placement"]
FILE = "jam_analysis.json"
G = PR.G  # unit weight, as the predictor: forces are in units of the brick's weight


@dataclass(frozen=True)
class JamConfig:
    seeds: int = 3                  # re-runs of each cell per friction coefficient
    mus: tuple = PR.MUS             # 0.2, 0.3, 0.4, 0.5
    jitter: float = 0.05            # each re-run shifts the pose by up to this share of a map cell
    repeat_share: float = 2 / 3     # repeatable: fails this share of the seeds at some friction
    min_force_share: float = 0.05   # a contact site must carry this share of the brick's weight
    touch_mm: float = 0.01          # contacts further apart than this are not touching
    merge_mm: float = 0.5           # points closer than this in X-Z ...
    merge_deg: float = 10.0         # ... with normals within this angle are one site
    line_mm: float = 0.1            # sites on one line within this ...
    face_deg: float = 3.0           # ... with normals within this angle are one flat patch
    # "A large share" of the sites' force along Y: a 3D effect. Measured on A/B V0 (course 1, sim
    # mu 0.35; tests/test_jam_analysis.py): a clean seat has 0.03 once its mirror-image gutter
    # contacts are merged, and an in-plane jam at the capture edge about 0.10 to 0.13.
    y_share: float = 0.25
    out_of_plane_mm: float = 1.0    # the brick resting further than this off the plane in Y ...
    out_of_plane_deg: float = 2.0   # ... or rolled or yawed by more than this: a 3D effect
    corner_mm: float = 0.6          # a contact this near a vertex is named as the corner
    max_sites: int = 10             # sites searched for the holding set (the most loaded ones)

    def __post_init__(self):
        if not isinstance(self.seeds, int) or self.seeds < 1 or not self.mus:
            raise ValueError("seeds must be an integer >= 1 and mus must not be empty")
        if not all(isinstance(m, (int, float)) and math.isfinite(m) and m > 0 for m in self.mus):
            raise ValueError(f"mus must be finite and > 0, got {self.mus}")
        if not 0 < self.repeat_share <= 1 or not 0 <= self.jitter < 0.5:
            raise ValueError("repeat_share must be in (0, 1] and jitter in [0, 0.5)")


def _mu_key(mu):
    return f"{mu:g}"


# --- repeatable or one-off, and bands ---


def repeatability(reruns, cfg: JamConfig = JamConfig()):
    """From a cell's re-runs ([{mu, seed, outcome}]): how often it failed overall and at each
    friction, and whether it is repeatable (it fails ``repeat_share`` of the seeds at some
    friction). A design must seat at every friction, so one friction that reliably jams is real."""
    per = {}
    for mu in sorted({r["mu"] for r in reruns}):
        rows = [r for r in reruns if r["mu"] == mu]
        per[_mu_key(mu)] = sum(r["outcome"] != SEATED for r in rows) / len(rows)
    failed = sum(r["outcome"] != SEATED for r in reruns)
    return {"repeat_rate": failed / len(reruns) if reruns else 0.0, "per_mu": per,
            "repeatable": bool(per) and max(per.values()) >= cfg.repeat_share - 1e-9}


def bands(cells):
    """Group cells ({slice, course, cell: [i, j], ...}) into connected bands: same slice and
    course, touching side to side or corner to corner. Returns a list of lists, in a fixed order."""
    groups = []
    todo = sorted(cells, key=lambda c: (c["slice"], c["course"], c["cell"][1], c["cell"][0]))
    seen = set()
    for k, first in enumerate(todo):
        if k in seen:
            continue
        seen.add(k)
        group, queue = [first], [first]
        while queue:
            a = queue.pop()
            for m, b in enumerate(todo):
                if m in seen or (b["slice"], b["course"]) != (a["slice"], a["course"]):
                    continue
                if max(abs(a["cell"][0] - b["cell"][0]), abs(a["cell"][1] - b["cell"][1])) <= 1:
                    seen.add(m)
                    group.append(b)
                    queue.append(b)
        groups.append(sorted(group, key=lambda c: (c["cell"][1], c["cell"][0])))
    return groups


def representative(band):
    """The cell nearest the band's centre (the lower row, then the lower column, on a tie)."""
    pts = np.array([c["cell"] for c in band], float)
    d = np.linalg.norm(pts - pts.mean(axis=0), axis=1)
    order = sorted(range(len(band)), key=lambda k: (round(float(d[k]), 9), band[k]["cell"][1], band[k]["cell"][0]))
    return band[order[0]]


# --- the reduction: contacts at rest -> sites -> the smallest holding set ---


def _unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.zeros_like(v)


def _angle_deg(a, b):
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


def _merged(contacts, cfg):
    """Touching contacts flattened to X-Z and merged into point clusters, then flat patches.

    Returns a list of sites, most loaded first. Each holds its end points (one, or two for a flat
    patch), its unit normal in X-Z, the summed normal force and force vector, the 3D positions of
    its members, and ``mu_scale``: the summed normal force over its X-Z part. Faces that lean
    across Y (a V gutter) press harder than their in-plane resultant shows, so in plane their
    friction is larger by that factor."""
    clusters = []
    touching = [c for c in contacts if c["dist_mm"] <= cfg.touch_mm and c["normal_N"] > 0]
    for c in sorted(touching, key=lambda c: -c["normal_N"]):
        pos, n3, f3 = np.asarray(c["pos_mm"], float), np.asarray(c["normal"], float), np.asarray(c["force_N"], float)
        p, nxz = pos[[0, 2]], _unit(n3[[0, 2]])
        hit = None
        for q in clusters:
            if q["other"] == c["other"] and np.linalg.norm(q["p"] - p) <= cfg.merge_mm \
                    and _angle_deg(q["n"], nxz) <= cfg.merge_deg:
                hit = q
                break
        if hit is None:
            hit = {"other": c["other"], "p": p, "n": nxz, "fn": 0.0, "F": np.zeros(3), "N3": np.zeros(3),
                   "wp": np.zeros(2), "members": []}
            clusters.append(hit)
        hit["fn"] += c["normal_N"]
        hit["F"] += f3
        hit["N3"] += c["normal_N"] * n3
        hit["wp"] += c["normal_N"] * p
        hit["members"].append(pos)
    for q in clusters:
        q["p"] = q["wp"] / q["fn"]
        q["n"] = _unit(q["N3"][[0, 2]])
    # Flat patches: clusters with one normal on one line become a face with two end points.
    faces = []
    for q in sorted(clusters, key=lambda q: -q["fn"]):
        for f in faces:
            if f["other"] == q["other"] and _angle_deg(f["n"], q["n"]) <= cfg.face_deg \
                    and abs(float(f["n"] @ (q["p"] - f["parts"][0]["p"]))) <= cfg.line_mm:
                f["parts"].append(q)
                break
        else:
            faces.append({"other": q["other"], "n": q["n"], "parts": [q]})
    sites = []
    for f in faces:
        parts = f["parts"]
        fn = sum(q["fn"] for q in parts)
        N3 = np.sum([q["N3"] for q in parts], axis=0)
        n = _unit(N3[[0, 2]])
        t = np.array([-n[1], n[0]])
        s = [float(t @ q["p"]) for q in parts]
        ends = [parts[int(np.argmin(s))]["p"], parts[int(np.argmax(s))]["p"]] if len(parts) > 1 else [parts[0]["p"]]
        in_plane = float(np.linalg.norm(N3[[0, 2]]))
        sites.append({"kind": "face" if len(parts) > 1 else "point", "other": f["other"],
                      "points": [e.tolist() for e in ends], "normal": n.tolist(), "normal_N": float(fn),
                      "force_N": np.sum([q["F"] for q in parts], axis=0).tolist(),
                      "members": sum(len(q["members"]) for q in parts),
                      "mu_scale": float(fn / in_plane) if in_plane > 1e-12 else float("inf"),
                      "_positions": [m for q in parts for m in q["members"]]})
    return sorted(sites, key=lambda s: -s["normal_N"]), touching


def sites(contacts, weight_N, cfg: JamConfig = JamConfig()):
    """The contact sites that carry real force: steps a and b of the reduction."""
    merged, _ = _merged(contacts, cfg)
    return [_public(s) for s in merged if s["normal_N"] >= cfg.min_force_share * weight_N
            and math.isfinite(s["mu_scale"])]


def _public(site):
    out = {k: v for k, v in site.items() if not k.startswith("_")}
    ys = [float(m[1]) for m in site["_positions"]]
    out["y_mm"] = [min(ys), max(ys)]
    return out


def _system(chosen, com, L, r2):
    cs = [Contact(tuple(p), tuple(s["normal"])) for s in chosen for p in s["points"]]
    return ContactSystem(cs, com, np.diag([1.0, 1.0, r2]), L)


def _holds(chosen, com, mu, L, r2):
    """Can friction at these sites balance gravity? The predictor's weak-stability test (a cone
    membership test), with each site's friction scaled by its ``mu_scale``."""
    sys = _system(chosen, com, L, r2)
    mus = np.array([mu * s["mu_scale"] for s in chosen for _ in s["points"]])
    return sys._in_cone(np.hstack([sys.Wn + mus * sys.Wt, sys.Wn - mus * sys.Wt]), G), sys


def holding_set(site_list, com, mu, L, r2=None, cfg: JamConfig = JamConfig()):
    """The smallest set of sites whose friction alone can balance gravity at ``mu``; None if not
    even all of them can. Among sets of the same size, the one the sim loads most is reported."""
    r2 = L * L / 12 if r2 is None else r2
    pool = list(range(min(len(site_list), cfg.max_sites)))  # sites arrive most loaded first
    for size in range(1, len(pool) + 1):
        found = []
        for combo in itertools.combinations(pool, size):
            ok, sys = _holds([site_list[i] for i in combo], com, mu, L, r2)
            if ok:
                found.append((combo, sys))
        if found:
            combo, sys = max(found, key=lambda f: sum(site_list[i]["normal_N"] for i in f[0]))
            mu_eff = mu * max(site_list[i]["mu_scale"] for i in combo)
            return {"sites": list(combo), "size": size, "alternatives": len(found) - 1, "mu": float(mu),
                    "mu_effective": float(mu_eff), "stick_margin": float(sys.stick_margin(G, mu_eff)),
                    "wedged": bool(sys.wedged(mu_eff)), "opposing": bool(sys.opposing()),
                    "needs_friction": not bool(sys.frictionless_stable(G))}
    return None


def _mechanism(site_list, holding, all_margin, mu):
    if holding is None:
        return (f"not explained by 2D friction at the sim's mu {mu:g}: no set of the contacts at rest balances "
                f"gravity (stick margin {all_margin:+.3f} of the weight)")
    chosen = [site_list[i] for i in holding["sites"]]
    n = holding["size"]
    if not holding["needs_friction"]:
        what = "one contact holds" if n == 1 else f"{n} contacts hold"
        return f"{what} the brick without friction: a stable rest away from the seat, not a friction jam"
    if n == 1:
        nx, nz = chosen[0]["normal"]
        slope = math.degrees(math.atan2(abs(nx), abs(nz)))
        return (f"one contact: stuck on a face too shallow for its friction (R6): the face is {slope:.0f} deg from "
                f"horizontal and holds while tan(slope) = {math.tan(math.radians(slope)):.2f} <= mu "
                f"{holding['mu_effective']:.2f}")
    if n == 2:
        if holding["opposing"]:
            tail = "the pair can wedge it (Whitney 1982, two-point wedging)" if holding["wedged"] else \
                "both sticking (Balkcom & Trinkle 2002)"
            return f"two contacts pushing from opposite sides: {tail}"
        return "two contacts, both sticking (Balkcom & Trinkle 2002)"
    return f"{n} contacts hold the brick (reported as found)"


def reduce_contacts(contacts, com, weight_N, mu, L, cfg: JamConfig = JamConfig(), out_of_plane=None, r2=None):
    """Steps a to d on one settled drop's contacts.

    ``com`` is the brick's centre of mass (x, z) in the world, ``L`` a length that scales moments
    (the voxel pitch), ``out_of_plane`` the settled pose's {'dy_mm', 'tilt_deg'} if known. Returns
    the sites, whether it is a 3D effect, the smallest holding set and the mechanism."""
    return _reduce(contacts, com, weight_N, mu, L, cfg, out_of_plane, r2)[0]


def _reduce(contacts, com, weight_N, mu, L, cfg, out_of_plane, r2):
    """reduce_contacts, plus the kept sites with their members' 3D positions (for naming)."""
    r2 = L * L / 12 if r2 is None else r2
    merged, touching = _merged(contacts, cfg)
    f_sites = sum(float(np.linalg.norm(s["force_N"])) for s in merged)
    f_raw = sum(float(np.linalg.norm(c["force_N"])) for c in touching)
    y_net = sum(abs(s["force_N"][1]) for s in merged) / f_sites if f_sites > 0 else 0.0
    y_raw = sum(abs(c["force_N"][1]) for c in touching) / f_raw if f_raw > 0 else 0.0
    kept = [s for s in merged if s["normal_N"] >= cfg.min_force_share * weight_N and math.isfinite(s["mu_scale"])]
    reason = None
    if out_of_plane and (abs(out_of_plane["dy_mm"]) > cfg.out_of_plane_mm or out_of_plane["tilt_deg"] > cfg.out_of_plane_deg):
        reason = (f"the brick rests rolled or yawed {out_of_plane['tilt_deg']:.1f} deg and "
                  f"{abs(out_of_plane['dy_mm']):.1f} mm off its plane in Y")
    elif y_net > cfg.y_share:
        reason = f"{y_net:.0%} of the contact force is along Y"
    out = {"three_d": reason is not None, "y_share": float(y_net), "y_share_raw": float(y_raw),
           "out_of_plane": out_of_plane, "mu": float(mu), "contacts_touching": len(touching),
           # The most loaded single contact point, as a share of the weight: a seated face spreads
           # its load over many points, which is why the force threshold applies to sites.
           "max_point_share": max((c["normal_N"] for c in touching), default=0.0) / weight_N,
           "sites": [_public(s) for s in kept]}
    if reason:
        return {**out, "holding": None, "all_sites": None,
                "mechanism": f"3D effect: {reason}; the 2D reduction stops here"}, kept
    if kept:
        ok, sys = _holds(kept, com, mu, L, r2)
        every = {"weakly_stable": bool(ok), "frictionless_stable": bool(sys.frictionless_stable(G)),
                 "stick_margin": float(sys.stick_margin(G, mu))}
    else:
        every = {"weakly_stable": False, "frictionless_stable": False, "stick_margin": 1.0}
    holding = holding_set(kept, com, mu, L, r2, cfg) if kept else None
    return {**out, "holding": holding, "all_sites": every,
            "mechanism": _mechanism(kept, holding, every["stick_margin"], mu)}, kept


# --- naming the faces ---

_DIRECTIONS = ("right", "up-right", "up", "up-left", "left", "down-left", "down", "down-right")


def _direction(v):
    return _DIRECTIONS[int(round(math.degrees(math.atan2(v[1], v[0])) / 45.0)) % 8]


def name_feature(outline, guide_faces, point, corner_mm=0.6):
    """Which feature of a section outline a point (x, z) touches: a corner (within ``corner_mm``
    of a vertex) or a face (its nearest edge), with enough geometry to find it on a drawing."""
    ring = np.asarray(orient(outline).exterior.coords)[:-1]  # counter-clockwise
    pt = np.asarray(point, float)
    n = len(ring)
    dv = np.linalg.norm(ring - pt, axis=1)
    k = int(np.argmin(dv))
    if dv[k] <= corner_mm:
        prev, cur, nxt = ring[k - 1], ring[k], ring[(k + 1) % n]
        a, b = cur - prev, nxt - cur
        convex = bool(a[0] * b[1] - a[1] * b[0] > 0)
        out = {"feature": "corner", "index": k, "at_mm": [float(cur[0]), float(cur[1])], "convex": convex, "guide": False}
        label = f"{'convex' if convex else 'concave'} corner at ({cur[0]:.1f}, {cur[1]:.1f}) mm"
        if convex:
            out["pointing"] = _direction(-(_unit(prev - cur) + _unit(nxt - cur)))
            label += f", pointing {out['pointing']}"
        return {**out, "label": label}
    best, best_d = 0, math.inf
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        d = b - a
        t = float(np.clip((pt - a) @ d / (d @ d), 0.0, 1.0))
        dist = float(np.linalg.norm(pt - (a + t * d)))
        if dist < best_d:
            best, best_d = i, dist
    a, b = ring[best], ring[(best + 1) % n]
    d = b - a
    angle = math.degrees(math.atan2(abs(d[1]), abs(d[0])))
    facing = _direction(np.array([d[1], -d[0]]))  # the outward normal of a counter-clockwise ring
    mid = (a + b) / 2
    guide = any(_on_segment(mid, g.a, g.b) for g in guide_faces)
    label = (f"{angle:.0f} deg {'guide ' if guide else ''}face facing {facing}, from ({a[0]:.1f}, {a[1]:.1f}) "
             f"to ({b[0]:.1f}, {b[1]:.1f}) mm")
    return {"feature": "face", "index": best, "angle_deg": float(angle), "facing": facing,
            "from_mm": [float(a[0]), float(a[1])], "to_mm": [float(b[0]), float(b[1])], "guide": bool(guide),
            "label": label}


def _on_segment(pt, a, b, tol=1e-3):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    t = float(np.clip((pt - a) @ d / (d @ d), 0.0, 1.0))
    return float(np.linalg.norm(pt - (a + t * d))) <= tol


def _rot(quat):
    return Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])


class _Frames:
    """Maps world contact points into the section frames of the falling brick and of what it
    rests on, so each contact can be named on both."""

    def __init__(self, family, p, course, rest):
        self.family, self.p = family, p
        g = family.geometry
        self.extrusion = g.extrusion(p) if isinstance(g, Prismatic) else None
        self.part = family.part_for(course, 1)
        self.section = B.section(family, p, self.part)
        self.pos, self.rot = np.asarray(rest["pos_mm"], float), _rot(rest["quat"])
        self.below_part, self.below_section, self.below = None, None, None
        if rest.get("support"):
            self.below_part = family.part_for(0, 1)
            self.below_section = B.section(family, p, self.below_part)
            self.below = (np.asarray(rest["support"]["pos_mm"], float), _rot(rest["support"]["quat"]))
        base = family.base(p, 4, 1)  # the rig's base: voxels 0 to 4, base frame = world
        self.base_section = base if isinstance(base, Section) else None

    def _shift(self, y):
        if self.extrusion is None:
            return 0.0
        half = self.extrusion.depth_mm / 2
        return B._band_shift(self.extrusion, float(np.clip(y, -half, half)))

    def _local(self, world, frame):
        q = np.asarray(world, float) if frame is None else frame[1].inv().apply(np.asarray(world, float) - frame[0])
        return np.array([q[0], q[2] - self._shift(q[1])])

    def _mean(self, positions, frame):
        return np.mean([self._local(m, frame) for m in positions], axis=0)

    def name(self, site, corner_mm):
        on_brick = name_feature(self.section.outer, self.section.guide_faces,
                                self._mean(site["_positions"], (self.pos, self.rot)), corner_mm)
        on_brick = {"part": self.part, **on_brick, "label": f"{self.part}: {on_brick['label']}"}
        if site["other"].startswith("brick") and self.below is not None:
            f = name_feature(self.below_section.outer, self.below_section.guide_faces,
                             self._mean(site["_positions"], self.below), corner_mm)
            below = {"part": self.below_part, **f, "label": f"{self.below_part}: {f['label']}"}
        elif self.base_section is not None:
            f = name_feature(self.base_section.outer, (), self._mean(site["_positions"], None), corner_mm)
            below = {"part": "base", **f, "label": f"base: {f['label']}"}
        else:
            below = {"part": "base", "feature": "unknown", "label": "base"}
        return on_brick, below

    def com(self, bodies):
        c = self.pos + self.rot.apply(np.asarray(bodies.com_mm, float))
        return (float(c[0]), float(c[2]))

    def tilt_deg(self):
        """How far the brick's Y axis has left the world's Y axis (roll and yaw together)."""
        return math.degrees(math.acos(float(np.clip(abs(self.rot.as_matrix()[1, 1]), 0.0, 1.0))))


def reduce_row(family, p, course, row, mu, cfg: JamConfig = JamConfig()):
    """The full reduction (steps a to e) of one drop row that carries its settled state
    (DropRig.drop(..., rest=True)): contact sites named on both bodies, the 3D check, the smallest
    holding set and the mechanism."""
    rest = row["rest"]
    frames = _Frames(family, p, course, rest)
    bodies = B.sim_bodies(family, p, frames.part)
    U = float(family.lattice(p).U_mm)
    plane = {"dy_mm": float(row["err_mm"][1]), "tilt_deg": float(frames.tilt_deg())}
    out, kept = _reduce(rest["contacts"], frames.com(bodies), rest["weight_N"], mu, U, cfg, plane,
                        float(bodies.inertia_g_mm2[1][1]) / bodies.mass_g)
    for public, site in zip(out["sites"], kept):
        public["on_brick"], public["on_support"] = frames.name(site, cfg.corner_mm)
    return {"outcome": row["outcome"], "err_mm": [float(v) for v in row["err_mm"]], "angle_deg": float(row["angle_deg"]),
            "weight_N": float(rest["weight_N"]), "com_mm": list(frames.com(bodies)), **out}


def analyse_pose(family, p, course, sample, sim_cfg=None, cfg: JamConfig = JamConfig()):
    """Drop one pose and reduce its contacts at rest (for a look at a single failure by hand)."""
    sim_cfg = sim_cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    row = T.DropRig(family, p, sim_cfg, course=course).drop(sample, trace=False, rest=True)
    return {"labels": dict(LABELS), "course": course, "sample": {k: v for k, v in sample.items() if k != "vel"},
            **reduce_row(family, p, course, row, float(sim_cfg.physics.friction), cfg)}


# --- the analysis of a map ---


def _jittered(cell, seed, salt, cfg, z_axis):
    """The cell shifted by a seeded fraction of a cell on each axis (never below release height 0)."""
    rng = np.random.default_rng([seed, *salt, cell[0] + 100000, cell[1] + 100000])
    d = rng.uniform(-cfg.jitter, cfg.jitter, 2)
    out = [cell[0] + float(d[0]), cell[1] + float(d[1])]
    if z_axis is not None:
        out[z_axis] = max(out[z_axis], 0.0)
    return tuple(out)


def analyse(family, p, sim_map, cfg: JamConfig = JamConfig(), sim_cfg=None, workers=None):
    """Analyse the failures inside the green zone of a fitted map (sim_map.sim_map's first result,
    or map.json): re-run each, group the repeatable ones into bands and reduce each band's
    contacts. Returns jam_analysis.json as a dict."""
    sim_cfg = sim_cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    sim_mu = float(sim_cfg.physics.friction)
    aim = sim_map["aim"]
    t0 = time.time()
    cells, jobs = [], []
    for g in sim_map["grids"]:
        axes = tuple(g["axes"])
        steps = {**SM.MapConfig().steps(float(family.lattice(p).U_mm)), **g["step"]}
        z_axis = axes.index("z") if "z" in axes else None
        for f in g["inside_failures"]:
            cell = {"slice": g["slice"], "course": g["course"], "cell": list(f["cell"]), "outcome": f["outcome"],
                    **{a: f[a] for a in axes}, "_steps": steps}
            cells.append(cell)
            for mu in cfg.mus:
                for seed in range(cfg.seeds):
                    at = _jittered(f["cell"], seed, (g["course"], sorted(SM.SLICES).index(g["slice"])), cfg, z_axis)
                    jobs.append((len(cells) - 1, mu, seed,
                                 (g["course"], SM.cell_sample(sim_cfg, g["slice"], at, steps, aim),
                                  {"friction": float(mu), "trace": False})))
    n_workers = workers or (1 if len(jobs) <= 60 else SM.DEFAULT_WORKERS)
    out_bands, references = [], {}
    with BS.DropPool(family, p, sim_cfg, workers=n_workers) as pool:
        for c in cells:
            c["reruns"] = []
        for (k, mu, seed, _), row in zip(jobs, pool.imap([j[3] for j in jobs])):
            cells[k]["reruns"].append({"mu": float(mu), "seed": seed, "outcome": row["outcome"]})
        for c in cells:
            c.update(repeatability(c["reruns"], cfg))
        groups = bands([c for c in cells if c["repeatable"]])
        reps = [representative(g) for g in groups]
        rep_jobs = [(r["course"], SM.cell_sample(sim_cfg, r["slice"], r["cell"], r["_steps"], aim),
                     {"rest": True, "trace": False}) for r in reps]
        for k, (group, rep, job, row) in enumerate(zip(groups, reps, rep_jobs, pool.imap(rep_jobs))):
            types = {}
            for c in group:
                types[c["outcome"]] = types.get(c["outcome"], 0) + 1
            axes = SM.SLICES[rep["slice"]]
            out_bands.append({
                "id": k, "slice": rep["slice"], "course": rep["course"], "cells": [c["cell"] for c in group],
                "extent": {a: [min(c[a] for c in group), max(c[a] for c in group)] for a in axes},
                "failure_types": dict(sorted(types.items())), "failure_type": max(sorted(types), key=types.get),
                "repeat_rate": float(np.mean([c["repeat_rate"] for c in group])),
                "per_mu": {m: float(np.mean([c["per_mu"][m] for c in group])) for m in group[0]["per_mu"]},
                "representative": {"cell": rep["cell"], **{a: rep[a] for a in axes},
                                   "sample": {q: v for q, v in job[1].items() if q != "vel"}},
                **reduce_row(family, p, rep["course"], row, sim_mu, cfg)})
        # A seated drop at the aim on each course, for comparison: its Y force share shows what a
        # clean seat looks like on this family (a V gutter leans its faces across Y).
        courses = sorted({c["course"] for c in cells})
        aim_jobs = [(course, SM.cell_sample(sim_cfg, "xz", (0, 0), {"x": 1.0, "z": 1.0, "t": 1.0}, aim),
                     {"rest": True, "trace": False}) for course in courses]
        for course, row in zip(courses, pool.imap(aim_jobs)):
            r = reduce_row(family, p, course, row, sim_mu, cfg)
            scales = [s["mu_scale"] for s in r["sites"]]
            references[str(course)] = {k: r[k] for k in ("outcome", "y_share", "y_share_raw", "contacts_touching",
                                                         "max_point_share", "out_of_plane", "three_d")} | {
                "sites": len(r["sites"]), "mu_scale": [min(scales), max(scales)] if scales else None}
    for c in cells:
        c.pop("_steps")
    return {
        "labels": dict(LABELS), "family": family.name, "family_version": family.version, "params": p.to_dict(),
        "box": sim_map["box"], "config": {**asdict(cfg), "mus": list(cfg.mus)}, "sim_mu": sim_mu,
        "cells": cells, "bands": out_bands, "one_offs": [c for c in cells if not c["repeatable"]],
        "seated_reference": references,
        "counts": {"inside_failures": len(cells), "repeatable": sum(c["repeatable"] for c in cells),
                   "bands": len(out_bands), "three_d": sum(b["three_d"] for b in out_bands),
                   "reruns": len(jobs)},
        "workers": n_workers, "seconds": time.time() - t0,
    }


def write_analysis(result, map_dir):
    """Write jam_analysis.json into the map's run directory; never overwrites one."""
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"
    path = Path(map_dir) / FILE
    with open(path, "x", encoding="utf-8") as f:
        f.write(text)
    return path


def summary(result):
    """A short text summary, for the terminal."""
    c = result["counts"]
    lines = [f"{result['family']} placement {result['box']['statistic']}: {c['inside_failures']} failures inside the "
             f"green zone, {c['repeatable']} repeatable in {c['bands']} bands ({c['three_d']} of them 3D effects), "
             f"{len(result['one_offs'])} one-off; {c['reruns']} re-runs, {result['seconds']:.1f} s "
             f"(sim: {LABELS['sim']}; mechanism: {LABELS['pred']})"]
    for b in result["bands"]:
        ext = ", ".join(f"{a} {lo:+g} to {hi:+g} {SM.AXIS_UNITS[a]}" for a, (lo, hi) in b["extent"].items())
        lines.append(f"  band {b['id']} ({b['slice']} course {b['course']}, {ext}; {len(b['cells'])} cells): "
                     f"{b['failure_type']}, repeat rate {b['repeat_rate']:.0%} {b['per_mu']}")
        lines.append(f"    {b['mechanism']}")
        show = b["sites"] if b["holding"] is None else [b["sites"][i] for i in b["holding"]["sites"]]
        for s in show:
            pts = " to ".join(f"({x:.1f}, {z:.1f})" for x, z in s["points"])
            lines.append(f"    - {s['on_brick']['label']}  ON  {s['on_support']['label']}; at {pts} mm, normal "
                         f"({s['normal'][0]:+.2f}, {s['normal'][1]:+.2f}), {s['normal_N'] / b['weight_N']:.2f} of the weight")
        if b["holding"] is not None:
            h = b["holding"]
            lines.append(f"    holding set of {h['size']}: stick margin {h['stick_margin']:+.3f}, wedged {h['wedged']}, "
                         f"needs friction {h['needs_friction']}")
    return "\n".join(lines)


def parser():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--map", required=True, help="a map run directory (runs/map-<stamp>-<family>-<target>)")
    ap.add_argument("--seeds", type=int, default=JamConfig().seeds)
    ap.add_argument("--workers", type=int, default=None)
    return ap


def main(argv=None):
    from brixzle.families import registry

    args = parser().parse_args(argv)
    map_dir = Path(args.map)
    sim_map = json.loads((map_dir / "map.json").read_text(encoding="utf-8"))
    family = registry.get(sim_map["family"])
    p = family.params_type.from_dict(sim_map["params"])
    result = analyse(family, p, sim_map, cfg=JamConfig(seeds=args.seeds), workers=args.workers)
    result["map"] = map_dir.name
    path = write_analysis(result, map_dir)
    print(summary(result))
    print(f"-> {path}")


if __name__ == "__main__":
    main()
