"""The staged search's stages and driver (#166). See the package docstring."""
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, field, fields
from datetime import datetime, timezone
import json
import math
import multiprocessing
from pathlib import Path
import time

import numpy as np

LABEL_PRED = "predicted, quasi-static 2D"
LABEL_SIM = "simulated, uncalibrated contact"


@dataclass(frozen=True)
class SearchSettings:
    n: int = 2000                   # candidates sampled
    seed: int = 0
    max_length_mm: float = 70.0     # #164: brick length (2U here)
    max_release_mm: float = 30.0    # #164: release lift
    mass_max_g: float = 25.0        # R9
    mass_min_g: float = 1.0
    payload_g: float = 35.0         # R10, with the 5 g gripper allowance
    gripper_g: float = 5.0
    predict_r_mm: float = 10.0      # stage 1: X offsets the predictor must seat
    drops_min: float = 0.6          # stage 2: (S - J) / N to pass
    keep_after_drops: int = 20      # at most this many go on to stage 3, best first
    full: int = 3                   # at most this many get the full evaluation
    workers: int = 8
    fixed: dict = field(default_factory=lambda: {"end_gap": 0.3})
    # prim allows y_deg 0 (straight) or >= 30 deg (R6); sampling 0-55 wasted 43 % of a first search
    # (runs/search-20261120T184736Z-prim), so the search samples the V across Y only.
    bounds: dict = field(default_factory=lambda: {"y_deg": (30.0, 55.0)})


def family():
    from brixzle.families.prim.family import FAMILY
    return FAMILY


def sample(settings=SearchSettings()):
    """``n`` parameter dicts from a scrambled Sobol sequence over prim's bounds."""
    from scipy.stats import qmc

    fam = family()
    bounds = {**fam.bounds, **settings.bounds}
    names = sorted(bounds)
    lo = np.array([bounds[k][0] for k in names])
    hi = np.array([bounds[k][1] for k in names])
    m = int(math.ceil(math.log2(max(2, settings.n))))
    pts = qmc.Sobol(len(names), scramble=True, seed=settings.seed).random_base2(m)[: settings.n]
    vals = lo + pts * (hi - lo)
    return [{**{k: float(v) for k, v in zip(names, row)}, **settings.fixed} for row in vals]


# --- stage 0: rules on the parameters ---

MASS_MARGIN_G = 1.0   # stage 0 rejects on mass only beyond this margin: the estimate is not exact (calibrated)


def bore_problem(p, g):
    """The carrying bores' placement, analytically (R13 walls, R10 either side of and above the centre
    of mass), or ''. Each bore sits on a ramp, ``bore_z`` of the way up the strip; the strip's faces
    there slope at the ramp angle, so the wall is the vertical gap times cos(ramp)."""
    from brixzle.families.prim.family import BORE_D_MM

    r, c = BORE_D_MM / 2, math.cos(math.radians(p.ramp_deg))
    if min(p.bore_z, 1 - p.bore_z) * p.H0 * c - r < 1.2:
        return "bore wall"
    x_right = g["vu"] + p.bore_x * g["run"]
    if not 0 < x_right < p.U:                       # one bore each side of the middle
        return "bores straddle"
    com_z = p.H0 / 2 + g["h"] / 2                   # the strip's mid-height, over the profile's mean height
    if p.bore_x * g["h"] + p.bore_z * p.H0 <= com_z:
        return "bores above the centre of mass"
    return ""


def mass_estimate(p, g):
    """Grams, without geometry: the strip (constant vertical thickness H0 over 2U), plus the teeth, minus
    the two bores and the part of each slot inside the body (the slot polygon's area, half of it)."""
    from brixzle.families import builders as B
    from brixzle.families.prim.family import BORE_D_MM, SLOT_EXTRA_MM

    area = 2 * p.U * p.H0 - 2 * math.pi * (BORE_D_MM / 2) ** 2
    if p.hook_L > 0:
        grow = (p.side_gap - p.seat_gap) / max(p.hook_L, 1e-9)
        st = [(-6.0, -(p.side_gap + grow * 6.0)), (p.hook_L, -p.seat_gap), (p.hook_L + SLOT_EXTRA_MM, -p.seat_gap),
              (p.hook_L + SLOT_EXTRA_MM, p.hook_w + p.hook_gap), (-6.0, p.hook_w + p.hook_gap)]
        slot = 0.5 * abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(st, st[1:] + st[:1])))
        area += 2 * (p.hook_L * p.hook_w - 0.5 * slot)
    return area * p.D * B.PLA_DENSITY


def stage_rules(params, settings=SearchSettings()):
    """(ok, reason, metrics) from the parameters alone. Predicted, quasi-static 2D."""
    from brixzle.families import builders as B
    from brixzle.families.prim.family import PrimParams, SLOT_EXTRA_MM, geom

    try:
        p = PrimParams(**params)
    except (ValueError, TypeError) as e:
        return False, f"invalid: {e}"[:120], {}
    g = geom(p)
    length = 2 * p.U
    release = g["h"] + (p.hook_L + SLOT_EXTRA_MM) * g["cos"] + 1.0     # estimate: the stroke over the ramp
    mass = mass_estimate(p, g)
    m = {"length_mm": length, "release_est_mm": release, "mass_est_g": mass, "capture_mm": min(g["run"], g["left_run"])}
    if length > settings.max_length_mm:
        return False, "length", m
    if release > settings.max_release_mm:
        return False, "release", m
    why = bore_problem(p, g)
    if why:
        return False, why, m
    if not settings.mass_min_g <= mass <= settings.mass_max_g + MASS_MARGIN_G or mass + settings.gripper_g > settings.payload_g + MASS_MARGIN_G:
        return False, "mass", m
    return True, "", m


# --- stage 1: exact geometry and the predictor ---


def _params(params):
    from brixzle.families.prim.family import PrimParams
    return PrimParams(**params)


def stage_predict(params, settings=SearchSettings()):
    """(ok, reason, metrics): exact release lift and rule margins, then the predictor at the aim and
    at +-r on both courses (all must seat at every friction 0.2-0.5)."""
    from brixzle.engine.predictor import predict as PR
    from brixzle.families import builders as B
    from brixzle.families import rules as R

    fam, p = family(), _params(params)
    try:
        lift = fam.release_lift_mm(p)
        bodies = B.sim_bodies(fam, p, "A")
        margins = R.margins(fam, p, "A", bodies)
    except Exception as e:  # noqa: BLE001 - geometry that cannot be built
        return False, f"geometry: {e}"[:120], {}
    m = {"release_mm": lift, "mass_g": bodies.mass_g}
    if lift > settings.max_release_mm:
        return False, "release", m
    broken = sorted(k for k, v in margins.items() if v is not None and v < 0)
    if broken:
        return False, f"rules: {broken}", m
    aim = 3.0  # trials.ErrorModel aim_bias
    for course in (0, 1):
        for dx in (0.0, settings.predict_r_mm, -settings.predict_r_mm):
            r = PR.predict_pose(fam, p, {"dx": aim + dx, "clear": 5.0}, course=course, trace=False)
            if r.get("outcome") != "seated":
                return False, f"predictor: course {course} dx {dx:+.0f} {r.get('outcome')}", m
    return True, "", m


# --- stage 2: reduced drops ---


def key_poses(radius_mm=14.8, clear=(5.0, 11.0)):
    """The reduced drop set: per course, the aim, X at +-r/2 and +-r, Y at +-r/2, a high release and
    a 0.4 deg pitch."""
    aim = 3.0
    base = {"dx": aim, "dy": 0.0, "aim": aim, "yaw": 0.0, "roll": 0.0, "pitch": 0.0, "clear": clear[0], "vel": [0.0, 0.0, 0.0]}
    out = []
    for course in (0, 1):
        for dx, dy in ((0, 0), (radius_mm / 2, 0), (-radius_mm / 2, 0), (radius_mm, 0), (-radius_mm, 0),
                       (0, radius_mm / 2), (0, -radius_mm / 2), (radius_mm / 3, radius_mm / 3)):
            out.append((course, {**base, "dx": aim + dx, "dy": float(dy)}))
        out.append((course, {**base, "clear": clear[1]}))
        out.append((course, {**base, "pitch": 0.4}))
    return out


def stage_drops(params, settings=SearchSettings(), workers=1):
    from brixzle.engine import box_search as BS
    from brixzle.engine.sim import trials as T

    fam, p = family(), _params(params)
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    jobs = [(c, s, {"trace": False}) for c, s in key_poses()]
    with BS.DropPool(fam, p, cfg, workers=workers) as pool:
        outs = [r["outcome"] for r in pool.imap(jobs)]
    n = len(outs)
    s, j = outs.count("seated"), outs.count("jammed")
    score = (s - j) / n
    return score >= settings.drops_min, ("" if score >= settings.drops_min else f"drops {score:+.2f}"), \
        {"drops_score": score, "S": s, "J": j, "N": n, "outcomes": outs}


# --- stage 3: the half-supported step and the grip ---


def stage_build(params, settings=SearchSettings()):
    from brixzle.engine import objective as OB
    from brixzle.engine import structure_gate as SG

    fam, p = family(), _params(params)
    step = SG.step_check(fam, p)
    m = {"half_supported": {k: v["status"] for k, v in step["steps"].items()}}
    if not step["passed"]:
        return False, "half-supported step", m
    grip = OB.grip_rule(fam, p)
    m["grip_rule"] = grip["passed"]
    return grip["passed"], ("" if grip["passed"] else "grip"), m


STAGES = (("rules", stage_rules), ("predict", stage_predict), ("drops", stage_drops), ("build", stage_build))


def _run_one(args):
    name, params, settings = args
    fn = dict(STAGES)[name]
    t = time.time()
    ok, why, m = fn(params, settings)
    return ok, why, m, time.time() - t


def run_stage(name, cands, settings, parallel=True):
    """Run one stage on ``cands`` ([{'id', 'params', ...}]); returns (passed, report)."""
    t0 = time.time()
    if parallel and name != "rules" and len(cands) > 1:
        with ProcessPoolExecutor(min(settings.workers, len(cands)), mp_context=multiprocessing.get_context("spawn")) as ex:
            res = list(ex.map(_run_one, [(name, c["params"], settings) for c in cands]))
    else:
        res = [_run_one((name, c["params"], settings)) for c in cands]
    wall = time.time() - t0
    passed, reasons = [], {}
    for c, (ok, why, m, sec) in zip(cands, res):
        c.setdefault("stages", {})[name] = {"ok": ok, "reason": why, "seconds": round(sec, 4),
                                            **{k: v for k, v in m.items() if k != "outcomes"}}
        if ok:
            passed.append(c)
        else:
            key = why.split(":")[0]
            reasons[key] = reasons.get(key, 0) + 1
    rep = {"stage": name, "in": len(cands), "passed": len(passed), "wall_s": round(wall, 3),
           "evals_per_s": round(len(cands) / wall, 3) if wall > 0 else None,
           "cpu_s_per_eval": round(sum(r[3] for r in res) / max(1, len(res)), 4), "rejected_by": reasons,
           "label": LABEL_PRED if name in ("rules", "predict") else LABEL_SIM}
    return passed, rep


def write_package(root, cid, params):
    """A candidate as a package ``<root>/<cid>/family.py`` (prim with these defaults), for the tools."""
    d = Path(root) / cid
    d.mkdir(parents=True, exist_ok=False)
    (d / "__init__.py").write_text("")
    fields_txt = "\n".join(f"    {k}: float = {float(v)!r}" for k, v in sorted(params.items()))
    (d / "family.py").write_text(f'''"""Search candidate {cid} (#166): prim at these defaults."""
from dataclasses import dataclass, replace

from brixzle.families.prim.family import FAMILY as _PRIM, PrimParams, cell


@dataclass(frozen=True)
class {cid.title().replace("_", "")}Params(PrimParams):
{fields_txt}


def _family():
    from brixzle.engine import ring_cells as RC
    fam = replace(_PRIM, name="{cid}", version="0.1.0", params_type={cid.title().replace("_", "")}Params, ring=None)
    return replace(fam, ring=RC.kit_from_cell(fam, cell))


FAMILY = _family()
''')
    return f"{root}:{cid}"


def run_search(settings=SearchSettings(), runs="runs", full_eval=True):
    from brixzle.tools import core as TC

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = Path(runs) / f"search-{stamp}-prim"
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.time()
    cands = [{"id": f"c{k:05d}", "params": prm} for k, prm in enumerate(sample(settings))]
    t_sample = time.time() - t0
    reports, alive = [], cands
    for name, _ in STAGES:
        alive, rep = run_stage(name, alive, settings)
        reports.append(rep)
        print(json.dumps(rep), flush=True)
        if name == "drops":
            alive = sorted(alive, key=lambda c: -c["stages"]["drops"]["drops_score"])[: settings.keep_after_drops]
        if not alive:
            break
    full = []
    if full_eval and alive:
        TC.cache_on()
        for c in alive[: settings.full]:
            spec = write_package(out / "candidates", c["id"], c["params"])
            t = time.time()
            ev = TC.evaluate(spec, out / "eval" / c["id"], stages=("check", "quick", "map", "held_out", "gate", "ring"))
            full.append({"id": c["id"], "spec": spec, "z": {k: v["Z"] for k, v in (ev.get("z") or {}).items()},
                         "gate": {k: (ev.get("gate") or {}).get(k) for k in ("passed", "overhang_n")},
                         "ring": (ev.get("ring") or {}).get("status"), "seconds": round(time.time() - t, 1)})
            print(json.dumps(full[-1]), flush=True)
        if full:
            reports.append({"stage": "full", "in": len(full), "wall_s": round(sum(f["seconds"] for f in full), 1),
                            "evals_per_s": round(len(full) / sum(f["seconds"] for f in full), 5), "label": LABEL_SIM})
    result = {"settings": asdict(settings), "sample_s": round(t_sample, 3), "stages": reports, "full": full,
              "survivors": [{"id": c["id"], "params": c["params"], "stages": c["stages"]} for c in alive],
              "seconds": round(time.time() - t0, 1), "run": str(out)}
    (out / "search.json").write_text(json.dumps(TC._finite(result), indent=1, default=str) + "\n")
    (out / "candidates.json").write_text(json.dumps(TC._finite(cands), default=str) + "\n")
    return result


def main(argv=None):
    import argparse

    ap = argparse.ArgumentParser(prog="python -m brixzle.search")
    ap.add_argument("--n", type=int, default=SearchSettings.n)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--full", type=int, default=SearchSettings.full)
    ap.add_argument("--workers", type=int, default=SearchSettings.workers)
    ap.add_argument("--runs", default="runs")
    a = ap.parse_args(argv)
    s = SearchSettings(n=a.n, seed=a.seed, full=a.full, workers=a.workers)
    res = run_search(s, runs=a.runs, full_eval=a.full > 0)
    print(json.dumps({"stages": res["stages"], "full": res["full"], "seconds": res["seconds"], "run": res["run"]}, indent=1))
    return 0
