"""Scorer and diagnoser (issue #13): one family's scores, failures and predictor cross-check, as report.json.

For each target box (the drone's mean, p95 and worst error; engine/boxes.py) and each task,
placement (R11) and pickup (R12):

- **Score:** the sim box scale (engine/box_search.py); a target passes at scale >= 1. Placement is the
  smaller over the two courses. Pickup also needs the clean-release check, which does not depend
  on the box, so it runs once per design; a failed release scores pickup 0 at every target (no
  box in which every trial succeeds). The overall score of a target is the smaller of the two.
- **Failures:** every failing sim drop, counted by category: jam, fall and wrong_valley
  (placement); jam, pickup_miss, off_centre and forceful (pickup); and release. Counts are of the
  drops the bisection ran, not rates. The limiting failures, the failing drops at the smallest
  failing scale at or above the score, are listed with their contact traces: they are the ones
  that cap the box, and the ones the critic (#19) should read.
- **Cross-check:** the predictor (engine/predictor/) is run on exactly the poses the sim dropped.
  A pose where only the predictor succeeds is 'optimistic' (the dangerous case: the tuner would
  overvalue the design); one where only the sim succeeds is 'conservative'. Both are listed, with
  each target's predicted scale minus the sim's. When both fail, their classes are compared but
  not trusted: the predictor confuses jammed and wrong valley and has no falls
  (docs/predictor-validation.md).

report.json holds the summary; the full sim and predictor results (megabytes of contact traces)
are written next to it and named under 'files'. Every run writes a new directory under runs/.

Sim results are simulated, uncalibrated contact; predictor results are predicted, quasi-static 2D.

    uv run python -m brixzle.engine.report --family ab --seed 0
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from brixzle.engine import box_search as BS
from brixzle.engine import boxes as BX
from brixzle.engine.predictor import pickup as PP
from brixzle.engine.predictor import predict as PR
from brixzle.engine.sim import pickup as PK
from brixzle.engine.sim import trials as T
from brixzle.engine.validate import outcome_class

SCHEMA_VERSION = 1
LABELS = {"sim": BS.LABEL, "pred": PR.LABEL}
BUDGET_S = 30 * 60  # a family should take no more than 30 min (CLAUDE.md, Speed)
CATEGORIES = {
    "placement": {"jammed": "jam", "fell": "fall", "wrong_valley": "wrong_valley"},
    "pickup": {"jammed": "jam", "missed": "pickup_miss", "off_centre": "off_centre", "forceful": "forceful"},
}
ALL_CATEGORIES = frozenset(c for m in CATEGORIES.values() for c in m.values()) | {"release"}
KINDS = ("agree_ok", "agree_failed", "optimistic", "conservative")
DISAGREE = ("optimistic", "conservative")
FLAGS = (
    "failure counts are of the drops the bisection ran, not rates",
    "the predictor's failure class is not trusted; take classes and traces from the sim "
    "(docs/predictor-validation.md)",
    *(f"pickup predictor: {f}" for f in PP.FLAGS),
)


def category(task, outcome):
    """The failure category of a sim outcome; raises on a success or an unknown outcome."""
    try:
        return CATEGORIES[task][outcome]
    except KeyError:
        raise ValueError(f"no failure category for {task!r} outcome {outcome!r}") from None


def limiting_failures(course_result, success):
    """The failing drops at the smallest failing scale at or above the course's score (with traces)."""
    failing = [d for d in course_result["drops"]
               if d["outcome"] != success and d["scale"] >= course_result["scale"]]
    if not failing:
        return []
    s = min(d["scale"] for d in failing)
    return [d for d in failing if d["scale"] == s]


def agreement(task, sim_outcome, pred_outcome):
    """'agree_ok', 'agree_failed', 'optimistic' (only the predictor succeeds) or 'conservative'."""
    success = BS.SUCCESS[task]
    sim_ok, pred_ok = sim_outcome == success, pred_outcome == success
    if sim_ok == pred_ok:
        return "agree_ok" if sim_ok else "agree_failed"
    return "optimistic" if pred_ok else "conservative"


# --- cross-check: the predictor on the sim's own poses ---


def _pose_key(course, sample):
    return json.dumps([course, sample], sort_keys=True)


def cross_check(family, p, box, sim_result, cfg=None, pool=None, geo=None):
    """One row per sim drop: the sim's and the predictor's outcome at the same pose, and their kind.

    Each distinct pose is predicted once. Placement poses use ``cfg`` (a TrialConfig, as the sim
    search) for the release height; ``pool`` (a predict.PosePool) runs them in parallel."""
    task = box.task
    if task == "placement":
        cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    else:
        geo = geo or PP.geometry(family, p)
    jobs = {}
    for course, res in sim_result["courses"].items():
        for d in res["drops"]:
            sample = BS.pose_sample(box, d["scale"], d["unit"], cfg)
            jobs.setdefault(_pose_key(course, sample), (course, sample))
    keys = list(jobs)
    if task == "pickup":
        preds = [PP.predict_pick(family, p, s, geo=geo)["outcome"] for _, s in jobs.values()]
    elif pool is not None:
        scenes = {c: PR.scene(family, p, int(c)) for c in {c for c, _ in jobs.values()}}
        preds = [r["outcome"] for r in pool.map(PR._pose_job, [
            (family, p, int(c), s, PR.MUS, False, scenes[c]) for c, s in jobs.values()])]
    else:
        preds = [PR.predict_pose(family, p, s, int(c), early_stop=True, trace=False)["outcome"]
                 for c, s in jobs.values()]
    pred_of = dict(zip(keys, preds))
    rows = []
    for course, res in sim_result["courses"].items():
        for d in res["drops"]:
            pred = pred_of[_pose_key(course, BS.pose_sample(box, d["scale"], d["unit"], cfg))]
            rows.append({"task": task, "statistic": box.statistic, "course": course, "scale": d["scale"],
                         "point": d["point"], "unit": list(d["unit"]), "sim_outcome": d["outcome"],
                         "pred_outcome": pred, "kind": agreement(task, d["outcome"], pred)})
    return rows


# --- the report ---


def _failure(task, stat, course, d):
    row = {k: v for k, v in d.items() if k != "frames"}
    return {"task": task, "statistic": stat, "course": course, "category": category(task, d["outcome"]), **row}


def build_report(family, p, sims, preds, release, checks, timing, files, seed):
    """report.json as a dict, from the sim and predicted results ({(task, statistic): result}), the
    pickup release check (or None if pickup was not scored), the cross-check rows, the stage
    timings (seconds; 'total' required) and the full-result files (name -> relative path)."""
    stats = [s for s in BX.STATISTICS if any(k[1] == s for k in sims)]
    targets, score, counts, fails, dcounts, mismatches = {}, {}, {}, [], {}, {}
    config = {}
    for stat in stats:
        targets[stat], counts[stat], dcounts[stat], mismatches[stat] = {}, {}, {}, {}
        for task in BX.TASKS:
            sim, pred = sims[(task, stat)], preds[(task, stat)]
            success = BS.SUCCESS[task]
            config.setdefault(task, sim.get("config", {}))
            passed = sim["scale"] >= 1.0
            t = {"box": sim["box"],
                 "sim": {"scale": sim["scale"], "passed": passed, "limiting_course": sim["limiting_course"]},
                 "pred": {"scale": pred["scale"], "passed": pred["scale"] >= 1.0},
                 "scale_gap": pred["scale"] - sim["scale"]}
            c = {}
            for course, res in sim["courses"].items():
                for d in res["drops"]:
                    if d["outcome"] != success:
                        k = category(task, d["outcome"])
                        c[k] = c.get(k, 0) + 1
                fails += [_failure(task, stat, course, d) for d in limiting_failures(res, success)]
            if task == "pickup":
                if release is None:
                    raise ValueError("pickup was scored without its release check")
                t["release"] = {"passed": bool(release["passed"]), "drift_mm": release["drift_mm"]}
                t["sim"]["passed"] = passed and bool(release["passed"])
                if not release["passed"]:
                    c["release"] = 1
                    fails.append({"task": task, "statistic": stat, "course": "release", "category": "release",
                                  "outcome": "release", **release})
            targets[stat][task], counts[stat][task] = t, c
            rows = [r for r in checks if r["task"] == task and r["statistic"] == stat]
            dcounts[stat][task] = {k: sum(r["kind"] == k for r in rows) for k in KINDS}
            mismatches[stat][task] = sum(r["kind"] == "agree_failed" and outcome_class(r["pred_outcome"])
                                         != r["sim_outcome"] for r in rows)
        place = targets[stat]["placement"]["sim"]
        pick = targets[stat]["pickup"]["sim"]
        pick_score = pick["scale"] if release["passed"] else 0.0
        score[stat] = {"placement": place["scale"], "pickup": pick_score,
                       "overall": min(place["scale"], pick_score)}
    timing = {**timing, "over_budget": bool(timing["total"] > BUDGET_S)}
    return {
        "schema_version": SCHEMA_VERSION, "labels": dict(LABELS),
        "family": family.name, "family_version": family.version, "params": p.to_dict(), "seed": seed,
        "config": config, "statistics": stats,
        "score": score,
        "covers_all_targets": all(targets[s][t]["sim"]["passed"] for s in stats for t in BX.TASKS),
        "targets": targets,
        "failure_counts": counts, "failures": fails,
        "disagreements": [r for r in checks if r["kind"] in DISAGREE],
        "disagreement_counts": dcounts, "class_mismatches": mismatches, "class_trusted": False,
        "timing": timing, "flags": list(FLAGS), "files": dict(files),
    }


_REQUIRED = ("schema_version", "labels", "family", "family_version", "params", "seed", "config", "statistics",
             "score", "covers_all_targets", "targets", "failure_counts", "failures", "disagreements",
             "disagreement_counts", "class_mismatches", "class_trusted", "timing", "flags", "files")


def _finite(v, where="report"):
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return
    if isinstance(v, (int, float)):
        if not math.isfinite(v):
            raise ValueError(f"non-finite number at {where}")
    elif isinstance(v, dict):
        for k, x in v.items():
            _finite(x, f"{where}.{k}")
    elif isinstance(v, (list, tuple)):
        for i, x in enumerate(v):
            _finite(x, f"{where}[{i}]")
    else:
        raise ValueError(f"not JSON data at {where}: {type(v).__name__}")


def _need(cond, msg):
    if not cond:
        raise ValueError(msg)


def validate_report(d):
    """Raise ValueError unless ``d`` is a well-formed report.json (schema SCHEMA_VERSION)."""
    _need(isinstance(d, dict), "report must be a dict")
    missing = [k for k in _REQUIRED if k not in d]
    _need(not missing, f"report is missing {missing}")
    _need(d["schema_version"] == SCHEMA_VERSION, f"schema_version must be {SCHEMA_VERSION}")
    _need(d["labels"] == LABELS, f"labels must be {LABELS}")
    _need(d["class_trusted"] is False and isinstance(d["covers_all_targets"], bool), "bad flags")
    _need(d["statistics"] and set(d["statistics"]) <= set(BX.STATISTICS), "bad statistics")
    for stat in d["statistics"]:
        _need(set(d["targets"].get(stat, {})) == set(BX.TASKS), f"targets[{stat}] needs {BX.TASKS}")
        _need(set(d["score"].get(stat, {})) == {*BX.TASKS, "overall"}, f"score[{stat}] is incomplete")
        for task in BX.TASKS:
            t = d["targets"][stat][task]
            for side in ("sim", "pred"):
                s = t[side]["scale"]
                _need(isinstance(s, (int, float)) and 0 <= s, f"targets[{stat}][{task}][{side}] scale")
                _need(isinstance(t[side]["passed"], bool), f"targets[{stat}][{task}][{side}] passed")
            _need(task != "pickup" or "release" in t, f"targets[{stat}][pickup] needs its release check")
            _need(set(d["failure_counts"][stat][task]) <= ALL_CATEGORIES, f"failure_counts[{stat}][{task}]")
            _need(set(d["disagreement_counts"][stat][task]) == set(KINDS), f"disagreement_counts[{stat}][{task}]")
    for f in d["failures"]:
        _need(f.get("category") in ALL_CATEGORIES, f"unknown failure category {f.get('category')!r}")
        _need(f.get("task") in BX.TASKS and f.get("statistic") in d["statistics"], "bad failure row")
    for r in d["disagreements"]:
        _need(r.get("kind") in DISAGREE, f"a disagreement must be {DISAGREE}, got {r.get('kind')!r}")
    _need(isinstance(d["timing"].get("total"), (int, float)) and isinstance(d["timing"].get("over_budget"), bool),
          "timing needs total and over_budget")
    _finite(d)


def _write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def new_run_dir(root, family_name):
    """A new runs/report-<UTC stamp>-<family> directory; never reuses one."""
    root = Path(root)
    while True:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        path = root / f"report-{stamp}-{family_name}"
        try:
            path.mkdir(parents=True, exist_ok=False)
            return path
        except FileExistsError:
            continue


def score_family(family, p, statistics=BX.STATISTICS, seed=0, tol=0.05, n_interior=4, workers=None,
                 runs="runs", boxes=None):
    """Score one family against every target (predictor, sim, cross-check); writes a new run
    directory with report.json and the full results, and returns its path."""
    bad = [s for s in statistics if s not in BX.STATISTICS]
    _need(statistics and not bad, f"statistics must be from {BX.STATISTICS}, got {list(statistics)}")
    boxes = boxes or BX.default_boxes()
    out = new_run_dir(runs, family.name)
    t_all = time.time()
    timing, files, sims, preds, checks = {}, {}, {}, {}, []
    place_cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    pick_cfg = PK.PickConfig()
    n_workers = workers or BS.DEFAULT_WORKERS
    pool = PR.PosePool(n_workers) if n_workers > 1 else None
    try:
        t = time.time()
        for stat in statistics:
            preds[("placement", stat)] = PR.predicted_box(family, p, boxes["placement"][stat], seed=seed, tol=tol,
                                                          n_interior=n_interior, cfg=place_cfg, trace=False,
                                                          pool=pool)
            preds[("pickup", stat)] = PP.predicted_pickup_box(family, p, boxes["pickup"][stat], seed=seed, tol=tol,
                                                              n_interior=n_interior)
        timing["predict"] = time.time() - t
        t = time.time()
        release = PK.release_check(family, p, pick_cfg)
        timing["sim_release"] = time.time() - t
        _write_json(out / "release.json", release)
        files["release"] = "release.json"
        for stat in statistics:
            for task, cfg in (("placement", place_cfg), ("pickup", pick_cfg)):
                t = time.time()
                sims[(task, stat)] = BS.box_search(family, p, boxes[task][stat], cfg=cfg, seed=seed, tol=tol,
                                                   n_interior=n_interior, workers=workers, release=False)
                timing[f"sim_{task}_{stat}"] = time.time() - t
        t = time.time()
        geo = PP.geometry(family, p)
        for (task, stat), sim in sims.items():
            checks += cross_check(family, p, boxes[task][stat], sim, cfg=place_cfg, pool=pool, geo=geo)
        timing["cross_check"] = time.time() - t
    finally:
        if pool is not None:
            pool.close()
    for (task, stat), res in sims.items():
        files[f"sim_{task}_{stat}"] = f"sim-{task}-{stat}.json"
        _write_json(out / files[f"sim_{task}_{stat}"], res)
    for (task, stat), res in preds.items():
        files[f"pred_{task}_{stat}"] = f"pred-{task}-{stat}.json"
        _write_json(out / files[f"pred_{task}_{stat}"], res)
    files["cross_check"] = "cross-check.json"
    _write_json(out / "cross-check.json", checks)
    timing["total"] = time.time() - t_all
    rep = build_report(family, p, sims, preds, release, checks, timing, files, seed)
    rep.update(tol=tol, n_interior=n_interior, workers=n_workers)
    validate_report(rep)
    _write_json(out / "report.json", rep)
    return out


def summary(rep):
    """A short text table of a report, for the terminal."""
    lines = [f"{rep['family']} ({LABELS['sim']}; predictor {LABELS['pred']}), seed {rep['seed']}",
             f"{'target':<6} {'placement':>9} {'pickup':>7} {'overall':>7}  pred place/pick  failures",
             ]
    for stat in rep["statistics"]:
        s, t = rep["score"][stat], rep["targets"][stat]
        fc = "; ".join(f"{task}: " + ", ".join(f"{k} {n}" for k, n in sorted(rep["failure_counts"][stat][task].items()))
                       for task in BX.TASKS if rep["failure_counts"][stat][task])
        lines.append(f"{stat:<6} {s['placement']:>9.3f} {s['pickup']:>7.3f} {s['overall']:>7.3f}  "
                     f"{t['placement']['pred']['scale']:>5.3f}/{t['pickup']['pred']['scale']:<5.3f}    {fc or 'none'}")
    n = {k: sum(c[t][k] for c in rep["disagreement_counts"].values() for t in BX.TASKS) for k in DISAGREE}
    lines.append(f"covers all targets: {rep['covers_all_targets']}; disagreements: "
                 f"{n['optimistic']} optimistic, {n['conservative']} conservative")
    tm = rep["timing"]
    lines.append("wall-clock: " + ", ".join(f"{k} {v:.1f} s" for k, v in tm.items() if k != "over_budget")
                 + (" — OVER the 30 min budget" if tm["over_budget"] else ""))
    return "\n".join(lines)


def main(argv=None):
    from brixzle.families import registry

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--family", default="ab")
    ap.add_argument("--statistics", nargs="+", choices=BX.STATISTICS, default=list(BX.STATISTICS))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--n-interior", type=int, default=4)
    ap.add_argument("--workers", type=int, default=None, help=f"parallel processes (default {BS.DEFAULT_WORKERS})")
    ap.add_argument("--runs", default="runs")
    args = ap.parse_args(argv)
    family = registry.get(args.family)
    out = score_family(family, family.defaults, statistics=tuple(args.statistics), seed=args.seed, tol=args.tol,
                       n_interior=args.n_interior, workers=args.workers, runs=args.runs)
    print(summary(json.loads((out / "report.json").read_text(encoding="utf-8"))))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
