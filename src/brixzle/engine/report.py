"""Scorer and diagnoser (issue #13): one family's scores, failures and predictor cross-check, as report.json.

The loop scores placement (R11) only for now (issue #46): ``tasks`` defaults to ('placement',), and
then the pickup predictor, sim, release check and cross-check are not run at all. Pickup (R12) comes
back once the gripper is designed for the chosen brick (#24); ``--tasks placement pickup`` gives the
full report.

For each target box (the drone's mean, p95 and worst error; engine/boxes.py) and each scored task:

- **Score:** the sim box scale (engine/box_search.py); a target passes at scale >= 1. Placement is the
  smaller over the two courses. Pickup also needs the clean-release check, which does not depend
  on the box, so it runs once per design; a failed release scores pickup 0 at every target (no
  box in which every trial succeeds). The overall score of a target is the smallest over the
  scored tasks, so with placement only it is the placement score.
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

A ``Solid3D`` family is scored by the sim only (issue #51; the decision on #18): the predictor works on
a 2D section, so it and the cross-check are skipped. The report then says so under 'predictor', and
holds no predicted scale, scale gap or disagreement.

report.json holds the summary; the full sim and predictor results (megabytes of contact traces)
are written next to it and named under 'files'. Every run writes a new directory under runs/.

Sim results are simulated, uncalibrated contact; predictor results are predicted, quasi-static 2D.

    uv run python -m brixzle.engine.report --family ab --seed 0
    uv run python -m brixzle.engine.report --family ab --seed 0 --tasks placement pickup
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
from brixzle.families.interface import Solid3D

# 2: 'tasks' names the scored tasks; targets, scores and counts hold only those (#46).
# 3: 'predictor' says whether the predictor was used; a sim-only report holds no predictor results (#51).
SCHEMA_VERSION = 3
DEFAULT_TASKS = ("placement",)  # pickup (R12) is deferred until the gripper is designed (#46, #24)
LABELS = {"sim": BS.LABEL, "pred": PR.LABEL}
BUDGET_S = 30 * 60  # a family should take no more than 30 min (CLAUDE.md, Speed)
CATEGORIES = {
    "placement": {"jammed": "jam", "fell": "fall", "wrong_valley": "wrong_valley"},
    "pickup": {"jammed": "jam", "missed": "pickup_miss", "off_centre": "off_centre", "forceful": "forceful"},
}
ALL_CATEGORIES = frozenset(c for m in CATEGORIES.values() for c in m.values()) | {"release"}
KINDS = ("agree_ok", "agree_failed", "optimistic", "conservative")
DISAGREE = ("optimistic", "conservative")
COUNT_FLAG = "failure counts are of the drops the bisection ran, not rates"
CLASS_FLAG = ("the predictor's failure class is not trusted; take classes and traces from the sim "
              "(docs/predictor-validation.md)")
FLAGS = (COUNT_FLAG, CLASS_FLAG)
SOLID3D_REASON = ("Solid3D family: the analytic predictor works on a 2D section, so the family is scored by "
                  "the sim only (issue #51)")
PICKUP_FLAGS = tuple(f"pickup predictor: {f}" for f in PP.FLAGS)
NOT_SCORED = "pickup (R12) was not scored: it is deferred until the gripper is designed (issue #46)"


def scored_tasks(tasks):
    """``tasks`` checked and put in boxes.TASKS order; raises ValueError unless it is a non-empty
    collection of distinct task names."""
    if isinstance(tasks, str) or not tasks:
        raise ValueError(f"tasks must be a non-empty collection from {BX.TASKS}, got {tasks!r}")
    tasks = list(tasks)
    if len(set(tasks)) != len(tasks) or any(t not in BX.TASKS for t in tasks):
        raise ValueError(f"tasks must be distinct names from {BX.TASKS}, got {tasks}")
    return tuple(t for t in BX.TASKS if t in tasks)


def predictor_skip_reason(family):
    """Why the analytic predictor is not run for this family, or None if it is run."""
    return SOLID3D_REASON if isinstance(family.geometry, Solid3D) else None


def flags(tasks, predictor_reason=None):
    """The report's caveats for the tasks it scored. ``predictor_reason`` says why the predictor
    was skipped (a sim-only report), and replaces the caveats about its results."""
    if predictor_reason:
        return [COUNT_FLAG, f"the analytic predictor was skipped: {predictor_reason}",
                *(() if "pickup" in tasks else (NOT_SCORED,))]
    return [*FLAGS, *(PICKUP_FLAGS if "pickup" in tasks else (NOT_SCORED,))]


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


def build_report(family, p, sims, preds, release, checks, timing, files, seed, tasks=None, predictor_reason=None):
    """report.json as a dict, from the sim and predicted results ({(task, statistic): result}), the
    pickup release check (or None if pickup was not scored), the cross-check rows, the stage
    timings (seconds; 'total' required) and the full-result files (name -> relative path).

    ``tasks`` names the tasks to report (default: those the sim results hold). Only they appear
    under targets, score and the counts, and a target's overall score is the smallest of theirs.

    ``preds=None`` gives a sim-only report (issue #51), which needs ``predictor_reason``: targets
    then hold no 'pred' or 'scale_gap', and there are no disagreements or class mismatches."""
    used = preds is not None
    _need(used or predictor_reason, "a report without predictor results must say why (predictor_reason)")
    _need(not (used and predictor_reason), "predictor_reason is for a report without predictor results")
    tasks = scored_tasks(tasks if tasks is not None else {k[0] for k in sims})
    stats = [s for s in BX.STATISTICS if any(k[1] == s for k in sims)]
    targets, score, counts, fails, dcounts, mismatches = {}, {}, {}, [], {}, {}
    config = {}
    for stat in stats:
        targets[stat], counts[stat] = {}, {}
        if used:
            dcounts[stat], mismatches[stat] = {}, {}
        for task in tasks:
            sim = sims[(task, stat)]
            success = BS.SUCCESS[task]
            config.setdefault(task, sim.get("config", {}))
            passed = sim["scale"] >= 1.0
            t = {"box": sim["box"],
                 "sim": {"scale": sim["scale"], "passed": passed, "limiting_course": sim["limiting_course"]}}
            if used:
                pred = preds[(task, stat)]
                t["pred"] = {"scale": pred["scale"], "passed": pred["scale"] >= 1.0}
                t["scale_gap"] = pred["scale"] - sim["scale"]
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
            if not used:
                continue
            rows = [r for r in checks if r["task"] == task and r["statistic"] == stat]
            dcounts[stat][task] = {k: sum(r["kind"] == k for r in rows) for k in KINDS}
            mismatches[stat][task] = sum(r["kind"] == "agree_failed" and outcome_class(r["pred_outcome"])
                                         != r["sim_outcome"] for r in rows)
        per = {task: targets[stat][task]["sim"]["scale"] for task in tasks}
        if "pickup" in tasks and not release["passed"]:
            per["pickup"] = 0.0
        score[stat] = {**per, "overall": min(per.values())}
    timing = {**timing, "over_budget": bool(timing["total"] > BUDGET_S)}
    return {
        "schema_version": SCHEMA_VERSION, "labels": dict(LABELS) if used else {"sim": LABELS["sim"]},
        "predictor": {"used": used, "reason": predictor_reason},
        "family": family.name, "family_version": family.version, "params": p.to_dict(), "seed": seed,
        "config": config, "statistics": stats, "tasks": list(tasks),
        "score": score,
        "covers_all_targets": all(targets[s][t]["sim"]["passed"] for s in stats for t in tasks),
        "targets": targets,
        "failure_counts": counts, "failures": fails,
        "disagreements": [r for r in checks if r["kind"] in DISAGREE] if used else [],
        "disagreement_counts": dcounts, "class_mismatches": mismatches, "class_trusted": False,
        "timing": timing, "flags": flags(tasks, predictor_reason), "files": dict(files),
    }


_REQUIRED = ("schema_version", "labels", "predictor", "family", "family_version", "params", "seed", "config", "statistics",
             "tasks", "score", "covers_all_targets", "targets", "failure_counts", "failures", "disagreements",
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
    pred = d["predictor"]
    _need(isinstance(pred, dict) and isinstance(pred.get("used"), bool), "predictor needs a boolean 'used'")
    used = pred["used"]
    _need(used or (isinstance(pred.get("reason"), str) and pred["reason"]), "a sim-only report must say why")
    labels = LABELS if used else {"sim": LABELS["sim"]}
    _need(d["labels"] == labels, f"labels must be {labels}")
    _need(used or (d["disagreements"] == [] and d["disagreement_counts"] == {} and d["class_mismatches"] == {}),
          "a sim-only report holds no disagreements or class mismatches")
    _need(d["class_trusted"] is False and isinstance(d["covers_all_targets"], bool), "bad flags")
    _need(d["statistics"] and set(d["statistics"]) <= set(BX.STATISTICS), "bad statistics")
    _need(isinstance(d["tasks"], list), "tasks must be a list")
    tasks = scored_tasks(d["tasks"])
    _need(list(tasks) == d["tasks"], f"tasks must be in the order of {BX.TASKS}")
    for stat in d["statistics"]:
        _need(set(d["targets"].get(stat, {})) == set(tasks), f"targets[{stat}] needs exactly {tasks}")
        _need(set(d["score"].get(stat, {})) == {*tasks, "overall"}, f"score[{stat}] needs {tasks} and overall")
        _need(d["score"][stat]["overall"] == min(d["score"][stat][t] for t in tasks),
              f"score[{stat}].overall must be the smallest of its task scores")
        _need(set(d["failure_counts"].get(stat, {})) == set(tasks), f"failure_counts[{stat}] needs exactly {tasks}")
        _need(not used or set(d["disagreement_counts"].get(stat, {})) == set(tasks),
              f"disagreement_counts[{stat}] needs exactly {tasks}")
        for task in tasks:
            t = d["targets"][stat][task]
            _need(used or not {"pred", "scale_gap"} & set(t), f"targets[{stat}][{task}]: a sim-only report "
                  "holds no predictor results")
            for side in ("sim", "pred") if used else ("sim",):
                s = t[side]["scale"]
                _need(isinstance(s, (int, float)) and 0 <= s, f"targets[{stat}][{task}][{side}] scale")
                _need(isinstance(t[side]["passed"], bool), f"targets[{stat}][{task}][{side}] passed")
            _need(task != "pickup" or "release" in t, f"targets[{stat}][pickup] needs its release check")
            _need(set(d["failure_counts"][stat][task]) <= ALL_CATEGORIES, f"failure_counts[{stat}][{task}]")
            _need(not used or set(d["disagreement_counts"][stat][task]) == set(KINDS),
                  f"disagreement_counts[{stat}][{task}]")
    for f in d["failures"]:
        _need(f.get("category") in ALL_CATEGORIES, f"unknown failure category {f.get('category')!r}")
        _need(f.get("task") in tasks and f.get("statistic") in d["statistics"], "bad failure row")
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
                 runs="runs", boxes=None, tasks=DEFAULT_TASKS):
    """Score one family against every target (predictor, sim, cross-check); writes a new run
    directory with report.json and the full results, and returns its path.

    ``tasks``: the tasks to score, placement only by default (issue #46). A task that is not
    scored is not run: without 'pickup' there is no pickup predictor, sim, release check or
    cross-check.

    A ``Solid3D`` family is scored by the sim only (issue #51): the predictor and the cross-check
    are not run. If scoring raises, the run directory it made is removed again."""
    bad = [s for s in statistics if s not in BX.STATISTICS]
    _need(statistics and not bad, f"statistics must be from {BX.STATISTICS}, got {list(statistics)}")
    tasks = scored_tasks(tasks)
    out = new_run_dir(runs, family.name)
    try:
        _score_into(out, family, p, statistics, seed, tol, n_interior, workers, boxes or BX.default_boxes(), tasks)
    except BaseException:
        # Results are written only at the end, so a run that raised has left at most partial files.
        for path in out.iterdir():
            path.unlink()
        out.rmdir()
        raise
    return out


def _score_into(out, family, p, statistics, seed, tol, n_interior, workers, boxes, tasks):
    pickup = "pickup" in tasks
    skip = predictor_skip_reason(family)
    t_all = time.time()
    timing, files, sims, preds, checks = {}, {}, {}, {}, []
    place_cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    cfgs = {"placement": place_cfg, "pickup": PK.PickConfig()}
    release = None
    n_workers = workers or BS.DEFAULT_WORKERS
    pool = PR.PosePool(n_workers) if n_workers > 1 and not skip else None
    try:
        if not skip:
            t = time.time()
            for stat in statistics:
                if "placement" in tasks:
                    preds[("placement", stat)] = PR.predicted_box(family, p, boxes["placement"][stat], seed=seed,
                                                                  tol=tol, n_interior=n_interior, cfg=place_cfg,
                                                                  trace=False, pool=pool)
                if pickup:
                    preds[("pickup", stat)] = PP.predicted_pickup_box(family, p, boxes["pickup"][stat], seed=seed,
                                                                      tol=tol, n_interior=n_interior)
            timing["predict"] = time.time() - t
        if pickup:
            t = time.time()
            release = PK.release_check(family, p, cfgs["pickup"])
            timing["sim_release"] = time.time() - t
        for stat in statistics:
            for task in tasks:
                t = time.time()
                sims[(task, stat)] = BS.box_search(family, p, boxes[task][stat], cfg=cfgs[task], seed=seed, tol=tol,
                                                   n_interior=n_interior, workers=workers, release=False)
                timing[f"sim_{task}_{stat}"] = time.time() - t
        if not skip:
            t = time.time()
            geo = PP.geometry(family, p) if pickup else None
            for (task, stat), sim in sims.items():
                checks += cross_check(family, p, boxes[task][stat], sim, cfg=place_cfg, pool=pool, geo=geo)
            timing["cross_check"] = time.time() - t
    finally:
        if pool is not None:
            pool.close()
    if release is not None:
        files["release"] = "release.json"
        _write_json(out / "release.json", release)
    for (task, stat), res in sims.items():
        files[f"sim_{task}_{stat}"] = f"sim-{task}-{stat}.json"
        _write_json(out / files[f"sim_{task}_{stat}"], res)
    for (task, stat), res in preds.items():
        files[f"pred_{task}_{stat}"] = f"pred-{task}-{stat}.json"
        _write_json(out / files[f"pred_{task}_{stat}"], res)
    if not skip:
        files["cross_check"] = "cross-check.json"
        _write_json(out / "cross-check.json", checks)
    timing["total"] = time.time() - t_all
    rep = build_report(family, p, sims, None if skip else preds, release, checks, timing, files, seed, tasks=tasks,
                       predictor_reason=skip)
    rep.update(tol=tol, n_interior=n_interior, workers=n_workers)
    validate_report(rep)
    _write_json(out / "report.json", rep)


def summary(rep):
    """A short text table of a report, for the terminal."""
    tasks = rep["tasks"]
    used = rep["predictor"]["used"]
    models = f"predictor {LABELS['pred']}" if used else "sim only"
    lines = [f"{rep['family']} ({LABELS['sim']}; {models}), seed {rep['seed']}, tasks {' + '.join(tasks)}",
             f"{'target':<6} " + " ".join(f"{t:>9}" for t in (*tasks, "overall"))
             + (f"  pred {'/'.join(tasks)}" if used else "") + "  failures"]
    for stat in rep["statistics"]:
        s, t = rep["score"][stat], rep["targets"][stat]
        fc = "; ".join(f"{task}: " + ", ".join(f"{k} {n}" for k, n in sorted(rep["failure_counts"][stat][task].items()))
                       for task in tasks if rep["failure_counts"][stat][task])
        pred = "/".join(f"{t[task]['pred']['scale']:.3f}" for task in tasks) + "  " if used else ""
        lines.append(f"{stat:<6} " + " ".join(f"{s[k]:>9.3f}" for k in (*tasks, "overall")) + "  "
                     + pred + f"  {fc or 'none'}")
    if used:
        n = {k: sum(c[t][k] for c in rep["disagreement_counts"].values() for t in tasks) for k in DISAGREE}
        lines.append(f"covers all targets: {rep['covers_all_targets']}; disagreements: "
                     f"{n['optimistic']} optimistic, {n['conservative']} conservative")
    else:
        lines.append(f"covers all targets: {rep['covers_all_targets']}; the analytic predictor was skipped: "
                     f"{rep['predictor']['reason']}")
    tm = rep["timing"]
    lines.append("wall-clock: " + ", ".join(f"{k} {v:.1f} s" for k, v in tm.items() if k != "over_budget")
                 + (" — OVER the 30 min budget" if tm["over_budget"] else ""))
    return "\n".join(lines)


def parser():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--family", default="ab")
    ap.add_argument("--statistics", nargs="+", choices=BX.STATISTICS, default=list(BX.STATISTICS))
    ap.add_argument("--tasks", nargs="+", choices=BX.TASKS, default=list(DEFAULT_TASKS),
                    help="tasks to score (default: placement only, issue #46)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--n-interior", type=int, default=4)
    ap.add_argument("--workers", type=int, default=None, help=f"parallel processes (default {BS.DEFAULT_WORKERS})")
    ap.add_argument("--runs", default="runs")
    return ap


def main(argv=None):
    from brixzle.families import registry

    args = parser().parse_args(argv)
    family = registry.get(args.family)
    out = score_family(family, family.defaults, statistics=tuple(args.statistics), seed=args.seed, tol=args.tol,
                       n_interior=args.n_interior, workers=args.workers, runs=args.runs, tasks=tuple(args.tasks))
    print(summary(json.loads((out / "report.json").read_text(encoding="utf-8"))))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
