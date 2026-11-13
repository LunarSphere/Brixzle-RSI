"""Scorer (issues #13, #61): Z per target box from the full sim map, as report.json (schema 4).

The loop's one objective is Z (engine/zscore.py, issue #61). For each target box (the drone's
mean, p95 and worst error; engine/boxes.py), every drop of the full sim map (engine/sim_map.py)
whose pose lies inside the box counts, across all slices and both courses:

    Z = (S - J) / N     S seated (+1), J jammed (-1), fell and wrong valley 0, N every drop in the box

Z = 1 passes. The report holds Z per target with S, J, N and the fell, wrong-valley and other
counts behind it. Nothing else is scored: the box scale and the predictor's terms are gone from
the score (issue #61). The predictor stays in the jam analysis (#49), to explain jams.

``score_family`` runs the quick check (#47), the full map with the grid covering all three target
boxes (aimed at the run's target), and writes map.json, drops.json.gz and report.json to one new
run directory. The jam analysis (#49) is written beside them later.

Placement only (issue #46). Sim results are simulated, uncalibrated contact.

    uv run python -m brixzle.engine.report --family ab --statistic mean --seed 0
"""

import argparse
import json
import math
from pathlib import Path
import time

from brixzle.engine import boxes as BX
from brixzle.engine import quick_check as QC
from brixzle.engine import sim_map as SM
from brixzle.engine import zscore as ZS

# 4: Z per target from the full sim map replaces the box scale and the predictor terms (#61).
# 5: Z counts the X-Y disc; z_x_only keeps the X-only Z, and y_capture_mm the measured Y capture (#86).
SCHEMA_VERSION = 5
LABEL = ZS.LABEL
BUDGET_S = 30 * 60  # a family should take no more than 30 min (CLAUDE.md, Speed)
FILE = "report.json"
COUNT_KEYS = ("Z", "S", "J", "N", "fell", "wrong_valley", "other", "passed")
FLAGS = (
    "Z is counted over the target box, not the green zone: the box is fixed by the drone's measured error",
    "the target boxes' X is measured (Exp 3); their Z and pitch are hover estimates (docs/requirements.md)",
    "zoom drops (overrides) are not counted",
)


def _need(cond, msg):
    if not cond:
        raise ValueError(msg)


def build_report(sim_map, drops, boxes, timing, map_dir=None):
    """report.json as a dict, from a map (map.json as a dict), its drops, the target boxes
    ({statistic: TargetBox}) and the stage timings (seconds; 'total' required).

    Every box must be one the map's grid covered (its ``cover``), or Z would miss drops."""
    missing = [s for s in boxes if s not in sim_map.get("cover", [])]
    _need(not missing, f"the map does not cover the target boxes {missing} (cover {sim_map.get('cover')}); "
                       "run it with cover= every target box")
    stats = [s for s in BX.STATISTICS if s in boxes]
    clearance0 = float(sim_map["config"]["error"]["clearance"][0])
    z = ZS.z_from_drops(drops, {s: boxes[s] for s in stats}, clearance0)
    z_x = ZS.z_from_drops(drops, {s: boxes[s] for s in stats}, clearance0, x_only=True)
    timing = {**timing, "over_budget": bool(timing["total"] > BUDGET_S)}
    return {
        "schema_version": SCHEMA_VERSION, "label": LABEL, "objective": ZS.DEFINITION,
        "family": sim_map["family"], "family_version": sim_map["family_version"], "params": sim_map["params"],
        "statistic": sim_map["box"]["statistic"], "statistics": stats,
        "boxes": {s: boxes[s].to_dict() for s in stats},
        "z": z, "z_x_only": z_x, "y_capture_mm": SM.y_capture(sim_map),
        "covers_all_targets": all(z[s]["passed"] for s in stats),
        "check": sim_map.get("check"),
        "map": {"dir": map_dir, "drops": sim_map["drops"], "seconds": sim_map["seconds"],
                "cover": list(sim_map["cover"]), "inside_failures": sim_map.get("inside_failures")},
        "timing": timing, "flags": list(FLAGS),
    }


def _finite(v, where="report"):
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return
    if isinstance(v, (int, float)):
        _need(math.isfinite(v), f"non-finite number at {where}")
    elif isinstance(v, dict):
        for k, x in v.items():
            _finite(x, f"{where}.{k}")
    elif isinstance(v, (list, tuple)):
        for i, x in enumerate(v):
            _finite(x, f"{where}[{i}]")
    else:
        raise ValueError(f"not JSON data at {where}: {type(v).__name__}")


_REQUIRED = ("schema_version", "label", "objective", "family", "family_version", "params", "statistic",
             "statistics", "boxes", "z", "z_x_only", "y_capture_mm", "covers_all_targets", "check", "map", "timing",
             "flags")


def validate_report(d):
    """Raise ValueError unless ``d`` is a well-formed report.json (schema SCHEMA_VERSION)."""
    _need(isinstance(d, dict), "report must be a dict")
    missing = [k for k in _REQUIRED if k not in d]
    _need(not missing, f"report is missing {missing}")
    _need(d["schema_version"] == SCHEMA_VERSION, f"schema_version must be {SCHEMA_VERSION}")
    _need(d["label"] == LABEL, f"label must be {LABEL!r}")
    stats = d["statistics"]
    _need(stats and set(stats) <= set(BX.STATISTICS), "bad statistics")
    _need(set(d["z"]) == set(stats) and set(d["boxes"]) == set(stats) and set(d["z_x_only"]) == set(stats),
          "z, z_x_only and boxes need exactly the statistics")
    for s, c in [(s, d[key][s]) for key in ("z", "z_x_only") for s in stats]:
        _need(set(c) == set(COUNT_KEYS), f"z[{s}] needs {COUNT_KEYS}")
        n = c["N"]
        _need(isinstance(n, int) and n > 0, f"z[{s}].N must be a positive integer")
        _need(c["S"] + c["J"] + c["fell"] + c["wrong_valley"] + c["other"] == n, f"z[{s}] counts must add up to N")
        _need(-1 <= c["Z"] <= 1 and abs(c["Z"] - (c["S"] - c["J"]) / n) < 1e-12, f"z[{s}].Z must be (S - J) / N")
        _need(c["passed"] == (c["S"] == n), f"z[{s}].passed must mean every drop seated")
    _need(d["covers_all_targets"] == all(d["z"][s]["passed"] for s in stats), "covers_all_targets")
    _need(isinstance(d["timing"].get("total"), (int, float)) and isinstance(d["timing"].get("over_budget"), bool),
          "timing needs total and over_budget")
    _finite(d)


def write_report(rep, map_dir):
    """Write report.json into the map's run directory; never overwrites one."""
    validate_report(rep)
    text = json.dumps(rep, indent=2, allow_nan=False) + "\n"
    path = Path(map_dir) / FILE
    with open(path, "x", encoding="utf-8") as f:
        f.write(text)
    return path


def score_family(family, p, statistic="mean", seed=0, runs="runs", workers=None, check=None, config=None,
                 config_reason=None, allow_failed_check=False, boxes=None, y_error=True):
    """Quick check (unless ``check`` is given), full map covering every target box, Z per target.

    Writes map.json, drops.json.gz and report.json to a new runs/map-<UTC>-<family>-<target>
    directory; returns (report, directory). ``allow_failed_check`` maps a design that failed the
    quick check (the loop never does; a baseline may). ``y_error=False`` leaves out the X-Y slice, as
    before #86. The quick check samples along X either way (issue #97)."""
    boxes = boxes or BX.default_boxes()["placement"]
    t0 = time.time()
    timing = {}
    if check is None:
        check = QC.quick_check(family, p, boxes[statistic], seed=seed)
        timing["check"] = time.time() - t0
    t = time.time()
    m, drops = SM.sim_map(family, p, boxes[statistic], check=check, config=config, config_reason=config_reason,
                          workers=workers, allow_failed_check=allow_failed_check, cover=tuple(boxes.values()),
                          slices=ZS.slices_for(y_error))
    timing["map"] = time.time() - t
    out = SM.write_map(m, drops, runs)
    timing["total"] = time.time() - t0
    rep = build_report(m, drops, boxes, timing, map_dir=out.name)
    write_report(rep, out)
    return rep, out


def summary(rep):
    """A short text table of a report, for the terminal."""
    lines = [f"{rep['family']} ({rep['label']}), map aimed at the {rep['statistic']} box; {rep['objective']}",
             f"{'target':<6} {'Z':>7} {'S':>5} {'J':>5} {'fell':>5} {'wrong':>5} {'other':>5} {'N':>5}"]
    for s in rep["statistics"]:
        c = rep["z"][s]
        lines.append(f"{s:<6} {c['Z']:>+7.3f} {c['S']:>5} {c['J']:>5} {c['fell']:>5} {c['wrong_valley']:>5} "
                     f"{c['other']:>5} {c['N']:>5}" + ("  PASS" if c["passed"] else ""))
    tm = rep["timing"]
    lines.append("wall-clock: " + ", ".join(f"{k} {v:.1f} s" for k, v in tm.items() if k != "over_budget")
                 + (" — OVER the 30 min budget" if tm["over_budget"] else ""))
    return "\n".join(lines)


def parser():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--family", default="ab")
    ap.add_argument("--statistic", choices=BX.STATISTICS, default="mean", help="the run's target: where the map aims")
    ap.add_argument("--seed", type=int, default=0, help="the quick check's seed")
    ap.add_argument("--workers", type=int, default=None, help=f"map drop processes (default {SM.DEFAULT_WORKERS})")
    ap.add_argument("--even-if-check-fails", action="store_true", help="score a design that failed the quick check")
    ap.add_argument("--runs", default="runs")
    return ap


def main(argv=None):
    from brixzle.families import registry

    args = parser().parse_args(argv)
    family = registry.get(args.family)
    rep, out = score_family(family, family.defaults, statistic=args.statistic, seed=args.seed, runs=args.runs,
                            workers=args.workers, allow_failed_check=args.even_if_check_fails)
    print(summary(rep))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
