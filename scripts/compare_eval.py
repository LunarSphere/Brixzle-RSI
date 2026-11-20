"""Compare two evaluations (eval.json from runs/fable/eval_family.py) drop by drop and build by build.

    uv run python scripts/compare_eval.py <reference eval dir> <new eval dir> [--out compare.json]

Used to check that a port or a refactor (#162) changed nothing: the same seed and poses must give the
same outcome and rest error for every map drop, the same Z counts, the same held-out counts, the same
statuses, stable bricks and failures for every gate build, and the same ring. Exits 1 on any
difference. Simulated, uncalibrated contact.
"""
import argparse
import gzip
import json
from pathlib import Path
import sys


def _drops(map_dir):
    rows = json.loads(gzip.open(Path(map_dir) / "drops.json.gz").read())
    out = {}
    for r in rows:
        key = (r.get("slice"), r["course"], tuple(r.get("cell") or ()), r["dx_mm"], r.get("dy_mm", 0.0),
               r["clear_mm"], r["pitch_deg"], bool(r.get("zoom")))
        out[key] = (r["outcome"], tuple(r.get("err_mm") or ()))
    return out


def _gate(gate_dir):
    g = json.loads((Path(gate_dir) / "gate.json").read_text())
    keep = ("status", "complete", "n", "statuses", "stable_bricks", "failed_brick", "max_sag_mm")
    return {"overhang_n": g.get("overhang_n"), "passed": g.get("passed"),
            "builds": {k: {f: b.get(f) for f in keep} for k, b in g["builds"].items()}}


def compare(ref_dir, new_dir):
    ref = json.loads((Path(ref_dir) / "eval.json").read_text())
    new = json.loads((Path(new_dir) / "eval.json").read_text())
    diffs = []

    da, db = _drops(ref["map"]), _drops(new["map"])
    only = set(da) ^ set(db)
    if only:
        diffs.append(f"map: {len(only)} drop poses in only one map")
    changed = [k for k in set(da) & set(db) if da[k] != db[k]]
    outcome_changed = [k for k in changed if da[k][0] != db[k][0]]
    if changed:
        diffs.append(f"map: {len(changed)} drops differ ({len(outcome_changed)} in outcome)")

    for box in ref["z"]:
        a = {k: ref["z"][box].get(k) for k in ("S", "J", "N", "Z")}
        b = {k: new["z"].get(box, {}).get(k) for k in ("S", "J", "N", "Z")}
        if a != b:
            diffs.append(f"Z {box}: {a} vs {b}")
    for name, a in (ref.get("held_out") or {}).items():
        b = (new.get("held_out") or {}).get(name)
        if a != b:
            diffs.append(f"held-out {name}: {a} vs {b}")

    ga, gb = _gate(ref["gate"]["run"]), _gate(new["gate"]["run"])
    if ga != gb:
        for k in sorted(set(ga["builds"]) | set(gb["builds"])):
            if ga["builds"].get(k) != gb["builds"].get(k):
                diffs.append(f"gate {k}: {ga['builds'].get(k)} vs {gb['builds'].get(k)}")
        if ga["overhang_n"] != gb["overhang_n"] or ga["passed"] != gb["passed"]:
            diffs.append(f"gate overhang_n/passed: {ga['overhang_n']}/{ga['passed']} vs {gb['overhang_n']}/{gb['passed']}")

    ring_keys = ("status", "seated", "total", "failed_brick")
    ra = {k: (ref.get("ring") or {}).get(k) for k in ring_keys}
    rb = {k: (new.get("ring") or {}).get(k) for k in ring_keys}
    if ra != rb:
        diffs.append(f"ring: {ra} vs {rb}")
    if ref.get("mass_g") != new.get("mass_g"):
        diffs.append(f"mass: {ref.get('mass_g')} vs {new.get('mass_g')}")

    return {
        "label": "simulated, uncalibrated contact",
        "reference": str(ref_dir), "new": str(new_dir),
        "drops_compared": len(set(da) & set(db)),
        "gate_builds_compared": len(ga["builds"]),
        "identical": not diffs, "differences": diffs,
        "seconds": {"reference": ref.get("seconds"), "new": new.get("seconds")},
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("reference")
    ap.add_argument("new")
    ap.add_argument("--out")
    args = ap.parse_args(argv)
    result = compare(args.reference, args.new)
    text = json.dumps(result, indent=1, allow_nan=False)
    if args.out:
        Path(args.out).write_text(text + "\n")
    print(text)
    return 0 if result["identical"] else 1


if __name__ == "__main__":
    sys.exit(main())
