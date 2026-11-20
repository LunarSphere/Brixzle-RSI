"""The build-aware objective's terms and ranking for the baseline designs (#164).

    uv run python scripts/objective_baselines.py [--designs ab,fable_twinhook4,...]

Writes runs/objective-<UTC>/report.json: per design every term of engine/objective.evaluate_design and
the ranking under objective.Settings. Z on the three boxes and the gate's overhang are read from each
design's evaluation (cited per design), Z at mu 0.8 from its friction run; A/B's friction margin is
rerun here (tools.core.friction, seed 1). Simulated, uncalibrated contact.
"""
import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
import time

from brixzle.engine import objective as OB
from brixzle.engine.sim import arm as A
from brixzle.families import registry
from brixzle.tools import core as TC

EVALS = {  # design -> (eval.json with Z and the gate, friction.json or None to rerun)
    "ab": ("runs/loop/wrapup-20261116-mean/design_log.jsonl", None),
    "fable_twinhook4": ("RSI_RESULTS/refactor162-fable_twinhook4-20261120/eval.json",
                        "RSI_RESULTS/fable-twinhook4-verify-20261117/friction.json"),
    "fable_gentle": ("RSI_RESULTS/refactor162-fable_gentle-20261120/eval.json",
                     "RSI_RESULTS/fable_gentle-verify-20261117/friction.json"),
    "fable_allround": ("RSI_RESULTS/refactor162-fable_allround-20261120/eval.json",
                       "RSI_RESULTS/fable_allround-20261117-opus/friction.json"),
}


def inputs(name, fam):
    src, fr = EVALS[name]
    if src.endswith(".jsonl"):  # A/B: the loop's iteration 0
        row = json.loads(Path(src).read_text().splitlines()[0])
        z = {k: v["Z"] if isinstance(v, dict) else v for k, v in row["z"].items()}
        overhang = row.get("overhang_n")
    else:
        e = json.loads(Path(src).read_text())
        z = {k: v["Z"] for k, v in e["z"].items()}
        overhang = e["gate"]["overhang_n"]
    if fr:
        f = json.loads(Path(fr).read_text())
        mu08 = next(v["Z"] for k, v in f["z"].items() if k.endswith("0.8"))
        fsrc = fr
    else:
        f = TC.friction(fam, fam.defaults, mus=(0.8,))
        mu08 = next(iter(f["z"].values()))["Z"]
        fsrc = "rerun here (tools.core.friction, seed 1, mean box)"
    return z, mu08, overhang, {"z_and_gate": src, "z_mu08": fsrc}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--designs", default=",".join(EVALS))
    ap.add_argument("--runs", default="runs")
    a = ap.parse_args()
    out = Path(a.runs) / f"objective-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.time()
    env = A.top_down_envelope(step_mm=5.0)
    secs = {"envelope": round(time.time() - t0, 1)}
    recs = []
    for name in a.designs.split(","):
        fam = registry.get(name)
        z, mu08, overhang, sources = inputs(name, fam)
        rec = OB.evaluate_design(fam, settings=OB.Settings(), env=env, z=z, z_mu08=mu08, overhang_n=overhang)
        rec["sources"] = sources
        recs.append(rec)
        print(name, json.dumps({k: rec[k] for k in ("violations", "overhang_mean_steps", "tolerance_mm")}), rec["overhang_tolerance"]["median_steps"],
              rec["grip"]["lift_mm"], rec["size"]["length_mm"], rec["arm"], flush=True)
        (out / "report.json").write_text(json.dumps(TC._finite({"label": OB.LABEL, "designs": recs, "seconds": secs}), indent=1, default=str) + "\n")
    ranked = OB.rank(recs)
    secs["total"] = round(time.time() - t0, 1)
    report = {"label": OB.LABEL, "settings": recs[0]["settings"], "designs": recs,
              "ranking": [{"name": r["name"], "layer": r["layer"], "feasible": r["feasible"], "violations": r["violations"]}
                          for r in ranked], "seconds": secs}
    (out / "report.json").write_text(json.dumps(TC._finite(report), indent=1, default=str) + "\n")
    print(json.dumps(report["ranking"], indent=1))
    print(out)


if __name__ == "__main__":
    main()
