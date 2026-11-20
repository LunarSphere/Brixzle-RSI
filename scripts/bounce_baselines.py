"""Bounce-out against drop height and impact energy (#167).

    uv run python scripts/bounce_baselines.py [--designs ...] [--dampratios 1.0,0.3,0.1] [--repeats 5]

Writes runs/bounce-<UTC>/report.json with engine/bounce.bounce for each design. Simulated, uncalibrated
contact; the damping ratio 1.0 is the repo's contact, lower ones stand in for a bouncier one.
"""
import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
import time

from brixzle.engine import bounce as BO
from brixzle.families import registry

DESIGNS = ("ab", "fable_twinhook4", "fable_gentle", "fable_allround", "fable_gentle_longhook", "fable_gentle_tpuhook")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--designs", default=",".join(DESIGNS))
    ap.add_argument("--dampratios", default="1.0,0.3,0.1")
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--runs", default="runs")
    a = ap.parse_args()
    out = Path(a.runs) / f"bounce-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.time()
    rep = {"label": BO.LABEL, "repeats": a.repeats, "designs": {}}
    for name in a.designs.split(","):
        fam = registry.get(name)
        r = BO.bounce(fam, dampratios=tuple(float(d) for d in a.dampratios.split(",")), repeats=a.repeats)
        rep["designs"][name] = r
        print(name, r["seconds"], [(x["seat"], x["dampratio"], round(x["height_mm"]), x["kinds"]) for x in r["rows"]], flush=True)
        rep["seconds"] = round(time.time() - t0, 1)
        (out / "report.json").write_text(json.dumps(rep, indent=1) + "\n")
    print(out)


if __name__ == "__main__":
    main()
