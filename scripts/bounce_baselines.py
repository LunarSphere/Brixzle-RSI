"""Bounce-out against drop height and impact energy (#167).

    uv run python scripts/bounce_baselines.py [--designs ...] [--dampratios 1.0,0.3,0.1]
        [--heights-mm 0,25,50,75,100,150] [--seats half,full] [--repeats 5] [--seed 0]

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
    ap.add_argument("--heights-mm", default=",".join(map(str, BO.HEIGHTS_MM)))
    ap.add_argument("--seats", default=",".join(BO.SEATS))
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--runs", default="runs")
    a = ap.parse_args()
    out = Path(a.runs) / f"bounce-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.time()
    dampratios = tuple(float(d) for d in a.dampratios.split(","))
    heights_mm = tuple(float(h) for h in a.heights_mm.split(","))
    seats = tuple(seat.strip() for seat in a.seats.split(","))
    rep = {"label": BO.LABEL, "seed": a.seed, "repeats": a.repeats, "heights_mm": heights_mm,
           "dampratios": dampratios, "seats": seats, "designs": {}}
    for name in a.designs.split(","):
        fam = registry.get(name.strip())
        r = BO.bounce(fam, heights_mm=heights_mm, seats=seats, dampratios=dampratios, repeats=a.repeats, seed=a.seed)
        rep["designs"][name] = r
        print(name, r["seconds"], [(x["seat"], x["dampratio"], round(x["height_mm"]), x["kinds"]) for x in r["rows"]], flush=True)
        rep["seconds"] = round(time.time() - t0, 1)
        (out / "report.json").write_text(json.dumps(rep, indent=1, allow_nan=False) + "\n")
    print(out)


if __name__ == "__main__":
    main()
