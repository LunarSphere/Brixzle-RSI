"""Record a predicted placement box: uv run python -m brixzle.engine.predictor --family ab --statistic mean

Writes runs/predict-<UTC>-<family>-placement-<statistic>/result.json (never overwrites a run).
Results are "predicted, quasi-static 2D".
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from brixzle.engine import boxes as BX
from brixzle.engine.predictor import predict as PR


def main(argv=None):
    from brixzle.families import registry

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--family", default="ab")
    ap.add_argument("--statistic", choices=BX.STATISTICS, default="mean")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--courses", type=int, nargs="+", choices=(0, 1), default=[0, 1])
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--runs", default="runs")
    args = ap.parse_args(argv)
    family = registry.get(args.family)
    box = BX.default_boxes()["placement"][args.statistic]
    result = PR.predicted_box(family, family.defaults, box, courses=tuple(args.courses), seed=args.seed, tol=args.tol)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = Path(args.runs) / f"predict-{stamp}-{family.name}-placement-{args.statistic}"
    out.mkdir(parents=True, exist_ok=False)
    (out / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    per = ", ".join(f"course {c} {r['scale']:.3f}" for c, r in result["courses"].items())
    print(f"{family.name} placement {args.statistic} (predicted, quasi-static 2D): scale {result['scale']:.3f} "
          f"({per}), {result['seconds']:.1f} s -> {out}")


if __name__ == "__main__":
    main()
