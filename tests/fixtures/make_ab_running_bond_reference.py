"""Record the A/B running-bond drops that tests/test_ab_drop_regression.py pins (issue #82).

Since #82 a course-1 drop lands across two seated course-0 bricks (running bond), so aerial-assembly's
stack-drop reference (ab_drop_reference.json) no longer gives the outcomes or settled poses. This
script replays the same cases (same params, seeds and configs) with this repo's DropRig and writes
their outcomes and settled poses. The sampled errors are still checked against aerial-assembly.

    uv run python tests/fixtures/make_ab_running_bond_reference.py tests/fixtures/ab_drop_running_bond.json

Run it on macOS arm64 (the reference platform). Results are simulated, uncalibrated contact.
"""

import json
import platform
import sys
from datetime import date
from pathlib import Path

import mujoco

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import test_ab_drop_regression as R  # noqa: E402
from brixzle.engine.sim import trials as T  # noqa: E402


def main(out):
    out = Path(out)
    if out.exists():
        raise SystemExit(f"{out} exists; remove it first if you mean to regenerate it")
    cases = []
    for case in R.CASES:
        rows, _ = T.drop_trials(R.AB, R.V0, R._config(case["error"]), samples=case["samples"], seed=case["seed"],
                                course=R.COURSE[case["part"]])
        cases.append({"name": case["name"], "rows": [{"outcome": r["outcome"], "floor": bool(r["floor"]),
                                                      "err_mm": [float(v) for v in r["err_mm"]],
                                                      "angle_deg": float(r["angle_deg"])} for r in rows]})
    doc = {"label": "simulated, uncalibrated contact",
           "source": {"what": "this repo's DropRig, course 1 across two seated course-0 bricks (issue #82)",
                      "platform": f"{platform.system()} {platform.machine()}".replace("Darwin", "macOS"),
                      "mujoco": mujoco.__version__, "date": date.today().isoformat()},
           "cases": cases}
    out.write_text(json.dumps(doc, indent=1, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main(sys.argv[1])
