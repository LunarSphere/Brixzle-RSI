"""Record the aerial-assembly A/B drop outcome that tests/test_ab_drop_regression.py reproduces.

Not part of the brixzle package and not run by pytest. It runs under aerial-assembly's own
environment (its brixzle package), so the reference comes from the original code:

    ~/Repos/aerial-assembly/brixzle/.venv/bin/python -I \
        tests/fixtures/make_ab_drop_reference.py ~/Repos/aerial-assembly \
        tests/fixtures/ab_drop_reference.json

It refuses to run unless aerial-assembly's brixzle/src and brixzle/configs match SOURCE_COMMIT,
the commit the sim core was ported from. Results are simulated, uncalibrated contact.
"""

import json
import platform
import subprocess
import sys
from datetime import date
from pathlib import Path

SOURCE_COMMIT = "a649663"
PATHS = ("brixzle/src", "brixzle/configs")

# The cases the regression test replays. Keep in step with CASES in the test.
CASES = (
    {"name": "gauss-seed0", "part": "B", "seed": 0, "samples": 4, "error": "default"},
    {"name": "exp3-mean-seed0", "part": "B", "seed": 0, "samples": 4, "error": "exp3-r4:mean"},
)


def _git(repo, *args):
    return subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True,
                          text=True).stdout.strip()


def main(repo, out):
    repo = Path(repo).resolve()
    out = Path(out)
    if out.exists():
        raise SystemExit(f"{out} exists; remove it first if you mean to regenerate it")
    if _git(repo, "status", "--porcelain", "--", *PATHS):
        raise SystemExit("aerial-assembly has uncommitted changes under brixzle/src or brixzle/configs")
    if subprocess.run(["git", "-C", str(repo), "diff", "--quiet", SOURCE_COMMIT, "HEAD", "--", *PATHS]).returncode:
        raise SystemExit(f"aerial-assembly brixzle/src or brixzle/configs differ from {SOURCE_COMMIT}")

    import mujoco
    import numpy as np
    from brixzle import approach_error as AE, cad, trials as T
    from brixzle.params import V0

    bricks = cad.build_bricks(V0)
    geometry = {
        name: {
            "mass_g": b["mass_g"],
            "volume_mm3": b["volume"],
            "com_mm": list(b["com"]),
            "inertia_g_mm2": [list(map(float, row)) for row in b["inertia"]],
            "n_collision": len(b["collision"]),
            "n_collision_bored": len(b["collision_bored"]),
        }
        for name, b in bricks.items()
    }

    cases = []
    for case in CASES:
        if case["error"] == "default":
            cfg = T.TrialConfig()
        else:
            _, stat = case["error"].split(":")
            summary = AE.load(AE.CONFIG_DIR / "error-exp3-r4.json")
            cfg = T.TrialConfig(error=AE.error_model(summary, stat))
        rows, _ = T.drop_trials(V0, bricks, cfg, samples=case["samples"], seed=case["seed"],
                                part=case["part"])
        cases.append({**case, "config": T.as_dict(cfg), "rows": rows})

    reference = {
        "label": "simulated, uncalibrated contact",
        "source": {
            "repo": "LunarSphere/aerial-assembly",
            "commit": _git(repo, "rev-parse", SOURCE_COMMIT),
            "checked_out_head": _git(repo, "rev-parse", "HEAD"),
            "paths_identical_to_commit": list(PATHS),
            "generator": "tests/fixtures/make_ab_drop_reference.py",
            "date": date.today().isoformat(),
            "platform": platform.platform(),
            "python": platform.python_version(),
            "mujoco": mujoco.__version__,
            "numpy": np.__version__,
        },
        "params": V0.to_dict(),
        "geometry": geometry,
        "cases": cases,
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(reference, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    main(sys.argv[1], sys.argv[2])
