"""Estimate the drone's Z, pitch and yaw error from aerial-assembly's committed Exp 3 hover logs.

These size the unmeasured sides of the target boxes (src/brixzle/engine/boxes.py) until
aerial-assembly#27 measures them properly. They are hover estimates, not errors measured at
release or pickup, and Z is the drone's own Lighthouse estimate against the commanded height,
so it likely underestimates the true height error.

Reads data only (``git show``), never aerial-assembly code:

    uv run python scripts/make_hover_estimates.py ~/Repos/aerial-assembly src/brixzle/data/hover-exp3-r2.json

For each tag dwell it uses aerial-assembly's settled window (flight/flight/calibration/accuracy.py
@ a649663: from SETTLE_S = 0.5 s after the segment ends, for dwell_s, only if the next segment was
commanded) and takes the RMS deviation from the command over that window: height against
target_m[2], pitch against 0, yaw against 0 (the Exp 3 rig requires launch yaw 0, rig.py; hover.py
checks yaw against zero heading).
"""

import gzip
import io
import json
import math
import subprocess
import sys
from datetime import date
from pathlib import Path

import numpy as np

COMMIT = "a649663"
DATA = "figures/exp3-approach-accuracy/data"
SETTLE_S = 0.5  # aerial-assembly accuracy.SETTLE_S @ a649663
PERCENTILE = 95


def _git(repo, *args, text=True):
    return subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True, text=text).stdout


def _rms(values):
    return float(np.sqrt(np.mean(np.square(values))))


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def approaches(events, trial):
    targets = [e for e in events if e.get("event") == "target"]
    telemetry = [(e["host_time"], e["values"]) for e in events if e.get("event") == "telemetry"]
    rows = []
    for i, target in enumerate(targets):
        if "tag_id" not in target:
            continue
        start = target["t"] + target["duration_s"] + SETTLE_S
        end = start + target["dwell_s"]
        if i + 1 >= len(targets) or targets[i + 1]["t"] < end:
            continue  # dwell not completed (same rule as accuracy.approaches)
        win = [v for t, v in telemetry if start <= t <= end]
        z = [1e3 * (v["stateEstimate.z"] - target["target_m"][2]) for v in win if "stateEstimate.z" in v]
        pitch = [v["stateEstimate.pitch"] for v in win if "stateEstimate.pitch" in v]
        yaw = [_wrap_deg(v["stateEstimate.yaw"]) for v in win if "stateEstimate.yaw" in v]
        if not (z and pitch and yaw):
            continue
        rows.append({"trial": trial, "tag_id": int(target["tag_id"]), "samples": len(win),
                     "z_rms_mm": _rms(z), "pitch_rms_deg": _rms(pitch), "yaw_rms_deg": _rms(yaw)})
    return rows


def main(repo, out):
    repo, out = Path(repo).resolve(), Path(out)
    if out.exists():
        raise SystemExit(f"{out} exists; remove it first if you mean to regenerate it")
    commit = _git(repo, "rev-parse", COMMIT).strip()
    files = sorted(f for f in _git(repo, "ls-tree", "-r", "--name-only", commit, DATA).split()
                   if f.endswith("events.jsonl.gz"))
    rows, trials = [], []
    for f in files:
        trial = f.split("/")[-2]
        raw = _git(repo, "show", f"{commit}:{f}", text=False)
        events = [json.loads(line) for line in gzip.open(io.BytesIO(raw), "rt")]
        found = approaches(events, trial)
        if found:
            trials.append(trial)
            rows += found
    if not rows:
        raise SystemExit("no completed dwells found")
    out_d = {
        "version": 1, "kind": "hover-deviation",
        "units": {"z": "mm", "pitch": "deg", "yaw": "deg"},
        "label": ("estimated from hover, not measured at release or pickup; z is the drone's own "
                  "Lighthouse estimate against the commanded height (likely an underestimate)"),
        "source": {
            "repo": "LunarSphere/aerial-assembly", "commit": commit, "data": DATA, "session": "exp3-r2",
            "trials": trials, "generator": "scripts/make_hover_estimates.py", "date": date.today().isoformat(),
            "window": "accuracy.approaches settled window: from 0.5 s after each segment, for dwell_s",
            "statistic_per_approach": "RMS deviation from the command over the window",
            "commanded_yaw_deg": 0, "commanded_pitch_deg": 0, "z_reference": "target_m[2]",
        },
        "n": len(rows),
        "approaches": rows,
    }
    for key in ("z_mm", "pitch_deg", "yaw_deg"):
        values = [r[key.replace("_mm", "_rms_mm").replace("_deg", "_rms_deg")] for r in rows]
        out_d[f"mean_{key}"] = float(np.mean(values))
        out_d[f"p95_{key}"] = float(np.percentile(values, PERCENTILE))
        out_d[f"max_{key}"] = float(np.max(values))
    if not all(math.isfinite(v) for k, v in out_d.items() if k.startswith(("mean_", "p95_", "max_"))):
        raise SystemExit("non-finite statistic")
    out.write_text(json.dumps(out_d, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"wrote {out}: {len(rows)} approaches from {len(trials)} trials")
    for key in ("z_mm", "pitch_deg", "yaw_deg"):
        print(f"  {key}: mean {out_d[f'mean_{key}']:.2f}  p95 {out_d[f'p95_{key}']:.2f}  max {out_d[f'max_{key}']:.2f}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    main(sys.argv[1], sys.argv[2])
