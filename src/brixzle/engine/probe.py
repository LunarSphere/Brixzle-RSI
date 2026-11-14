"""Drop probe (issue #107): a few drops of a draft design, at the poses its parent failed.

The design agent gets one scored result per iteration. The probe lets it test an idea inside its
own call: it drops the draft at up to 30 poses and returns, per pose, the outcome (seated, jammed,
wrong_valley, fell) and where the brick came to rest.

**The poses** (``probe_poses``), seeded so a call can be repeated:

- with a parent map: a sample of the parent's jammed and wrong-valley drops inside the current
  target box, a few of its seated ones as a control, and the aim;
- without one (an explore call, #106): a seeded spread over the target box's X-Y disc on both
  courses, and the aim.

They are map grid cells or the disc spread, never the held-out poses: those are each cell shifted
off the grid by another seed (zscore.held_out_poses), so the held-out check stays independent.

The design agent runs it as a tool, in a subprocess with a time limit (``probe_in_subprocess``), as
it runs check_family. Results are simulated, uncalibrated contact.

    uv run python -m brixzle.engine.probe --root <root> --module <module> --poses <poses.json>
"""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from brixzle.engine import box_search as BS
from brixzle.engine import quick_check as QC
from brixzle.engine import zscore as ZS
from brixzle.engine.sim import trials as T

LABEL = "simulated, uncalibrated contact"
N_PROBE = 30        # poses per probe, the aim included
N_CONTROL = 4       # seated poses of the parent, as a control
FAILURES = ("jammed", "wrong_valley")
PROBE_TIMEOUT_S = 120
WORKERS = 4


def _pose_entry(source, course, sample, **extra):
    x, y = sample["dx"] - sample["aim"], sample.get("dy", 0.0)
    return {"source": source, "course": int(course), "x_mm": round(float(x), 3), "y_mm": round(float(y), 3),
            "sample": {k: (float(v) if not isinstance(v, list) else [float(q) for q in v]) for k, v in sample.items()},
            **extra}


def _aim(cfg):
    return [_pose_entry("aim", course, cfg.error.sample(None)) for course in (0, 1)]


def _map_sample(d, cfg):
    aim = cfg.error.aim_bias
    return {"dx": aim + float(d["dx_mm"]), "dy": float(d.get("dy_mm", 0.0)), "aim": aim, "yaw": 0.0, "roll": 0.0,
            "pitch": float(d["pitch_deg"]), "clear": float(d["clear_mm"]), "vel": [0.0, 0.0, 0.0]}


def probe_poses(box, drops=None, seed=0, n=N_PROBE, n_control=N_CONTROL, cfg=None):
    """Up to ``n`` probe poses for ``box`` (a placement TargetBox), the aim included.

    ``drops``: the parent's map drops (sim_map.read_drops); the poses are its failures inside the
    box, a few seated drops and the aim. None: a seeded spread over the box's X-Y disc."""
    if not isinstance(n, int) or isinstance(n, bool) or n < 2:
        raise ValueError(f"n must be an integer >= 2, got {n!r}")
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    rng = np.random.default_rng(seed)
    out = _aim(cfg)
    if drops is None:
        for course, unit in (QC.check_poses(n - len(out), seed, disc=True) if n > len(out) else []):
            out.append(_pose_entry("disc", course, QC.disc_sample(box, unit, cfg)))
        return out
    c0 = cfg.error.clearance[0]
    inside = [d for d in drops if not d.get("zoom") and ZS.inside(ZS.drop_pose(d, c0), box)]
    inside.sort(key=lambda d: (d["slice"], d["course"], tuple(d["cell"])))
    fails = [d for d in inside if d["outcome"] in FAILURES]
    seated = [d for d in inside if d["outcome"] == "seated"]
    k_control = min(n_control, len(seated), n - len(out))
    k_fail = min(len(fails), n - len(out) - k_control)
    k_control = min(len(seated), n - len(out) - k_fail)  # fill with controls if there are few failures
    for group, k in ((fails, k_fail), (seated, k_control)):
        idx = sorted(rng.choice(len(group), size=k, replace=False).tolist()) if k else []
        for i in idx:
            d = group[i]
            out.append(_pose_entry(d["outcome"], d["course"], _map_sample(d, cfg), slice=d["slice"],
                                   cell=list(d["cell"])))
    return out


def run_probe(family, p, poses, cfg=None, workers=WORKERS):
    """Drop ``family`` at ``poses``; one row per pose: its source, pose, outcome, rest error and tilt."""
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    t0 = time.time()
    with BS.DropPool(family, p, cfg, workers=min(workers, len(poses))) as pool:
        rows = list(pool.imap([(q["course"], q["sample"], {"trace": False}) for q in poses]))
    out = []
    for q, r in zip(poses, rows):
        out.append({"source": q["source"], "course": q["course"], "x_mm": q["x_mm"], "y_mm": q["y_mm"],
                    "outcome": r["outcome"], "rest_err_mm": [round(float(v), 2) for v in r["err_mm"]],
                    "tilt_deg": round(float(r["angle_deg"]), 2)})
    counts = {}
    for r in out:
        counts[r["outcome"]] = counts.get(r["outcome"], 0) + 1
    fixed = sum(1 for r in out if r["source"] in FAILURES and r["outcome"] == "seated")
    broken = sum(1 for r in out if r["source"] == "seated" and r["outcome"] != "seated")
    return {"label": LABEL, "drops": len(out), "outcomes": counts,
            "parent_failures_now_seated": fixed, "parent_seated_now_failing": broken,
            "seconds": round(time.time() - t0, 2), "rows": out}


def probe_in_subprocess(root, module, poses_path, timeout=PROBE_TIMEOUT_S):
    """``run_probe`` on the package ``<root>/<module>`` in a fresh process with a time limit; a crash
    or a hang is reported, not raised."""
    cmd = [sys.executable, "-m", "brixzle.engine.probe", "--root", str(root), "--module", module,
           "--poses", str(poses_path)]
    t0 = time.time()
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"error": f"the probe took longer than {timeout} s", "seconds": round(time.time() - t0, 2)}
    try:
        return json.loads(done.stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError):
        tail = (done.stderr or "").strip().splitlines()[-5:]
        return {"error": f"the probe crashed (exit {done.returncode}): {' | '.join(tail)}",
                "seconds": round(time.time() - t0, 2)}


def write_poses(poses, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(poses, indent=1, allow_nan=False) + "\n", encoding="utf-8")
    return path


def main(argv=None):
    from brixzle.agents import check_family as CF

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--root", required=True)
    ap.add_argument("--module", required=True)
    ap.add_argument("--poses", required=True)
    args = ap.parse_args(argv)
    poses = json.loads(Path(args.poses).read_text(encoding="utf-8"))
    try:
        fam = CF.load_family(args.root, args.module)
        out = run_probe(fam, fam.defaults, poses)
    except Exception as e:  # noqa: BLE001 - reported to the agent
        out = {"error": f"{type(e).__name__}: {e}"[:500]}
    print(json.dumps(out, allow_nan=False, default=lambda v: v if math.isfinite(v) else None))


if __name__ == "__main__":
    main()
