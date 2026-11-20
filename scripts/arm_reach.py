"""Measure the SO-101's top-down reach and how far each design's overhang and bridge fall inside it (#152).

    uv run python scripts/arm_reach.py [--fable-root runs/fable] [--step 5]

Writes runs/arm-reach-<UTC time>/ with report.json (the envelope's size, each design's overhang
courses and bridge arm courses and the arm base used, stage timings) and envelope.png. Reach is
judged top-down at the nominal grasp point, the centre of the brick's bounding box (#153 replaces
it with the real grasp). Kinematics only: no contact is simulated.
"""
import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from brixzle.agents.check_family import load_family
from brixzle.engine.sim import arm as A
from brixzle.families import registry


def families(fable_root):
    out = {"ab": registry.get("ab")}
    for name in ("fable_twinhook4", "fable_allround"):
        if (Path(fable_root) / name / "family.py").exists():
            out[name] = load_family(fable_root, name)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fable-root", default="runs/fable")
    ap.add_argument("--step", type=float, default=5.0)
    ap.add_argument("--runs", default="runs")
    a = ap.parse_args()
    out = Path(a.runs) / f"arm-reach-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    out.mkdir(parents=True, exist_ok=False)
    secs = {}
    t = time.time()
    env = A.top_down_envelope(step_mm=a.step)
    secs["envelope"] = round(time.time() - t, 1)
    cols = env["reachable"].any(axis=0)
    rec = {"label": "kinematics only, top-down at the nominal grasp point (bounding-box centre)",
           "model": "mujoco_menagerie robotstudio_so101 @ 370c439", "step_mm": a.step,
           "max_reach_mm": float(env["r_mm"][cols].max()),
           "top_down_ceiling_mm": float(env["z_mm"][env["reachable"].any(axis=1)].max()),
           "reach_at_floor_mm": float(env["r_mm"][env["reachable"][np.argmin(abs(env["z_mm"]))]].max(initial=0.0)),
           "designs": {}}
    for name, fam in families(a.fable_root).items():
        t = time.time()
        p = fam.defaults
        oh = A.max_overhang_courses(fam, env, p)
        br = A.max_bridge_courses(fam, env, p)
        lat = fam.lattice(p)
        rec["designs"][name] = {"pitch_mm": lat.U_mm, "course_mm": lat.H0_mm, "overhang_courses": oh["courses"],
                                "overhang_base_mm": oh["base_mm"], "overhang_base_yaw_deg": None if oh["yaw"] is None else math.degrees(oh["yaw"]),
                                "overhang_riser_mm": oh["riser_mm"], "bridge_riser_mm": br["riser_mm"],
                                "bridge_arm_courses": br["arm_courses"], "bridge_base_mm": br["base_mm"],
                                "bridge_base_yaw_deg": None if br["yaw"] is None else math.degrees(br["yaw"])}
        secs[name] = round(time.time() - t, 1)
        print(name, rec["designs"][name], flush=True)
    rec["seconds"] = secs
    (out / "report.json").write_text(json.dumps(rec, indent=1, allow_nan=False) + "\n")
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.imshow(env["reachable"], origin="lower", aspect="equal", cmap="Greens",
              extent=[env["r_mm"][0], env["r_mm"][-1], env["z_mm"][0], env["z_mm"][-1]])
    ax.set_xlabel("plan distance from the pan axis (mm)")
    ax.set_ylabel("height above the arm's base (mm)")
    ax.set_title("SO-101: where the tool point reaches pointing straight down")
    fig.tight_layout()
    fig.savefig(out / "envelope.png", dpi=110)
    print(out)


if __name__ == "__main__":
    main()
