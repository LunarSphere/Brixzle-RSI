"""Record the stock-jaw grasp of each design, the pick from the mould station, and whether the arm
clears the overhang and bridge at every placement (#153).

    uv run python scripts/arm_grasp.py

Writes runs/arm-grasp-<UTC time>/report.json. Simulated, uncalibrated contact (MuJoCo); the clash
check is kinematic. The overhang is the longest the arm reaches with the real grasp (up to 12).
"""
import argparse
import json
import math
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from brixzle.engine import structures as S
from brixzle.engine.sim import arm as A
from brixzle.engine.sim import grasp as G
from brixzle.engine.sim import pickplace as PP
from brixzle.families import registry

DESIGNS = ("ab", "fable_twinhook4", "fable_allround")


def longest_overhang(fam, p, grasps, env):
    for n in range(12, 0, -1):
        st = S.overhang(n, +1, fam.lattice(p).span_voxels)
        tcp, yaws = PP.grasp_tcp_mm(fam, p, st.bricks, grasps, with_yaw=True)
        if A.plan_base(fam, p, st, tcp, env, yaws=yaws) is not None:
            return st
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", default="runs")
    a = ap.parse_args()
    out = Path(a.runs) / f"arm-grasp-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    out.mkdir(parents=True, exist_ok=False)
    secs = {}
    t = time.time()
    env = A.top_down_envelope(step_mm=5.0)
    jaw = G.jaws()
    secs["envelope_and_jaws"] = round(time.time() - t, 1)
    rec = {"label": "simulated, uncalibrated contact (MuJoCo); clash check kinematic",
           "settings": {"grip_torque_Nm": PP.GRIP_TORQUE, "impratio": PP.IMPRATIO, "jaw_solref_s": PP.JAW_SOLREF,
                        "timestep_s": PP.PHYSICS.timestep, "approach_mm": PP.APPROACH_MM},
           "jaws": {"fixed_x_mm": jaw.fixed_x_mm, "pad_z_mm": jaw.pad_z_mm, "tip_below_mm": jaw.tip_below_mm,
                    "opening_range_mm": [float(jaw.opening_mm.min()), float(jaw.opening_mm.max())]},
           "designs": {}}
    for name in DESIGNS:
        fam = registry.get(name)
        p = fam.defaults
        d = {"grasps": {}, "picks": {}, "structures": {}}
        t = time.time()
        grasps = {pt: G.choose_grasp(fam, p, pt) for pt in fam.parts}
        d["grasps"] = {pt: asdict(g) for pt, g in grasps.items()}
        secs[f"{name}/grasp"] = round(time.time() - t, 1)
        for pt in fam.parts:
            t = time.time()
            try:
                d["picks"][pt] = PP.pick(PP.station_rig(fam, p, pt, grasp=grasps[pt]))
            except RuntimeError as e:
                d["picks"][pt] = {"error": str(e)}
            secs[f"{name}/pick_{pt}"] = round(time.time() - t, 1)
        for st in (longest_overhang(fam, p, grasps, env), S.bridge(3, fam.lattice(p).span_voxels)):
            if st is None:
                continue
            t = time.time()
            r = PP.placement_clashes(fam, p, st, env, grasps=grasps)
            base = r["base"]
            d["structures"][st.name] = {
                "bricks": len(st.bricks), "clashes": r["clashes"], "unreached": r["unreached"], "bases_tried": r["tried"],
                "arm_base_mm": None if base is None else list(base[:3]),
                "arm_yaw_deg": None if base is None else math.degrees(base[3]), "riser_mm": None if base is None else base[4]}
            secs[f"{name}/{st.name}"] = round(time.time() - t, 1)
        rec["designs"][name] = d
        print(name, json.dumps({"picks": d["picks"], "structures": {k: (v["bricks"], len(v["clashes"]), v["unreached"], v["riser_mm"])
                                                                       for k, v in d["structures"].items()}}), flush=True)
    rec["seconds"] = secs
    (out / "report.json").write_text(json.dumps(rec, indent=1, allow_nan=False) + "\n")
    print(out)


if __name__ == "__main__":
    main()
