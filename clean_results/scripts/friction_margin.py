"""Seating under gravity alone as friction rises: held-out Z on the mean box (seed 1's shifted poses)
at each mu, and the fork's clean release check. Simulated, uncalibrated contact.
Usage: friction_margin.py <root|registry> <module> [mu ...]"""
import json, sys, time
from brixzle.agents import check_family as CF
from brixzle.engine import boxes as BX, zscore as ZS
from brixzle.engine.sim import pickup as PK

def main():
    root, module = sys.argv[1], sys.argv[2]
    mus = tuple(float(m) for m in sys.argv[3:]) or (0.5, 0.6, 0.7, 0.8)
    if root == "registry":
        from brixzle.families.registry import get
        fam = get(module)
    else:
        fam = CF.load_family(root, module)
    p, box = fam.defaults, BX.default_boxes()["placement"]["mean"]
    t = time.time()
    out = {"family": fam.name, "label": "simulated, uncalibrated contact", "box": "mean", "seed": 1, "z": {}}
    for name, fn in ZS.held_out_scores(fam, box, 1, mus=mus).items():
        c = fn(p)
        c = c.get("counts", c)
        out["z"][name] = {k: c.get(k) for k in ("Z", "S", "J", "N", "fell")}
    try:
        rel = PK.release_check(fam, p)
        out["release"] = {k: rel.get(k) for k in ("passed", "drift_mm", "outcome")}
    except Exception as e:  # noqa: BLE001 - a family the fork cannot hold
        out["release"] = {"error": f"{type(e).__name__}: {e}"[:200]}
    out["seconds"] = round(time.time() - t, 1)
    print(json.dumps(out, default=str))

if __name__ == "__main__":
    main()
