"""Z of a family at another map's drop poses, per placement box: for a brick whose own map is coarse.

The sim map's cells scale with the brick's pitch, so a long brick's boxes hold few cells (fable_wide:
60 / 128 / 196 drops against twinhook4's 180 / 592 / 1100). This drops the family at every pose of a
reference map that lies inside each box, as the loop's map would score them. Simulated, uncalibrated
contact. Usage: fixed_poses.py <root> <module> <reference map dir> <out.json>
"""

import json
from pathlib import Path
import sys
import time

from brixzle.agents import check_family as CF
from brixzle.engine import boxes as BX, probe as PB, sim_map as SM, zscore as ZS
from brixzle.engine.sim import trials as T


def main():
    root, module, ref, dest = sys.argv[1:5]
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    c0 = cfg.error.clearance[0]
    drops = [d for d in SM.read_drops(ref) if not d.get("zoom")]
    boxes = BX.default_boxes()["placement"]
    fam = CF.load_family(root, module)
    t0 = time.time()
    out = {"label": "simulated, uncalibrated contact", "family": fam.name, "reference_map": ref, "boxes": {}}
    for name in ("mean", "p95", "max"):
        inside = [d for d in drops if ZS.inside(ZS.drop_pose(d, c0), boxes[name])]
        poses = [PB._pose_entry(d["outcome"], d["course"], PB._map_sample(d, cfg)) for d in inside]
        rows = PB.run_probe(fam, fam.defaults, poses, cfg, workers=8)["rows"]
        n = len(rows)
        c = {k: sum(1 for r in rows if r["outcome"] == k) for k in ("seated", "jammed", "wrong_valley", "fell")}
        out["boxes"][name] = {"N": n, **c, "Z": (c["seated"] - c["jammed"]) / n if n else None}
        print(name, out["boxes"][name], flush=True)
    out["seconds"] = round(time.time() - t0, 1)
    Path(dest).write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
