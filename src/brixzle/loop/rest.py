"""Where a failed brick rests, for the critic (issue #112).

The critic worked from text only and never saw where a jammed brick rests; its briefs asked for a
change of shape without saying which. So for a few representative failures inside the target box
(seeded: at least one jam, one wrong valley and one drop from the X-Y slice when the map has them)
the drop is run again, and the loop writes:

- two pictures of the brick at rest, from the side (X-Z) and end-on (Y-Z): ``rest-K-xz.png``,
  ``rest-K-yz.png``;
- the contacts at rest: which brick, the face or corner of the falling brick and of what it rests
  on (named by the jam analysis on a Prismatic family; a family that varies along Y gets the
  sim's raw contacts), and the normal;
- ``rest.json`` with both.

The critic may read that directory and must name the faces in contact in its brief. Simulated,
uncalibrated contact.
"""

import json
from pathlib import Path
import time

import numpy as np

from brixzle.engine import jam_analysis as JA
from brixzle.engine import zscore as ZS
from brixzle.engine.sim import trials as T
from brixzle.families import builders as B
from brixzle.loop import videos as VD

N_RESTS = 4
SIZE = (480, 360)


def pick_rests(drops, box, n=N_RESTS, seed=0, clearance0=5.0):
    """Up to ``n`` failed map drops inside ``box``, seeded: first a jam, a wrong valley and an X-Y
    drop if there are any, then the rest at random. Each is a pick as videos.pick_drops makes."""
    pool = sorted((d for d in drops if not d.get("zoom") and d["outcome"] != "seated"
                   and ZS.inside(ZS.drop_pose(d, clearance0), box)),
                  key=lambda d: (d["slice"], d["course"], tuple(d["cell"])))
    rng = np.random.default_rng(seed)
    chosen = []
    for want in (lambda d: d["outcome"] == "jammed", lambda d: d["outcome"] == "wrong_valley",
                 lambda d: d["slice"] == "xy"):
        if any(want(d) for d in chosen):
            continue
        group = [d for d in pool if want(d) and d not in chosen]
        if group and len(chosen) < n:
            chosen.append(group[int(rng.integers(len(group)))])
    rest = [d for d in pool if d not in chosen]
    if rest and len(chosen) < n:
        idx = rng.choice(len(rest), size=min(n - len(chosen), len(rest)), replace=False)
        chosen += [rest[int(i)] for i in sorted(idx)]
    out = []
    for k, d in enumerate(chosen, start=1):
        out.append({"k": k, "slice": d["slice"], "course": d["course"], "cell": list(d["cell"]), "outcome": d["outcome"],
                    "dx_mm": float(d["dx_mm"]), "dy_mm": float(d.get("dy_mm", 0.0)), "clear_mm": float(d["clear_mm"]),
                    "pitch_deg": float(d["pitch_deg"]), "pose": ZS.drop_pose(d, clearance0)})
    return out


def _contacts(family, p, course, row, mu):
    if B.varies_along_y(family):
        return [{"brick": family.part_for(course, 1), "on_brick": f"piece {c['piece']}", "on_support": c["other"],
                 "normal": [round(float(v), 3) for v in c["normal"]], "force_N": round(float(c["normal_N"]), 4)}
                for c in row["rest"]["contacts"] if c["normal_N"] > 0]
    r = JA.reduce_row(family, p, course, row, mu)
    return [{"brick": s["on_brick"].get("part"), "on_brick": s["on_brick"]["label"], "on_support": s["on_support"]["label"],
             "normal": [round(float(v), 3) for v in s["normal"]], "force_N": round(float(s["normal_N"]), 4)}
            for s in r["sites"]]


def rest_of(family, p, pick, cfg=None, record=False):
    """Drop a pick again and return its rest: outcome, rest pose and contact table (and the rig and
    recorded frames if ``record``)."""
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    rig = T.DropRig(family, p, cfg, course=pick["course"], record=record, record_fps=VD.RECORD_FPS)
    row = rig.drop(VD.drop_sample(pick, cfg), trace=False, rest=True)
    out = {"outcome": row["outcome"], "err_mm": [round(float(v), 2) for v in row["err_mm"]],
           "rest_pose": {"pos_mm": [float(v) for v in row["rest"]["pos_mm"]], "quat": [float(v) for v in row["rest"]["quat"]]},
           "contacts": _contacts(family, p, pick["course"], row, float(cfg.physics.friction))}
    if record:
        out["_rig"], out["_frames"] = rig, row["frames"]
    return out


def rest_pictures(family, p, drops, box, out_dir, n=N_RESTS, seed=0, cfg=None):
    """Pick, drop again, picture and tabulate up to ``n`` failed rests into ``out_dir``; returns
    {'dir', 'drops', 'seconds'} and writes rest.json. A rest that cannot be rendered keeps its table."""
    from PIL import Image

    t0 = time.time()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    picks = pick_rests(drops, box, n=n, seed=seed)
    for pick in picks:
        try:
            r = rest_of(family, p, pick, cfg, record=True)
        except Exception as e:  # noqa: BLE001 - a rest must not stop the loop
            pick["error"] = f"{type(e).__name__}: {e}"[:300]
            continue
        rig, frames = r.pop("_rig"), r.pop("_frames")
        pick.update(r)
        pick["pictures"] = []
        for view, az in (("xz", 90), ("yz", 0)):
            try:
                img = VD.render_frames(rig.model, frames[-1:], size=SIZE, azimuth=az,
                                       text=f"rest {pick['k']}: {pick['outcome']}, {view} ({VD.LABEL})")[0]
                name = f"rest-{pick['k']}-{view}.png"
                Image.fromarray(img).save(out_dir / name)
                pick["pictures"].append(name)
            except Exception as e:  # noqa: BLE001 - no GL context: the table still goes to the critic
                pick["render_error"] = f"{type(e).__name__}: {e}"[:300]
    result = {"dir": str(out_dir), "drops": picks, "seconds": time.time() - t0, "label": VD.LABEL}
    (out_dir / "rest.json").write_text(json.dumps(result, indent=1, allow_nan=False) + "\n", encoding="utf-8")
    return result
