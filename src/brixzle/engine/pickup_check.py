"""Cheap pickup check on the run's final design (issue #113): a report, not a gate.

The loop scores placement only (R11); pickup (R12) waits for the gripper (#24, #46). Once the loop
makes new concepts it could keep a brick the fork cannot enter, lift level or release, and a brick
that places well but cannot be picked up is not a result. So the final design (after the end tune)
gets:

- **R10**: the shared rule margins that start with ``R10.``: two large through-Y openings above
  and either side of the centre of mass, and brick plus gripper within 35 g (the 5 g gripper
  allowance is a guess until #24). The smallest over the parts.
- **The fork**: today's fixed fork (engine/sim/pickup.py), a stand-in, not the gripper: one pick
  with ideal alignment and ``n_offsets`` seeded picks inside the mean pickup box, then the clean
  release check. Only for a Prismatic family with two usable openings; otherwise the report says
  plainly why it was not checked.

Whether a failed pickup rejects a design is the maintainer's call once there are results (#113).
Results are simulated, uncalibrated contact. Written to ``runs/pickup-<UTC>-<family>/pickup.json``.
"""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import numpy as np

from brixzle.engine import box_search as BS
from brixzle.engine import boxes as BX
from brixzle.families import builders as B
from brixzle.families import rules as R
from brixzle.families.interface import Prismatic

LABEL = "simulated, uncalibrated contact"
N_OFFSETS = 6


def r10_margins(family, p):
    """The smallest of each R10 margin over the family's parts."""
    out = {}
    for part in family.parts:
        for k, v in R.margins(family, p, part, B.sim_bodies(family, p, part)).items():
            if k.startswith("R10."):
                out[k] = min(out.get(k, math.inf), float(v))
    return {k: (v if math.isfinite(v) else None) for k, v in out.items()}


def _num(v):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return v
    return v if math.isfinite(v) else None


def _clean(obj):
    if isinstance(obj, dict):
        return {k: _clean(v) for k, v in obj.items() if k not in ("contacts", "trace")}
    if isinstance(obj, (list, tuple)):
        return [_clean(v) for v in obj]
    if isinstance(obj, (int, float)) and not isinstance(obj, bool):
        return _num(obj)
    return obj


def pickup_check(family, p, seed=0, n_offsets=N_OFFSETS, runs="runs"):
    """The R10 margins and, where it applies, fork picks and the release check; writes pickup.json."""
    from brixzle.engine.sim import pickup as PK

    t0 = time.time()
    out = {"label": LABEL, "family": family.name, "params": p.to_dict(), "seed": seed, "gate": False,
           "r10": r10_margins(family, p), "checked": False, "reason": None, "picks": [], "release": None}
    if not isinstance(family.geometry, Prismatic):
        out["reason"] = ("not checked: the pickup trial needs a Prismatic family (engine/sim/pickup.py); this family "
                         "varies along Y (Solid3D)")
    elif out["r10"].get("R10.two_openings") is None or out["r10"]["R10.two_openings"] < 0:
        out["reason"] = "not checked: the brick has no two through-Y openings a 5 mm circle fits in (R10)"
    else:
        box = BX.default_boxes()["pickup"]["mean"]
        rng = np.random.default_rng(seed)
        samples = [{"dx": 0.0, "dz": 0.0, "psi": 0.0}] + [BS.pose_sample(box, 1.0, rng.uniform(-1, 1, 3), None)
                                                          for _ in range(n_offsets)]
        rig = PK.PickRig(family, p)
        for s in samples:
            row = rig.pick(s, trace=False)
            out["picks"].append(_clean({"sample": s, "outcome": row["outcome"], "carry_err_mm": row.get("carry_err_mm"),
                                        "carry_angle_deg": row.get("carry_angle_deg")}))
        rel = PK.release_check(family, p)
        out["release"] = _clean({k: v for k, v in rel.items() if k in ("passed", "drift_mm", "outcome")})
        out["checked"] = True
        out["box"] = box.to_dict()
    out["picked"] = sum(1 for q in out["picks"] if q["outcome"] == "picked")
    out["n"] = len(out["picks"])
    out["seconds"] = time.time() - t0
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    run = Path(runs) / f"pickup-{stamp}-{family.name}"
    run.mkdir(parents=True, exist_ok=False)
    out["run"] = str(run)
    (run / "pickup.json").write_text(json.dumps(_clean(out), indent=1, allow_nan=False) + "\n", encoding="utf-8")
    return out


def summary_lines(pk):
    """The run summary's pickup section."""
    if not pk:
        return []
    lines = ["## Pickup (#113: a report, not a gate)", "",
             f"Final design, with today's fixed fork as a stand-in for the gripper (#24); {LABEL}.", ""]
    r10 = pk.get("r10") or {}
    lines.append("R10 margins (>= 0 passes): " + (", ".join(f"{k} {v:+.2f}" if v is not None else f"{k} n/a"
                                                          for k, v in sorted(r10.items())) or "none") + ".")
    if pk.get("error"):
        lines.append(f"The check raised: {pk['error']}")
    elif not pk.get("checked"):
        lines.append(f"Fork: {pk.get('reason')}.")
    else:
        rel = pk.get("release") or {}
        lines.append(f"Fork: {pk['picked']} of {pk['n']} picks picked (the first at ideal alignment: "
                     f"{pk['picks'][0]['outcome']}); release {'clean' if rel.get('passed') else 'NOT clean'}"
                     + (f", drift {rel['drift_mm']:.2f} mm" if rel.get("drift_mm") is not None else "") + ".")
    lines += [f"Run: `{pk.get('run')}`; {pk.get('seconds', 0.0):.1f} s.", ""]
    return lines
