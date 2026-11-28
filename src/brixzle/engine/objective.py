# The noisy corbel (radius_error, noisy_corbel, build_record) is from LunarSphere/Brixzle-RSI
# src/brixzle/engine/edge_capture.py @ a6a4152 (local branch 145-edge-capture, closed issue #148).
# Changed: the error is a disc of any radius (not the mean box only), there is no edge-stage comparison,
# and the steps are summarised per radius (#164).
"""The noisy corbel (#164, #206): a back-first corbel built brick by brick, every brick released with
error inside a disc about its supporter's measured pose. ``radius.noisy_builds`` runs it for finalists
at 16 mm (#210, #222). The rest of #164's build-aware objective (grip, size, the arm's reach, the
error-tolerant overhang score and its Pareto ranking) was removed in #227 with the SO-101 arm and the
#164 goal; the tag ``pre-debloat-2026-11-28`` has it. Results are simulated, uncalibrated contact.
"""
import math
import re
import time

import numpy as np

LABEL = "simulated, uncalibrated contact"


# --- the error-tolerant overhang ---


def radius_error(radius_mm, n=64):
    """Placement error spread evenly over a disc of ``radius_mm`` and the mean box's release heights,
    aimed at the supporter's measured pose. Angles and residual speed are zero."""
    from brixzle.engine import boxes as BX
    from brixzle.engine.sim import trials as T

    if not (math.isfinite(radius_mm) and radius_mm >= 0):
        raise ValueError(f"radius_mm must be finite and >= 0, got {radius_mm!r}")
    box = BX.default_boxes()["placement"]["mean"]
    c0 = T.ErrorModel().clearance[0]
    radii = (0.0,) if radius_mm == 0 else tuple(radius_mm * math.sqrt((k + 0.5) / n) for k in range(n))
    return T.ErrorModel(radial_mode="bootstrap", radii_mm=radii, sigma_yaw=0.0, sigma_tilt=0.0, sigma_vel=0.0,
                        clearance=(c0, c0 + 2 * box.half_widths["z_mm"]), track_supporter=True)


def _placement(brick):
    course, voxel = re.fullmatch(r"b(\d+)_(-?\d+)m?", brick).groups()
    return int(course), int(voxel)


def build_record(build):
    """One noisy corbel: the reaching steps seated from course 0 up, and where it stopped."""
    stages = build["stages"]
    good = {st["brick"] for st in stages if st["outcome"] == "seated" and not st["collapsed"]}
    steps = 0
    while f"b{steps}_{steps}" in good:
        steps += 1
    failed = stages[-1]["brick"] if build["status"] != "complete" and stages else None
    kind = None
    if failed is not None:
        c, v = _placement(failed)
        kind = "reach" if c >= 1 and v == c else "back" if v == c - 2 else "first"
    last = stages[-1] if failed is not None else {}
    return {"status": build["status"], "steps": steps, "failed_brick": failed, "failed_kind": kind,
            "failed_outcome": last.get("outcome"),
            # the failed brick's release (error sample) and settled error, for replaying it (#210)
            "failed_sample": last.get("sample"), "failed_err_mm": last.get("err_mm"),
            "seconds": float(build["seconds"])}


def noisy_corbel(family, p, radius_mm, builds=17, steps=6, seed=0, workers=None, held=False):
    """The corbel built ``builds`` times with error inside a disc of ``radius_mm`` (seeds seed..);
    ``held``: every brick placed by held release (trials.TrialConfig.held, #206)."""
    from brixzle.engine import structures as S
    from brixzle.engine.sim import assemble as A
    from brixzle.engine.sim import trials as T

    t0 = time.time()
    cfg = T.TrialConfig(error=radius_error(radius_mm), held=held)
    res = A.repeat_builds(family, p, S.corbel(steps, family.lattice(p).span_voxels), cfg, n=builds, seed=seed,
                          jitter_mm=0.0, workers=workers)
    recs = [build_record(b) for b in res["builds"]]
    return {"radius_mm": radius_mm, "builds": recs, "seeds": res["seeds"],
            "median_steps": float(np.median([b["steps"] for b in recs])),
            "complete": sum(b["status"] == "complete" for b in recs),
            "failed_at": {k: sum(b["failed_kind"] == k for b in recs) for k in ("first", "back", "reach")},
            "seconds": round(time.time() - t0, 1)}


