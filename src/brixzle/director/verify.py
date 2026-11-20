"""The verifier (#165): every number a designer claims is rerun, not taken on trust.

A claim (``claim.json``) lists numbers as {what, path, value}, where ``path`` names the number in the
verifier's own measurement (dotted keys, for example ``z.mean.Z`` or ``gate.overhang_n``). ``measure``
evaluates the candidate itself, with the tools and the objective (the drop cache only reads drops
the engine simulated for the same geometry), and ``compare`` checks each claim against it. A
number the measurement does not have is a mismatch.
"""
import json
import math
from pathlib import Path
import time

from brixzle.director.run import write_json


def lookup(obj, path):
    for key in path.split("."):
        if isinstance(obj, dict) and key in obj:
            obj = obj[key]
        else:
            return None
    return obj


def _same(a, b, tol):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        return math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=tol)
    return a == b


def compare(claim, measured, tol=1e-6):
    rows = []
    for n in claim.get("numbers", []):
        m = lookup(measured, n["path"])
        rows.append({**n, "measured": m, "match": m is not None and _same(n["value"], m, tol)})
    bad = sum(not r["match"] for r in rows)
    return {"ok": bad == 0, "mismatches": bad, "numbers": rows}


def _with_violations(measured, stages):
    """Combine objective violations with verdicts for the requested feasibility stages."""
    violations = list((measured.get("objective") or {}).get("violations") or [])
    if "check" in stages and (measured.get("check_family") or {}).get("ok") is not True:
        violations.append("check_family")
    if "gate" in stages and (measured.get("gate") or {}).get("passed") is not True:
        violations.append("structure_gate")
    if "ring" in stages and (measured.get("ring") or {}).get("status") != "complete":
        violations.append("ring")
    return {**measured, "violations": violations}


def measure(spec, out_dir, stages=("check", "quick", "map", "held_out", "gate", "ring"), objective=True, seed=0):
    """The candidate's evaluation (tools.core.evaluate) and, with ``objective``, the #164 terms. Z at
    mu 0.8 comes from tools.core.friction (seed 1, mean box).

    Resumable: a foreground step is cut off after 10 minutes, so each part is saved as it finishes
    (``eval/eval.json``, ``objective.json``, then ``measured.json``) and a second call reuses what is
    there. An evaluation cut off part-way is redone in a new directory (``eval-2``, ...); the drop and
    stage caches make the redone stages fast."""
    from brixzle.engine import objective as OB
    from brixzle.tools import core as TC

    out_dir = Path(out_dir)
    done = out_dir / "measured.json"
    if done.exists():
        return _with_violations(json.loads(done.read_text()), stages)
    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    ev = None
    for k in range(1, 10):
        d = out_dir / ("eval" if k == 1 else f"eval-{k}")
        if not d.exists():
            ev = TC.evaluate(spec, d, stages=stages, seed=seed)
            break
        rec = json.loads((d / "eval.json").read_text()) if (d / "eval.json").exists() else {}
        if rec.get("complete"):
            ev = rec
            break
    if ev is None:
        raise RuntimeError(f"no usable evaluation under {out_dir}")
    out = {"eval": str(d), "z": ev.get("z"), "gate": ev.get("gate"), "ring": ev.get("ring"),
           "held_out": ev.get("held_out"), "check_family": ev.get("check_family"), "mass_g": ev.get("mass_g")}
    if objective:
        obj_path = out_dir / "objective.json"
        if obj_path.exists():
            rec = json.loads(obj_path.read_text())
        else:
            fam = TC.load(spec)
            fr = TC.friction(fam, fam.defaults, mus=(0.8,))
            mu08 = next(iter(fr["z"].values()))
            rec = OB.evaluate_design(fam, z={k: v["Z"] for k, v in (ev.get("z") or {}).items()}, z_mu08=mu08["Z"],
                                     overhang_n=(ev.get("gate") or {}).get("overhang_n"))
            rec["z_counts"] = {k: (ev.get("z") or {}).get("mean", {}).get(k) for k in ("S", "J", "N")}
            rec["z_mu08_counts"] = {"S": mu08["S"], "J": mu08["J"], "N": mu08["N"]}
            write_json(obj_path, TC._finite(rec))
        out["objective"] = rec
        out["overhang_mean_steps"] = rec.get("overhang_mean_steps")
        out["tolerance_mm"] = rec.get("tolerance_mm")
    out["seconds"] = round(time.time() - t0, 1)
    out = _with_violations(out, stages)
    write_json(done, TC._finite(out))
    return out
