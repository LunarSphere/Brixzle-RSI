"""The verifier (#165): every number a designer claims is rerun, not taken on trust.

A claim (``claim.json``) lists numbers as {what, path, value}, where ``path`` names the number in the
verifier's own measurement (dotted keys, for example ``z.mean.Z`` or ``gate.overhang_n``). ``measure``
evaluates the candidate itself, with the tools and the objective (the drop cache only reads drops
the engine simulated for the same geometry), and ``compare`` checks each claim against it. A
number the measurement does not have is a mismatch.
"""
import math
from pathlib import Path
import time


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


def measure(spec, out_dir, stages=("check", "quick", "map", "held_out", "gate", "ring"), objective=True, seed=0):
    """The candidate's evaluation (tools.core.evaluate) and, with ``objective``, the #164 terms. Z at
    mu 0.8 comes from tools.core.friction (seed 1, mean box)."""
    from brixzle.engine import objective as OB
    from brixzle.tools import core as TC

    t0 = time.time()
    ev = TC.evaluate(spec, Path(out_dir) / "eval", stages=stages, seed=seed)
    out = {"eval": str(Path(out_dir) / "eval"), "z": ev.get("z"), "gate": ev.get("gate"), "ring": ev.get("ring"),
           "held_out": ev.get("held_out"), "check_family": ev.get("check_family"), "mass_g": ev.get("mass_g")}
    if objective:
        fam = TC.load(spec)
        fr = TC.friction(fam, fam.defaults, mus=(0.8,))
        mu08 = next(iter(fr["z"].values()))
        rec = OB.evaluate_design(fam, z={k: v["Z"] for k, v in (ev.get("z") or {}).items()}, z_mu08=mu08["Z"],
                                 overhang_n=(ev.get("gate") or {}).get("overhang_n"))
        rec["z_counts"] = {k: (ev.get("z") or {}).get("mean", {}).get(k) for k in ("S", "J", "N")}
        rec["z_mu08_counts"] = {"S": mu08["S"], "J": mu08["J"], "N": mu08["N"]}
        out["objective"] = rec
        out["tolerance_mm"] = rec["tolerance_mm"]
        out["violations"] = rec["violations"]
    out["seconds"] = round(time.time() - t0, 1)
    return out
