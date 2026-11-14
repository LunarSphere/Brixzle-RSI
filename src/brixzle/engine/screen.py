"""Cheap screen of candidate designs (issue #108): which are worth a full map.

A full score (gate, map, jam analysis, held-out, ring) costs about 3 to 5 minutes per design, so
an explore round of several concepts (#106) cannot map them all inside the run's 30 minutes. The
screen scores each candidate from two cheap checks:

- the quick check as the loop runs it (40 drops along X, issue #97), which is also the candidate's
  pass/fail filter;
- a small seeded X-Y check on the target box's disc (20 drops, ``quick_check(disc=True)``), since
  most of A/B's loss is in Y.

The score is (seated - jammed) / drops over both: the form of Z.

**What it can tell (checked before trusting it, #108).** On A/B and the four designs of
``ab4iter2-20261114-mean`` the screen did not rank them as their maps did (``runs/screen-compare-
20261113.json``): those five lie within 0.1 of each other in Z, and the screen's standard error at
60 drops is about 0.08. It did put the yfunnel fixture (Z -0.169) far below them. So the screen does
not pick a winner among close candidates: ``select`` drops only a clear loser, one whose score is
more than two standard errors of the difference below the best, and maps the rest by rank up to a
cap. Results are simulated, uncalibrated contact.

    uv run python -m brixzle.engine.screen --root <root> --module <module> --target mean
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import subprocess
import sys
import time

from brixzle.engine import boxes as BX
from brixzle.engine import quick_check as QC
from brixzle.engine import zscore as ZS

N_XY = 20
CLEAR_LOSER_SE = 2.0     # screened out only below the best by more than this many standard errors
SCREEN_TIMEOUT_S = 180
LABEL = "simulated, uncalibrated contact"


def _counts(outcomes, n):
    s, j = int(outcomes.get("seated", 0)), int(outcomes.get("jammed", 0))
    return {"Z": (s - j) / n, "S": s, "J": j, "N": int(n)}


def combine(x, xy):
    """The screen score from the X check and the X-Y check (each with 'outcomes' and 'dropped')."""
    cx, cxy = _counts(x["outcomes"], x["dropped"]), _counts(xy["outcomes"], xy["dropped"])
    s, j, n = cx["S"] + cxy["S"], cx["J"] + cxy["J"], cx["N"] + cxy["N"]
    return {"Z": (s - j) / n, "S": s, "J": j, "N": n, "x": cx, "xy": cxy}


def screen(family, p, box, seed=0, n_x=QC.N_DROPS, n_xy=N_XY, check=None):
    """The quick check (or ``check`` if it already ran) and the X-Y check, and their combined score.
    'check' in the result is the quick check's verdict, so the screen is the candidate's filter too."""
    t0 = time.time()
    need = QC.NEED_SEATED if n_x == QC.N_DROPS else math.ceil(0.8 * n_x)
    x = check or QC.quick_check(family, p, box, n=n_x, need=need, seed=seed)
    xy = QC.quick_check(family, p, box, n=n_xy, need=0, seed=seed, disc=True)
    out = combine(x, xy)
    out["check"] = {"passed": bool(x["passed"]), "seated": x["seated"], "dropped": x["dropped"], "outcomes": x["outcomes"],
                    "need": x.get("need", need)}
    out["check_full"] = x  # the whole quick check, for the full map's ranges (#106)
    out["label"] = LABEL
    out["seconds"] = round(time.time() - t0, 2)
    return out


def _band(c):
    return ZS.z_band(c) if c.get("N") else float("inf")


def select(scores, max_maps):
    """(mapped ids best first, {screened-out id: reason}) from {id: screen counts}. A candidate is
    screened out if its score is below the best's by more than CLEAR_LOSER_SE standard errors of
    the difference; of the rest, the best ``max_maps`` by score are mapped."""
    if isinstance(max_maps, bool) or not isinstance(max_maps, int) or max_maps < 1:
        raise ValueError(f"max_maps must be an integer >= 1, got {max_maps!r}")
    order = sorted(scores, key=lambda k: (-scores[k]["Z"], k))
    top = scores[order[0]]
    mapped, out = [], {}
    for k in order:
        c = scores[k]
        se = math.hypot(_band(c), _band(top))
        gap = top["Z"] - c["Z"]
        if k != order[0] and gap > CLEAR_LOSER_SE * se:
            out[k] = (f"screen {c['Z']:+.3f} is below the best's {top['Z']:+.3f} by {gap:.3f}, more than "
                      f"{CLEAR_LOSER_SE:g} standard errors ({CLEAR_LOSER_SE * se:.3f})")
        elif len(mapped) >= max_maps:
            out[k] = f"screen {c['Z']:+.3f}; inside the noise of the best, but the cap of {max_maps} maps was reached"
        else:
            mapped.append(k)
    return mapped, out


def screen_in_subprocess(root, module, target, seed=0, n_x=QC.N_DROPS, n_xy=N_XY, timeout=SCREEN_TIMEOUT_S):
    """``screen`` of the package ``<root>/<module>`` in a fresh process; a crash or a hang is reported."""
    cmd = [sys.executable, "-m", "brixzle.engine.screen", "--root", str(root), "--module", module, "--target", target,
           "--seed", str(seed), "--n-x", str(n_x), "--n-xy", str(n_xy)]
    t0 = time.time()
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"error": f"the screen took longer than {timeout} s", "seconds": round(time.time() - t0, 2)}
    try:
        return json.loads(done.stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError):
        tail = (done.stderr or "").strip().splitlines()[-5:]
        return {"error": f"the screen crashed (exit {done.returncode}): {' | '.join(tail)}",
                "seconds": round(time.time() - t0, 2)}


def screen_many(jobs, target, seed=0, n_x=QC.N_DROPS, n_xy=N_XY, workers=3):
    """{id: screen} for ``jobs`` [(id, root, module)], each in its own process, ``workers`` at a time."""
    with ThreadPoolExecutor(max(1, min(workers, len(jobs)))) as ex:
        futs = {k: ex.submit(screen_in_subprocess, root, module, target, seed, n_x, n_xy) for k, root, module in jobs}
        return {k: f.result() for k, f in futs.items()}


def main(argv=None):
    from brixzle.agents import check_family as CF

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--root", required=True)
    ap.add_argument("--module", required=True)
    ap.add_argument("--target", choices=BX.STATISTICS, default="mean")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n-x", type=int, default=QC.N_DROPS)
    ap.add_argument("--n-xy", type=int, default=N_XY)
    args = ap.parse_args(argv)
    try:
        fam = CF.load_family(args.root, args.module)
        out = screen(fam, fam.defaults, BX.default_boxes()["placement"][args.target], seed=args.seed,
                     n_x=args.n_x, n_xy=args.n_xy)
    except Exception as e:  # noqa: BLE001 - reported to the loop
        out = {"error": f"{type(e).__name__}: {e}"[:500]}
    print(json.dumps(out, allow_nan=False))


if __name__ == "__main__":
    main()
