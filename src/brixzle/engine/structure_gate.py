"""The structure gate (issue #83): a design must build the basic shapes before it is scored.

A tower, a running-bond wall and a bridge must each complete, and an overhang asked for
``overhang_ask`` bricks (+x) must hold at least ``overhang_floor``. ``overhang_n``, the overhang's
length, is also the loop's second objective. Builds use ideal placement (#81's ``assemble``), and
each structure is judged on seeded repeats with a tiny release jitter (``repeat_structures``): a
structure passes when most repeats complete, and ``overhang_n`` is the count most repeats reach.
The ring joins the gate in #89. Every result is simulated, uncalibrated contact.

    uv run python -m brixzle.engine.structure_gate --family ab
"""

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from brixzle.engine import structures as S
from brixzle.engine.sim import assemble as A
from brixzle.engine.sim import trials as T

LABEL = A.LABEL
NAMES = ("tower", "wall", "bridge", "overhang")


@dataclass(frozen=True)
class GateSettings:
    overhang_floor: int = 3     # the least that counts as forming an overhang (the issue's choice)
    overhang_ask: int = 12      # bricks asked of the overhang, as aerial-assembly's trial
    bridge_arms: int = 3        # courses per arm
    wall: tuple = (3, 4)        # bricks wide, courses high
    tower: int = 3              # courses
    repeats: int = A.REPEATS    # seeded builds per structure (maintainer, 2026-11-13)
    jitter_mm: float = A.JITTER_MM

    def __post_init__(self):
        ints = {"overhang_floor": self.overhang_floor, "overhang_ask": self.overhang_ask,
                "bridge_arms": self.bridge_arms, "tower": self.tower, "repeats": self.repeats}
        for k, v in ints.items():
            if not isinstance(v, int) or isinstance(v, bool) or v < 1:
                raise ValueError(f"{k} must be an integer >= 1, got {v!r}")
        if self.overhang_floor > self.overhang_ask:
            raise ValueError(f"overhang_floor ({self.overhang_floor}) is more than overhang_ask ({self.overhang_ask})")
        if (len(self.wall) != 2 or not all(isinstance(v, int) and not isinstance(v, bool) and v >= 1
                                           for v in self.wall)):
            raise ValueError(f"wall must be (bricks wide, courses high), got {self.wall!r}")
        if not (isinstance(self.jitter_mm, (int, float)) and math.isfinite(self.jitter_mm) and self.jitter_mm >= 0):
            raise ValueError(f"jitter_mm must be finite and >= 0, got {self.jitter_mm!r}")


def majority_count(counts):
    """The largest count that more than half of the repeats reach (the median of an odd number)."""
    counts = sorted(counts, reverse=True)
    return counts[len(counts) // 2]


def _structures(family, p, settings, span):
    """{name: Structure} for the structures that can be planned, and {name: message} for those that
    cannot: an unsupported span, or no clear insertion path for some brick (it would have to pass
    through a placed one), which counts as that structure failing."""
    builders = {"tower": lambda: S.tower(settings.tower, span),
                "wall": lambda: S.wall(*settings.wall, span),
                "bridge": lambda: S.bridge(settings.bridge_arms, span),
                "overhang": lambda: S.overhang(settings.overhang_ask, 1, span)}
    out, errors = {}, {}
    for name in NAMES:
        try:
            structure = builders[name]()
            A.assembly_order(family, p, structure)
            out[name] = structure
        except ValueError as e:
            errors[name] = str(e)
    return out, errors


def _summary(name, res, settings):
    """One structure's verdict, compact: no stages."""
    builds = res["builds"]
    failed = [None if b["status"] == "complete" else b["stages"][-1]["brick"] if b["stages"] else None
              for b in builds]
    out = {"passed": res["passed"], "status": Counter(res["statuses"]).most_common(1)[0][0],
           "complete": res["complete"], "n": res["n"], "statuses": res["statuses"],
           "stable_bricks": res["stable_bricks"], "total": res["total"],
           "max_sag_mm": [b["max_sag_mm"] for b in builds], "failed_brick": failed,
           "seconds": [b["seconds"] for b in builds], "error": None}
    if name == "overhang":
        n = majority_count(res["stable_bricks"])
        out.update(passed=n >= settings.overhang_floor, overhang_n=n)
    return out


def run_gate(family, p, settings: GateSettings = GateSettings(), runs="runs", seed=0, workers=None):
    """Build the gate's structures (``settings.repeats`` seeded builds each, in one process pool),
    judge them and write ``runs/gate-<UTC>-<family>/``. Returns the verdict: passed, overhang_n,
    builds {name: summary}, seconds and the run directory; 'repeat_results' holds every build (it
    is written beside gate.json, not in it)."""
    t0 = time.time()
    span = family.lattice(p).span_voxels
    structures, errors = _structures(family, p, settings, span)
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    results = {}
    if structures:
        by_name = A.repeat_structures(family, p, list(structures.values()), cfg, n=settings.repeats, seed=seed,
                                      jitter_mm=settings.jitter_mm, workers=workers)
        results = {name: by_name[s.name] for name, s in structures.items()}
    builds = {}
    for name in NAMES:
        if name in errors:
            builds[name] = {"passed": False, "status": "error", "error": errors[name], "complete": 0,
                            "n": 0, "statuses": [], "stable_bricks": [], "total": None, "max_sag_mm": [],
                            "failed_brick": [], "seconds": []}
            if name == "overhang":
                builds[name]["overhang_n"] = 0
        else:
            builds[name] = _summary(name, results[name], settings)
    overhang_n = builds["overhang"]["overhang_n"]
    out = {"passed": all(b["passed"] for b in builds.values()), "overhang_n": overhang_n,
           "overhang_floor": settings.overhang_floor, "builds": builds, "family": family.name,
           "params": p.to_dict(), "settings": asdict(settings), "repeats": settings.repeats, "seed": seed,
           "config": T.as_dict(cfg), "seconds": time.time() - t0, "label": LABEL}
    run = write_gate(out, results, runs)
    return {**out, "run": str(run), "repeat_results": results}


def write_gate(result, repeat_results, runs="runs"):
    """Write ``runs/gate-<UTC>-<family>/gate.json`` (the verdict) and one ``builds-<name>.json`` per
    structure (every repeat, in full); never reuses a directory. Finite numbers only."""
    text = json.dumps(A._jsonable({**result, "label": LABEL}), indent=1, allow_nan=False) + "\n"
    root = Path(runs)
    while True:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        out = root / f"gate-{stamp}-{result['family']}"
        try:
            out.mkdir(parents=True, exist_ok=False)
            break
        except FileExistsError:
            continue
    (out / "gate.json").write_text(text, encoding="utf-8")
    for name, res in repeat_results.items():
        (out / f"builds-{name}.json").write_text(
            json.dumps(A._jsonable({**res, "label": LABEL}), indent=1, allow_nan=False) + "\n", encoding="utf-8")
    return out


def failures_for_agent(gate):
    """The gate's result for the design agent, short: each structure's verdict, and for each that
    failed, where its builds stopped."""
    rows, failed = [], []
    for name, b in gate["builds"].items():
        row = {"structure": name, "passed": b["passed"], "complete": f"{b['complete']} of {b['n']}",
               "stable_bricks": b["stable_bricks"], "total": b["total"], "statuses": b["statuses"]}
        if name == "overhang":
            row["overhang_n"] = b["overhang_n"]
        rows.append(row)
        if not b["passed"]:
            stops = [x for x in b["failed_brick"] if x]
            failed.append({"structure": name, "error": b["error"], "statuses": b["statuses"],
                           "failed_brick": stops[0] if stops else None, "failed_bricks": b["failed_brick"],
                           "max_sag_mm": [round(v, 2) for v in b["max_sag_mm"]]})
    return {"passed": gate["passed"], "overhang_n": gate["overhang_n"], "overhang_floor": gate["overhang_floor"],
            "structures": rows, "failed": failed, "run": gate.get("run"), "label": LABEL}


def failures_text(gate):
    """One line per structure, for logs and the terminal."""
    out = []
    for name, b in gate["builds"].items():
        if b["error"]:
            out.append(f"{name}: error, {b['error']}")
            continue
        verdict = "passed" if b["passed"] else "FAILED"
        extra = f", overhang_n {b['overhang_n']} (floor {gate['overhang_floor']})" if name == "overhang" else ""
        stops = sorted({x for x in b["failed_brick"] if x})
        out.append(f"{name}: {verdict}, {b['complete']} of {b['n']} complete, stable {b['stable_bricks']} of "
                   f"{b['total']}{extra}" + (f", stopped at {', '.join(stops)}" if stops else ""))
    return "\n".join(out)


def main(argv=None):
    from brixzle.families import registry

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--family", default="ab")
    ap.add_argument("--repeats", type=int, default=A.REPEATS)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--runs", default="runs")
    args = ap.parse_args(argv)
    fam = registry.get(args.family)
    g = run_gate(fam, fam.defaults, GateSettings(repeats=args.repeats), runs=args.runs, seed=args.seed)
    print(failures_text(g))
    print(f"gate {'passed' if g['passed'] else 'FAILED'}, overhang_n {g['overhang_n']}, {g['seconds']:.1f} s "
          f"({LABEL}) -> {g['run']}")


if __name__ == "__main__":
    main()
