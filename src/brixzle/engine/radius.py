"""The error-radius score (#206): how far off the drone may release a brick and still have it seat.

The drone's placement error is 16 to 40 mm (Exp 3 and the maintainer's range). For a radius r,
each placement type is dropped ``drops`` times at error spread evenly over a disc of radius r about
its aim (a Vogel spiral, so the points cover the disc without clumping) and over the mean box's
release heights. A brick passes r when every placement type seats at least ``level`` (90 %) of
its drops; a jam, a wrong valley and a fall all count against it.

Placement types (``PLACEMENTS``), all on the drop rigs the rest of the engine uses:

- ``full``: a brick across two seated bricks (running bond, course 1): stacking;
- ``plus_bare`` and ``minus_bare``: a brick reaching one voxel past a bare edge, each way;
- ``corbel_reach``: the first reaching brick of a back-first corbel (``plus_anchored``).

``radius_search`` goes up the radii and stops one past the first that fails. ``r90_mm`` is where
the weakest placement's seat rate crosses ``level``, interpolated between the last radius that
passed and the first that failed (the seat rate is taken as 1 at r = 0), so a brick that fails
the smallest radius still gets a score that can improve. ``noisy_builds`` then builds a tower and
a back-first corbel brick by brick at that radius, until they fail: the stack height and the
overhang length the brick reaches under its own tolerated error.

All results are simulated, uncalibrated contact.
"""
from dataclasses import asdict, dataclass
import math
import time

import numpy as np

LABEL = "simulated, uncalibrated contact"
PLACEMENTS = {"full": 1, "plus_bare": "plus_bare", "minus_bare": "minus_bare", "corbel_reach": "plus_anchored"}
GOLDEN_ANGLE = math.pi * (3.0 - math.sqrt(5.0))


@dataclass(frozen=True)
class RadiusSettings:
    radii_mm: tuple = (8.0, 12.0, 16.0, 22.0, 28.0, 34.0, 40.0)
    drops: int = 30          # per placement and radius: 0 failures in 30 bounds the failure rate near 10 % (rule of three)
    level: float = 0.9       # the seat rate a radius needs on every placement
    floor_mm: float = 16.0   # the least error a brick must tolerate to be worth printing (maintainer, 2026-11-26)
    max_mm: float = 40.0     # the largest error the score credits
    placements: tuple = tuple(PLACEMENTS)
    seed: int = 0
    held: bool = True        # held release (trials.TrialConfig.held); False: the free drop

    def __post_init__(self):
        if not self.radii_mm or any(not (math.isfinite(r) and r > 0) for r in self.radii_mm):
            raise ValueError(f"radii_mm must be finite and > 0, got {self.radii_mm!r}")
        if list(self.radii_mm) != sorted(self.radii_mm):
            raise ValueError("radii_mm must be in increasing order")
        if not isinstance(self.drops, int) or isinstance(self.drops, bool) or self.drops < 1:
            raise ValueError(f"drops must be an integer >= 1, got {self.drops!r}")
        if not 0.0 < self.level <= 1.0:
            raise ValueError(f"level must be in (0, 1], got {self.level!r}")
        unknown = set(self.placements) - set(PLACEMENTS)
        if unknown or not self.placements:
            raise ValueError(f"placements must be some of {tuple(PLACEMENTS)}, got {self.placements!r}")


def disc_points(radius_mm, n, seed=0):
    """``n`` (x, y) offsets in mm spread evenly over a disc of ``radius_mm``, rotated by a seeded
    angle, and a release-height fraction in [0, 1) for each, stratified the same way."""
    if not (math.isfinite(radius_mm) and radius_mm >= 0):
        raise ValueError(f"radius_mm must be finite and >= 0, got {radius_mm!r}")
    rng = np.random.default_rng(seed)
    turn, shift = float(rng.uniform(0, 2 * math.pi)), float(rng.uniform(0, 1))
    out = []
    for k in range(n):
        r = radius_mm * math.sqrt((k + 0.5) / n)
        a = turn + k * GOLDEN_ANGLE
        out.append((r * math.cos(a), r * math.sin(a), (shift + k * 0.6180339887) % 1.0))
    return out


def _ideal_cfg(held=True):
    from brixzle.engine.sim import trials as T
    return T.TrialConfig(error=T.ErrorModel(ideal=True), held=held)


def _jobs(cfg, placement, radius_mm, n, seed):
    from brixzle.engine import boxes as BX
    from brixzle.engine import zscore as ZS

    span_z = 2 * BX.default_boxes()["placement"]["mean"].half_widths["z_mm"]
    jobs = []
    for x, y, h in disc_points(radius_mm, n, seed):
        sample = ZS._sample(cfg, {"x": x, "y": y, "z": h * span_z, "t": 0.0})
        jobs.append((PLACEMENTS[placement], sample, {"trace": False}))
    return jobs


def _counts(outcomes):
    c = {"seated": 0, "jammed": 0, "wrong_valley": 0, "fell": 0, "other": 0}
    for o in outcomes:
        c[o if o in c else "other"] += 1
    n = len(outcomes)
    c["N"] = n
    c["rate"] = c["seated"] / n if n else 0.0
    return c


_COURSE = {"full": 1, "plus_bare": 1, "minus_bare": 1, "corbel_reach": 1}


def seat_rates(pool, cfg, radius_mm, settings: RadiusSettings, drops=None):
    """{placement: counts with 'rate'} at one radius, every placement's drops in one batch. Each drop
    is appended to ``drops`` (if given) in the form ``tools contacts`` and ``tools redrop`` read:
    ``slice`` is the placement, ``edge_case`` the rig's layout."""
    jobs, owner = [], []
    for k, placement in enumerate(settings.placements):
        batch = _jobs(cfg, placement, radius_mm, settings.drops, settings.seed * 1000 + k)
        jobs += batch
        owner += [placement] * len(batch)
    rows = list(pool.imap(jobs))
    if drops is not None:
        for (job, sample, _), r, placement in zip(jobs, rows, owner):
            drops.append({"slice": placement, "edge_case": job, "course": _COURSE[placement], "radius_mm": radius_mm,
                          "dx_mm": round(sample["dx"] - sample["aim"], 3), "dy_mm": round(sample["dy"], 3),
                          "clear_mm": sample["clear"], "pitch_deg": sample["pitch"], "sample": sample,
                          "outcome": r["outcome"], "err_mm": [round(float(v), 3) for v in r.get("err_mm", [])],
                          "angle_deg": round(float(r.get("angle_deg", 0.0)), 2)})
    out = {}
    for placement in settings.placements:
        out[placement] = _counts([r["outcome"] for r, o in zip(rows, owner) if o == placement])
    return out


def r90(curve, level=0.9, max_mm=40.0):
    """Where the weakest placement's seat rate crosses ``level``: ``curve`` is [(radius, min rate)]
    in increasing radius. The rate is taken as 1 at r = 0."""
    last_r, last_rate = 0.0, 1.0
    for r, rate in curve:
        if rate < level:
            if last_rate <= rate:  # cannot happen with a pass then a fail; kept safe
                return last_r
            return last_r + (r - last_r) * (last_rate - level) / (last_rate - rate)
        last_r, last_rate = r, rate
    return min(last_r, max_mm)


def radius_search(family, p, settings: RadiusSettings = RadiusSettings(), workers=None):
    """Seat rates going up the radii, stopping at the first radius that fails. Returns r90_mm, whether
    it reaches the floor, the weakest placement, and the counts at every radius."""
    from brixzle.engine import box_search as BS

    t0 = time.time()
    cfg = _ideal_cfg(settings.held)
    span = family.lattice(p).span_voxels
    if span != 2:
        return {"label": LABEL, "error": f"the placements are laid out for a two-voxel brick; span_voxels = {span}",
                "r90_mm": 0.0, "floor_passed": False, "settings": asdict(settings), "by_radius": {},
                "seconds": round(time.time() - t0, 1)}
    by_radius, curve, drops = {}, [], []
    with BS.DropPool(family, p, cfg, workers=workers) as pool:
        for r in settings.radii_mm:
            rates = seat_rates(pool, cfg, r, settings, drops)
            worst = min(rates, key=lambda k: rates[k]["rate"])
            by_radius[str(r)] = {"placements": rates, "min_rate": rates[worst]["rate"], "weakest": worst}
            curve.append((r, rates[worst]["rate"]))
            if rates[worst]["rate"] < settings.level:
                break
        cache = dict(getattr(pool, "cache_stats", {}) or {})
    score = round(r90(curve, settings.level, settings.max_mm), 2)
    first_fail = next((k for k, v in by_radius.items() if v["min_rate"] < settings.level), None)
    return {"label": LABEL, "r90_mm": score, "floor_passed": score >= settings.floor_mm,
            "weakest": by_radius[first_fail]["weakest"] if first_fail else None,
            "first_failing_radius_mm": float(first_fail) if first_fail else None,
            "curve": [{"radius_mm": r, "min_rate": rate} for r, rate in curve],
            "by_radius": by_radius, "settings": asdict(settings), "cache": cache, "drops": drops,
            "seconds": round(time.time() - t0, 1)}


def write_search(result, out_dir):
    """``radius.json`` (without the drops) and ``drops.json.gz`` in ``out_dir``: the drops are what
    ``python -m brixzle.tools contacts <family> --map <out_dir> --slices <placement>`` replays."""
    import gzip
    import json
    from pathlib import Path

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    with gzip.open(out_dir / "drops.json.gz", "wt") as stream:
        json.dump(result.get("drops", []), stream)
    rec = {k: v for k, v in result.items() if k != "drops"}
    rec["drops_file"] = str(out_dir / "drops.json.gz")
    (out_dir / "radius.json").write_text(json.dumps(rec, indent=1, default=str) + "\n")
    return rec


def main(argv=None):
    """python -m brixzle.engine.radius <family> [--drops N] [--radii 8,16] [--out DIR] [--builds]

    The error-radius score of one design (a registry name or <root>:<module>), for probing a draft:
    fewer drops and radii make it quick, and the verifier reruns the full default. Prints a short
    JSON summary; with --out, writes radius.json and drops.json.gz there for ``tools contacts``."""
    import argparse
    import json

    from brixzle.tools import core as TC

    ap = argparse.ArgumentParser(prog="python -m brixzle.engine.radius")
    ap.add_argument("family")
    ap.add_argument("--drops", type=int, default=RadiusSettings.drops)
    ap.add_argument("--radii", help="comma-separated radii in mm (default: the full ladder)")
    ap.add_argument("--placements", help=f"comma-separated, some of {tuple(PLACEMENTS)}")
    ap.add_argument("--out")
    ap.add_argument("--builds", action="store_true", help="also build the noisy tower and corbel at r90")
    ap.add_argument("--workers", type=int)
    a = ap.parse_args(argv)
    TC.cache_on()
    kw = {"drops": a.drops}
    if a.radii:
        kw["radii_mm"] = tuple(float(v) for v in a.radii.split(","))
    if a.placements:
        kw["placements"] = tuple(a.placements.split(","))
    fam = TC.load(a.family)
    res = radius_search(fam, fam.defaults, RadiusSettings(**kw), workers=a.workers)
    if a.out:
        res = write_search(res, a.out)
    summary = {k: res.get(k) for k in ("r90_mm", "floor_passed", "weakest", "first_failing_radius_mm", "curve",
                                       "seconds", "error", "drops_file")}
    summary["rates"] = {r: {k: round(v["rate"], 3) for k, v in d["placements"].items()}
                        for r, d in res["by_radius"].items()}
    if a.builds:
        b = noisy_builds(fam, fam.defaults, res["r90_mm"], workers=a.workers)
        summary["builds"] = {k: b[k] for k in ("radius_mm", "overhang_steps", "stack_courses", "seconds")}
    summary["label"] = LABEL
    print(json.dumps(summary, indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


# --- noisy builds at the tolerated radius ---


@dataclass(frozen=True)
class BuildSettings:
    builds: int = 7           # seeded builds per structure; the median is reported (7: speed, #206)
    tower: int = 6            # courses asked of the stack
    corbel: int = 8           # reaching steps asked of the overhang
    seed: int = 0
    held: bool = True         # held release, as in RadiusSettings


def _tower_courses(build):
    ok = {st["brick"] for st in build["stages"] if st["outcome"] == "seated" and not st["collapsed"]}
    k = 0
    while f"b{k}_0" in ok:
        k += 1
    return k


def noisy_builds(family, p, radius_mm, settings: BuildSettings = BuildSettings(), workers=None):
    """A tower and a back-first corbel built brick by brick, every brick released with error inside a
    disc of ``radius_mm`` (objective.radius_error), each new brick aimed at its supporter's measured
    pose. Returns the median stack height (courses) and overhang length (reaching steps)."""
    from brixzle.engine import objective as OB
    from brixzle.engine import structures as S
    from brixzle.engine.sim import assemble as A
    from brixzle.engine.sim import trials as T

    t0 = time.time()
    radius_mm = max(0.0, float(radius_mm))
    corbel = OB.noisy_corbel(family, p, radius_mm, builds=settings.builds, steps=settings.corbel,
                             seed=settings.seed, workers=workers, held=settings.held)
    cfg = T.TrialConfig(error=OB.radius_error(radius_mm), held=settings.held)
    res = A.repeat_builds(family, p, S.tower(settings.tower, family.lattice(p).span_voxels), cfg,
                          n=settings.builds, seed=settings.seed, jitter_mm=0.0, workers=workers)
    courses = [_tower_courses(b) for b in res["builds"]]
    return {"label": LABEL, "radius_mm": radius_mm,
            "overhang_steps": corbel["median_steps"], "overhang_by_build": [b["steps"] for b in corbel["builds"]],
            "overhang_failed_at": corbel["failed_at"],
            "stack_courses": float(np.median(courses)), "stack_by_build": courses,
            "settings": asdict(settings), "seconds": round(time.time() - t0, 1)}
