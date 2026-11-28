"""The grid score (#208, scored as four quarters since #222): Z, and the green zone.

**The score (#222).** Z = 1/4 Z_stack + 1/4 Z_overhang1 + 1/4 Z_overhang2 + 1/4 Z_overhang3. Each term
is Z = (S - J) / N over that placement's grid cells whose centre lies inside the ``z_ring_mm`` (16 mm)
disc, every cell counted by its area in fine cells: S seated (+1), J jammed (-1), a wrong valley or a
fall 0. The four placements are ``radius.QUARTERS``: the full seat (stacking) and the reaching brick of
a back-first corbel's courses 1, 2 and 3, everything below it ideally placed. If one brick stacks,
many do; a corbel needs each reach to seat. Z = 1 means every release within 16 mm seats on all four.
Z over the 28 and 40 mm discs (``z_rings``) break ties.

Per placement type, the brick is released in a free drop (#218) on an X-Y grid of offsets about its
aim out to ``max_mm`` (40 mm):

1. **Coarse.** Cells of ``coarse_mm`` (7.5 mm) whose square meets the 40 mm disc, one drop at each
   cell's centre.
2. **Refine.** A coarse cell that touches a cell with another outcome is split into
   the 3 x 3 fine cells (``fine_mm``, 2.5 mm) around its own centre, so its drop is reused as the
   middle fine cell and every drop lies on one 2.5 mm lattice. This repeats until nothing is left
   to split: every outcome change ends up at fine resolution (so a lone failing coarse cell is
   resolved, and so are its seated neighbours), while plains of one outcome stay coarse.
3. **Early stop.** Once a placement has a failure within ``floor_mm`` (16 mm), cells farther out
   than its nearest failure are refined only if they reach inside the ``z_ring_mm`` disc, where
   they still change Z (#222); the rest cannot change the score.

``green_mm``, a diagnostic since #222, is the largest disc radius about the aim inside which every
cell seats: the distance from the aim to the nearest point of the nearest cell that did not seat
(jammed, wrong valley or fell), capped at ``max_mm``. The weakest placement sets the design's
``green_mm``. Any placement of ``radius.PLACEMENTS`` (the bare edges, the corbel's back brick) can
still be gridded as a diagnostic with ``placements``; Z averages whichever placements were gridded.

``search`` is the grid logic on any ``outcome_of``; ``grid_search`` runs it in the sim with one
``box_search.DropPool`` across every placement (all cores, drop cache on). All results are
simulated, uncalibrated contact.
"""
from dataclasses import asdict, dataclass
import math
from pathlib import Path
import time

from brixzle.engine import radius as RA

LABEL = RA.LABEL
SEATED = "seated"
FAILURES = ("jammed", "wrong_valley", "fell")


@dataclass(frozen=True)
class GridSettings:
    max_mm: float = 40.0        # the largest error the score credits (#206)
    fine_mm: float = 2.5        # refined cell; a coarse cell is 3 x 3 of these (7.5 mm)
    floor_mm: float = 16.0      # the least error a brick must tolerate to be worth printing (#206)
    rings_mm: tuple = (16.0, 28.0, 40.0)   # Z rings: floor, p95 and worst (#206's error range)
    placements: tuple = RA.QUARTERS   # stack and overhangs 1-3 (#222)
    z_ring_mm: float = 16.0           # the disc the score Z is counted over (#222): the floor
    z_frac: float = 1.0         # release height, as a share of the mean box's height span: its top (#218), 11.2 mm
    held: bool = False          # hover and drop (#218): a free drop; True: the held release (#206)
    early_stop: bool = True

    def __post_init__(self):
        for name in ("max_mm", "fine_mm", "floor_mm"):
            v = getattr(self, name)
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
                raise ValueError(f"{name} must be finite and > 0, got {v!r}")
        unknown = set(self.placements) - set(RA.PLACEMENTS)
        if unknown or not self.placements:
            raise ValueError(f"placements must be some of {tuple(RA.PLACEMENTS)}, got {self.placements!r}")
        if not 0.0 <= self.z_frac <= 1.0:
            raise ValueError(f"z_frac must be in [0, 1], got {self.z_frac!r}")
        if not self.rings_mm or any(not (math.isfinite(r) and r > 0) for r in self.rings_mm):
            raise ValueError(f"rings_mm must be finite and > 0, got {self.rings_mm!r}")
        if self.z_ring_mm not in self.rings_mm:
            raise ValueError(f"z_ring_mm must be one of rings_mm {self.rings_mm!r}, got {self.z_ring_mm!r}")

    @property
    def coarse_mm(self):
        return 3 * self.fine_mm


def release_z_mm(s: GridSettings):
    """The release height above the minimum clearance: ``z_frac`` of the mean box's height span (#218)."""
    from brixzle.engine import boxes as BX
    return s.z_frac * 2 * BX.default_boxes()["placement"]["mean"].half_widths["z_mm"]


def ring_key(r):
    return f"{r:g}"


# A cell is (i, j, size): its centre at (i, j) fine cells from the aim, its side ``size`` fine cells
# (3: coarse, 1: fine). Coarse centres are multiples of 3.


def centre_mm(cell, s: GridSettings):
    return cell[0] * s.fine_mm, cell[1] * s.fine_mm


def distance_mm(cell, s: GridSettings):
    """From the aim to the nearest point of the cell's square."""
    half = cell[2] / 2
    return s.fine_mm * math.hypot(max(abs(cell[0]) - half, 0.0), max(abs(cell[1]) - half, 0.0))


def contains(cell, x, y, s: GridSettings):
    half = cell[2] * s.fine_mm / 2
    cx, cy = centre_mm(cell, s)
    return abs(x - cx) <= half + 1e-9 and abs(y - cy) <= half + 1e-9


def coarse_cells(s: GridSettings):
    n = int(math.ceil(s.max_mm / s.coarse_mm)) + 1
    return [(3 * a, 3 * b, 3) for b in range(-n, n + 1) for a in range(-n, n + 1)
            if distance_mm((3 * a, 3 * b, 3), s) <= s.max_mm + 1e-9]


def children(cell):
    i, j, _ = cell
    return [(i + a, j + b, 1) for b in (-1, 0, 1) for a in (-1, 0, 1)]


def _owners(cells):
    """Each fine lattice point -> the cell that covers it."""
    own = {}
    for c in cells:
        if c[2] == 3:
            for k in children(c):
                own[k[:2]] = c
        else:
            own[c[:2]] = c
    return own


def _touching(cell, owners):
    i, j, size = cell
    r = size // 2 + 1
    out = set()
    for b in range(j - r, j + r + 1):
        for a in range(i - r, i + r + 1):
            if max(abs(a - i), abs(b - j)) == r and (a, b) in owners:
                out.add(owners[(a, b)])
    return out


def green_radius(cells, s: GridSettings, ignore=()):
    """Largest disc radius about the aim inside which every cell seats (``cells``: {cell: outcome}).
    Cells in ``ignore`` (failures tolerated as solver noise) do not count."""
    bad = [distance_mm(c, s) for c, o in cells.items() if o != SEATED and c not in ignore]
    return round(min([s.max_mm, *bad]), 4)


def _bound(cells, s: GridSettings):
    """The nearest failure's centre: the green zone cannot reach past it."""
    return min([s.max_mm, *(s.fine_mm * math.hypot(c[0], c[1]) for c, o in cells.items() if o != SEATED)])


def to_split(cells, s: GridSettings):
    """The coarse cells to refine, and whether the placement has stopped early."""
    bound = _bound(cells, s)
    early = s.early_stop and bound < s.floor_mm
    owners = _owners(cells)
    out = []
    for c, o in cells.items():
        if c[2] != 3:
            continue
        if early and distance_mm(c, s) >= max(bound, s.z_ring_mm):
            continue  # outside both the green zone's bound and the disc Z is counted over
        if any(cells[n] != o for n in _touching(c, owners)):
            out.append(c)
    return sorted(out, key=lambda c: (c[1], c[0])), early


def ring_z(cells, s: GridSettings, rings=None):
    """Z = (S - J) / N per ring, over the cells whose centre lies within it, each counted by its area
    in fine cells (a coarse cell is 9)."""
    out = {}
    for r in rings or s.rings_mm:
        S = J = N = 0
        for c, o in cells.items():
            x, y = centre_mm(c, s)
            if math.hypot(x, y) > r + 1e-9:
                continue
            w = c[2] ** 2
            N += w
            S += w * (o == SEATED)
            J += w * (o == "jammed")
        out[f"{r:g}"] = {"S": S, "J": J, "N": N, "Z": round((S - J) / N, 4) if N else 0.0}
    return out


def _counts(cells):
    c = {k: 0 for k in (SEATED, *FAILURES, "other")}
    for o in cells.values():
        c[o if o in c else "other"] += 1
    return c


def _seen(cells, tolerated):
    """The outcomes refinement and the bound see: a tolerated one-off counts as seated."""
    return {c: (SEATED if c in tolerated else o) for c, o in cells.items()}


def nearest_unchecked_failure(cells, s: GridSettings, tolerated=(), checked=()):
    """The failure nearest the aim, if it is a fine cell not yet rerun: the next candidate for solver
    noise. None if the nearest failure was already rerun (and repeated) or is a coarse cell."""
    view = _seen(cells, tolerated)
    bad = sorted((distance_mm(c, s), c) for c, o in view.items() if o != SEATED)
    if not bad:
        return None
    c = bad[0][1]
    if c[2] != 1 or c in checked:
        return None
    return c


def search(outcome_of, s: GridSettings = GridSettings(), noise_check=None):
    """The grid search on ``outcome_of([(placement, x_mm, y_mm), ...]) -> [outcome, ...]`` (one call
    per round, every placement's drops in one batch). Returns the green zone, ring Z and the cells
    per placement, weakest placement first.

    ``noise_check(placement, [cell]) -> {cell: [{mu, seed, outcome}]}`` reruns cells (at other
    frictions and seeded shifts). Once nothing is left to refine, the failure nearest the aim is
    rerun; if it is not repeatable (jam_analysis.repeatability: it fails less than 2/3 of the seeds at
    every friction) it is solver noise and no longer bounds the green zone, whether or not its
    neighbours failed too (maintainer, 2026-11-26, #218; isolated cells only before), and the search
    goes on to the next nearest, until the nearest failure repeats. It stays in the cells, listed under ``tolerated``."""
    from brixzle.engine import jam_analysis as JA

    t0 = time.time()
    cells = {pl: {} for pl in s.placements}
    drops = {pl: 0 for pl in s.placements}
    early = {pl: False for pl in s.placements}
    tolerated = {pl: set() for pl in s.placements}
    noise = {pl: {} for pl in s.placements}
    pending = [(pl, c) for pl in s.placements for c in coarse_cells(s)]
    rounds = []

    def refine():
        out = []
        for pl in s.placements:
            split, early[pl] = to_split(_seen(cells[pl], tolerated[pl]), s)
            for c in split:
                o = cells[pl].pop(c)
                for k in children(c):
                    if k[:2] == c[:2]:
                        cells[pl][k] = o  # the coarse drop is the middle fine cell
                    else:
                        out.append((pl, k))
        return out

    while pending:
        while pending:
            t = time.time()
            outs = outcome_of([(pl, *centre_mm(c, s)) for pl, c in pending])
            for (pl, c), o in zip(pending, outs):
                cells[pl][c] = o
                drops[pl] += 1
            rounds.append({"round": len(rounds), "kind": "coarse" if not rounds else "refine",
                           "drops": len(pending), "seconds": round(time.time() - t, 2)})
            pending = refine()
        if noise_check is None:
            break
        t, n, changed = time.time(), 0, False
        for pl in s.placements:
            while True:
                c = nearest_unchecked_failure(cells[pl], s, tolerated[pl], noise[pl])
                if c is None:
                    break
                reruns = noise_check(pl, [c])[c]
                n += len(reruns)
                noise[pl][c] = {"outcome": cells[pl][c], "reruns": reruns, **JA.repeatability(reruns)}
                if noise[pl][c]["repeatable"]:
                    break
                tolerated[pl].add(c)
                changed = True
        if n:
            rounds.append({"round": len(rounds), "kind": "noise", "drops": n, "seconds": round(time.time() - t, 2)})
        if changed:
            pending = refine()
    per = {}
    for pl in s.placements:
        cs = cells[pl]
        per[pl] = {"green_mm": green_radius(cs, s, ignore=tolerated[pl]), "early_stop": early[pl], "drops": drops[pl],
                   "tolerated": sorted([list(c) for c in tolerated[pl]]),
                   "noise": {",".join(map(str, c)): v for c, v in noise[pl].items()},
                   "rings": ring_z(cs, s), "counts": _counts(cs),
                   "cells": [{"cell": list(c), "x_mm": centre_mm(c, s)[0], "y_mm": centre_mm(c, s)[1],
                              "size_mm": c[2] * s.fine_mm, "outcome": o}
                             for c, o in sorted(cs.items(), key=lambda kv: (kv[0][1], kv[0][0], kv[0][2]))]}
    order = sorted(s.placements, key=lambda pl: (per[pl]["green_mm"], per[pl]["rings"][ring_key(s.z_ring_mm)]["Z"]))
    green = per[order[0]]["green_mm"]
    return {"label": LABEL, **quarter_z(per, s), "green_mm": green, "weakest": order[0], "order": order,
            "floor_passed": green >= s.floor_mm, "placements": per, "rounds": rounds,
            "drops": sum(drops.values()), "settings": asdict(s), "seconds": round(time.time() - t0, 2)}


def quarter_z(per, s: GridSettings):
    """The score (#222): ``z``, the mean over the placements of each one's Z in the ``z_ring_mm`` disc;
    ``z_quarters``, each placement's; and ``z_rings``, the same mean in every ring (the tie-breaks).
    ``per`` is {placement: {'rings': ring_z(...)}}."""
    if not per:
        raise ValueError("no placements: Z is undefined")
    rings = {ring_key(r): round(sum(v["rings"][ring_key(r)]["Z"] for v in per.values()) / len(per), 4)
             for r in s.rings_mm}
    return {"z": rings[ring_key(s.z_ring_mm)], "z_rings": rings,
            "z_quarters": {pl: v["rings"][ring_key(s.z_ring_mm)]["Z"] for pl, v in per.items()}}


# --- in the sim ---


def grid_search(family, p, s: GridSettings = GridSettings(), workers=None, noise=True, jams=True):
    """``search`` in the sim: held drops in one DropPool (all cores unless ``workers``), the drop
    cache on if the caller turned it on (tools.core.cache_on). Every drop is kept in ``drops_rows``
    in the form ``tools contacts`` reads, and the rounds carry their wall-clock. With ``noise``,
    the failures nearest the aim are rerun until one repeats (``search``'s noise_check); with ``jams``, the failures that bound each
    placement's green zone are analysed (``analyse_jams``)."""
    import os

    from brixzle.engine import box_search as BS
    from brixzle.engine import jam_analysis as JA
    from brixzle.engine import zscore as ZS

    t0 = time.time()
    span = family.lattice(p).span_voxels
    if span != 2:
        return {"label": LABEL, "error": f"the placements are laid out for a two-voxel brick; span_voxels = {span}",
                "z": None, "z_rings": {}, "z_quarters": {}, "green_mm": 0.0, "floor_passed": False, "placements": {}, "settings": asdict(s), "drops_rows": [],
                "seconds": round(time.time() - t0, 1)}
    cfg = RA._ideal_cfg(s.held)
    jcfg = JA.JamConfig()
    z = release_z_mm(s)
    rows = []

    def sample(x, y):
        return ZS._sample(cfg, {"x": x, "y": y, "z": z, "t": 0.0})

    def rerun_jobs(pl, cells):
        """Each cell at every friction and seed, shifted by up to ``jitter`` of a fine cell."""
        out = []
        for c in cells:
            for mu in jcfg.mus:
                for seed in range(jcfg.seeds):
                    i, j = JA._jittered(c[:2], seed, (s.placements.index(pl),), jcfg, None)
                    out.append((c, mu, seed, (RA.PLACEMENTS[pl], sample(i * s.fine_mm, j * s.fine_mm),
                                              {"friction": float(mu), "trace": False})))
        return out

    with BS.DropPool(family, p, cfg, workers=workers or os.cpu_count()) as pool:
        def outcome_of(points):
            jobs = [(RA.PLACEMENTS[pl], sample(x, y), {"trace": False}) for pl, x, y in points]
            out = []
            for (pl, x, y), (job, smp, _), r in zip(points, jobs, pool.imap(jobs)):
                rows.append({"slice": pl, "edge_case": job, "course": 1, "x_mm": x, "y_mm": y,
                             "dx_mm": round(smp["dx"] - smp["aim"], 3), "dy_mm": round(smp["dy"], 3),
                             "clear_mm": smp["clear"], "pitch_deg": smp["pitch"], "sample": smp,
                             "outcome": r["outcome"], "err_mm": [round(float(v), 3) for v in r.get("err_mm", [])],
                             "angle_deg": round(float(r.get("angle_deg", 0.0)), 2)})
                out.append(r["outcome"])
            return out

        def noise_check(pl, cells):
            jobs = rerun_jobs(pl, cells)
            out = {c: [] for c in cells}
            for (c, mu, seed, _), r in zip(jobs, pool.imap([j[3] for j in jobs])):
                out[c].append({"mu": float(mu), "seed": seed, "outcome": r["outcome"]})
            return out

        res = search(outcome_of, s, noise_check=noise_check if noise else None)
        if jams:
            t = time.time()
            res["jams"] = analyse_jams(family, p, res, s, pool, noise_check, sample, float(cfg.physics.friction), jcfg)
            res["rounds"].append({"round": len(res["rounds"]), "kind": "jams", "drops": res["jams"]["counts"]["drops"],
                                  "seconds": round(time.time() - t, 2)})
        res["cache"] = dict(getattr(pool, "cache_stats", {}) or {})
        res["workers"] = pool.workers
    res.update(drops_rows=rows, family=family.name, release_z_mm=z, seconds=round(time.time() - t0, 1))
    return res


JAM_CELLS = 6  # failing cells analysed per placement: the nearest to the aim


def jam_candidates(placement_result, s: GridSettings, n=JAM_CELLS):
    """The failures that bound a placement's green zone: fine failing cells (not tolerated) within
    two fine cells of its edge, nearest first, at most ``n``."""
    tol = {tuple(c) for c in placement_result["tolerated"]}
    g = placement_result["green_mm"]
    out = []
    for c in placement_result["cells"]:
        key = tuple(c["cell"])
        if c["outcome"] == SEATED or key in tol or key[2] != 1:
            continue
        d = distance_mm(key, s)
        if d <= g + 2 * s.fine_mm + 1e-9:
            out.append((d, key, c["outcome"]))
    return [(k, o) for _, k, o in sorted(out)[:n]]


def analyse_jams(family, p, res, s, pool, noise_check, sample, sim_mu, jcfg):
    """jam_analysis (#49) on the grid (#208), for every placement including the bare edges and the
    corbel reach: each failure that bounds the green zone is rerun at each friction with seeded
    shifts (reusing the noise check's reruns), the repeatable ones are grouped into bands of
    touching cells, and the drop at each band's representative cell is run again and its contacts
    at rest reduced to the holding set, with named faces. Sim results are simulated, uncalibrated
    contact; the mechanism is predicted, quasi-static 2D."""
    from brixzle.engine import jam_analysis as JA

    t0 = time.time()
    out, n_drops = {}, 0
    for pl in res["order"]:
        r = res["placements"][pl]
        cand = jam_candidates(r, s)
        known = {tuple(int(v) for v in k.split(",")): v["reruns"] for k, v in r["noise"].items()}
        todo = [c for c, _ in cand if c not in known]
        fresh = noise_check(pl, todo) if todo else {}
        n_drops += sum(len(v) for v in fresh.values())
        cells = []
        for c, o in cand:
            reruns = known.get(c) or fresh[c]
            x, y = centre_mm(c, s)
            cells.append({"slice": pl, "course": 1, "cell": [c[0], c[1]], "x_mm": x, "y_mm": y,
                          "distance_mm": round(distance_mm(c, s), 3), "outcome": o, "reruns": reruns,
                          **JA.repeatability(reruns, jcfg)})
        groups = JA.bands([c for c in cells if c["repeatable"]])
        reps = [JA.representative(g) for g in groups]
        jobs = [(RA.PLACEMENTS[pl], sample(rep["x_mm"], rep["y_mm"]), {"rest": True, "trace": False}) for rep in reps]
        bands = []
        for k, (group, rep, row) in enumerate(zip(groups, reps, pool.imap(jobs))):
            n_drops += 1
            types = {}
            for c in group:
                types[c["outcome"]] = types.get(c["outcome"], 0) + 1
            try:
                reduced = JA.reduce_row(family, p, RA.PLACEMENTS[pl], row, sim_mu, jcfg)
            except Exception as e:  # a reduction that cannot run is reported, not fatal to the score
                reduced = {"outcome": row["outcome"], "mechanism": f"not reduced: {type(e).__name__}: {e}"[:300]}
            bands.append({"id": k, "placement": pl, "cells": [c["cell"] for c in group],
                          "extent_mm": {"x": [min(c["x_mm"] for c in group), max(c["x_mm"] for c in group)],
                                        "y": [min(c["y_mm"] for c in group), max(c["y_mm"] for c in group)]},
                          "nearest_mm": min(c["distance_mm"] for c in group),
                          "failure_types": dict(sorted(types.items())), "failure_type": max(sorted(types), key=types.get),
                          "repeat_rate": float(sum(c["repeat_rate"] for c in group) / len(group)),
                          "representative": {"cell": rep["cell"], "x_mm": rep["x_mm"], "y_mm": rep["y_mm"]},
                          **reduced})
        out[pl] = {"green_mm": r["green_mm"], "cells": cells, "bands": bands,
                   "one_offs": [c for c in cells if not c["repeatable"]],
                   "tolerated": r["tolerated"]}
    return {"labels": dict(JA.LABELS), "family": family.name, "config": {**asdict(jcfg), "mus": list(jcfg.mus)},
            "sim_mu": sim_mu, "placements": out,
            "counts": {"cells": sum(len(v["cells"]) for v in out.values()),
                       "bands": sum(len(v["bands"]) for v in out.values()), "drops": n_drops},
            "seconds": round(time.time() - t0, 2)}


def write_grid(result, out_dir, family_name=None):
    """``grid.json`` (without the drop rows), ``drops.json.gz`` and one map PNG per placement in
    ``out_dir``. Returns the record written to grid.json."""
    import gzip
    import json

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    with gzip.open(out_dir / "drops.json.gz", "wt") as stream:
        json.dump(result.get("drops_rows", []), stream)
    t = time.time()
    maps = write_maps(result, out_dir, family_name or result.get("family"))
    rec = {k: v for k, v in result.items() if k not in ("drops_rows", "jams")}
    if result.get("jams") is not None:
        (out_dir / "jams.json").write_text(json.dumps(result["jams"], indent=1, default=str, allow_nan=False) + "\n")
        rec["jams_file"] = str(out_dir / "jams.json")
        rec["jams_summary"] = jams_summary(result["jams"])
    rec["maps"] = [str(m) for m in maps]
    rec["map_seconds"] = round(time.time() - t, 2)
    rec["drops_file"] = str(out_dir / "drops.json.gz")
    (out_dir / "grid.json").write_text(json.dumps(rec, indent=1, default=str, allow_nan=False) + "\n")
    return rec


def jams_summary(jams):
    """One line per band: placement, failure type, where, how repeatable, and the mechanism."""
    lines = []
    for pl, v in jams["placements"].items():
        for b in v["bands"]:
            ex = b["extent_mm"]
            lines.append(f"{pl}: {b['failure_type']} from {b['nearest_mm']:.1f} mm (x {ex['x'][0]:+g}..{ex['x'][1]:+g}, "
                         f"y {ex['y'][0]:+g}..{ex['y'][1]:+g}; {len(b['cells'])} cells, repeat {b['repeat_rate']:.0%}): "
                         f"{b.get('mechanism')}")
            for c3 in (b.get("contacts_3d") or [])[:3]:  # a 3D brick: contacts in its own frame (#214)
                at = ", ".join(f"{v:+.1f}" for v in c3["at_mm"])
                lines.append(f"    touches {c3['other']} at ({at}) mm in its own frame, normal "
                             f"({', '.join(f'{v:+.2f}' for v in c3['normal'])}), {c3['weight_share']:.2f} of the weight")
            sites = sorted(b.get("sites") or [], key=lambda x: -x.get("normal_N", 0.0))[:3]
            for site in sites:
                share = site["normal_N"] / b["weight_N"] if b.get("weight_N") else 0.0
                lines.append(f"    {site.get('on_brick', {}).get('label')} ON {site.get('on_support', {}).get('label')}"
                             f" ({share:.2f} of the weight)")
        for c in v["one_offs"]:
            lines.append(f"{pl}: one-off {c['outcome']} at ({c['x_mm']:+g}, {c['y_mm']:+g}) mm, not repeatable")
    return lines


# --- the maps ---

# Outcome -> (fill, hatch). Status colours (good / critical / warning) and a neutral grey for a
# fall; each failure also has its own hatch, so the map reads without colour.
STYLE = {SEATED: ("#0aac23", ""), "jammed": ("#dc3c3d", "////"), "wrong_valley": ("#fbb315", "...."),
         "fell": ("#5b5b5b", "xxxx"), "other": ("#6a4ce2", "oo")}
NAMES = {SEATED: "seated", "jammed": "jammed", "wrong_valley": "wrong valley", "fell": "fell", "other": "other"}


def write_maps(result, out_dir, family_name=None):
    """One PNG per placement (``map-<placement>.png``): the outcome of every cell, the 16, 28 and 40 mm
    rings, and the green zone's radius (dashed)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Circle, Patch, Rectangle

    out_dir = Path(out_dir)
    s = result["settings"]
    rings = s["rings_mm"]
    paths = []
    for pl in result.get("order", []):
        r = result["placements"][pl]
        fig, ax = plt.subplots(figsize=(6.4, 6.9), dpi=130)
        seen = set()
        for c in r["cells"]:
            o = c["outcome"] if c["outcome"] in STYLE else "other"
            seen.add(o)
            fill, hatch = STYLE[o]
            h = c["size_mm"] / 2
            ax.add_patch(Rectangle((c["x_mm"] - h, c["y_mm"] - h), 2 * h, 2 * h, facecolor=fill, hatch=hatch,
                                   edgecolor="white", linewidth=0.6))
        for ring in rings:
            ax.add_patch(Circle((0, 0), ring, fill=False, color="#191919", linewidth=1.0))
            ax.annotate(f"{ring:g} mm", (ring * 0.7071, ring * 0.7071), xytext=(3, 3), textcoords="offset points",
                        fontsize=8, color="#191919", bbox={"boxstyle": "round,pad=0.15", "fc": "white", "ec": "none",
                                                          "alpha": 0.85})
        tol = [c for c in r["cells"] if c["cell"] in r.get("tolerated", [])]
        for c in tol:
            h = c["size_mm"] / 2
            ax.add_patch(Rectangle((c["x_mm"] - h, c["y_mm"] - h), 2 * h, 2 * h, fill=False, edgecolor="#191919",
                                   linewidth=1.6))
        g = r["green_mm"]
        ax.add_patch(Circle((0, 0), g, fill=False, color="#257de0", linewidth=2.2, linestyle=(0, (5, 3))))
        ax.plot([0], [0], marker="+", color="#191919", markersize=10, markeredgewidth=1.5)
        lim = s["max_mm"] + 1.5 * s["fine_mm"] * 3
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect("equal")
        ax.set_xlabel("release error x (mm, along the brick)")
        ax.set_ylabel("release error y (mm, across)")
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        rz = r["rings"]
        zline = "   ".join(f"Z{k} {v['Z']:+.2f}" for k, v in rz.items())
        ax.set_title(f"{family_name or ''} {pl}: Z{ring_key(s.get('z_ring_mm', 16.0))} "
                     f"{rz[ring_key(s.get('z_ring_mm', 16.0))]['Z']:+.2f}, green zone {g:.1f} mm\n{zline}   ({r['drops']} drops"
                     f"{', stopped early' if r['early_stop'] else ''})", fontsize=10, loc="left")
        handles = [Patch(facecolor=STYLE[o][0], hatch=STYLE[o][1], edgecolor="white", label=NAMES[o])
                   for o in STYLE if o in seen]
        if tol:
            handles.append(Patch(facecolor="none", edgecolor="#191919", linewidth=1.6,
                                 label="one-off, tolerated (solver noise)"))
        handles.append(Line2D([], [], color="#257de0", linestyle=(0, (5, 3)), linewidth=2.2,
                              label=f"green zone {g:.1f} mm"))
        ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=3, frameon=False, fontsize=8)
        fig.text(0.99, 0.01, LABEL, ha="right", va="bottom", fontsize=7, color="#5b5b5b")
        fig.tight_layout()
        path = out_dir / f"map-{pl}.png"
        fig.savefig(path)
        plt.close(fig)
        paths.append(path)
    return paths


def main(argv=None):
    """python -m brixzle.engine.gridscore <family> [--placements full,corbel_reach] [--out DIR] [--builds] [--no-jams]

    The grid score of one design (a registry name or <root>:<module>): Z on the four quarters by
    default (#222), or on the placements named. Prints a short JSON summary
    with the wall-clock per round; with --out (a new directory), writes grid.json, drops.json.gz
    and the maps there."""
    import argparse
    import json

    from brixzle.tools import core as TC

    ap = argparse.ArgumentParser(prog="python -m brixzle.engine.gridscore")
    ap.add_argument("family")
    ap.add_argument("--placements", help=f"comma-separated, some of {tuple(RA.PLACEMENTS)}")
    ap.add_argument("--out")
    ap.add_argument("--workers", type=int)
    ap.add_argument("--no-early-stop", action="store_true")
    ap.add_argument("--no-jams", action="store_true", help="skip the jam analysis (a quicker probe)")
    ap.add_argument("--builds", action="store_true", help="also build the noisy tower and corbel (at 16 mm, #210)")
    a = ap.parse_args(argv)
    TC.cache_on()
    kw = {}
    if a.placements:
        kw["placements"] = tuple(a.placements.split(","))
    if a.no_early_stop:
        kw["early_stop"] = False
    fam = TC.load(a.family)
    res = grid_search(fam, fam.defaults, GridSettings(**kw), workers=a.workers, jams=not a.no_jams)
    if a.out:
        if Path(a.out).exists():
            raise SystemExit(f"{a.out} exists; every run writes a new directory")
        res = write_grid(res, a.out, fam.name)
    summary = {k: res.get(k) for k in ("z", "z_quarters", "z_rings", "green_mm", "weakest", "order", "floor_passed", "rounds", "cache", "workers",
                                       "seconds", "map_seconds", "error")}
    summary["drops"] = res["drops"] if isinstance(res.get("drops"), int) else len(res.get("drops_rows", []))
    summary["placements"] = {pl: {k: v[k] for k in ("green_mm", "early_stop", "drops", "rings", "counts")}
                             for pl, v in res.get("placements", {}).items()}
    if res.get("jams_summary") or res.get("jams"):
        summary["jams"] = res.get("jams_summary") or jams_summary(res["jams"])
    if a.builds:
        b = RA.noisy_builds(fam, fam.defaults, None, workers=a.workers)
        if a.out:  # beside grid.json, for the design library (#215)
            (Path(a.out) / "builds.json").write_text(json.dumps(b, indent=1, default=str) + "\n")
        summary["builds"] = {k: b[k] for k in ("radius_mm", "overhang_steps", "stack_courses", "overhang_failed_at",
                                               "seconds")}
        summary["builds"]["corbel_failures"] = [
            {k: f.get(k) for k in ("build", "brick", "kind", "outcome", "release_mm", "layout", "replay_outcome",
                                   "mechanism")} for f in b["corbel_failures"]]
    summary["label"] = LABEL
    print(json.dumps(summary, indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
