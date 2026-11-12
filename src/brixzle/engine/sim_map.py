"""Full sim map on a grid fitted to the brick (issue #48).

For a design that passed the quick check (#47), drop the brick over three slices through the aim
point, on both courses, and record seated, jammed, wrong valley and fell with every drop's
contact trace:

- ``xz``: X against release height, at zero pitch;
- ``xt``: X against pitch (theta), at the target box's centre height;
- ``zt``: release height against pitch, at the aim in X.

Coordinates, around the aim: ``x`` is the X offset (mm), ``z`` the release height above the
minimum release clearance (mm, never below 0), ``t`` the pitch (deg). A cell is index * step, so
the aim's X and zero pitch are always grid lines.

The grid is fitted in code, not chosen by an agent (``GridFit``):

1. start from the quick check's drops, widened to the target box, plus a margin;
2. grow any edge that still has a seated (green) cell on it, until it has none or reaches the
   axis limit;
3. trim outer rows and columns with no green, keeping one such row past the last green one: where
   the misses start is the capture limit, which the critic needs.

Guardrail: the grid never shrinks inside the target boxes it covers (``cover``). The loop's
objective, Z (engine/zscore.py, issue #61), is counted over the target box whatever else the map
covers, so cropping the map cannot raise it. Extents are also reported in voxel pitches (U), so
bricks of different sizes get comparable maps.

Overrides: a caller may add a zoom (a finer grid over part of a slice, for example a jam band) or
change the grid settings. Either needs a reason, and both are written to ``overrides`` in
map.json for the design log (#20). A zoom is an extra grid: it never replaces a fitted slice.

Placement only for now (issue #46). Results are simulated, uncalibrated contact.

    uv run python -m brixzle.engine.sim_map --family ab --statistic mean --seed 0
"""

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import gzip
import json
import math
from pathlib import Path
import time

from brixzle.engine import box_search as BS
from brixzle.engine import boxes as BX
from brixzle.engine import quick_check as QC
from brixzle.engine.sim import trials as T

LABEL = BS.LABEL
SEATED = BS.SUCCESS["placement"]
DROPS_FILE = "drops.json.gz"  # every drop with its contact trace: tens of MB as plain JSON
CODES = {"seated": "S", "jammed": "J", "wrong_valley": "W", "fell": "F"}
OTHER = "?"  # any other outcome (for example 'unstable')
SLICES = {"xz": ("x", "z"), "xt": ("x", "t"), "zt": ("z", "t")}
AXIS_UNITS = {"x": "mm", "z": "mm", "t": "deg"}
AXIS_NAMES = {"x": "X offset about the aim", "z": "release height above the minimum clearance", "t": "pitch"}
FIXED_GRID_MM = 30.0  # the old fixed grid's half-width in X (aerial-assembly figures/biasbrick-drop-grid)
# Measured on A/B V0's mean-box map, 2026-11-11 (14-core Mac, 1,254 drops): 48.8 s on 4 workers,
# 40.1 s on 6 and 37.8 s on 8 (runs/map-20261111T193311241213Z-ab-mean, ...193402487600Z...,
# ...193451515332Z...). Six is most of the gain, and the grids are the same for any count.
DEFAULT_WORKERS = 6


@dataclass(frozen=True)
class MapConfig:
    """Cell sizes and growth limits. Sizes in X and Z are in voxel pitches (U), so a map has about
    the same number of cells whatever the brick's size."""

    x_step_U: float = 0.04          # 1 mm at U = 25 mm
    z_step_U: float = 0.2           # 5 mm at U = 25 mm
    theta_step_deg: float = 2.0
    x_limit_U: float = 2.0          # the grid never grows past +-2 U about the aim (a whole brick)
    # Release height and pitch are capped well past the worst target box (7.2 mm, 0.4 deg). On A/B
    # V0 the green zone still reached 2 U and +-15 deg, the largest values tried, on every slice
    # (runs/map-20261111T193029503482Z-ab-mean), so growing these axes "until red" only spends drops.
    z_limit_U: float = 1.0          # nor above 1 U of release height
    theta_limit_deg: float = 10.0   # nor past +-10 deg of pitch
    margin: int = 1                 # cells added round the starting extent
    grow: int = 2                   # rows or columns added to an edge per growth step
    max_drops: int = 12000          # stop growing once this many drops are planned

    def __post_init__(self):
        for name, v in asdict(self).items():
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
                raise ValueError(f"{name} must be a finite number > 0, got {v!r}")
        for name in ("margin", "grow", "max_drops"):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f"{name} must be an integer, got {getattr(self, name)!r}")

    def steps(self, U):
        """Cell size per axis: mm for x and z, degrees for t."""
        return {"x": self.x_step_U * U, "z": self.z_step_U * U, "t": self.theta_step_deg}

    def limits(self, U):
        """Hard index limits per axis as (lo, hi). Release height never goes below 0."""
        s = self.steps(U)
        nx, nz = math.floor(self.x_limit_U * U / s["x"] + 1e-9), math.floor(self.z_limit_U * U / s["z"] + 1e-9)
        nt = math.floor(self.theta_limit_deg / s["t"] + 1e-9)
        return {"x": (-nx, nx), "z": (0, nz), "t": (-nt, nt)}


@dataclass(frozen=True)
class Zoom:
    """An override: an extra, finer grid over part of one slice (for example a jam band)."""

    slice: str     # 'xz', 'xt' or 'zt'
    course: int    # 0 or 1
    extent: dict   # axis -> (lo, hi), in mm or degrees, for the slice's two axes
    reason: str    # why; written to map.json for the design log (issue #20)
    step: dict | None = None  # axis -> cell size; default: half the map's cell


# --- the fit (no sim: cells are integer index pairs) ---


class GridFit:
    """Fit one 2D grid of cells to its green zone.

    Extents are inclusive index ranges ((i_lo, i_hi), (j_lo, j_hi)). ``start`` is where the grid
    begins, ``must`` the extent it may never shrink inside (the target box), ``limit`` the extent
    it may never grow past. Feed it outcomes until it is done:

        while not fit.done:
            for cell in fit.pending():
                fit.add(cell, outcome_of(cell))
            fit.step()
    """

    def __init__(self, start, must, limit, grow=2, budget=None):
        for name, e in (("start", start), ("must", must), ("limit", limit)):
            if any(lo > hi for lo, hi in e):
                raise ValueError(f"{name} extent {e} has lo > hi")
        # The guardrail comes first: the limit always holds the target box, and so does the grid.
        self.limit = tuple((min(l[0], m[0]), max(l[1], m[1])) for l, m in zip(limit, must))
        self.must = tuple(tuple(m) for m in must)
        self.extent = [[max(min(s[0], m[0]), l[0]), min(max(s[1], m[1]), l[1])]
                       for s, m, l in zip(start, self.must, self.limit)]
        if not isinstance(grow, int) or grow < 1:
            raise ValueError("grow must be an integer >= 1")
        self.grow, self.budget = grow, budget
        self.cells = {}            # (i, j) -> outcome, for every cell simulated (trimmed ones too)
        self.done, self.over_budget = False, False
        self._pending = self._missing()

    def _missing(self):
        (i0, i1), (j0, j1) = self.extent
        return [(i, j) for j in range(j0, j1 + 1) for i in range(i0, i1 + 1) if (i, j) not in self.cells]

    def pending(self):
        """The cells whose outcomes the fit is waiting for."""
        return list(self._pending)

    def add(self, cell, outcome):
        self.cells[tuple(cell)] = outcome

    def line(self, axis, index):
        """The cells of one row (axis 1) or column (axis 0) inside the current extent."""
        lo, hi = self.extent[1 - axis]
        return [(index, k) if axis == 0 else (k, index) for k in range(lo, hi + 1)]

    def _green(self, axis, index):
        return any(self.cells[c] == SEATED for c in self.line(axis, index))

    def step(self):
        """After the pending cells are in: grow every edge that has green on it, or finish."""
        self._pending = self._missing()
        if self._pending or self.done:
            return
        # Judge all four edges on the extent as it stands (every cell of it is in), then grow.
        before = [list(e) for e in self.extent]
        grow = [(axis, side) for axis in (0, 1) for side in (0, 1)
                if self.extent[axis][side] != self.limit[axis][side] and self._green(axis, self.extent[axis][side])]
        for axis, side in grow:
            edge, limit = self.extent[axis][side], self.limit[axis][side]
            self.extent[axis][side] = max(edge - self.grow, limit) if side == 0 else min(edge + self.grow, limit)
        self._pending = self._missing()
        if grow and self.budget is not None and len(self.cells) + len(self._pending) > self.budget:
            # Over the drop budget: give up this growth step and finish on the extent we have.
            self.over_budget, grow, self.extent = True, [], before
        if not grow:
            self._trim()
            self.done, self._pending = True, []

    def _trim(self):
        """Drop outer lines with no green, keeping one past the last green line and never cutting
        into ``must``."""
        for axis in (0, 1):
            for side, inward in ((0, 1), (1, -1)):
                while True:
                    edge = self.extent[axis][side]
                    if edge == self.must[axis][side] or self.extent[axis][0] == self.extent[axis][1]:
                        break
                    if self._green(axis, edge) or self._green(axis, edge + inward):
                        break
                    self.extent[axis][side] = edge + inward

    def edges(self):
        """Per edge ('lo'/'hi' per axis): 'closed' (no green on it), 'limit' (green at the axis
        limit) or 'budget' (green, and growth stopped at the drop budget)."""
        out = []
        for axis in (0, 1):
            row = {}
            for side, name in ((0, "lo"), (1, "hi")):
                edge = self.extent[axis][side]
                if not self._green(axis, edge):
                    row[name] = "closed"
                else:
                    row[name] = "limit" if edge == self.limit[axis][side] else "budget"
            out.append(row)
        return out

    def kept(self):
        """{cell: outcome} inside the final extent."""
        (i0, i1), (j0, j1) = self.extent
        return {c: o for c, o in self.cells.items() if i0 <= c[0] <= i1 and j0 <= c[1] <= j1}


def run_fit(fit, outcome_of):
    """Drive a GridFit with a function cell -> outcome (for tests and simple callers)."""
    while not fit.done:
        for cell in fit.pending():
            fit.add(cell, outcome_of(cell))
        fit.step()
    return fit


def inside_green(grid, edges=None):
    """The non-seated cells that lie inside the green zone of one grid ({(i, j): outcome}).

    Along a row or a column, a failing cell is *enclosed* if the green zone lies on both sides of
    it: a seated cell, or a grid edge the zone runs on past (``edges``, as GridFit.edges gives
    them: 'limit' or 'budget'). A failing cell is inside the zone if it is

    - enclosed along its row and along its column (a hole, or part of a band inside the zone), or
    - enclosed along one of them while the other has no green at all (a band cutting right across).

    The failures beyond the zone's outer edge are the capture limit, and are left out. So is a
    ragged edge: a failing cell with green above and below it but only on one side."""
    open_ = [[bool(edges) and edges[axis][side] in ("limit", "budget") for side in ("lo", "hi")] for axis in (0, 1)]
    rows, cols = {}, {}
    for (i, j), o in grid.items():
        if o == SEATED:
            rows.setdefault(j, []).append(i)
            cols.setdefault(i, []).append(j)

    def enclosed(k, green, axis):
        return (any(g < k for g in green) or open_[axis][0]) and (any(g > k for g in green) or open_[axis][1])

    out = []
    for (i, j), o in sorted(grid.items(), key=lambda kv: (kv[0][1], kv[0][0])):
        if o == SEATED:
            continue
        row, col = rows.get(j, []), cols.get(i, [])
        in_row, in_col = enclosed(i, row, 0), enclosed(j, col, 1)
        if (in_row and in_col) or (in_row and not col) or (in_col and not row):
            out.append((i, j))
    return out


# --- from the box and the quick check to index extents ---


def _ceil(v):
    return math.ceil(v - 1e-9)


def _floor(v):
    return math.floor(v + 1e-9)


def box_ranges(box):
    """The target box as (lo, hi) per axis, in mm and degrees about the aim."""
    w = box.half_widths
    return {"x": (-w["x_mm"], w["x_mm"]), "z": (0.0, 2 * w["z_mm"]), "t": (-w["theta_deg"], w["theta_deg"])}


def aim_point(box):
    """The aim, where the slices cross: zero X offset, the box's centre height, zero pitch."""
    return {"x": 0.0, "z": box.half_widths["z_mm"], "t": 0.0}


def index_extent(ranges, steps, axes, pad=0):
    """Index extents covering (lo, hi) ranges outward, widened by ``pad`` cells; z stays >= 0."""
    out = []
    for a in axes:
        lo, hi = _floor(ranges[a][0] / steps[a]) - pad, _ceil(ranges[a][1] / steps[a]) + pad
        out.append((max(lo, 0) if a == "z" else lo, hi))
    return tuple(out)


def check_ranges(check, box):
    """(lo, hi) per axis covering the quick check's drops, or the target box if there is no check."""
    ranges = box_ranges(box)
    if not check or not check.get("drops"):
        return ranges
    c0 = check["config"]["error"]["clearance"][0]
    got = {"x": [d["dx_mm"] for d in check["drops"]], "z": [d["clear_mm"] - c0 for d in check["drops"]],
           "t": [d["pitch_deg"] for d in check["drops"]]}
    return {a: (min(got[a]), max(got[a])) for a in ranges}


def cell_sample(cfg, slice_name, cell, steps, aim):
    """The trial sample (ErrorModel.sample keys) of one cell of a slice."""
    v = dict(aim)
    for axis, index in zip(SLICES[slice_name], cell):
        v[axis] = index * steps[axis]
    bias = cfg.error.aim_bias
    return {"dx": bias + v["x"], "dy": 0.0, "aim": bias, "yaw": 0.0, "roll": 0.0, "pitch": v["t"] + 0.0,
            "clear": cfg.error.clearance[0] + v["z"], "vel": [0.0, 0.0, 0.0]}


# --- the map ---


def _grid_summary(slice_name, course, extent, cells, edges, steps, U, box, aim, simulated):
    a, b = SLICES[slice_name]
    (i0, i1), (j0, j1) = extent
    ranges = box_ranges(box)
    counts = {}
    for o in cells.values():
        counts[o] = counts.get(o, 0) + 1
    in_box = [c for c in cells if all(ranges[ax][0] - 1e-9 <= k * steps[ax] <= ranges[ax][1] + 1e-9
                                      for ax, k in zip((a, b), c))]
    inside = inside_green(cells, edges)
    return {
        "slice": slice_name, "course": course, "axes": [a, b], "units": [AXIS_UNITS[a], AXIS_UNITS[b]],
        "fixed": {k: aim[k] for k in ("x", "z", "t") if k not in (a, b)},
        "step": {a: steps[a], b: steps[b]},
        "index_extent": {a: [i0, i1], b: [j0, j1]},
        "extent": {a: [i0 * steps[a], i1 * steps[a]], b: [j0 * steps[b], j1 * steps[b]]},
        "extent_U": {ax: [lo * steps[ax] / U, hi * steps[ax] / U] for ax, (lo, hi) in ((a, (i0, i1)), (b, (j0, j1)))
                     if ax != "t"},
        "edges": {a: edges[0], b: edges[1]},
        # One string per row, from the lowest value of the second axis to the highest; each
        # character is a cell along the first axis.
        "rows": ["".join(CODES.get(cells[(i, j)], OTHER) for i in range(i0, i1 + 1)) for j in range(j0, j1 + 1)],
        "counts": dict(sorted(counts.items())), "cells": len(cells), "simulated": simulated,
        "target_cells": len(in_box), "target_cells_seated": sum(cells[c] == SEATED for c in in_box),
        "inside_failures": [{"cell": list(c), a: c[0] * steps[a], b: c[1] * steps[b], "outcome": cells[c]}
                            for c in inside],
    }


def _edges_in_state(grids, state):
    return [f"{g['slice']} course {g['course']} {ax} {side}" for g in grids
            for ax, e in g["edges"].items() for side, st in e.items() if st == state]


def fixed_grid_drops(grids, half_width_mm=FIXED_GRID_MM):
    """Drops a fixed +-``half_width_mm`` X grid would take at the same cell sizes: the same rows,
    with X spanning the fixed width. Slices without an X axis count as they are."""
    total = 0
    for g in grids:
        (a, b), ext = g["axes"], g["index_extent"]
        rows = ext[b][1] - ext[b][0] + 1
        cols = 2 * _floor(half_width_mm / g["step"]["x"]) + 1 if a == "x" else ext[a][1] - ext[a][0] + 1
        total += rows * cols
    return total


def sim_map(family, p, box, check=None, config=None, config_reason=None, zooms=(), cfg=None, courses=(0, 1),
            slices=tuple(SLICES), workers=None, allow_failed_check=False, cover=None):
    """Map one design over the fitted slices; returns (map, drops), both JSON-safe.

    ``check`` is the quick check's result for this design and box (quick_check.quick_check): the
    grid starts from its drops, and a design that failed it is refused unless
    ``allow_failed_check``. ``config`` is a MapConfig; changing it from the default is an override
    and needs ``config_reason``. ``zooms`` are Zoom overrides. ``cfg`` is a trials.TrialConfig
    (default: the ideal error model, as the box search).

    ``cover``: target boxes the grid must also cover (the guardrail), so Z (engine/zscore.py, issue
    #61) can be counted on each of them; ``box`` is always covered, and its aim is where the slices
    cross. The loop passes the mean, p95 and worst boxes."""
    cover = _cover(box, cover)
    if box.task != "placement":
        raise ValueError(f"the sim map covers placement boxes (issue #46), got {box.task!r}")
    if check is not None and not check["passed"] and not allow_failed_check:
        raise ValueError("the design failed the quick check; the full map runs only for designs that pass it")
    default = MapConfig()
    config = config or default
    changed = {k: v for k, v in asdict(config).items() if v != getattr(default, k)}
    overrides = []
    if changed:
        if not (config_reason and config_reason.strip()):
            raise ValueError(f"grid settings changed ({sorted(changed)}): give config_reason, overrides are logged")
        overrides.append({"kind": "config", "changed": changed, "reason": config_reason.strip()})
    for s in slices:
        if s not in SLICES:
            raise ValueError(f"unknown slice {s!r}; expected one of {sorted(SLICES)}")
    for z in zooms:
        _check_zoom(z)
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    U = float(family.lattice(p).U_mm)
    steps, limits, aim = config.steps(U), config.limits(U), aim_point(box)
    start_ranges, must_ranges = check_ranges(check, box), _union(cover)
    fits = {}
    for s in slices:
        axes = SLICES[s]
        for course in courses:
            fits[(s, course)] = GridFit(index_extent(start_ranges, steps, axes, pad=config.margin),
                                        index_extent(must_ranges, steps, axes), tuple(limits[a] for a in axes),
                                        grow=config.grow, budget=max(1, config.max_drops // (len(slices) * len(courses))))
    t0 = time.time()
    drops = []
    with BS.DropPool(family, p, cfg, workers=workers or DEFAULT_WORKERS) as pool:
        t_ready = time.time()
        rounds = 0
        while not all(f.done for f in fits.values()):
            batch = [(key, cell) for key, f in fits.items() for cell in f.pending()]
            jobs = [(key[1], cell_sample(cfg, key[0], cell, steps, aim)) for key, cell in batch]
            for (key, cell), (_, sample), row in zip(batch, jobs, pool.imap(jobs)):
                d = QC.drop_row(len(drops), key[1], (0.0, 0.0, 0.0), sample, {**row, "contacts": row.get("contacts", [])})
                d.pop("unit")
                d.update(slice=key[0], cell=list(cell), contacts=row.get("contacts", []))
                fits[key].add(cell, d["outcome"])
                drops.append(d)
            for f in fits.values():
                f.step()
            rounds += 1
        grids = []
        for (s, course), f in fits.items():
            grids.append(_grid_summary(s, course, f.extent, f.kept(), f.edges(), steps, U, box, aim, len(f.cells)))
        kept = {(s, c): f.kept() for (s, c), f in fits.items()}
        for d in drops:
            d["kept"] = tuple(d["cell"]) in kept[(d["slice"], d["course"])]
        zoom_grids = []
        for z in zooms:
            zoom_grids.append(_run_zoom(z, pool, cfg, steps, U, box, aim, drops))
            overrides.append({"kind": "zoom", "slice": z.slice, "course": z.course, "extent": {k: list(v) for k, v in z.extent.items()},
                              "step": zoom_grids[-1]["step"], "reason": z.reason.strip(), "cells": zoom_grids[-1]["cells"]})
        used = pool.workers
    seconds = time.time() - t0
    fitted = len([d for d in drops if "zoom" not in d])
    out = {
        "label": LABEL, "task": "placement", "family": family.name, "family_version": family.version,
        "params": p.to_dict(), "box": box.to_dict(), "U_mm": U, "aim": aim,
        "cover": [b.statistic for b in cover], "cover_boxes": [b.to_dict() for b in cover],
        "config": json.loads(json.dumps(T.as_dict(cfg))), "map_config": asdict(config),
        "check": None if check is None else {"passed": check["passed"], "seated": check["seated"],
                                             "dropped": check["dropped"], "seed": check["seed"]},
        "codes": {**{v: k for k, v in CODES.items()}, OTHER: "other"},
        "grids": grids, "zooms": zoom_grids, "overrides": overrides,
        "drops": fitted, "cells_kept": sum(g["cells"] for g in grids),
        "fixed_grid_drops": fixed_grid_drops(grids), "fixed_grid_half_width_mm": FIXED_GRID_MM,
        # Edges that still have green on them: at an axis limit (the zone runs on past the grid),
        # or short of one because the drop budget ran out. Every other edge closed on failures.
        "limit_edges": _edges_in_state(grids, "limit"), "open_edges": _edges_in_state(grids, "budget"),
        "covers_target": all(g["target_cells"] == g["target_cells_seated"] for g in grids),
        "inside_failures": sum(len(g["inside_failures"]) for g in grids),
        "workers": used, "rounds": rounds, "seconds": seconds,
        "stage_seconds": {"build": t_ready - t0, "drops": seconds - (t_ready - t0)},
        "files": {"drops": DROPS_FILE},
    }
    return out, drops


def _cover(box, cover):
    """``box`` and the other boxes the grid must cover, one per statistic, ``box`` first."""
    out = [box]
    for b in cover or ():
        if b.task != "placement":
            raise ValueError(f"the map covers placement boxes only, got a {b.task!r} box in cover")
        if b.statistic != box.statistic and b.statistic not in [c.statistic for c in out]:
            out.append(b)
    return tuple(out)


def _union(boxes):
    rs = [box_ranges(b) for b in boxes]
    return {a: (min(r[a][0] for r in rs), max(r[a][1] for r in rs)) for a in ("x", "z", "t")}


def _check_zoom(z):
    if z.slice not in SLICES or z.course not in (0, 1):
        raise ValueError(f"a zoom needs a slice from {sorted(SLICES)} and course 0 or 1")
    if not (isinstance(z.reason, str) and z.reason.strip()):
        raise ValueError("a zoom needs a reason: overrides are logged for the design log (issue #20)")
    axes = SLICES[z.slice]
    if set(z.extent) != set(axes) or any(len(v) != 2 or not v[0] <= v[1] for v in z.extent.values()):
        raise ValueError(f"a zoom on {z.slice} needs an extent (lo, hi) for each of {axes}")
    if "z" in z.extent and z.extent["z"][0] < 0:
        raise ValueError("release height cannot go below the minimum clearance (z >= 0)")
    if z.step is not None and (set(z.step) != set(axes) or any(not v > 0 for v in z.step.values())):
        raise ValueError(f"a zoom's step needs a size > 0 for each of {axes}")


def _run_zoom(z, pool, cfg, steps, U, box, aim, drops):
    axes = SLICES[z.slice]
    zsteps = {**steps, **(z.step or {a: steps[a] / 2 for a in axes})}
    ext = [(_ceil(z.extent[a][0] / zsteps[a]), _floor(z.extent[a][1] / zsteps[a])) for a in axes]
    cells = [(i, j) for j in range(ext[1][0], ext[1][1] + 1) for i in range(ext[0][0], ext[0][1] + 1)]
    if not cells:
        raise ValueError(f"the zoom on {z.slice} holds no cell at its step; widen the extent or shrink the step")
    jobs = [(z.course, cell_sample(cfg, z.slice, c, zsteps, aim)) for c in cells]
    got = {}
    for cell, (_, sample), row in zip(cells, jobs, pool.imap(jobs)):
        d = QC.drop_row(len(drops), z.course, (0.0, 0.0, 0.0), sample, {**row, "contacts": row.get("contacts", [])})
        d.pop("unit")
        d.update(slice=z.slice, cell=list(cell), contacts=row.get("contacts", []), kept=True, zoom=z.reason.strip())
        got[cell] = d["outcome"]
        drops.append(d)
    edges = [{"lo": "override", "hi": "override"}] * 2
    g = _grid_summary(z.slice, z.course, ext, got, edges, zsteps, U, box, aim, len(got))
    g["reason"] = z.reason.strip()
    return g


# --- output ---


def ascii_grid(g):
    """One grid as text: the second axis runs up the page, the first across it."""
    a, b = g["axes"]
    lo = g["index_extent"][b][0]
    lines = [f"{g['slice']} course {g['course']}: {a} {g['extent'][a][0]:+.2f} to {g['extent'][a][1]:+.2f} "
             f"{AXIS_UNITS[a]} (left to right), step {g['step'][a]:g}"]
    for k in range(len(g["rows"]) - 1, -1, -1):
        lines.append(f"{b} {(lo + k) * g['step'][b]:+7.2f} {AXIS_UNITS[b]:<3} {g['rows'][k]}")
    return "\n".join(lines)


def summary(m):
    """A short text summary of a map, for the terminal."""
    lines = [f"{m['family']} placement {m['box']['statistic']} ({LABEL}): {m['drops']} drops in {m['seconds']:.1f} s "
             f"on {m['workers']} workers; a fixed +-{m['fixed_grid_half_width_mm']:g} mm grid at the same cell "
             f"sizes would take {m['fixed_grid_drops']}",
             f"covers the target box: {m['covers_target']}; failures inside the green zone: {m['inside_failures']}; "
             f"edges still green at an axis limit: {len(m['limit_edges'])}; "
             f"still green short of a limit (drop budget): {m['open_edges'] or 'none'}"]
    for g in m["grids"]:
        ext = ", ".join(f"{ax} {g['extent'][ax][0]:+.2f} to {g['extent'][ax][1]:+.2f} {AXIS_UNITS[ax]}"
                        + (f" ({g['extent_U'][ax][0]:+.2f} to {g['extent_U'][ax][1]:+.2f} U)" if ax in g["extent_U"] else "")
                        for ax in g["axes"])
        edges = ", ".join(f"{ax} {side} {st}" for ax, e in g["edges"].items() for side, st in e.items() if st != "closed")
        lines.append(f"  {g['slice']} course {g['course']}: {ext}; {g['cells']} cells, {g['counts']}; "
                     f"edges not closed: {edges or 'none'}")
    return "\n".join(lines)


def new_run_dir(root, family_name, statistic):
    """A new runs/map-<UTC stamp>-<family>-<target> directory; never reuses one."""
    root = Path(root)
    while True:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        path = root / f"map-{stamp}-{family_name}-{statistic}"
        try:
            path.mkdir(parents=True, exist_ok=False)
            return path
        except FileExistsError:
            continue


def _rounded(v, digits=4):
    """Floats rounded for storage (0.1 um, 0.1 ms, 0.1 mN): a trace holds about a hundred contacts."""
    if isinstance(v, float):
        return round(v, digits)
    if isinstance(v, dict):
        return {k: _rounded(x, digits) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_rounded(x, digits) for x in v]
    return v


def write_map(m, drops, runs="runs"):
    """Write map.json (the grids) and drops.json.gz (every drop with its contact trace, floats
    rounded to 4 decimals) to a new run directory; returns it."""
    text = json.dumps(m, indent=2, allow_nan=False) + "\n"   # validate both before making the directory
    drops_text = json.dumps(_rounded(drops), allow_nan=False, separators=(",", ":")) + "\n"
    out = new_run_dir(runs, m["family"], m["box"]["statistic"])
    (out / "map.json").write_text(text, encoding="utf-8")
    with gzip.open(out / m["files"]["drops"], "wt", encoding="utf-8") as f:
        f.write(drops_text)
    return out


def read_drops(run_dir):
    """The drops of a written map (a list of rows, in the order they were simulated)."""
    with gzip.open(Path(run_dir) / DROPS_FILE, "rt", encoding="utf-8") as f:
        return json.load(f)


def parser():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--family", default="ab")
    ap.add_argument("--statistic", choices=BX.STATISTICS, default="mean")
    ap.add_argument("--seed", type=int, default=0, help="the quick check's seed (the map itself has none)")
    ap.add_argument("--workers", type=int, default=None, help=f"parallel drop processes (default {DEFAULT_WORKERS})")
    ap.add_argument("--even-if-check-fails", action="store_true",
                    help="map a design that failed the quick check (the loop never does)")
    ap.add_argument("--ascii", action="store_true", help="print every grid")
    ap.add_argument("--runs", default="runs")
    return ap


def main(argv=None):
    from brixzle.families import registry

    args = parser().parse_args(argv)
    family = registry.get(args.family)
    box = BX.default_boxes()["placement"][args.statistic]
    check = QC.quick_check(family, family.defaults, box, seed=args.seed)
    print(QC.summary(check))
    if not check["passed"] and not args.even_if_check_fails:
        raise SystemExit("the design failed the quick check, so it is not mapped (pass --even-if-check-fails to map it anyway)")
    m, drops = sim_map(family, family.defaults, box, check=check, workers=args.workers,
                       allow_failed_check=args.even_if_check_fails)
    out = write_map(m, drops, args.runs)
    print(summary(m))
    if args.ascii:
        print("\n\n".join(ascii_grid(g) for g in m["grids"]))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
