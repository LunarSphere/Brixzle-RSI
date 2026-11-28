"""The verifier (#165): every number a designer claims is rerun, not taken on trust.

A claim (``claim.json``) lists numbers as {what, path, value}, where ``path`` names the number in the
verifier's own measurement (dotted keys, for example ``z_quarters.corbel_reach`` or
``gate.overhang_n``). ``measure_radius`` measures the candidate itself (the drop cache only reads
drops the engine simulated for the same geometry), and ``compare`` checks each claim against it. A
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


# --- the error-radius stage (#206) ---


def _part(path, fn):
    """Read a saved part of a measurement, or compute and save it (a cut-off step resumes here)."""
    path = Path(path)
    if path.exists():
        return json.loads(path.read_text())
    from brixzle.tools import core as TC
    rec = TC._finite(fn())
    write_json(path, rec)
    return rec


def measure_radius(spec, out_dir, grid_settings=None, build_settings=None, workers=None):
    """The candidate's measurement for the error-radius goal (#206; the grid since #208; scored as Z on
    four quarters since #222), cheapest first:

    1. ``check``: the family contract and rule margins (``check_family``), and the gripper interface
       (``engine/gripper_spec.target_check``);
    2. ``gate``: the MVP build with ideal placement, a stack of 3 and an overhang of at least 3;
    3. ``screen``: the quarter screen (``engine/quick_check.quarter_screen``): 40 random drops in the
       16 mm disc on the stack and overhangs 1-3, at least 32 seated. A design that fails it skips
       the grid and keeps the screen's Z, with the violation ``screen``;
    4. ``grid``: the grid score (``engine/gridscore.grid_search``) on the four quarters: ``z``, the
       mean of each quarter's Z in the 16 mm disc, ``z_quarters``, ``z_rings`` (Z16, Z28, Z40),
       ``green_mm`` as a diagnostic, the jam analysis (``grid/jams.json``) and one map per placement
       (``grid/map-<placement>.png``);
    5. ``builds``: only when ``build_settings`` is given (for finalists, #222): the noisy tower and
       corbel at a fixed 16 mm of error (#210), each corbel failure replayed on its layout with its
       contacts reduced (``engine/radius.noisy_builds``), only for a brick that passed the gate.

    Each part is saved as it finishes (``check.json``, ``gate.json``, ``screen.json``,
    ``grid/grid.json``, ``builds.json``), so a step cut off at 10 minutes resumes. A brick that fails
    the check stops there. Every result is simulated, uncalibrated contact."""
    from brixzle.tools import check_family as CF
    from brixzle.engine import gridscore as GS
    from brixzle.engine import gripper_spec as GSP
    from brixzle.engine import quick_check as QC
    from brixzle.engine import radius as RA
    from brixzle.engine import structure_gate as SG
    from brixzle.tools import core as TC

    out_dir = Path(out_dir)
    done = out_dir / "measured.json"
    if done.exists():
        return json.loads(done.read_text())
    out_dir.mkdir(parents=True, exist_ok=True)
    TC.cache_on()
    t0 = time.time()
    gs = grid_settings or GS.GridSettings()
    bs = build_settings  # None: no noisy builds (#222)
    root, module = TC._root_module(spec)
    seconds = {}

    def timed(name, fn):
        def run():
            t = time.time()
            rec = fn()
            seconds[name] = round(time.time() - t, 1)
            return rec
        return run

    def check():
        chk = CF.check_in_subprocess(root, module)
        rec = {k: chk.get(k) for k in ("ok", "violations", "mass_g", "error", "size_mm") if k in chk}
        rec["broken"] = sorted({b for part in (chk.get("parts") or {}).values() for b in part.get("broken", [])})
        if chk.get("ok") or not chk.get("violations"):
            try:
                fam = TC.load(spec)
                rec["gripper"] = GSP.target_check(fam, fam.defaults)
            except Exception as e:  # a family that cannot even be loaded has failed the check already
                rec["gripper"] = {"violations": [f"{type(e).__name__}: {e}"[:300]]}
        return rec

    measured = {"stage": "radius", "score": "grid", "label": RA.LABEL, "spec": spec}
    measured["check_family"] = _part(out_dir / "check.json", timed("check", check))
    violations = []
    if measured["check_family"].get("ok") is not True:
        violations.append("check_family")
    if (measured["check_family"].get("gripper") or {}).get("violations"):
        violations.append("gripper_interface")
    if "check_family" not in violations:
        fam = TC.load(spec)
        p = fam.defaults

        def gate():
            g = SG.run_gate(fam, p, SG.MVP, runs=str(out_dir / "runs"), workers=workers)
            return {k: g[k] for k in ("passed", "overhang_n", "builds", "seconds", "run")}

        measured["gate"] = _part(out_dir / "gate.json", timed("gate", gate))
        if not measured["gate"].get("passed"):
            violations.append("mvp_gate")
        measured["screen"] = _part(out_dir / "screen.json", timed("screen", lambda: QC.quarter_screen(fam, p)))
        if not measured["screen"].get("passed"):
            violations.append("screen")
        else:
            measured["grid"] = _part(out_dir / "grid" / "grid.json", timed("grid", lambda: GS.write_grid(
                GS.grid_search(fam, p, gs, workers=workers), out_dir / "grid", getattr(fam, "name", None))))
            if measured["grid"].get("error"):
                violations.append("placements")
        if bs is not None and measured["gate"].get("passed"):
            measured["builds"] = _part(out_dir / "builds.json", timed("builds", lambda: RA.noisy_builds(
                fam, p, None, bs, workers=workers)))
    g = measured.get("grid") or {}
    b = measured.get("builds") or {}
    greens = {pl: v["green_mm"] for pl, v in (g.get("placements") or {}).items()}
    order = sorted(greens.values())
    sc = measured.get("screen") or {}
    z = g.get("z") if g else sc.get("z")  # a design that failed the screen keeps the screen's Z
    measured.update(z=z, z_quarters=g.get("z_quarters") or {}, z_rings=g.get("z_rings") or {},
                    green_mm=g.get("green_mm", 0.0), floor_passed=bool(g.get("floor_passed")),
                    weakest=g.get("weakest"), green_by_placement=greens,
                    second_mm=order[1] if len(order) > 1 else (order[0] if order else 0.0),
                    build_radius_mm=b.get("radius_mm"), maps=g.get("maps", []), jams=g.get("jams_file"),
                    overhang_steps=b.get("overhang_steps", 0.0), stack_courses=b.get("stack_courses", 0.0),
                    violations=violations, stage_seconds=seconds, seconds=round(time.time() - t0, 1))
    write_json(done, measured)
    return measured
