"""The design library (#215): every past design's code, maps, scores and failure analysis, for the
roles to read.

``runs/library/`` (git-ignored, shared by every worktree through the ``runs/`` link) holds one folder
per design and an ``INDEX.md`` with one line each:

    <name>/family.py            the code (copyable; the maintainer's own designs may be reused)
    <name>/card.md              the idea, scores per placement, strengths and weaknesses with numbers,
                                the jam bands with named contacts, and the critic's diagnosis
    <name>/scores.json          Z (#222: the stack and overhangs 1-3 in the 16 mm disc), green zone and
                                Z per placement, overhang, stack (finalists only), rules, sources
    <name>/map-<placement>.png  the grid maps; jams.json the failure analysis
    <name>/module.png, *.stl    three orthographic views and the printable solid

``decide`` adds each iteration once its critique exists (``add_iteration``), and
``python -m brixzle.director.library add <spec> --grid <dir>`` adds a registry family or any design
from an earlier grid run. A library entry is never a parent: the roles read it, and refinement
happens only inside a run's archive. All sim results are simulated, uncalibrated contact.
"""
import argparse
import json
from pathlib import Path
import shutil

LABEL = "simulated, uncalibrated contact"
RULES = "222"   # the score the entries were measured under: the #208 grid, #218 free drop, #222 Z on four quarters
FLOOR_MM = 16.0


def _idea(text, limit=400):
    """The first plain paragraph of a note or a docstring."""
    for para in (text or "").split("\n\n"):
        p = " ".join(line.strip() for line in para.strip().splitlines())
        if p and not p.startswith("#") and not p.startswith("```"):
            return p[:limit]
    return ""


def entry_from(name, spec, origin, grid, builds=None, idea="", diagnosis=None, violations=(), verdict=None,
               sources=None):
    """One library entry, from a grid record (``gridscore`` grid.json or ``measured.grid``)."""
    builds = builds or {}
    places = grid.get("placements") or {}
    return {"name": name, "spec": spec, "origin": origin, "idea": idea, "rules": RULES, "label": LABEL,
            "z": grid.get("z"), "z_quarters": grid.get("z_quarters") or {}, "z_mean_rings": grid.get("z_rings") or {},
            "green_mm": grid.get("green_mm"), "weakest": grid.get("weakest"),
            "green_by_placement": {pl: v["green_mm"] for pl, v in places.items()},
            "z_rings": {pl: {k: z["Z"] for k, z in v.get("rings", {}).items()} for pl, v in places.items()},
            "jams": list(grid.get("jams_summary") or []),
            "overhang_steps": builds.get("overhang_steps"), "stack_courses": builds.get("stack_courses"),
            "build_radius_mm": builds.get("radius_mm"), "violations": list(violations or []),
            "verdict": verdict, "diagnosis": diagnosis, "sources": sources or {}}


def _num(v):
    return "–" if v is None else f"{round(float(v), 2):g}"


def card_md(e):
    """The card: what the design is, what it does well and badly, with numbers, and why it failed."""
    greens = e["green_by_placement"]
    order = sorted(greens, key=greens.get)
    lines = [f"# {e['name']}", "",
             f"`{e['spec']}`, from {e['origin']}. Scored under rules {e['rules']}; every number is {LABEL}.", ""]
    if e.get("idea"):
        lines += ["## Idea", "", e["idea"], ""]
    zr = e.get("z_mean_rings") or {}
    builds = (f" Noisy builds at {_num(e['build_radius_mm'])} mm: overhang {_num(e['overhang_steps'])}, "
              f"stack {_num(e['stack_courses'])}." if e.get("build_radius_mm") is not None else "")
    lines += ["## Scores", "",
              f"Z {_num(e.get('z'))} (the mean over the stack and overhangs 1-3 of Z in the 16 mm disc; Z28 "
              f"{_num(zr.get('28'))}, Z40 {_num(zr.get('40'))}). Green zone {_num(e['green_mm'])} mm "
              f"(weakest: {e['weakest']}).{builds}",
              "", "| placement | green zone (mm) | Z16 | Z28 | Z40 |", "|---|---|---|---|---|"]
    for pl in order:
        z = e["z_rings"].get(pl, {})
        lines.append(f"| {pl} | {_num(greens[pl])} | {_num(z.get('16'))} | {_num(z.get('28'))} | {_num(z.get('40'))} |")
    if e.get("violations"):
        lines += ["", f"Hard constraints broken: {', '.join(e['violations'])}."]
    if e.get("verdict"):
        lines += ["", f"Archive: {e['verdict']}."]
    strong = [pl for pl in order if greens[pl] >= FLOOR_MM]
    weak = [pl for pl in order if greens[pl] < FLOOR_MM]
    lines += ["", "## Strengths", ""]
    lines += [f"- {pl}: {_num(greens[pl])} mm, past the {FLOOR_MM:g} mm floor" for pl in reversed(strong)] or [
        f"- none of the placements reaches the {FLOOR_MM:g} mm floor"]
    lines += ["", "## Weaknesses", ""]
    lines += [f"- {pl}: {_num(greens[pl])} mm" for pl in weak] or ["- every placement reaches the floor"]
    if e.get("jams"):
        lines += ["", "What bounds the green zones (jam analysis; contacts named on both bodies):", "", "```"]
        lines += e["jams"] + ["```"]
    if e.get("diagnosis"):
        lines += ["", "## The critic's diagnosis", "", e["diagnosis"].strip()]
    if e.get("sources"):
        lines += ["", "## Sources", ""] + [f"- {k}: `{v}`" for k, v in e["sources"].items()]
    return "\n".join(lines) + "\n"


def write_index(lib):
    """``INDEX.md``: one line per design, best Z first (#222); an entry without Z (scored before #222) last."""
    lib = Path(lib)
    rows = []
    for p in sorted(lib.glob("*/scores.json")):
        s = json.loads(p.read_text())
        greens = s.get("green_by_placement") or {}
        per = ", ".join(f"{pl} {_num(g)}" for pl, g in sorted(greens.items(), key=lambda kv: kv[1]))
        quarters = ", ".join(f"{pl} {_num(z)}" for pl, z in (s.get("z_quarters") or {}).items())
        rows.append((-2.0 if s.get("z") is None else s["z"],
                     f"| [{s['name']}]({s['name']}/card.md) | {s['origin']} | {(s.get('idea') or '')[:120]} | "
                     f"{_num(s.get('z'))} | {quarters or '–'} | {per} | {s.get('rules') or '–'} | "
                     f"{s.get('verdict') or '–'} |"))
    text = ["# Design library", "",
            f"Every design measured so far, best Z first (#222: the mean over the stack and overhangs 1-3 of "
            f"Z = (S - J) / N in the 16 mm disc). An entry scored under older rules has no Z: rescore it to compare. "
            f"Open a card for its idea, scores, strengths, weaknesses and the contacts that bound it. Read, don't "
            f"refine: no entry is a parent. All numbers are {LABEL}.", "",
            "| design | from | idea | Z | Z per quarter | green zone per placement (mm, weakest first) | rules | "
            "archive |", "|---|---|---|---|---|---|---|---|"]
    text += [r for _, r in sorted(rows, key=lambda t: -t[0])]
    (lib / "INDEX.md").write_text("\n".join(text) + "\n")


def write_entry(lib, entry, files):
    """Write (or replace) ``<lib>/<name>/``: card, scores and the ``files`` ({name: source path}), then
    the index. Returns the folder."""
    lib = Path(lib)
    out = lib / entry["name"]
    out.mkdir(parents=True, exist_ok=True)
    for name, src in files.items():
        if src and Path(src).exists():
            shutil.copyfile(src, out / name)
    (out / "card.md").write_text(card_md(entry))
    (out / "scores.json").write_text(json.dumps({k: v for k, v in entry.items() if k != "diagnosis"}, indent=1) + "\n")
    write_index(lib)
    return out


def _renders(spec, out):
    """The three orthographic views and the STL of each part; skipped (not fatal) if they cannot be made."""
    try:
        from brixzle.tools import render as RD
        from brixzle.tools import videos as VD
        from brixzle.tools import core as TC

        fam = TC.load(spec)
        RD.module_png(fam, fam.defaults, Path(out) / "module.png", title=fam.name)
        VD.export_stl(fam, fam.defaults, out)
    except Exception as e:  # a render is a convenience, never a reason to lose the card
        (Path(out) / "render-error.txt").write_text(f"{type(e).__name__}: {e}\n")


def _docstring(path):
    import ast
    try:
        return ast.get_docstring(ast.parse(Path(path).read_text())) or ""
    except (OSError, SyntaxError):
        return ""


def _bases(family_file):
    """The shared base a thin family subclasses (the twinhook line, #162), copied beside it so the card's
    code is complete: {"<package>_base.py": path}."""
    import re
    from brixzle import families

    try:
        text = Path(family_file).read_text()
    except OSError:
        return {}
    out = {}
    for pkg in set(re.findall(r"from brixzle\.families\.(\w+)(?:\.base)? import", text)):
        base = Path(families.__file__).parent / pkg / "base.py"
        if base.exists():
            out[f"{pkg}_base.py"] = base
    return out


def _grid_files(grid_dir):
    grid_dir = Path(grid_dir)
    files = {p.name: p for p in grid_dir.glob("map-*.png")}
    files["jams.json"] = grid_dir / "jams.json"
    return files


def add_iteration(run, n, lib, renders=True):
    """Add iteration ``n`` of ``run`` from its own files: the measurement, the design's code, the
    designer's note (the idea) and the critic's diagnosis. None if it was not measured."""
    run = Path(run)
    d = run / f"iter-{int(n):02d}"
    vpath = d / "verified.json"
    if not vpath.exists():
        return None
    v = json.loads(vpath.read_text())
    m = v.get("measured") or {}
    if not m.get("grid"):
        return None
    spec = m["spec"]
    module = spec.rsplit(":", 1)[-1]
    note = d / "design_note.md"
    diag = d / "diagnosis.md"
    offer = v.get("offer") or {}
    verdict = None
    if offer:
        verdict = f"{'kept' if offer.get('kept') else 'not kept'}: {offer.get('reason')}"
    entry = entry_from(name=f"{run.name}-iter{int(n):02d}-{module}", spec=spec, origin=f"{run.name}/iter-{int(n):02d}",
                       grid=m["grid"], builds=m.get("builds"), idea=_idea(note.read_text() if note.exists() else ""),
                       diagnosis=diag.read_text() if diag.exists() else None, violations=m.get("violations"),
                       verdict=verdict, sources={"iteration": str(d), "grid": str(d / "measured" / "grid")})
    files = {"family.py": d / module / "family.py", **_bases(d / module / "family.py"),
             **_grid_files(d / "measured" / "grid")}
    out = write_entry(lib, entry, files)
    if renders:
        _renders(spec, out)
    return out


def add_design(spec, grid_dir, lib, builds=None, name=None, renders=True, diagnosis=None, origin=None):
    """Add a registry family (or ``<root>:<module>``) from a grid run's folder (``gridscore --out``)."""
    from brixzle.tools import core as TC

    grid_dir = Path(grid_dir)
    grid = json.loads((grid_dir / "grid.json").read_text())
    if builds is None and (grid_dir / "builds.json").exists():  # gridscore --out DIR --builds writes it
        builds = json.loads((grid_dir / "builds.json").read_text())
    fam = TC.load(spec)
    root, module = TC._root_module(spec)
    module_file = Path(root) / module / "family.py"
    run_dir = grid_dir.parent.parent if grid_dir.name == "grid" and grid_dir.parent.name == "measured" else grid_dir
    measured = grid_dir.parent / "measured.json"  # a verify's measurement beside its grid: the constraints broken
    violations = json.loads(measured.read_text()).get("violations") if measured.exists() else ()
    entry = entry_from(name=name or fam.name, spec=spec, origin=origin or f"run {run_dir.name}", grid=grid,
                       builds=builds, idea=_idea(_docstring(module_file)), violations=violations,
                       diagnosis=Path(diagnosis).read_text() if diagnosis else None,
                       sources={"grid": str(grid_dir), "code": str(module_file)})
    out = write_entry(lib, entry, {"family.py": module_file, **_bases(module_file), **_grid_files(grid_dir)})
    if renders:
        _renders(spec, out)
    return out


def default_library(run):
    """The shared library beside a run's ``runs/rsi/`` folder: ``runs/library``."""
    return Path(run).resolve().parents[1] / "library"


def main(argv=None):
    """python -m brixzle.director.library add <spec> --grid <dir> [--builds builds.json] [--lib runs/library]
    python -m brixzle.director.library add-run <run> [--lib runs/library]      every measured iteration
    python -m brixzle.director.library index [--lib runs/library]"""
    ap = argparse.ArgumentParser(prog="python -m brixzle.director.library")
    sub = ap.add_subparsers(dest="cmd", required=True)
    a1 = sub.add_parser("add")
    a1.add_argument("spec")
    a1.add_argument("--grid", required=True)
    a1.add_argument("--builds", help="a builds.json (radius.noisy_builds) to include")
    a1.add_argument("--name")
    a1.add_argument("--diagnosis", help="a critic's diagnosis.md to include (e.g. from the run that made it)")
    a1.add_argument("--origin", help="where the design came from, for the card (default: the grid run)")
    a2 = sub.add_parser("add-run")
    a2.add_argument("run")
    sub.add_parser("index")
    for p in (a1, a2, sub.choices["index"]):
        p.add_argument("--lib", default="runs/library")
    a = ap.parse_args(argv)
    lib = Path(a.lib)
    if a.cmd == "add":
        builds = json.loads(Path(a.builds).read_text()) if a.builds else None
        print(add_design(a.spec, a.grid, lib, builds=builds, name=a.name, diagnosis=a.diagnosis, origin=a.origin))
    elif a.cmd == "add-run":
        for d in sorted(Path(a.run).glob("iter-*")):
            out = add_iteration(a.run, int(d.name.split("-")[1]), lib)
            if out:
                print(out)
    else:
        lib.mkdir(parents=True, exist_ok=True)
        write_index(lib)
        print(lib / "INDEX.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
