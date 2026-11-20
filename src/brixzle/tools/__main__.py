"""python -m brixzle.tools <tool> ... (#163). Each tool writes runs/<tool>-<UTC>-<family>/result.json
(``evaluate`` writes eval.json to --out) and prints a short JSON summary. Simulated, uncalibrated contact.

    evaluate   <family> --out DIR [--stages map,gate,...]   the loop's stages, with the caches
    edgemap    <family>                                     half-supported seat map
    redrop     <family> --map DIR [selection] [--friction MU]  re-drop chosen map poses
    contacts   <family> --map DIR [selection]                  rest pose and contacts of chosen drops
    sweep      <family> --param NAME --values V,V,.. --map DIR [selection]
    slices     --map DIR                                        Z per box, slice and course (no sim)
    mapdiff    --a DIR --b DIR                                   two maps cell by cell (no sim)
    diffdesign <family> <family>                                 changed parameters and geometry
    gate | ring | friction <family>

A family is a registry name or <root>:<module>. Selection: --box mean|p95|max, --outcomes jammed,..,
--slices xz,.., --courses 0,1, --limit N, --seed S.
"""
import argparse
import json
import sys

from brixzle.tools import core as TC

SUMMARY_ROWS = 12


def _csv(s, cast=str):
    return [cast(x) for x in s.split(",") if x != ""] if s else None


def _selection(ap, needs_map=True):
    ap.add_argument("--map", required=needs_map, help="a map run directory (runs/map-...)")
    ap.add_argument("--box", choices=("mean", "p95", "max"))
    ap.add_argument("--outcomes")
    ap.add_argument("--slices")
    ap.add_argument("--courses")
    ap.add_argument("--edge-case", choices=("plus_bare", "minus_bare", "plus_anchored", "plus_cantilever"))
    ap.add_argument("--limit", type=int)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--friction", type=float)
    ap.add_argument("--workers", type=int, default=8)


def _drops(a):
    return TC.select_drops(TC.read_drops(a.map), box=a.box, outcomes=_csv(a.outcomes), slices=_csv(a.slices),
                           courses=_csv(a.courses, int), limit=a.limit, seed=a.seed, edge_case=a.edge_case)


def parser():
    ap = argparse.ArgumentParser(prog="python -m brixzle.tools", description=__doc__.splitlines()[0])
    ap.add_argument("--runs", default="runs")
    sub = ap.add_subparsers(dest="tool", required=True)
    e = sub.add_parser("evaluate")
    e.add_argument("family")
    e.add_argument("--out", required=True)
    e.add_argument("--stages", default=",".join(TC.STAGES))
    e.add_argument("--seed", type=int, default=0)
    e.add_argument("--no-cache", action="store_true")
    s = sub.add_parser("edgemap")
    s.add_argument("family")
    s.add_argument("--workers", type=int, default=8)
    s.add_argument("--no-cache", action="store_true")
    for name in ("redrop", "contacts"):
        s = sub.add_parser(name)
        s.add_argument("family")
        _selection(s)
    s = sub.add_parser("sweep")
    s.add_argument("family")
    s.add_argument("--param", required=True)
    s.add_argument("--values", required=True)
    _selection(s)
    s = sub.add_parser("slices")
    s.add_argument("--map", required=True)
    s = sub.add_parser("mapdiff")
    s.add_argument("--a", required=True)
    s.add_argument("--b", required=True)
    s = sub.add_parser("diffdesign")
    s.add_argument("a")
    s.add_argument("b")
    for name in ("gate", "ring", "friction"):
        s = sub.add_parser(name)
        s.add_argument("family")
        s.add_argument("--seed", type=int, default=0 if name != "friction" else 1)
        s.add_argument("--no-cache", action="store_true")
        if name == "friction":
            s.add_argument("--mus", default="0.5,0.6,0.7,0.8")
    return ap


def _short(res):
    out = {k: v for k, v in res.items() if k not in ("rows", "changed_cells")}
    for k in ("rows", "changed_cells"):
        if k in res:
            out[k] = res[k][:SUMMARY_ROWS]
            if len(res[k]) > SUMMARY_ROWS:
                out[f"{k}_more"] = len(res[k]) - SUMMARY_ROWS
    return out


def main(argv=None):
    a = parser().parse_args(argv)
    TC.cache_on() if not getattr(a, "no_cache", False) else None
    if a.tool == "evaluate":
        res = TC.evaluate(a.family, a.out, stages=tuple(_csv(a.stages)), seed=a.seed, cache=not a.no_cache)
        print(json.dumps(_short(res), indent=1, default=str))
        return 0
    if a.tool in ("slices", "mapdiff"):
        res = TC.slices(a.map) if a.tool == "slices" else TC.mapdiff(a.a, a.b)
        name = "map"
    elif a.tool == "diffdesign":
        fa, fb = TC.load(a.a), TC.load(a.b)
        res, name = TC.diffdesign(fa, fb), f"{fa.name}-vs-{fb.name}"
    else:
        fam = TC.load(a.family)
        p, name = fam.defaults, fam.name
        if a.tool == "redrop":
            res = TC.redrop(fam, p, _drops(a), friction=a.friction, workers=a.workers)
        elif a.tool == "edgemap":
            res = TC.edgemap(fam, p, workers=a.workers)
        elif a.tool == "contacts":
            res = TC.contacts(fam, p, _drops(a), friction=a.friction, workers=a.workers)
        elif a.tool == "sweep":
            res = TC.sweep(fam, a.param, _csv(a.values, float), _drops(a), friction=a.friction, workers=a.workers)
        elif a.tool == "gate":
            res = TC.gate(fam, p, seed=a.seed, runs=a.runs, cache=not a.no_cache)
        elif a.tool == "ring":
            res = TC.ring(fam, p, seed=a.seed, runs=a.runs, cache=not a.no_cache)
        else:
            res = TC.friction(fam, p, mus=_csv(a.mus, float), seed=a.seed)
    res = TC.write(TC.new_run(a.tool, name, a.runs), res)
    print(json.dumps(_short(res), indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
