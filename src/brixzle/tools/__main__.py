"""python -m brixzle.tools <tool> ... (#163). Each tool writes runs/<tool>-<UTC>-<family>/result.json
and prints a short JSON summary. Simulated, uncalibrated contact.

    edgemap    <family>                                     half-supported seat map
    redrop     <family> --map DIR [selection] [--friction MU]  re-drop chosen poses (a grid dir's drops.json.gz)
    contacts   <family> --map DIR [selection]                  rest pose and contacts of chosen drops
    sweep      <family> --param NAME --values V,V,.. --map DIR [selection]
    diffdesign <family> <family>                                 changed parameters and geometry
    gate | ring <family>

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
    s = sub.add_parser("diffdesign")
    s.add_argument("a")
    s.add_argument("b")
    for name in ("gate", "ring"):
        s = sub.add_parser(name)
        s.add_argument("family")
        s.add_argument("--seed", type=int, default=0)
        s.add_argument("--no-cache", action="store_true")
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
    if a.tool == "diffdesign":
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
        else:
            res = TC.ring(fam, p, seed=a.seed, runs=a.runs, cache=not a.no_cache)
    res = TC.write(TC.new_run(a.tool, name, a.runs), res)
    print(json.dumps(_short(res), indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
