"""Load and check a family the design agent wrote (issue #18).

The design agent writes a package ``<root>/<module>/`` whose ``family.py`` exports ``FAMILY``,
outside ``src/``. It is imported as the top-level package ``<module>``, with ``<root>`` on
``sys.path``. Worker processes started with spawn get the parent's ``sys.path``, so the sim's
workers can import it too. Module names are unique per attempt, so two designs never share one.

``check`` runs the family contract, the rule margins and one ideal drop at the aim on each course
(issue #70; a passing check does not need the drops to seat, but reports them). The agent calls it as a tool, in a
separate process with a time limit (``check_in_subprocess``), so a design that hangs or crashes
cannot take the loop down with it:

    uv run python -m brixzle.agents.check_family --root <root> --module <module>
"""

import argparse
from dataclasses import asdict
import importlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

CHECK_TIMEOUT_S = 240


def load_family(root, module):
    """The FAMILY of ``<root>/<module>/family.py``, imported as package ``module``."""
    root = str(Path(root).resolve())
    cached = sys.modules.get(module)
    if cached is not None and not str(Path(getattr(cached, "__file__", "") or "").resolve()).startswith(root + "/"):
        # The same package name was imported from another root (another run or test): drop it and
        # its submodules, so this root's package is the one imported.
        for name in [n for n in sys.modules if n == module or n.startswith(module + ".")]:
            del sys.modules[name]
    if root in sys.path:
        sys.path.remove(root)
    sys.path.insert(0, root)
    importlib.invalidate_caches()
    mod = importlib.import_module(f"{module}.family")
    from brixzle.families.interface import Family

    fam = getattr(mod, "FAMILY", None)
    if not isinstance(fam, Family):
        raise ValueError(f"{module}/family.py must export FAMILY = Family(...)")
    return fam


def _num(v):
    v = float(v)
    return round(v, 4) if math.isfinite(v) else None


def check(root, module, samples=2, reference=None, share=None):
    """The contract violations, rule margins, mass and size of a written family, JSON-safe.
    ``reference`` (an explore call, #106): a registry family the design must not be a near copy of
    (loop/novelty.py); a near copy is not ok."""
    from brixzle.families import builders as B
    from brixzle.families import contract as C
    from brixzle.families import rules as R
    from brixzle.families.interface import Prismatic

    t0 = time.time()
    out = {"module": module, "ok": False, "violations": [], "parts": {}, "error": None}
    try:
        fam = load_family(root, module)
    except Exception as e:  # noqa: BLE001 - reported to the agent
        out["error"] = f"import failed: {type(e).__name__}: {e}"
        return out
    out.update(family=fam.name, version=fam.version, geometry=type(fam.geometry).__name__,
               params=fam.defaults.to_dict(), bounds={k: list(v) for k, v in fam.bounds.items()})
    try:
        out["violations"] = [str(v) for v in C.check_family(fam, samples=samples)]
    except Exception as e:  # noqa: BLE001
        out["error"] = f"contract check raised {type(e).__name__}: {e}"
        return out
    if not out["violations"]:
        try:
            p = fam.defaults
            for part in fam.parts:
                bodies = B.sim_bodies(fam, p, part)
                margins = {k: _num(v) for k, v in R.margins(fam, p, part, bodies).items()}
                info = {"margins": margins, "broken": sorted(k for k, v in margins.items() if v is None or v < 0),
                        "mass_g": _num(bodies.mass_g)}
                if isinstance(fam.geometry, Prismatic):
                    sec = fam.geometry.section(p, part)
                    x0, z0, x1, z1 = sec.outer.bounds
                    info["section_mm"] = {"x": [_num(x0), _num(x1)], "z": [_num(z0), _num(z1)]}
                out["parts"][part] = info
            out["lattice"] = {k: _num(v) for k, v in asdict(fam.lattice(p)).items()}
        except Exception as e:  # noqa: BLE001
            out["error"] = f"rule check raised {type(e).__name__}: {e}"
            return out
    if not out["violations"] and not out["error"]:
        try:
            out["aim_drops"] = aim_drops(fam, fam.defaults)
        except Exception as e:  # noqa: BLE001 - reported to the agent
            out["error"] = f"the aim drop raised {type(e).__name__}: {e}"
            return out
    if not out["violations"] and not out["error"]:
        out["ring"] = ring_report(fam)  # reported, not part of ok: the loop's ring build judges it (#89)
    if reference and not out["violations"] and not out["error"]:
        from brixzle.families import registry
        from brixzle.loop import novelty as NV

        ref = registry.get(reference)
        nv = NV.near_copy(fam, fam.defaults, ref, ref.defaults, share=NV.NEAR_COPY_SHARE if share is None else share)
        out["novelty"] = {"near_copy": bool(nv["near_copy"]), "share": _num(nv["share"]) if nv["share"] is not None else None,
                          "reason": nv["reason"]}
    out["ok"] = (not out["violations"] and not any(i["broken"] for i in out["parts"].values()) and not out["error"]
                 and not (out.get("novelty") or {}).get("near_copy"))
    out["seconds"] = round(time.time() - t0, 2)
    return out


def ring_report(fam):
    """Whether the family has a ring kit (issue #89) and whether its parts, plan and base build.
    A design without one is rejected by the loop's ring build, after its map."""
    from brixzle.families import builders as B

    if fam.ring is None:
        return {"kit": False, "parts": [], "built": False,
                "error": "no ring kit: set Family.ring (the simplest is engine/ring_cells.kit_from_cell from one "
                         "cell); a design that cannot close a ring is rejected"}
    out = {"kit": True, "parts": sorted(fam.ring.parts), "built": False, "error": None}
    try:
        p = fam.defaults
        plan = fam.ring.plan(p, 6, 2)
        for part in dict.fromkeys(b.part for b in plan):
            B.ring_bodies(fam, p, part)
        if not fam.ring.base(p, 6):
            raise ValueError("the ring base is empty")
        out["built"] = True
    except Exception as e:  # noqa: BLE001 - reported to the agent
        out["error"] = f"the ring kit raised {type(e).__name__}: {e}"
    return out


def aim_drops(fam, p):
    """One ideal drop at the aim on each course (issue #70): the outcome, where the brick came to
    rest against its seat (mm) and its tilt (deg). A design that cannot seat even here will fail the
    quick check, so the agent learns it in about a second. Simulated, uncalibrated contact."""
    from brixzle.engine.sim import trials as T

    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    out = []
    for course in (0, 1):
        row = T.DropRig(fam, p, cfg, course=course).drop(cfg.error.sample(None), trace=False)
        out.append({"course": course, "outcome": row["outcome"], "err_mm": [_num(v) for v in row["err_mm"]],
                    "angle_deg": _num(row["angle_deg"])})
    return out


def check_in_subprocess(root, module, timeout=CHECK_TIMEOUT_S, reference=None, share=None):
    """``check`` in a fresh process with a time limit; a crash or a hang is reported, not raised."""
    cmd = [sys.executable, "-m", "brixzle.agents.check_family", "--root", str(root), "--module", module]
    if reference:
        cmd += ["--reference", reference] + (["--share", str(share)] if share is not None else [])
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"module": module, "ok": False, "violations": [], "parts": {},
                "error": f"the check took longer than {timeout} s"}
    try:
        return json.loads(done.stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError):
        tail = (done.stderr or "").strip().splitlines()[-5:]
        return {"module": module, "ok": False, "violations": [], "parts": {},
                "error": f"the check crashed (exit {done.returncode}): {' | '.join(tail)}"}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--root", required=True)
    ap.add_argument("--module", required=True)
    ap.add_argument("--reference", help="a registry family the design must not be a near copy of (#106)")
    ap.add_argument("--share", type=float, default=None)
    args = ap.parse_args(argv)
    print(json.dumps(check(args.root, args.module, reference=args.reference, share=args.share), allow_nan=False))


if __name__ == "__main__":
    main()
