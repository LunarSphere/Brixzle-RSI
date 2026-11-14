"""Orchestrator (issue #21): runs the design loop headless, from intake to a tuned brick family.

**Iteration 0** (#85): the parent family (``--parent``, A/B by default) is scored with the same
stages as a design, with no Claude call; it becomes the best, so every design must beat it, and the
critic's first brief comes from its results. ``--parent none`` starts from scratch;
``--baseline-only`` scores the parent and stops.

**Explore rounds** (#106): the first design iteration, and the one after a brief that asks for a
new family or after ``stall_iterations`` iterations whose Z moved by no more than the noise band
(#110), make ``concepts`` designs side by side, each from its own capture principle
(rules/capture_principles.md), with A/B as the score to beat, not the starting file; a near copy
of A/B's section is refused. The cheap screen (#108) picks which to score in full, and the best of
those by Z on the target goes through the decision below as the iteration's design.

Each outer iteration:

1. **Design:** the design agent (#18) writes a family package, from the concept, the critic's
   last brief and the current best design. The loop checks the package itself (contract and
   rules); a package that fails goes straight back to the agent with the check's output. Inside
   its call the agent may probe its draft (#107): up to ``probe_calls`` times, 30 drops at the best
   design's map failures on the current target (never the held-out poses).
2. **Quick check** (#47): 40 drops in the target box. Below 32 seated, the design goes straight
   back to the agent with the failing drops. Up to ``design_attempts`` tries per iteration.
3. **Full sim map** (#48), covering the mean, p95 and worst boxes, and **Z** per target (#61).
4. **Jam analysis** (#49, #56).
5. **Acceptance:** the design replaces the current best only if Z on the current target improves,
   Z on no lower rung of the ladder falls below the best's, and held-out Z on the current target
   (its poses shifted by another seed, at friction 0.2 and 0.5) is not worse on either by more than
   the noise band, one standard error of Z (#110). Without a
   parent, the first design that is scored becomes the best. The critic never sees the held-out
   results.
6. **Critic** (#19): a redesign brief that cites the design rules; back to 1.

Every attempt goes in the design log (#20). Every iteration writes ``RSI_RESULTS/<run>/iter-NN/``
(jam_map.png, module.png, summary.md; #15) and the run's ``progress.png``. Engine results go to
``runs/`` as usual; the loop's own files go to ``runs/loop/<run>/`` (state.json, the design
log, the spend ledger and each attempt's package).

**Target ladder** (#84): the target climbs mean -> p95 -> max; the current target is the first
rung, from ``--target`` up, on which the best's Z is below 1 (``--no-ladder``: ``--target`` only).

**Stop** when Z reaches 1 on the last rung, Z stops improving (``patience`` iterations with no
kept design), the critic says stop, or the iterations, the API budget or the wall-clock run out.
Then the end tune (#14) runs on the best design, with U fixed if intake fixed it.

**Resume:** ``state.json`` is written after every iteration; ``--resume <run>`` carries on from the
next one. Placement only (#46). Sim results are simulated, uncalibrated contact.

    uv run --extra agents python -m brixzle.loop --intake runs/intake-<stamp>/intake.json
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from brixzle.agents import check_family as CF
from brixzle.agents import critic as CR
from brixzle.agents import design as DA
from brixzle.agents.runner import BudgetExceeded, ClaudeRunner, SpendLedger
from brixzle.engine import boxes as BX
from brixzle.engine import jam_analysis as JA
from brixzle.engine import probe as PB
from brixzle.engine import quick_check as QC
from brixzle.engine import report as RP
from brixzle.engine import screen as SC
from brixzle.engine import structure_gate as SG
from brixzle.engine.sim import ring_build as RB
from brixzle.engine import sim_map as SM
from brixzle.engine import tune as TU
from brixzle.engine import zscore as ZS
from brixzle.intake.intake import load_intake
from brixzle.loop import lessons as LS
from brixzle.loop import render as RD
from brixzle.loop import videos as VD
from brixzle.loop.config import LoopConfig
from brixzle.loop import novelty as NV
from brixzle.loop.design_log import DesignLog

STATE = "state.json"
LABEL = ZS.LABEL
BASELINE = "baseline"  # iteration 0's decision: the parent family, scored as the reference (#85)


# --- the engine, as the loop calls it (tests replace it) ---


class SimEngine:
    """The evaluation engine's stages for one run. Every stage writes its own run directory."""

    def __init__(self, cfg: LoopConfig):
        self.cfg = cfg
        self.boxes = BX.default_boxes()["placement"]

    def quick_check(self, family, p):
        res = QC.quick_check(family, p, self.boxes[self.cfg.target], seed=self.cfg.seed)  # along X (#97)
        return res, QC.write_check(res, self.cfg.runs)

    def gate(self, family, p):
        """(gate, run directory): the structure gate (#83), every structure built on seeded repeats."""
        g = SG.run_gate(family, p, runs=self.cfg.runs, seed=self.cfg.seed, workers=self.cfg.workers)
        g = {k: v for k, v in g.items() if k != "repeat_results"}  # the full builds are in the run directory
        return g, Path(g["run"])

    def ring(self, family, p):
        """(result, run directory): the ring build (#89), 6 x 6 cells, 2 courses, ideal placement."""
        r = RB.build_ring(family, p, runs=self.cfg.runs, seed=self.cfg.seed)
        return r, Path(r["run"])

    def score(self, family, p, check, allow_failed_check=False):
        """(report, map directory): the full map covering every target box, and Z per target.
        ``allow_failed_check`` maps a design that failed the quick check: only the baseline (#85)."""
        return RP.score_family(family, p, statistic=self.cfg.target, seed=self.cfg.seed, runs=self.cfg.runs,
                               workers=self.cfg.workers, check=check, allow_failed_check=allow_failed_check,
                               y_error=self.cfg.y_error)

    def masses(self, family, p):
        """{part: printed mass in g}."""
        from brixzle.families import builders as B

        return {part: float(B.sim_bodies(family, p, part).mass_g) for part in family.parts}

    def read_map(self, map_dir):
        return json.loads((Path(map_dir) / "map.json").read_text(encoding="utf-8"))

    def jam(self, family, p, map_dir):
        res = JA.analyse(family, p, self.read_map(map_dir), workers=self.cfg.workers)
        res["map"] = Path(map_dir).name
        JA.write_analysis(res, map_dir)
        return res

    def held_out(self, family, p, target=None):
        """Held-out Z on ``target``'s box (default: the run's target): its poses shifted by seed + 1,
        at friction 0.2 and 0.5."""
        fns = ZS.held_out_scores(family, self.boxes[target or self.cfg.target], self.cfg.seed + 1,
                                 workers=self.cfg.workers, y_error=self.cfg.y_error)
        return {name: fn(p)["counts"] for name, fn in fns.items()}

    def drop_videos(self, family, p, sim_map, map_dir, out_dir, n, seed):
        """``n`` drops of the map, picked at random with ``seed``, recorded to ``out_dir`` (#77)."""
        return VD.drop_videos(family, p, sim_map, SM.read_drops(map_dir), out_dir, n=n, seed=seed)

    def screen(self, jobs, target):
        """{id: screen} for candidates [(id, root, module)] (#108): the quick check and a 20-drop X-Y
        check on ``target``'s box, each candidate in its own process, side by side. Each quick check
        is written to its own run directory ('check_run')."""
        out = SC.screen_many(jobs, target, seed=self.cfg.seed, workers=max(1, min(3, self.cfg.workers or 3)))
        for r in out.values():
            if r.get("check_full"):
                r["check_run"] = str(QC.write_check(r["check_full"], self.cfg.runs))
        return out

    def probe_poses(self, map_dir, target, seed):
        """The drop probe's poses (#107) on ``target``'s box: the failures, some seats and the aim of
        the map in ``map_dir``, or a spread over the box's X-Y disc without one."""
        drops = SM.read_drops(map_dir) if map_dir else None
        return PB.probe_poses(self.boxes[target], drops, seed=seed, n=self.cfg.probe_drops)

    def probe(self, root, module, poses_path):
        """The package ``<root>/<module>`` dropped at the poses in ``poses_path``, in a subprocess (#107)."""
        return PB.probe_in_subprocess(root, module, poses_path)

    def tune(self, family, p, fixed=(), target=None):
        s = TU.TuneSettings(statistic=target or self.cfg.target, fixed=tuple(fixed), seed=self.cfg.seed,
                            workers=self.cfg.workers, max_seconds=self.cfg.tune_seconds, y_error=self.cfg.y_error)
        return TU.tune_family(family, p, s, runs=self.cfg.runs)


# --- the rules ---


def rungs(cfg: LoopConfig):
    """The target ladder (#84): the boxes from the run's target up (mean, p95, max); without the
    ladder, the run's target alone."""
    if not cfg.ladder:
        return (cfg.target,)
    return BX.STATISTICS[BX.STATISTICS.index(cfg.target):]


def z_on(best, statistic):
    """The best design's Z on one box (a record from before #84 has only ``z_target``)."""
    z = (best.get("z") or {}).get(statistic)
    return z["Z"] if z else best["z_target"]


def current_target(state, cfg: LoopConfig):
    """The box designs are judged on now: from the run's target up, the first on which the best
    design's Z is below 1 (the last rung if none is). With no best yet, the run's target."""
    best, ladder = state.get("best"), rungs(cfg)
    if best is None:
        return ladder[0]
    return next((s for s in ladder if z_on(best, s) < 1.0), ladder[-1])


def at_top(best, cfg: LoopConfig):
    """Z is 1 on the last rung (or on the run's target, without the ladder)."""
    return best is not None and z_on(best, rungs(cfg)[-1]) >= 1.0


def accept(z, held, best, lower=(), overhang=None):
    """(kept, reason). ``z`` is the new design's Z on the current target, ``held`` its held-out
    counts on that box; ``best`` the current best on the same box ({'z_target', 'held_out'} and,
    since #83, 'overhang_n') or None; ``lower`` is (box, new Z, best's Z) for every lower rung of
    the ladder (#84); ``overhang`` the new design's overhang length (#83).

    Kept only if, on the pair (Z on the target, overhang length), neither is lower than the best's
    and at least one is higher (Z alone if either has no overhang length); Z on no lower rung falls
    below the best's; and held-out Z is not worse at either friction."""
    if best is None:
        return True, "first scored design"
    bz, bo = best["z_target"], best.get("overhang_n")
    if overhang is None or bo is None:
        if not z > bz:
            return False, f"Z on the target did not improve ({z:+.3f}, best {bz:+.3f})"
        better = f"Z on the target improved ({bz:+.3f} -> {z:+.3f})"
    else:
        if z < bz:
            return False, f"Z on the target did not improve ({z:+.3f}, best {bz:+.3f})"
        if overhang < bo:
            return False, f"the overhang got shorter ({overhang} < best {bo} bricks)"
        if not (z > bz or overhang > bo):
            return False, (f"Z on the target did not improve ({z:+.3f}, best {bz:+.3f}) and the overhang did not "
                           f"grow ({overhang} bricks, best {bo})")
        better = "; ".join(x for x in (f"Z on the target improved ({bz:+.3f} -> {z:+.3f})" if z > bz else "",
                                       f"the overhang grew ({bo} -> {overhang} bricks)" if overhang > bo else "") if x)
    down = [f"{s} {zn:+.3f} < {zb:+.3f}" for s, zn, zb in lower if zn < zb]
    if down:
        return False, "Z on a lower rung got worse: " + "; ".join(down)
    # Held-out Z is worse only when it falls by more than one standard error of the best's Z (#110).
    worse, noise = [], []
    for k, v in best["held_out"].items():
        if k not in held or not held[k]["Z"] < v["Z"]:
            continue
        band, d = ZS.z_band(v), held[k]["Z"] - v["Z"]
        if ZS.within_band(held[k]["Z"], v["Z"], band):
            noise.append(f"{k} {d:+.3f}, inside the band of {band:.3f}")
        else:
            worse.append(f"{k} {held[k]['Z']:+.3f} < {v['Z']:+.3f} ({d:+.3f}, band {band:.3f})")
    if worse:
        return False, "held-out Z got worse: " + "; ".join(worse)
    if noise:
        return True, f"{better}, held-out not worse (held-out " + "; ".join(noise) + ")"
    return True, f"{better}, held-out not worse"


def stop_reason(state, cfg: LoopConfig, remaining_usd, elapsed_s):
    """Why the loop stops now, or None. Iteration 0, the parent's baseline (#85), is not one of the
    ``max_iterations`` design iterations."""
    best = state.get("best")
    its = state["iterations"]
    if cfg.baseline_only:
        return "baseline only" if its else None
    if at_top(best, cfg):
        return f"Z reached 1 on the last rung ({rungs(cfg)[-1]})" if cfg.ladder else "Z reached 1 on the target box"
    if its and (its[-1].get("brief") or {}).get("decision") == "stop":
        return f"the critic said stop: {its[-1]['brief']['reason']}"
    designs = [i for i in its if i["iteration"] >= 1]
    if len(designs) >= cfg.max_iterations:
        return f"{cfg.max_iterations} iterations run"
    last_kept = max((i["iteration"] for i in its if i.get("decision") in ("kept", BASELINE)), default=0)
    if designs and its[-1]["iteration"] - last_kept >= cfg.patience:
        return f"no kept design for {cfg.patience} iterations"
    if remaining_usd < cfg.design_budget_usd + cfg.critic_budget_usd:
        return f"API budget spent (${remaining_usd:.2f} left of ${cfg.budget_usd:.2f})"
    if elapsed_s > cfg.max_minutes * 60:
        return f"wall-clock budget spent ({elapsed_s / 60:.1f} of {cfg.max_minutes:g} min)"
    return None


def _stalled(it):
    """An iteration that moved Z on its rung by no more than the noise band (#110), or scored nothing."""
    ch = it.get("z_change")
    if not it.get("z"):
        return True
    return ch is not None and abs(ch["delta"]) <= ch["band"] + ZS.EPS


def explore_reason(state, cfg: LoopConfig):
    """Why the next iteration is an explore round of ``cfg.concepts`` designs (#106), or None: the
    first design iteration; a brief whose cluster asks for a new family; or ``stall_iterations``
    iterations in a row (no explore round among them) that moved Z by no more than the noise band."""
    if cfg.concepts < 2:
        return None
    designs = [it for it in state["iterations"] if it["iteration"] >= 1]
    if not designs:
        return "the first design iteration explores several concepts"
    brief = state["iterations"][-1].get("brief") or {}
    if any(c.get("scope") == "new_family" for c in brief.get("clusters") or []):
        return "the critic's brief asks for a new family"
    recent = designs[-cfg.stall_iterations:]
    if (len(recent) == cfg.stall_iterations and not any(it.get("mode") == "explore" for it in recent)
            and all(_stalled(it) for it in recent)):
        return f"Z moved by no more than the noise band for {cfg.stall_iterations} iterations"
    return None


def objective_z(report, cfg: LoopConfig):
    """Z per target as the loop judges it: with the X-Y disc (#86), or X offsets only with
    ``y_error=False`` (a report from before #86 has only ``z``)."""
    return report["z"] if cfg.y_error else report.get("z_x_only", report["z"])


def gate_record(gate, run_dir):
    """The gate's verdict as stored in an iteration: per structure, no stages (#83)."""
    return {"passed": gate["passed"], "overhang_n": gate["overhang_n"], "overhang_floor": gate["overhang_floor"],
            "builds": gate["builds"], "seconds": gate["seconds"], "repeats": gate.get("repeats"), "run": str(run_dir)}


def failures_for_agent(check, limit=9):
    """The quick check's failing drops, short: pose, outcome and where the brick ended up."""
    rows = []
    for d in check.get("failures", [])[:limit]:
        rows.append({"course": d["course"], "dx_mm": round(d["dx_mm"], 2), "release_mm": round(d["clear_mm"], 2),
                     "pitch_deg": round(d["pitch_deg"], 2), "outcome": d["outcome"],
                     "final_err_mm": d.get("err_mm"), "final_angle_deg": d.get("angle_deg")})
    return {"seated": check["seated"], "dropped": check["dropped"], "needed": check["need"],
            "outcomes": check["outcomes"], "failing_drops": rows}


# --- one run ---


def _now():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _save(run_dir, state):
    tmp = Path(run_dir) / (STATE + ".tmp")
    tmp.write_text(json.dumps(state, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    tmp.replace(Path(run_dir) / STATE)


def load_scored(rec):
    """The family of a scored record (an iteration or the best): from the registry for the
    baseline (#85), otherwise from the attempt's package."""
    if rec.get("registry"):
        from brixzle.families import registry

        return registry.get(rec["registry"])
    return CF.load_family(rec["root"], rec["module"])


def family_py(rec):
    """The path of a scored record's family.py."""
    return Path(rec["family_py"]) if rec.get("family_py") else Path(rec["root"]) / rec["module"] / "family.py"


def _free(path):
    """``path``, or ``path-2``, ``path-3`` ... if it exists (never reuse a directory)."""
    path, k = Path(path), 2
    out = path
    while out.exists():
        out, k = path.with_name(f"{path.name}-{k}"), k + 1
    return out


class Loop:
    """One loop run. ``runner`` talks to Claude (default: ClaudeRunner with the config's model and
    a ledger capped at ``budget_usd``); ``engine`` runs the sim stages (default: SimEngine)."""

    def __init__(self, intake_path, cfg: LoopConfig = LoopConfig(), runner=None, engine=None, name=None,
                 resume=False):
        self.cfg = cfg.validate()
        root = Path(cfg.runs) / "loop"
        if resume:
            self.run_dir = root / name
            self.state = json.loads((self.run_dir / STATE).read_text(encoding="utf-8"))
            intake_path = self.state["intake"]
            # A run started before #84 or #85 carries on as it was: no ladder, no baseline.
            old = {k: v for k, v in (("ladder", False), ("parent", None))
                   if k not in self.state.get("config", {})}
            if old:
                self.cfg = replace(self.cfg, **old, baseline_only=False).validate()
        else:
            name = name or f"{_now()}-{cfg.target}"
            self.run_dir = _free(root / name)
            self.run_dir.mkdir(parents=True)
            self.state = {"run": self.run_dir.name, "intake": str(intake_path), "config": cfg.to_dict(),
                          "started": _now(), "iterations": [], "best": None, "stop": None, "tune": None,
                          "elapsed_s": 0.0}
        self.name = self.run_dir.name
        self.results = Path(cfg.results) / self.name
        self.results.mkdir(parents=True, exist_ok=True)
        self.intake = load_intake(intake_path)
        self.ledger = SpendLedger(cfg.ledger or self.run_dir / "api-spend.jsonl", cfg.budget_usd)
        self.runner = runner or ClaudeRunner(cfg.model, self.ledger)
        self.engine = engine or SimEngine(cfg)
        self.log = DesignLog.in_dir(self.run_dir)
        _save(self.run_dir, self.state)

    # --- the run ---

    def run(self):
        t0 = self._t0 = time.time() - self.state.get("elapsed_s", 0.0)
        stop = self.state.get("stop")
        while stop is None:
            stop = stop_reason(self.state, self.cfg, self.ledger.remaining(), time.time() - t0)
            if stop:
                break
            its = self.state["iterations"]
            try:
                rec = self.iterate(its[-1]["iteration"] + 1 if its else (0 if self.cfg.parent else 1))
            except BudgetExceeded as e:
                stop = f"API budget spent: {e}"
                break
            self.state["iterations"].append(rec)
            self.record_lessons()
            self.state["elapsed_s"] = time.time() - t0
            _save(self.run_dir, self.state)
            self.render_progress()
        self.state["stop"] = stop
        _save(self.run_dir, self.state)
        if self.cfg.tune and self.state["best"] and self.state.get("tune") is None and not self.cfg.baseline_only:
            self.state["tune"] = self.end_tune()
        self.state["elapsed_s"] = time.time() - t0
        self.state["spent_usd"] = self.ledger.spent()
        _save(self.run_dir, self.state)
        self.write_run_summary()
        return self.state

    def lessons_text(self):
        """The lessons for the agents' system prompts (#88); '' with ``--no-lessons``."""
        return LS.lessons_text(self.cfg.lessons_file) if self.cfg.lessons else ""

    def record_lessons(self):
        """Once an iteration has a scored design, write the lessons of the last brief with its outcome (#88)."""
        its = self.state["iterations"]
        if not self.cfg.lessons or len(its) < 2:
            return
        try:
            its[-1]["lessons_written"] = [x["id"] for x in LS.record(self.cfg.lessons_file, self.name, its[-2], its[-1])]
        except (ValueError, OSError) as e:  # a lesson must not stop the loop
            its[-1]["lessons_error"] = f"{type(e).__name__}: {e}"[:300]

    def render_progress(self):
        its = self.state["iterations"]
        history = [{"iteration": i["iteration"], "decision": i["decision"], "z": i.get("z"),
                    "target": i.get("target"), "overhang_n": i.get("overhang_n")} for i in its if i["iteration"] >= 1]
        base = next((i for i in its if i["iteration"] == 0 and i.get("z")), None)
        RD.progress_png(history, self.results / "progress.png", self.cfg.target,
                        title=f"{self.name}: Z per iteration (target {self.cfg.target}); {LABEL}",
                        baseline=({"family": base["family"], "z": base["z"]} if base else None))

    # --- one iteration ---

    def baseline(self):
        """Iteration 0 (#85): the parent family from the registry, scored with the same stages as a
        design but with no Claude call. It is recorded even if it fails the quick check (it is the
        reference, not a candidate to turn back), and it becomes the best."""
        from brixzle.families import registry

        cfg, t_iter = self.cfg, time.time()
        iter_dir = _free(self.run_dir / "iter-00")
        iter_dir.mkdir(parents=True)
        res_dir = self.results / "iter-00"
        res_dir.mkdir(parents=True, exist_ok=True)
        fam = registry.get(cfg.parent)
        module, path = registry.source(cfg.parent)
        spent0 = self.ledger.spent()
        rec = {"iteration": 0, "dir": str(iter_dir), "results": str(res_dir), "brief_in": None, "attempts": [],
               "decision": BASELINE, "z": None, "runs": {}, "seconds": {}, "family": fam.name, "module": module,
               "root": str(path.parents[len(module.split("."))]), "family_py": str(path), "registry": cfg.parent,
               "note": {"summary": f"the parent family {fam.name!r} from the registry, scored as the baseline",
                        "changes": "none: this is the reference the designs must beat"}}
        t = time.time()
        check, check_dir = self.engine.quick_check(fam, fam.defaults)
        rec["seconds"]["quick_check"] = time.time() - t
        rec["quick_check"] = {"passed": check["passed"], "seated": check["seated"], "dropped": check["dropped"],
                              "outcomes": check["outcomes"], "run": str(check_dir)}
        rec["runs"]["quick_check"] = str(check_dir)
        t = time.time()
        gate, gate_dir = self.engine.gate(fam, fam.defaults)  # recorded, never turned back: it is the reference
        rec["seconds"]["gate"] = time.time() - t
        rec["gate"], rec["runs"]["gate"] = gate_record(gate, gate_dir), str(gate_dir)
        rec["overhang_n"] = gate["overhang_n"]
        return self._score(rec, fam, check, res_dir, None, spent0, t_iter)

    def iterate(self, i):
        if i == 0:
            return self.baseline()
        why = explore_reason(self.state, self.cfg)
        if why:
            return self.explore(i, why)
        cfg, t_iter = self.cfg, time.time()
        iter_dir = _free(self.run_dir / f"iter-{i:02d}")
        res_dir = self.results / f"iter-{i:02d}"
        res_dir.mkdir(parents=True, exist_ok=True)
        brief = self.state["iterations"][-1].get("brief") if self.state["iterations"] else None
        best = self.state["best"]
        parent = family_py(best) if best else None
        spent0 = self.ledger.spent()
        rec = {"iteration": i, "dir": str(iter_dir), "results": str(res_dir), "brief_in": brief, "attempts": [],
               "decision": "no_design", "z": None, "runs": {}, "seconds": {}}
        failures, fam, outcome, check = None, None, None, None
        last = self.state["iterations"][-1] if self.state["iterations"] else None
        if last and last.get("decision") == "failed_ring":
            failures = {"ring": last["ring"]["failure"]}  # the last design closed no ring (#89)
        last_err, repeated = None, None
        prober = self._prober(iter_dir, i, best)
        for a in range(1, cfg.design_attempts + 1):
            module = f"rsi_{self.name.replace('-', '_').lower()}_i{i:02d}_a{a}"
            t = time.time()
            try:
                outcome = DA.design(self.runner, self.intake, iter_dir / f"attempt-{a}", module, brief=brief,
                                    parent=parent, failures=failures, log_text=self.log.text(),
                                    budget_usd=cfg.design_budget_usd, max_turns=cfg.design_max_turns,
                                    lessons_text=self.lessons_text(), prober=prober, probe_calls=cfg.probe_calls)
            except BudgetExceeded:
                raise
            except Exception as e:  # noqa: BLE001 - an SDK or API error: log it and try again
                err = f"{type(e).__name__}: {e}"
                rec["attempts"].append({"attempt": a, "module": module, "root": str(iter_dir / f"attempt-{a}" / "pkg"),
                                        "decision": "error", "error": err, "design_s": time.time() - t})
                self.log.append({"iteration": i, "attempt": a, "family": module, "module": module,
                                 "decision": "error", "reason": err[:500]})
                failures = {"error": "the last design call failed before it finished; start again"}
                outcome = None
                if err == last_err:  # the same call would fail the same way and cost the same again (#79)
                    repeated = err
                    break
                last_err = err
                continue
            probe_s = sum(float(q.get("seconds") or 0.0) for q in outcome.probes)
            att = {"attempt": a, "module": module, "root": str(outcome.workspace.root),
                   "design_cost_usd": outcome.result.cost_usd, "design_s": time.time() - t,
                   "family": outcome.check.get("family") or module, "note": outcome.note,
                   "probes": list(outcome.probes), "probe_s": probe_s}
            rec["attempts"].append(att)
            if outcome.probes:
                rec["seconds"]["probe"] = rec["seconds"].get("probe", 0.0) + probe_s
            entry = {"iteration": i, "attempt": a, "family": att["family"], "module": module,
                     "note": outcome.note, "brief_followed": (brief or {}).get("reason"),
                     "probes": len(outcome.probes), "probe_s": round(probe_s, 2)}
            if not outcome.ok:
                problems = outcome.check.get("violations") or []
                broken = [f"{part}: {', '.join(info['broken'])}" for part, info in outcome.check.get("parts", {}).items()
                          if info.get("broken")]
                err = outcome.check.get("error")
                att["decision"] = "failed_contract"
                self.log.append({**entry, "decision": "failed_contract",
                                 "violations": [*problems, *broken, *([err] if err else [])]})
                failures = {"check": outcome.check}
                continue
            fam = CF.load_family(outcome.workspace.root, module)
            t = time.time()
            check, check_dir = self.engine.quick_check(fam, fam.defaults)
            att["quick_check"] = {"passed": check["passed"], "seated": check["seated"], "dropped": check["dropped"],
                                  "outcomes": check["outcomes"], "run": str(check_dir)}
            rec["seconds"]["quick_check"] = time.time() - t
            if not check["passed"]:
                att["decision"] = "failed_quick_check"
                self.log.append({**entry, "decision": "failed_quick_check", "quick_check": att["quick_check"],
                                 "runs": {"quick_check": str(check_dir)}})
                failures = failures_for_agent(check)
                fam = None
                continue
            t = time.time()
            gate, gate_dir = self.engine.gate(fam, fam.defaults)
            rec["seconds"]["gate"] = time.time() - t
            att["gate"] = gate_record(gate, gate_dir)
            if not gate["passed"]:
                att["decision"] = "failed_structure_gate"
                self.log.append({**entry, "decision": "failed_structure_gate", "quick_check": att["quick_check"],
                                 "gate": SG.failures_text(gate), "overhang_n": gate["overhang_n"],
                                 "runs": {"quick_check": str(check_dir), "gate": str(gate_dir)}})
                failures = {"structure_gate": SG.failures_for_agent(gate)}
                fam = None
                continue
            att["decision"] = "passed_structure_gate"
            rec.update(module=module, root=str(outcome.workspace.root), family=att["family"], note=outcome.note)
            rec["runs"]["quick_check"] = str(check_dir)
            rec["gate"], rec["runs"]["gate"], rec["overhang_n"] = att["gate"], str(gate_dir), gate["overhang_n"]
            break
        if fam is None:
            rec["reason"] = (f"the design call failed twice with the same error, so it was not tried again: "
                             f"{repeated[:300]}" if repeated else
                             f"no design passed the contract, the quick check and the structure gate in "
                             f"{cfg.design_attempts} attempts")
            rec["spent_usd"] = self.ledger.spent() - spent0
            rec["seconds"]["total"] = time.time() - t_iter
            if outcome is not None and outcome.ok:
                self._module_png(outcome, res_dir)
            self.write_iteration_summary(rec, res_dir)
            return rec
        return self._score(rec, fam, check, res_dir, brief, spent0, t_iter, outcome)

    def _measure(self, rec, fam, check, allow_failed_check=False):
        """The full map, Z per target and the jam analysis of ``fam``, into ``rec``; returns
        (report, map directory, jam analysis)."""
        p = fam.defaults
        t = time.time()
        report, map_dir = self.engine.score(fam, p, check, allow_failed_check=allow_failed_check)
        rec["seconds"]["map"] = time.time() - t
        rec["z"], rec["runs"]["map"] = objective_z(report, self.cfg), str(map_dir)
        rec["z_with_y"], rec["z_x_only"] = report["z"], report.get("z_x_only")
        rec["y_capture_mm"] = report.get("y_capture_mm")
        t = time.time()
        jam = self.engine.jam(fam, p, map_dir)
        rec["seconds"]["jam"] = time.time() - t
        rec["runs"]["jam"] = str(Path(map_dir) / JA.FILE)
        return report, map_dir, jam

    def _score(self, rec, fam, check, res_dir, brief, spent0, t_iter, outcome=None, measured=None):
        """Map, Z, jam analysis, held-out Z, the decision, renders and the critic, for a design
        that passed the quick check or for the baseline (iteration 0). ``measured`` (an explore
        round's best concept, #106): (report, map directory, jam analysis) already in ``rec``."""
        cfg, i, best = self.cfg, rec["iteration"], self.state["best"]
        baseline = rec["decision"] == BASELINE
        target = rec["target"] = current_target(self.state, cfg)  # the rung this design is judged on (#84)
        p = fam.defaults
        if measured is None:
            measured = self._measure(rec, fam, check, allow_failed_check=baseline and not check["passed"])
        report, map_dir, jam = measured
        held = {}
        if cfg.held_out:
            t = time.time()
            held = self.engine.held_out(fam, p, target=target)
            rec["seconds"]["held_out"] = time.time() - t
        rec["held_out"] = held
        try:
            rec["mass_g"] = self.engine.masses(fam, p)
        except Exception as e:  # noqa: BLE001 - a mass for the report must not stop the loop
            rec["mass_error"] = f"{type(e).__name__}: {e}"[:300]
        z_t = rec["z"][target]["Z"]
        if best is not None and (best.get("z") or {}).get(target, {}).get("N"):
            # How far Z moved against the best, and the noise band of the best's Z (#110): a stall (#106).
            rec["z_change"] = {"best_before": z_on(best, target), "delta": z_t - z_on(best, target),
                               "band": ZS.z_band(best["z"][target])}
        if baseline:
            kept, why = True, f"the parent family {rec['family']!r}, scored as the baseline the designs must beat"
            self._ring(rec, fam, p)  # recorded, not judged: it is the reference
        else:
            ladder = rungs(cfg)
            lower = [(s, rec["z"][s]["Z"], z_on(best, s)) for s in ladder[:ladder.index(target)]] if best else []
            view = ({"z_target": z_on(best, target), "held_out": {}, "overhang_n": best.get("overhang_n")}
                    if best else None)
            over = rec.get("overhang_n")
            kept, why = accept(z_t, held, view, lower, over)
            if kept and view is not None and held:
                # The best's held-out Z on this box, computed once when the rung moves up (#84).
                view["held_out"] = self._best_held_out(target, rec)
                kept, why = accept(z_t, held, view, lower, over)
            rec["decision"] = "kept" if kept else "rejected"
            if kept:
                # The ring is slow, so it runs only on a design the rule would keep (#89).
                ring = self._ring(rec, fam, p)
                if ring is not None and ring["status"] != "complete":
                    kept = False
                    stop = (rec["ring"]["failure"].get("stopped_at") or {}).get("brick")
                    why = (f"the ring did not close ({ring['status']}" + (f" at {stop}" if stop else "")
                           + f", {ring['seated']} of {ring['total']} seated); otherwise: {why}")
                    rec["decision"] = "failed_ring"
        rec["reason"] = why
        if kept:
            self.state["best"] = {"iteration": i, "module": rec["module"], "root": rec["root"], "family": rec["family"],
                                  "family_py": str(family_py(rec)), "registry": rec.get("registry"),
                                  "z_target": z_t, "target": target, "z": rec["z"], "held_out": held,
                                  "held_out_by": {target: held} if held else {}, "map": str(map_dir),
                                  "mass_g": rec.get("mass_g"), "overhang_n": rec.get("overhang_n"),
                                  "gate": (rec.get("runs") or {}).get("gate")}
        sim_map = self.engine.read_map(map_dir)
        last_att = rec["attempts"][-1] if rec["attempts"] else {}
        if rec.get("mode") == "explore":
            last_att = next((c for c in rec.get("concepts", []) if c.get("module") == rec["module"]), last_att)
        self.log.append({"iteration": i, "attempt": len(rec["attempts"]), "family": rec["family"],
                         "module": rec["module"], "decision": rec["decision"], "reason": why, "note": rec["note"],
                         **({"concept": last_att.get("concept")} if last_att.get("concept") else {}),
                         "probes": len(last_att.get("probes") or []), "probe_s": round(last_att.get("probe_s", 0.0), 2),
                         "z": rec["z"], "held_out": held, "overrides": sim_map.get("overrides", []),
                         "overhang_n": rec.get("overhang_n"),
                         "brief_followed": (brief or {}).get("reason"), "runs": dict(rec["runs"])})
        t = time.time()
        picks = []
        if cfg.videos:
            # The pick is repeatable: its seed is the run's seed and the iteration (issue #77).
            picks = self.engine.drop_videos(fam, p, sim_map, map_dir, res_dir, cfg.videos, seed=cfg.seed * 1000 + i)
            rec["videos"] = picks
            rec["seconds"]["videos"] = time.time() - t
        t = time.time()
        RD.jam_map(sim_map, rec["z"], res_dir / "jam_map.png", target=cfg.target, jam=jam,
                   title=f"{self.name} iteration {i}: {rec['family']} ({rec['decision']})", picks=picks)
        self._module_png_of(fam, res_dir)
        try:
            rec["stl"] = [path.name for path in VD.export_stl(fam, p, res_dir)]
        except Exception as e:  # noqa: BLE001 - an export must not stop the loop
            rec["stl_error"] = f"{type(e).__name__}: {e}"[:300]
        rec["seconds"]["render"] = time.time() - t
        if not at_top(self.state["best"], cfg) and i < cfg.max_iterations and not cfg.baseline_only:
            family_source = family_py(rec).read_text(encoding="utf-8")
            t = time.time()
            try:
                rec["critic_target"] = current_target(self.state, cfg)  # the rung the next design is judged on
                brief, result = CR.critique(self.runner, report, sim_map, jam, family_source, self.log.text(),
                                            rec["critic_target"], budget_usd=cfg.critic_budget_usd,
                                            lessons_text=self.lessons_text(), gate=rec.get("gate"))
                rec["brief"], rec["critic_cost_usd"] = brief, result.cost_usd
            except BudgetExceeded:
                raise
            except Exception as e:  # noqa: BLE001 - a malformed brief or an SDK error: the next design gets no brief
                rec["brief"] = None
                rec["critic_error"] = f"{type(e).__name__}: {e}"[:500]
            rec["seconds"]["critic"] = time.time() - t
        rec["spent_usd"] = self.ledger.spent() - spent0
        rec["seconds"]["total"] = time.time() - t_iter
        self.write_iteration_summary(rec, res_dir)
        return rec

    # --- explore rounds (#106) ---

    def _concept_count(self):
        """Concepts this round can pay for: up to ``concepts``, each holding a design call's budget,
        with a critic call left over."""
        cfg = self.cfg
        n = int((self.ledger.remaining() - cfg.critic_budget_usd) // cfg.design_budget_usd)
        return max(1, min(cfg.concepts, n))

    def _principles(self, n):
        """``n`` capture principles not tried yet in this run (round-robin once all are tried)."""
        pool = DA.principles()
        tried = self.state.get("principles_tried", [])
        fresh = [p for p in pool if p["id"] not in tried]
        chosen = (fresh + [p for p in pool if p not in fresh])[:n]
        self.state["principles_tried"] = tried + [p["id"] for p in chosen if p["id"] not in tried]
        return chosen, tried

    def explore(self, i, why):
        """An explore round (#106): ``concepts`` design calls side by side, each from its own capture
        principle and none a near copy of A/B; the cheap screen (#108) picks which to score in full;
        the best of those by Z on the target goes through the decision as one iteration's design."""
        cfg, t_iter = self.cfg, time.time()
        iter_dir = _free(self.run_dir / f"iter-{i:02d}")
        iter_dir.mkdir(parents=True)
        res_dir = self.results / f"iter-{i:02d}"
        res_dir.mkdir(parents=True, exist_ok=True)
        brief = self.state["iterations"][-1].get("brief") if self.state["iterations"] else None
        best = self.state["best"]
        target = current_target(self.state, cfg)
        spent0 = self.ledger.spent()
        rec = {"iteration": i, "mode": "explore", "explore_reason": why, "dir": str(iter_dir), "results": str(res_dir),
               "brief_in": brief, "attempts": [], "concepts": [], "decision": "no_design", "z": None, "runs": {},
               "seconds": {}}
        chosen, tried = self._principles(self._concept_count())
        beat = (f"{best['family']}, Z {target} {z_on(best, target):+.3f} (simulated, uncalibrated contact)"
                if best else "A/B")
        prober = self._prober(iter_dir, i, None)  # no parent map: a spread over the error disc (#107)
        ref = self._reference()

        def one(k, principle):
            module = f"rsi_{self.name.replace('-', '_').lower()}_i{i:02d}_c{k}"
            t = time.time()
            c = {"id": f"c{k}", "attempt": k, "concept": principle["id"], "principle": principle["title"],
                 "module": module, "root": str(iter_dir / f"concept-{k}" / "pkg")}
            try:
                out = DA.design(self.runner, self.intake, iter_dir / f"concept-{k}", module, brief=brief, parent=None,
                                log_text=self.log.text(), budget_usd=cfg.design_budget_usd,
                                max_turns=cfg.design_max_turns, lessons_text=self.lessons_text(), prober=prober,
                                probe_calls=cfg.probe_calls,
                                explore={"principle": principle, "tried": tried, "beat": beat})
            except BudgetExceeded:
                raise
            except Exception as e:  # noqa: BLE001 - an SDK or API error: this concept is lost, the round goes on
                c.update(decision="error", reason=f"{type(e).__name__}: {e}"[:500], design_s=time.time() - t)
                return c
            probe_s = sum(float(q.get("seconds") or 0.0) for q in out.probes)
            c.update(root=str(out.workspace.root), design_cost_usd=out.result.cost_usd, design_s=time.time() - t,
                     family=out.check.get("family") or module, note=out.note, probes=list(out.probes), probe_s=probe_s)
            if not out.ok:
                problems = [*(out.check.get("violations") or []),
                            *[f"{part}: {', '.join(info['broken'])}" for part, info in out.check.get("parts", {}).items()
                              if info.get("broken")], *([out.check["error"]] if out.check.get("error") else [])]
                c.update(decision="failed_contract", reason="; ".join(map(str, problems))[:500] or "the check failed")
                return c
            if ref is not None:
                fam = CF.load_family(out.workspace.root, module)
                nv = NV.near_copy(fam, fam.defaults, ref, ref.defaults, share=cfg.near_copy_share)
                c["novelty"] = {"share": nv["share"], "reason": nv["reason"]}
                if nv["near_copy"]:
                    c.update(decision="near_copy", reason=nv["reason"])
            return c

        t = time.time()
        with ThreadPoolExecutor(len(chosen)) as ex:
            cands = list(ex.map(lambda kp: one(*kp), enumerate(chosen, start=1)))
        rec["seconds"]["design"] = time.time() - t
        probe_s = sum(c.get("probe_s", 0.0) for c in cands)
        if probe_s:
            rec["seconds"]["probe"] = probe_s
        rec["concepts"] = cands
        rec["attempts"] = [{k: c.get(k) for k in ("attempt", "module", "root", "design_cost_usd", "design_s", "family",
                                                     "note", "probes", "probe_s", "concept")} for c in cands]
        for c in cands:
            if c.get("decision"):
                self.log.append({"iteration": i, "attempt": c["attempt"], "family": c.get("family") or c["module"],
                                 "module": c["module"], "concept": c["concept"], "decision": c["decision"],
                                 "reason": c["reason"], "note": c.get("note"), "probes": len(c.get("probes") or []),
                                 "probe_s": round(c.get("probe_s", 0.0), 2)})
        live = [c for c in cands if not c.get("decision")]
        mapped = self.screen_round(rec, live, target) if live else []
        scored = []
        for c in mapped:
            fam = CF.load_family(c["root"], c["module"])
            crec = {"iteration": i, "seconds": {}, "runs": {"quick_check": c.get("quick_check", {}).get("run")}}
            t = time.time()
            gate, gate_dir = self.engine.gate(fam, fam.defaults)
            rec["seconds"]["gate"] = rec["seconds"].get("gate", 0.0) + (time.time() - t)
            c["gate"] = gate_record(gate, gate_dir)
            if not gate["passed"]:
                c.update(decision="failed_structure_gate", reason=SG.failures_text(gate)[:500])
                self.log.append({"iteration": i, "attempt": c["attempt"], "family": c["family"], "module": c["module"],
                                 "concept": c["concept"], "decision": "failed_structure_gate", "reason": c["reason"],
                                 "note": c.get("note"), "overhang_n": gate["overhang_n"], "screen": c.get("screen")})
                continue
            c["overhang_n"] = gate["overhang_n"]
            check = c.pop("check_full", None) or c.get("quick_check")
            measured = self._measure(crec, fam, check)
            for k in ("map", "jam"):
                rec["seconds"][k] = rec["seconds"].get(k, 0.0) + crec["seconds"][k]
            c.update(z=crec["z"], runs={**crec["runs"], "gate": str(gate_dir)})
            scored.append((c, fam, crec, measured, gate, gate_dir, check))
        if not scored:
            rec["reason"] = ("no concept of the round passed the contract, the novelty check, the screen and the "
                             "structure gate")
            rec["spent_usd"] = self.ledger.spent() - spent0
            rec["seconds"]["total"] = time.time() - t_iter
            self.write_iteration_summary(rec, res_dir)
            return rec
        top = max(scored, key=lambda s_: (s_[0]["z"][target]["Z"], s_[0].get("screen", {}).get("Z", 0.0)))
        for c, *_ in scored:
            if c is not top[0]:
                c.update(decision="rejected", reason=(f"another concept of this round scored higher on {target} "
                                                      f"({c['z'][target]['Z']:+.3f} < {top[0]['z'][target]['Z']:+.3f})"))
                self.log.append({"iteration": i, "attempt": c["attempt"], "family": c["family"], "module": c["module"],
                                 "concept": c["concept"], "decision": "rejected", "reason": c["reason"],
                                 "note": c.get("note"), "z": c["z"], "screen": c.get("screen"), "runs": c["runs"]})
        c, fam, crec, measured, gate, gate_dir, check = top
        rec.update(module=c["module"], root=c["root"], family=c["family"], note=c["note"], concept=c["concept"])
        rec["runs"].update(crec["runs"], gate=str(gate_dir))
        rec["gate"], rec["overhang_n"] = c["gate"], gate["overhang_n"]
        for k in ("z", "z_with_y", "z_x_only", "y_capture_mm"):
            rec[k] = crec.get(k)
        out = self._score(rec, fam, check, res_dir, brief, spent0, t_iter, measured=measured)
        c["decision"], c["reason"] = out["decision"], out.get("reason")
        return out

    def _reference(self):
        """The family an explore design must not copy (#106): the run's parent, else A/B."""
        from brixzle.families import registry

        try:
            return registry.get(self.cfg.parent or "ab")
        except Exception:  # noqa: BLE001 - no reference: the novelty check is skipped
            return None

    SCORE_S = 240.0  # a full score's wall-clock before any was measured (gate, map, jam, held-out, ring, renders)
    SCORED_STAGES = ("gate", "map", "jam", "held_out", "held_out_best", "ring", "videos", "render")

    def score_estimate(self):
        """Seconds one full score takes in this run: the mean over the scored iterations so far."""
        done = [sum(it["seconds"].get(k, 0.0) for k in self.SCORED_STAGES) for it in self.state["iterations"]
                if it.get("z") and it.get("seconds")]
        return sum(done) / len(done) if done else self.SCORE_S

    def screen_round(self, rec, candidates, target, score_s=None):
        """Screen ``candidates`` [{'id', 'root', 'module'}] side by side (#108) and return the ones to
        score in full, best first. A candidate that fails the quick check, a clear loser on the screen,
        and any past the cap (``screen_max_maps``, or fewer if the run's wall-clock left would not
        hold them) get their decision and reason in place and a line in the design log."""
        cfg = self.cfg
        t = time.time()
        res = self.engine.screen([(c["id"], c["root"], c["module"]) for c in candidates], target)
        rec["seconds"]["screen"] = rec["seconds"].get("screen", 0.0) + (time.time() - t)
        rec["screen"] = {k: {q: v[q] for q in ("Z", "S", "J", "N")} for k, v in res.items() if "Z" in v}
        passing = {}
        for c in candidates:
            r = res.get(c["id"]) or {"error": "no screen result"}
            c["screen"] = rec["screen"].get(c["id"])
            if "error" in r:
                c["decision"], c["reason"] = "failed_screen", r["error"]
            elif not r["check"]["passed"]:
                c["decision"] = "failed_quick_check"
                c["reason"] = (f"the quick check seated {r['check']['seated']} of {r['check']['dropped']} "
                               f"(needs {r['check'].get('need', 32)})")
                c["quick_check"] = r["check"]
            else:
                c["quick_check"] = r["check"]
                passing[c["id"]] = rec["screen"][c["id"]]
        est = score_s or self.score_estimate()
        left = cfg.max_minutes * 60 - (time.time() - getattr(self, "_t0", time.time())) - (cfg.tune_seconds if cfg.tune else 0.0)
        fit = max(1, int(left // est))
        cap = min(cfg.screen_max_maps, fit)
        mapped, out = SC.select(passing, cap) if passing else ([], {})
        if fit < cfg.screen_max_maps and len(mapped) + sum("cap" in r for r in out.values()) > fit:
            rec.setdefault("warnings", []).append(
                f"wall-clock: {left / 60:.1f} min left after the end tune's {cfg.tune_seconds / 60 if cfg.tune else 0:.1f} min "
                f"holds {fit} full score(s) of about {est:.0f} s, so only {fit} of this round's candidates are mapped")
        for c in candidates:
            if c["id"] in out:
                c["decision"], c["reason"] = "screened_out", out[c["id"]]
        for c in candidates:
            if c.get("decision") in ("screened_out", "failed_quick_check", "failed_screen"):
                self.log.append({"iteration": rec["iteration"], "attempt": c.get("attempt", 1), "family": c["module"],
                                 "module": c["module"], "concept": c.get("concept"), "decision": c["decision"],
                                 "reason": c["reason"], "screen": c.get("screen"), "note": c.get("note")})
        by = {c["id"]: c for c in candidates}
        return [by[k] for k in mapped]

    def _prober(self, iter_dir, i, best):
        """The design agent's probe (#107): ``prober(root, module)`` drops a package at this
        iteration's poses, the best design's map failures on the current target (or a spread over
        the box's disc without a best), seeded with the run's seed and the iteration. None with
        ``probe_calls`` 0."""
        if not self.cfg.probe_calls:
            return None
        target = current_target(self.state, self.cfg)
        poses = self.engine.probe_poses((best or {}).get("map"), target, self.cfg.seed * 1000 + i)
        Path(iter_dir).mkdir(parents=True, exist_ok=True)
        path = PB.write_poses(poses, Path(iter_dir) / "probe_poses.json")
        engine = self.engine
        return lambda root, module: engine.probe(root, module, path)

    RING_MIN_LEFT_S = 120.0  # no ring is started with less of the run's wall-clock left (#89)

    def _ring(self, rec, fam, p):
        """Build the ring for ``rec`` (#89) and record it; returns the result, or None if it was not
        run because the run has less than two minutes left."""
        left = self.cfg.max_minutes * 60 - (time.time() - getattr(self, "_t0", time.time()))
        if left < self.RING_MIN_LEFT_S:
            rec["ring"] = {"status": "not_run",
                           "reason": f"less than two minutes of the run's {self.cfg.max_minutes:g} min were left "
                                     f"({left:.0f} s)"}
            return None
        t = time.time()
        ring, ring_dir = self.engine.ring(fam, p)
        rec["seconds"]["ring"] = time.time() - t
        rec["ring"] = {"status": ring["status"], "seated": ring["seated"], "total": ring["total"],
                       "seconds": ring["seconds"], "run": str(ring_dir), "failure": RB.failure_for_agent(ring)}
        rec["runs"]["ring"] = str(ring_dir)
        return ring

    def _best_held_out(self, target, rec):
        """The best design's held-out counts on ``target``'s box: stored, or computed once and stored
        (a best from before #84 has them only on the run's target)."""
        best = self.state["best"]
        by = best.setdefault("held_out_by", {})
        if not by and best.get("held_out"):
            by[best.get("target", self.cfg.target)] = best["held_out"]
        if target not in by:
            t = time.time()
            fam = load_scored(best)
            by[target] = self.engine.held_out(fam, fam.defaults, target=target)
            rec["seconds"]["held_out_best"] = time.time() - t
        return by[target]

    def _module_png(self, outcome, res_dir):
        try:
            fam = CF.load_family(outcome.workspace.root, outcome.workspace.module)
        except Exception as e:  # noqa: BLE001 - a render must not stop the loop
            (res_dir / "module.png.error.txt").write_text(f"{type(e).__name__}: {e}\n", encoding="utf-8")
            return
        self._module_png_of(fam, res_dir)

    def _module_png_of(self, fam, res_dir):
        try:
            RD.module_png(fam, fam.defaults, res_dir / "module.png")
        except Exception as e:  # noqa: BLE001 - a render must not stop the loop
            (res_dir / "module.png.error.txt").write_text(f"{type(e).__name__}: {e}\n", encoding="utf-8")

    # --- end of run ---

    def end_tune(self):
        best = self.state["best"]
        fam = load_scored(best)
        fixed = ("U",) if self.intake.get("fixed", {}).get("U_mm") and "U" in fam.bounds else ()
        target = current_target(self.state, self.cfg)  # the first box still below 1 (#84)
        t = time.time()
        try:
            result, out = self.engine.tune(fam, fam.defaults, fixed, target=target)
        except ValueError as e:  # for example a family with nothing left to tune
            return {"error": str(e), "target": target}
        return {"run": str(out), "target": target, "accepted": result["accepted"], "improved": result["improved"],
                "baseline": result["baseline"]["score"], "tuned": result["tuned"]["score"],
                "params": result["params"], "seconds": time.time() - t}

    def write_iteration_summary(self, rec, res_dir):
        text = RD.iteration_summary(rec, self.name, rec.get("target") or self.cfg.target, self.ledger.path,
                                    self.ledger.spent(), self.cfg.budget_usd)
        (res_dir / "summary.md").write_text(text, encoding="utf-8")

    def write_run_summary(self):
        text = RD.run_summary(self.state, self.cfg.target, self.ledger.path, self.ledger.spent(), self.cfg.budget_usd)
        (self.results / "summary.md").write_text(text, encoding="utf-8")


def parser():
    d = LoopConfig()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--intake", help="intake.json (python -m brixzle.intake writes it)")
    ap.add_argument("--resume", metavar="RUN", help="carry on a run in runs/loop/RUN")
    ap.add_argument("--name", help="the run's name (default: <UTC>-<target>)")
    ap.add_argument("--target", choices=BX.STATISTICS, default=d.target,
                    help="the box the run starts on; it climbs mean -> p95 -> max as Z reaches 1")
    ap.add_argument("--no-ladder", action="store_true", help="stay on --target (no climbing)")
    ap.add_argument("--no-lessons", action="store_true",
                    help="neither read nor write the lessons file (for repeatable benchmarks, #22)")
    ap.add_argument("--no-y-error", action="store_true",
                    help="score X offsets only, as before issue #86 (for comparisons with earlier runs)")
    ap.add_argument("--parent", default=d.parent,
                    help="a registry family scored as iteration 0, the baseline to beat ('none': from scratch)")
    ap.add_argument("--baseline-only", action="store_true",
                    help="score the parent (iteration 0) and stop; no Claude call")
    ap.add_argument("--max-iterations", type=int, default=d.max_iterations)
    ap.add_argument("--patience", type=int, default=d.patience)
    ap.add_argument("--budget-usd", type=float, default=d.budget_usd, help="cap on real API spend")
    ap.add_argument("--ledger", help="spend ledger to share with other runs (default: the run's own)")
    ap.add_argument("--max-minutes", type=float, default=d.max_minutes)
    ap.add_argument("--design-attempts", type=int, default=d.design_attempts)
    ap.add_argument("--concepts", type=int, default=d.concepts,
                    help="designs in an explore round, each from its own capture principle (#106; 1: no explore rounds)")
    ap.add_argument("--stall-iterations", type=int, default=d.stall_iterations,
                    help="explore again after this many iterations with Z inside the noise band (#106)")
    ap.add_argument("--screen-max-maps", type=int, default=d.screen_max_maps,
                    help="of a round's candidates, at most this many get a full score (#108)")
    ap.add_argument("--probe-calls", type=int, default=d.probe_calls,
                    help="probe_drops calls per design call (#107; 0: no probe tool)")
    ap.add_argument("--seed", type=int, default=d.seed)
    ap.add_argument("--workers", type=int, default=None)
    ap.add_argument("--no-tune", action="store_true", help="skip the end tune")
    ap.add_argument("--videos", type=int, default=d.videos, help="drop videos per iteration (0: none)")
    ap.add_argument("--runs", default=d.runs)
    ap.add_argument("--results", default=d.results)
    return ap


def _interrupt_on_sigterm():
    """Turn SIGTERM into KeyboardInterrupt, so a stopped run unwinds and the Agent SDK closes its
    Claude Code process (the first smoke run left one running after a kill, 2026-11-12)."""
    import signal

    def handler(_signum, _frame):
        raise KeyboardInterrupt("SIGTERM")

    signal.signal(signal.SIGTERM, handler)


def config_from_args(args):
    parent = None if str(args.parent).lower() == "none" else args.parent
    return LoopConfig(target=args.target, ladder=not args.no_ladder, y_error=not args.no_y_error,
                      lessons=not args.no_lessons, parent=parent,
                      baseline_only=args.baseline_only,
                      max_iterations=args.max_iterations, patience=args.patience,
                      budget_usd=args.budget_usd, ledger=args.ledger, max_minutes=args.max_minutes,
                      design_attempts=args.design_attempts, probe_calls=args.probe_calls,
                      screen_max_maps=args.screen_max_maps, concepts=args.concepts,
                      stall_iterations=args.stall_iterations, seed=args.seed, workers=args.workers,
                      tune=not args.no_tune, videos=args.videos, runs=args.runs, results=args.results).validate()


def main(argv=None):
    _interrupt_on_sigterm()
    args = parser().parse_args(argv)
    if not (args.intake or args.resume):
        raise SystemExit("give --intake (a new run) or --resume RUN")
    cfg = config_from_args(args)
    loop = Loop(args.intake, cfg, name=args.resume or args.name, resume=bool(args.resume))
    state = loop.run()
    for it in state["iterations"]:
        t = it.get("target") or cfg.target
        z = (it.get("z") or {}).get(t, {}).get("Z")
        print(f"iteration {it['iteration']}: {it.get('family', '-')} {it['decision']}"
              + (f", Z {t} {z:+.3f}" if z is not None else "") + f" ({it.get('reason', '')})")
    print(f"stopped: {state['stop']}; spent ${loop.ledger.spent():.4f} of ${cfg.budget_usd:.2f}")
    if state.get("tune"):
        print(f"end tune: {state['tune']}")
    print(f"-> {loop.results}  (state: {loop.run_dir / STATE})")


if __name__ == "__main__":
    main()
