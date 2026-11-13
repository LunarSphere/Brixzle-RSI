"""Orchestrator (issue #21): runs the design loop headless, from intake to a tuned brick family.

**Iteration 0** (#85): the parent family (``--parent``, A/B by default) is scored with the same
stages as a design, with no Claude call; it becomes the best, so every design must beat it, and the
critic's first brief comes from its results. ``--parent none`` starts from scratch;
``--baseline-only`` scores the parent and stops.

Each outer iteration:

1. **Design:** the design agent (#18) writes a family package, from the concept, the critic's
   last brief and the current best design. The loop checks the package itself (contract and
   rules); a package that fails goes straight back to the agent with the check's output.
2. **Quick check** (#47): 40 drops in the target box. Below 32 seated, the design goes straight
   back to the agent with the failing drops. Up to ``design_attempts`` tries per iteration.
3. **Full sim map** (#48), covering the mean, p95 and worst boxes, and **Z** per target (#61).
4. **Jam analysis** (#49, #56).
5. **Acceptance:** the design replaces the current best only if Z on the target box improves and
   held-out Z (the target box's poses shifted by another seed, at friction 0.2 and 0.5) is not
   worse on either. The first design that is scored becomes the best. The critic never sees the
   held-out results.
6. **Critic** (#19): a redesign brief that cites the design rules; back to 1.

Every attempt goes in the design log (#20). Every iteration writes ``RSI_RESULTS/<run>/iter-NN/``
(jam_map.png, module.png, summary.md; #15) and the run's ``progress.png``. Engine results go to
``runs/`` as usual; the loop's own files go to ``runs/loop/<run>/`` (state.json, the design
log, the spend ledger and each attempt's package).

**Stop** when Z reaches 1 on the target box, Z stops improving (``patience`` iterations with no
kept design), the critic says stop, or the iterations, the API budget or the wall-clock run out.
Then the end tune (#14) runs on the best design, with U fixed if intake fixed it.

**Resume:** ``state.json`` is written after every iteration; ``--resume <run>`` carries on from the
next one. Placement only (#46). Sim results are simulated, uncalibrated contact.

    uv run --extra agents python -m brixzle.loop --intake runs/intake-<stamp>/intake.json
"""

import argparse
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
from brixzle.engine import quick_check as QC
from brixzle.engine import report as RP
from brixzle.engine import sim_map as SM
from brixzle.engine import tune as TU
from brixzle.engine import zscore as ZS
from brixzle.intake.intake import load_intake
from brixzle.loop import render as RD
from brixzle.loop import videos as VD
from brixzle.loop.config import LoopConfig
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
        res = QC.quick_check(family, p, self.boxes[self.cfg.target], seed=self.cfg.seed)
        return res, QC.write_check(res, self.cfg.runs)

    def score(self, family, p, check, allow_failed_check=False):
        """(report, map directory): the full map covering every target box, and Z per target.
        ``allow_failed_check`` maps a design that failed the quick check: only the baseline (#85)."""
        return RP.score_family(family, p, statistic=self.cfg.target, seed=self.cfg.seed, runs=self.cfg.runs,
                               workers=self.cfg.workers, check=check, allow_failed_check=allow_failed_check)

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

    def held_out(self, family, p):
        fns = ZS.held_out_scores(family, self.boxes[self.cfg.target], self.cfg.seed + 1, workers=self.cfg.workers)
        return {name: fn(p)["counts"] for name, fn in fns.items()}

    def drop_videos(self, family, p, sim_map, map_dir, out_dir, n, seed):
        """``n`` drops of the map, picked at random with ``seed``, recorded to ``out_dir`` (#77)."""
        return VD.drop_videos(family, p, sim_map, SM.read_drops(map_dir), out_dir, n=n, seed=seed)

    def tune(self, family, p, fixed=()):
        s = TU.TuneSettings(statistic=self.cfg.target, fixed=tuple(fixed), seed=self.cfg.seed,
                            workers=self.cfg.workers, max_seconds=self.cfg.tune_seconds)
        return TU.tune_family(family, p, s, runs=self.cfg.runs)


# --- the rules ---


def accept(z, held, best):
    """(kept, reason). ``z`` is the new design's Z on the target, ``held`` its held-out counts;
    ``best`` the current best ({'z_target', 'held_out'}) or None."""
    if best is None:
        return True, "first scored design"
    if not z > best["z_target"]:
        return False, f"Z on the target did not improve ({z:+.3f}, best {best['z_target']:+.3f})"
    worse = [f"{k} {held[k]['Z']:+.3f} < {v['Z']:+.3f}" for k, v in best["held_out"].items()
             if k in held and held[k]["Z"] < v["Z"]]
    if worse:
        return False, "held-out Z got worse: " + "; ".join(worse)
    return True, f"Z on the target improved ({best['z_target']:+.3f} -> {z:+.3f}), held-out not worse"


def stop_reason(state, cfg: LoopConfig, remaining_usd, elapsed_s):
    """Why the loop stops now, or None. Iteration 0, the parent's baseline (#85), is not one of the
    ``max_iterations`` design iterations."""
    best = state.get("best")
    its = state["iterations"]
    if cfg.baseline_only:
        return "baseline only" if its else None
    if best and best["z_target"] >= 1.0:
        return "Z reached 1 on the target box"
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
        t0 = time.time() - self.state.get("elapsed_s", 0.0)
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

    def render_progress(self):
        its = self.state["iterations"]
        history = [{"iteration": i["iteration"], "decision": i["decision"], "z": i.get("z")} for i in its
                   if i["iteration"] >= 1]
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
        return self._score(rec, fam, check, res_dir, None, spent0, t_iter)

    def iterate(self, i):
        if i == 0:
            return self.baseline()
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
        last_err, repeated = None, None
        for a in range(1, cfg.design_attempts + 1):
            module = f"rsi_{self.name.replace('-', '_').lower()}_i{i:02d}_a{a}"
            t = time.time()
            try:
                outcome = DA.design(self.runner, self.intake, iter_dir / f"attempt-{a}", module, brief=brief,
                                    parent=parent, failures=failures, log_text=self.log.text(),
                                    budget_usd=cfg.design_budget_usd, max_turns=cfg.design_max_turns)
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
            att = {"attempt": a, "module": module, "root": str(outcome.workspace.root),
                   "design_cost_usd": outcome.result.cost_usd, "design_s": time.time() - t,
                   "family": outcome.check.get("family") or module, "note": outcome.note}
            rec["attempts"].append(att)
            entry = {"iteration": i, "attempt": a, "family": att["family"], "module": module,
                     "note": outcome.note, "brief_followed": (brief or {}).get("reason")}
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
            att["decision"] = "passed_quick_check"
            rec.update(module=module, root=str(outcome.workspace.root), family=att["family"], note=outcome.note)
            rec["runs"]["quick_check"] = str(check_dir)
            break
        if fam is None:
            rec["reason"] = (f"the design call failed twice with the same error, so it was not tried again: "
                             f"{repeated[:300]}" if repeated else
                             f"no design passed the contract and the quick check in {cfg.design_attempts} attempts")
            rec["spent_usd"] = self.ledger.spent() - spent0
            rec["seconds"]["total"] = time.time() - t_iter
            if outcome is not None and outcome.ok:
                self._module_png(outcome, res_dir)
            self.write_iteration_summary(rec, res_dir)
            return rec
        return self._score(rec, fam, check, res_dir, brief, spent0, t_iter, outcome)

    def _score(self, rec, fam, check, res_dir, brief, spent0, t_iter, outcome=None):
        """Map, Z, jam analysis, held-out Z, the decision, renders and the critic, for a design
        that passed the quick check or for the baseline (iteration 0)."""
        cfg, i, best = self.cfg, rec["iteration"], self.state["best"]
        baseline = rec["decision"] == BASELINE
        p = fam.defaults
        t = time.time()
        report, map_dir = self.engine.score(fam, p, check, allow_failed_check=baseline and not check["passed"])
        rec["seconds"]["map"] = time.time() - t
        rec["z"], rec["runs"]["map"] = report["z"], str(map_dir)
        t = time.time()
        jam = self.engine.jam(fam, p, map_dir)
        rec["seconds"]["jam"] = time.time() - t
        rec["runs"]["jam"] = str(Path(map_dir) / JA.FILE)
        held = {}
        if cfg.held_out:
            t = time.time()
            held = self.engine.held_out(fam, p)
            rec["seconds"]["held_out"] = time.time() - t
        rec["held_out"] = held
        try:
            rec["mass_g"] = self.engine.masses(fam, p)
        except Exception as e:  # noqa: BLE001 - a mass for the report must not stop the loop
            rec["mass_error"] = f"{type(e).__name__}: {e}"[:300]
        z_t = report["z"][cfg.target]["Z"]
        if baseline:
            kept, why = True, f"the parent family {rec['family']!r}, scored as the baseline the designs must beat"
        else:
            kept, why = accept(z_t, held, best)
            rec["decision"] = "kept" if kept else "rejected"
        rec["reason"] = why
        if kept:
            self.state["best"] = {"iteration": i, "module": rec["module"], "root": rec["root"], "family": rec["family"],
                                  "family_py": str(family_py(rec)), "registry": rec.get("registry"),
                                  "z_target": z_t, "z": report["z"], "held_out": held, "map": str(map_dir),
                                  "mass_g": rec.get("mass_g")}
        sim_map = self.engine.read_map(map_dir)
        self.log.append({"iteration": i, "attempt": len(rec["attempts"]), "family": rec["family"],
                         "module": rec["module"], "decision": rec["decision"], "reason": why, "note": rec["note"],
                         "z": report["z"], "held_out": held, "overrides": sim_map.get("overrides", []),
                         "brief_followed": (brief or {}).get("reason"), "runs": dict(rec["runs"])})
        t = time.time()
        picks = []
        if cfg.videos:
            # The pick is repeatable: its seed is the run's seed and the iteration (issue #77).
            picks = self.engine.drop_videos(fam, p, sim_map, map_dir, res_dir, cfg.videos, seed=cfg.seed * 1000 + i)
            rec["videos"] = picks
            rec["seconds"]["videos"] = time.time() - t
        t = time.time()
        RD.jam_map(sim_map, report["z"], res_dir / "jam_map.png", target=cfg.target, jam=jam,
                   title=f"{self.name} iteration {i}: {rec['family']} ({rec['decision']})", picks=picks)
        self._module_png_of(fam, res_dir)
        try:
            rec["stl"] = [path.name for path in VD.export_stl(fam, p, res_dir)]
        except Exception as e:  # noqa: BLE001 - an export must not stop the loop
            rec["stl_error"] = f"{type(e).__name__}: {e}"[:300]
        rec["seconds"]["render"] = time.time() - t
        z_best = (self.state["best"] or {}).get("z_target", -2.0)
        if z_best < 1.0 and i < cfg.max_iterations and not cfg.baseline_only:
            family_source = family_py(rec).read_text(encoding="utf-8")
            t = time.time()
            try:
                brief, result = CR.critique(self.runner, report, sim_map, jam, family_source, self.log.text(),
                                            cfg.target, budget_usd=cfg.critic_budget_usd)
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
        t = time.time()
        try:
            result, out = self.engine.tune(fam, fam.defaults, fixed)
        except ValueError as e:  # for example a family with nothing left to tune
            return {"error": str(e)}
        return {"run": str(out), "accepted": result["accepted"], "improved": result["improved"],
                "baseline": result["baseline"]["score"], "tuned": result["tuned"]["score"],
                "params": result["params"], "seconds": time.time() - t}

    def write_iteration_summary(self, rec, res_dir):
        text = RD.iteration_summary(rec, self.name, self.cfg.target, self.ledger.path, self.ledger.spent(),
                                    self.cfg.budget_usd)
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
    ap.add_argument("--target", choices=BX.STATISTICS, default=d.target)
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
    return LoopConfig(target=args.target, parent=parent, baseline_only=args.baseline_only,
                      max_iterations=args.max_iterations, patience=args.patience,
                      budget_usd=args.budget_usd, ledger=args.ledger, max_minutes=args.max_minutes,
                      design_attempts=args.design_attempts, seed=args.seed, workers=args.workers,
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
        z = (it.get("z") or {}).get(cfg.target, {}).get("Z")
        print(f"iteration {it['iteration']}: {it.get('family', '-')} {it['decision']}"
              + (f", Z {cfg.target} {z:+.3f}" if z is not None else "") + f" ({it.get('reason', '')})")
    print(f"stopped: {state['stop']}; spent ${loop.ledger.spent():.4f} of ${cfg.budget_usd:.2f}")
    if state.get("tune"):
        print(f"end tune: {state['tune']}")
    print(f"-> {loop.results}  (state: {loop.run_dir / STATE})")


if __name__ == "__main__":
    main()
