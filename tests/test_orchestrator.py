"""Orchestrator (issue #21): the loop end to end with stubbed agents and a stubbed engine.

The agents are scripted (no API call) and the engine is a fake that returns canned results, so a
whole run takes seconds. The real engine stages have their own tests.
"""

import json
from pathlib import Path

import pytest

from brixzle.agents import runner as RN
from brixzle.engine import boxes as BX
from brixzle.loop import orchestrator as OR
from brixzle.loop.config import LoopConfig

from brixzle.agents import check_family as CF

REPO = Path(__file__).resolve().parents[1]
WEDGE = (REPO / "src/brixzle/families/wedge/family.py").read_text(encoding="utf-8")
BOXES = BX.default_boxes()["placement"]
GRID = {"slice": "xz", "course": 0, "axes": ["x", "z"], "step": {"x": 1.0, "z": 5.0},
        "index_extent": {"x": [-3, 3], "z": [0, 1]}, "extent": {"x": [-3.0, 3.0], "z": [0.0, 5.0]},
        "rows": ["WSSJSSJ", "FSSSSSJ"], "fixed": {"t": 0.0}}


_CHECKS = {}


@pytest.fixture(autouse=True)
def _memo_family_check(monkeypatch):
    """The family check (contract, rules, two aim drops) runs in a subprocess in about 2.5 s, and
    every scripted design here writes one of a few identical sources: check each source once per
    session. tests/test_design_agent.py covers the real check."""
    real = CF.check_in_subprocess

    def memo(root, module, **kw):
        path = Path(root) / module / "family.py"
        key = path.read_text(encoding="utf-8") if path.exists() else None
        if key is None:
            return real(root, module, **kw)
        if key not in _CHECKS:
            _CHECKS[key] = real(root, module, **kw)
        return {**_CHECKS[key], "module": module}

    monkeypatch.setattr(CF, "check_in_subprocess", memo)


def counts(z, n=100):
    s = round((z + 1) / 2 * n)
    return {"Z": (s - (n - s)) / n, "S": s, "J": n - s, "fell": 0, "wrong_valley": 0, "other": 0, "N": n,
            "passed": s == n}


class FakeEngine:
    """Canned engine results. ``zs`` is, per scored design in order, Z on every box (a number) or
    Z per box ({statistic: Z}); ``checks`` whether each quick check passes (default: all pass);
    ``held`` held-out Z per held-out call."""

    def __init__(self, tmp, zs, checks=None, held=None):
        self.tmp, self.zs, self.checks, self.held = Path(tmp), list(zs), list(checks or []), list(held or [])
        self.calls, self.n = [], 0
        self.held_targets, self.tune_targets = [], []

    def _dir(self, name):
        self.n += 1
        d = self.tmp / f"{name}-{self.n}"
        d.mkdir(parents=True)
        return d

    def quick_check(self, family, p):
        self.calls.append("quick_check")
        ok = self.checks.pop(0) if self.checks else True
        res = {"passed": ok, "seated": 36 if ok else 20, "dropped": 40 if ok else 29, "need": 32,
               "outcomes": {"seated": 36} if ok else {"seated": 20, "jammed": 9},
               "failures": [] if ok else [{"course": 1, "dx_mm": 12.0, "clear_mm": 6.0, "pitch_deg": 0.1,
                                           "outcome": "jammed", "err_mm": [3.0, 0.0, 4.0], "angle_deg": 20.0}]}
        return res, self._dir("check")

    def score(self, family, p, check, allow_failed_check=False):
        self.calls.append(("score", family.name, allow_failed_check))
        z = self.zs.pop(0)
        z = z if isinstance(z, dict) else {s: z for s in ("mean", "p95", "max")}
        d = self._dir("map")
        m = {"family": family.name, "box": BOXES["mean"].to_dict(), "cover_boxes": [b.to_dict() for b in BOXES.values()],
             "grids": [GRID], "overrides": []}
        (d / "map.json").write_text(json.dumps(m), encoding="utf-8")
        return {"z": {s: counts(z[s]) for s in ("mean", "p95", "max")}, "statistics": ["mean", "p95", "max"]}, d

    def read_map(self, d):
        return json.loads((Path(d) / "map.json").read_text(encoding="utf-8"))

    def jam(self, family, p, map_dir):
        self.calls.append("jam")
        return {"family": family.name, "box": BOXES["mean"].to_dict(), "seconds": 1.0, "one_offs": [], "bands": [],
                "capture_limits": [], "counts": {"inside_failures": 0, "repeatable": 0, "bands": 0, "three_d": 0,
                                                 "reruns": 0, "capture_limits": 0}}

    def masses(self, family, p):
        return {part: 10.0 for part in family.parts}

    def held_out(self, family, p, target=None):
        self.calls.append("held_out")
        self.held_targets.append((family.name, target))
        z = self.held.pop(0) if self.held else 0.0
        return {"seed 1, friction 0.2": counts(z), "seed 1, friction 0.5": counts(z)}

    def drop_videos(self, family, p, sim_map, map_dir, out_dir, n, seed):
        self.calls.append(("videos", n, seed))
        picks = []
        for k in range(1, n + 1):
            (Path(out_dir) / f"drop-{k}.mp4").write_bytes(b"\x00")
            picks.append({"k": k, "slice": "xz", "course": 0, "cell": [k, 0], "pose": {"x": float(k), "z": 1.0, "t": 0.0},
                          "outcome": "seated", "video": f"drop-{k}.mp4", "rerun_outcome": "seated"})
        return picks

    def tune(self, family, p, fixed=(), target=None):
        self.calls.append(("tune", tuple(fixed)))
        self.tune_targets.append(target)
        return {"accepted": False, "improved": False, "baseline": {"score": 0.1}, "tuned": {"score": 0.1},
                "params": p.to_dict()}, self._dir("tune")


def design_step(name="loopwedge"):
    def step(call):
        pkg = Path(call.write_root)
        (pkg / "family.py").write_text(WEDGE.replace('name="wedge"', f'name="{name}"'), encoding="utf-8")
        (pkg / "design_note.json").write_text(json.dumps({"summary": f"{name} chevron", "changes": "x",
                                                          "geometry": "Prismatic"}), encoding="utf-8")
        return RN.AgentResult(text="done", cost_usd=0.10, turns=5)
    return step


def critic_step(decision="redesign"):
    brief = {"decision": decision, "reason": "jams at the +X edge", "clusters": [] if decision == "stop" else [
        {"failure": "jam band", "diagnosis": "two-point wedge", "rules": ["DR2"], "change": "longer face",
         "scope": "reshape", "expected_effect": "fewer jams"}]}
    return RN.AgentResult(structured=brief, cost_usd=0.02, turns=1)


@pytest.fixture
def intake(tmp_path):
    from brixzle.intake import intake as IN

    img = tmp_path / "sketch.png"
    img.write_bytes(b"\x89PNG\r\n\x1a\n" + b"\x00" * 16)
    out = IN.write_intake(IN.build_intake([img], [], fixed_U_mm=25.0), runs=tmp_path / "runs")
    return out / "intake.json"


def _cfg(tmp_path, **kw):
    """The existing cases run from scratch (``--parent none``) on a fixed target (``ladder=False``);
    the baseline tests pass parent='ab' and the ladder tests ladder=True."""
    return LoopConfig(runs=str(tmp_path / "runs"), results=str(tmp_path / "RSI_RESULTS"),
                      **{"max_iterations": 2, "budget_usd": 5.0, "parent": None, "ladder": False, **kw})


def _loop(tmp_path, intake, script, engine, **kw):
    cfg = _cfg(tmp_path, **kw)
    led = RN.SpendLedger(tmp_path / "spend.jsonl", cfg.budget_usd)
    runner = RN.ScriptedRunner(script, ledger=led)
    loop = OR.Loop(intake, cfg, runner=runner, engine=engine, name="t")
    loop.ledger = led
    return loop, runner


# --- the rules ---


def test_the_first_scored_design_is_kept():
    assert OR.accept(0.2, {}, None) == (True, "first scored design")


def test_a_redesign_is_kept_only_if_z_improves_and_held_out_is_not_worse():
    best = {"z_target": 0.5, "held_out": {"a": counts(0.4), "b": counts(0.4)}}
    assert OR.accept(0.6, {"a": counts(0.4), "b": counts(0.5)}, best)[0]
    kept, why = OR.accept(0.5, {"a": counts(0.9), "b": counts(0.9)}, best)
    assert not kept and "did not improve" in why
    kept, why = OR.accept(0.7, {"a": counts(0.2), "b": counts(0.9)}, best)
    assert not kept and "held-out" in why


def test_the_loop_stops_at_z_one_or_on_the_budgets():
    cfg = LoopConfig(max_iterations=5, patience=2, ladder=False)
    state = {"best": None, "iterations": []}
    assert OR.stop_reason(state, cfg, 5.0, 0.0) is None
    assert "Z reached 1" in OR.stop_reason({"best": {"z_target": 1.0}, "iterations": [{}]}, cfg, 5.0, 0.0)
    assert "budget" in OR.stop_reason(state, cfg, 0.5, 0.0)
    assert "wall-clock" in OR.stop_reason(state, cfg, 5.0, 31 * 60)
    stale = {"best": {"z_target": 0.3}, "iterations": [{"iteration": 1, "decision": "kept"},
                                                       {"iteration": 2, "decision": "rejected"},
                                                       {"iteration": 3, "decision": "rejected"}]}
    assert "no kept design for 2" in OR.stop_reason(stale, cfg, 5.0, 0.0)
    assert "critic said stop" in OR.stop_reason({"best": None, "iterations": [{"brief": {"decision": "stop",
                                                                                         "reason": "r"}}]},
                                                cfg, 5.0, 0.0)


# --- end to end with stubbed agents ---


def test_an_end_to_end_run_with_stubbed_agents(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.2, 0.5], held=[0.1, 0.3])
    loop, runner = _loop(tmp_path, intake, [design_step("w1"), critic_step(), design_step("w2"), critic_step()],
                         engine)
    state = loop.run()
    its = state["iterations"]
    assert [i["decision"] for i in its] == ["kept", "kept"] and state["best"]["iteration"] == 2
    assert state["stop"] == "2 iterations run"
    assert [c.name for c in runner.calls] == ["design", "critic", "design"]  # no critic on the last iteration
    for k in (1, 2):
        d = tmp_path / "RSI_RESULTS" / "t" / f"iter-{k:02d}"
        assert (d / "jam_map.png").stat().st_size > 0 and (d / "module.png").stat().st_size > 0
        text = (d / "summary.md").read_text(encoding="utf-8")
        assert "Z per target box" in text and "simulated, uncalibrated contact" in text and "Decision: kept" in text
        assert sorted(f.name for f in d.glob("drop-*.mp4")) == [f"drop-{j}.mp4" for j in range(1, 6)]
        assert (d / "loopwedge-A.stl").exists() or list(d.glob("*.stl"))
        assert "## Drop videos" in text
    assert (tmp_path / "RSI_RESULTS" / "t" / "progress.png").exists()
    assert (tmp_path / "RSI_RESULTS" / "t" / "summary.md").exists()
    log = [json.loads(line) for line in (loop.run_dir / "design_log.jsonl").read_text().splitlines()]
    assert [e["decision"] for e in log] == ["kept", "kept"]
    assert ("tune", ("U",)) in engine.calls  # U was fixed at intake, so the tune leaves it alone
    assert state["spent_usd"] == pytest.approx(0.22)
    # the second design was told the brief and given the first as its parent
    second = runner.calls[2].prompt
    assert "jams at the +X edge" in second and "Parent design" in second


def test_the_critic_never_sees_held_out_results(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.2, 0.5], held=[0.123, 0.3])
    loop, runner = _loop(tmp_path, intake, [design_step(), critic_step(), design_step(), critic_step()], engine)
    loop.run()
    for call in runner.calls:
        assert "friction 0.2" not in call.prompt and "held_out" not in call.prompt


def test_a_design_that_fails_the_quick_check_goes_back_with_its_failing_drops(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.4], checks=[False, True])
    loop, runner = _loop(tmp_path, intake, [design_step("a"), design_step("b"), critic_step()], engine,
                         max_iterations=1)
    state = loop.run()
    it = state["iterations"][0]
    assert [a["decision"] for a in it["attempts"]] == ["failed_quick_check", "passed_quick_check"]
    assert "failing_drops" in runner.calls[1].prompt and "jammed" in runner.calls[1].prompt
    log = [json.loads(line)["decision"] for line in (loop.run_dir / "design_log.jsonl").read_text().splitlines()]
    assert log == ["failed_quick_check", "kept"]


def test_a_rejected_redesign_keeps_the_old_best(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.5, 0.3], held=[0.2, 0.2])
    loop, _ = _loop(tmp_path, intake, [design_step(), critic_step(), design_step(), critic_step()], engine)
    state = loop.run()
    assert [i["decision"] for i in state["iterations"]] == ["kept", "rejected"]
    assert state["best"]["iteration"] == 1


def test_z_of_one_stops_the_loop_without_a_critic(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[1.0])
    loop, runner = _loop(tmp_path, intake, [design_step()], engine, max_iterations=4)
    state = loop.run()
    assert state["stop"] == "Z reached 1 on the target box" and [c.name for c in runner.calls] == ["design"]


def test_the_spend_cap_stops_the_loop(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.2, 0.3])
    loop, runner = _loop(tmp_path, intake, [design_step(), critic_step(), design_step()], engine,
                         budget_usd=1.95, max_iterations=4)
    state = loop.run()
    assert "budget" in state["stop"] and loop.ledger.spent() <= 1.95


def test_a_run_resumes_from_its_state(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.2, 0.5])
    loop, _ = _loop(tmp_path, intake, [design_step(), critic_step()], engine)
    rec = loop.iterate(1)
    loop.state["iterations"].append(rec)
    OR._save(loop.run_dir, loop.state)
    cfg = _cfg(tmp_path)
    runner = RN.ScriptedRunner([design_step("again")], ledger=loop.ledger)
    again = OR.Loop(None, cfg, runner=runner, engine=engine, name="t", resume=True)
    state = again.run()
    assert [i["iteration"] for i in state["iterations"]] == [1, 2]


def test_an_agent_error_is_logged_and_the_loop_carries_on(tmp_path, intake):
    def broken(call):
        raise RuntimeError("Claude Code returned an error result")

    def broken_critic(call):
        raise RuntimeError("Reached maximum number of turns")

    engine = FakeEngine(tmp_path / "eng", zs=[0.4])
    loop, runner = _loop(tmp_path, intake, [broken, design_step(), broken_critic], engine, max_iterations=2,
                         design_attempts=2)
    loop.cfg = loop.cfg  # unchanged
    rec = loop.iterate(1)
    assert [a["decision"] for a in rec["attempts"]] == ["error", "passed_quick_check"]
    assert rec["decision"] == "kept" and rec["brief"] is None and "maximum number of turns" in rec["critic_error"]
    log = [json.loads(line)["decision"] for line in (loop.run_dir / "design_log.jsonl").read_text().splitlines()]
    assert log == ["error", "kept"]


def test_a_design_call_that_fails_twice_the_same_way_is_not_called_a_third_time(tmp_path, intake):
    """Two identical failures cost $3.00 of the ledger before the cause was found (issue #79); a third
    identical call would only repeat them."""
    def broken(call):
        raise RuntimeError("JSON message exceeded maximum buffer size")

    engine = FakeEngine(tmp_path / "eng", zs=[])
    loop, runner = _loop(tmp_path, intake, [broken, broken, design_step()], engine, max_iterations=1,
                         design_attempts=3)
    rec = loop.iterate(1)
    assert len(runner.calls) == 2 and rec["decision"] == "no_design"
    assert [a["decision"] for a in rec["attempts"]] == ["error", "error"]
    assert "twice" in rec["reason"] and "maximum buffer size" in rec["reason"]


def test_two_different_errors_still_get_another_attempt(tmp_path, intake):
    def one(call):
        raise RuntimeError("one")

    def two(call):
        raise RuntimeError("two")

    engine = FakeEngine(tmp_path / "eng", zs=[0.4])
    loop, runner = _loop(tmp_path, intake, [one, two, design_step(), critic_step()], engine, max_iterations=1,
                         design_attempts=3)
    rec = loop.iterate(1)
    assert len(runner.calls) >= 3 and rec["decision"] == "kept"


# --- the parent family as iteration 0 (issue #85) ---


def test_with_a_parent_iteration_0_scores_it_with_no_design_call_and_it_is_the_best(tmp_path, intake):
    # The parent fails its quick check, and is still scored: it is the reference, not a candidate.
    engine = FakeEngine(tmp_path / "eng", zs=[0.4], checks=[False])
    loop, runner = _loop(tmp_path, intake, [critic_step()], engine, parent="ab", max_iterations=1)
    rec = loop.iterate(0)
    assert rec["iteration"] == 0 and rec["decision"] == "baseline" and rec["family"] == "ab"
    assert rec["attempts"] == [] and not any(c.name == "design" for c in runner.calls)
    assert ("score", "ab", True) in engine.calls and rec["quick_check"]["passed"] is False
    assert rec["z"]["mean"]["Z"] == pytest.approx(0.4) and rec["mass_g"] == {"A": 10.0, "B": 10.0}
    assert rec["spent_usd"] == pytest.approx(0.02) and rec.get("design_cost_usd") is None  # the critic only
    best = loop.state["best"]
    assert best["iteration"] == 0 and best["family"] == "ab" and best["z_target"] == pytest.approx(0.4)
    assert best["family_py"].endswith("src/brixzle/families/ab/family.py")
    d = tmp_path / "RSI_RESULTS" / "t" / "iter-00"
    assert (d / "jam_map.png").exists() and (d / "module.png").exists() and "baseline" in (d / "summary.md").read_text()


def test_baseline_only_scores_the_parent_and_spends_nothing(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.4])
    loop, runner = _loop(tmp_path, intake, [], engine, parent="ab", baseline_only=True, tune=False)
    state = loop.run()
    assert [i["iteration"] for i in state["iterations"]] == [0] and runner.calls == []
    assert state["stop"] == "baseline only" and loop.ledger.spent() == 0.0
    text = (tmp_path / "RSI_RESULTS" / "t" / "summary.md").read_text(encoding="utf-8")
    assert "## Best against the baseline" in text and "+0.400" in text
    assert "src/brixzle/families/ab/family.py" in text and "brixzle.families.ab`" not in text


def test_a_first_design_must_beat_the_parent(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.5, 0.3, 0.7], held=[0.2, 0.2, 0.2])
    loop, runner = _loop(tmp_path, intake, [critic_step(), design_step("lo"), critic_step(), design_step("hi")],
                         engine, parent="ab", max_iterations=2, patience=3)
    state = loop.run()
    assert [(i["iteration"], i["decision"]) for i in state["iterations"]] == [(0, "baseline"), (1, "rejected"),
                                                                              (2, "kept")]
    assert "did not improve" in state["iterations"][1]["reason"]
    assert state["best"]["iteration"] == 2 and state["stop"] == "2 iterations run"
    assert [c.name for c in runner.calls] == ["critic", "design", "critic", "design"]
    text = (tmp_path / "RSI_RESULTS" / "t" / "summary.md").read_text(encoding="utf-8")
    assert "## Best against the baseline" in text and "+0.700" in text and "+0.500" in text


def test_the_critic_reads_the_parent_and_its_brief_reaches_the_first_design(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.5, 0.6])
    loop, runner = _loop(tmp_path, intake, [critic_step(), design_step()], engine, parent="ab", max_iterations=1)
    state = loop.run()
    critic, design = runner.calls
    assert critic.name == "critic" and "FAMILY" in critic.prompt and "Target box: mean" in critic.prompt
    assert design.name == "design" and "jams at the +X edge" in design.prompt
    assert "src/brixzle/families/ab/family.py" in design.prompt and "Parent design" in design.prompt
    assert state["iterations"][1]["brief_in"]["reason"] == "jams at the +X edge"


def test_parent_none_runs_from_scratch_and_the_cli_takes_the_parent():
    assert LoopConfig().parent == "ab"
    assert OR.config_from_args(OR.parser().parse_args(["--intake", "x"])).parent == "ab"
    assert OR.config_from_args(OR.parser().parse_args(["--intake", "x", "--parent", "none"])).parent is None
    assert OR.config_from_args(OR.parser().parse_args(["--intake", "x", "--baseline-only"])).baseline_only
    assert LoopConfig().ladder and OR.config_from_args(OR.parser().parse_args(["--intake", "x"])).ladder
    assert not OR.config_from_args(OR.parser().parse_args(["--intake", "x", "--no-ladder"])).ladder
    with pytest.raises(ValueError, match="parent"):
        LoopConfig(parent="no_such_family").validate()


def test_parent_none_starts_at_iteration_1(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.2])
    loop, runner = _loop(tmp_path, intake, [design_step()], engine, max_iterations=1)
    state = loop.run()
    assert [i["iteration"] for i in state["iterations"]] == [1] and state["best"]["iteration"] == 1
    assert [c.name for c in runner.calls] == ["design"]


# --- the target ladder: mean, then p95, then max (issue #84) ---


def z3(mean, p95, mx):
    return {"mean": mean, "p95": p95, "max": mx}


def _best(mean, p95, mx, target="mean"):
    z = {"mean": counts(mean), "p95": counts(p95), "max": counts(mx)}
    return {"z_target": z[target]["Z"], "z": z, "target": target, "held_out": {}, "held_out_by": {}}


def test_the_current_target_is_the_first_rung_below_one():
    cfg = LoopConfig()
    assert OR.current_target({"best": None, "iterations": []}, cfg) == "mean"
    assert OR.current_target({"best": _best(0.8, 0.3, 0.1), "iterations": []}, cfg) == "mean"
    assert OR.current_target({"best": _best(1.0, 0.3, 0.1), "iterations": []}, cfg) == "p95"
    assert OR.current_target({"best": _best(1.0, 1.0, 0.1), "iterations": []}, cfg) == "max"
    assert OR.current_target({"best": _best(1.0, 0.3, 0.1), "iterations": []}, LoopConfig(target="p95")) == "p95"
    assert OR.current_target({"best": _best(1.0, 0.3, 0.1), "iterations": []}, LoopConfig(ladder=False)) == "mean"


def test_z_reached_one_stops_the_ladder_only_on_the_last_rung():
    cfg = LoopConfig(max_iterations=5)
    assert OR.stop_reason({"best": _best(1.0, 0.6, 0.2), "iterations": [{"iteration": 1, "decision": "kept"}]},
                          cfg, 5.0, 0.0) is None
    assert OR.stop_reason({"best": _best(1.0, 1.0, 0.9), "iterations": [{"iteration": 1, "decision": "kept"}]},
                          cfg, 5.0, 0.0) is None
    assert "Z reached 1" in OR.stop_reason({"best": _best(1.0, 1.0, 1.0),
                                            "iterations": [{"iteration": 1, "decision": "kept"}]}, cfg, 5.0, 0.0)
    flat = LoopConfig(max_iterations=5, ladder=False)
    assert "Z reached 1" in OR.stop_reason({"best": _best(1.0, 0.6, 0.2),
                                            "iterations": [{"iteration": 1, "decision": "kept"}]}, flat, 5.0, 0.0)


def test_a_design_that_lowers_a_lower_rung_is_rejected_with_the_reason():
    best = {"z_target": 0.5, "held_out": {"a": counts(0.4)}}
    kept, why = OR.accept(0.8, {"a": counts(0.5)}, best, lower=[("mean", 0.9, 1.0)])
    assert not kept and "lower rung" in why and "mean +0.900 < +1.000" in why
    assert OR.accept(0.8, {"a": counts(0.5)}, best, lower=[("mean", 1.0, 1.0)])[0]


def test_z_one_on_mean_climbs_to_p95_and_the_critic_is_told(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[z3(1.0, 0.5, 0.2), z3(1.0, 0.7, 0.3)], held=[0.5, 0.5, 0.5])
    loop, runner = _loop(tmp_path, intake, [design_step("w1"), critic_step(), design_step("w2")], engine,
                         ladder=True, max_iterations=2)
    state = loop.run()
    its = state["iterations"]
    assert [(i["iteration"], i["decision"], i["target"]) for i in its] == [(1, "kept", "mean"), (2, "kept", "p95")]
    assert state["stop"] == "2 iterations run"
    critic = [c for c in runner.calls if c.name == "critic"]
    assert len(critic) == 1 and "Target box: p95" in critic[0].prompt
    # held-out on mean for design 1; on p95 for design 2 and, once, for the best it was compared with
    assert [t for _, t in engine.held_targets] == ["mean", "p95", "p95"]
    assert engine.tune_targets == ["p95"]  # the end tune works on the first box still below 1
    assert state["best"]["iteration"] == 2 and set(state["best"]["held_out_by"]) == {"p95"}
    text = (tmp_path / "RSI_RESULTS" / "t" / "iter-02" / "summary.md").read_text(encoding="utf-8")
    assert "judged on the **p95** box" in text
    assert (tmp_path / "RSI_RESULTS" / "t" / "progress.png").exists()


def test_raising_p95_but_lowering_mean_is_rejected(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[z3(1.0, 0.5, 0.2), z3(0.9, 0.8, 0.3)])
    loop, _ = _loop(tmp_path, intake, [design_step("w1"), critic_step(), design_step("w2")], engine,
                    ladder=True, max_iterations=2)
    state = loop.run()
    second = state["iterations"][1]
    assert second["decision"] == "rejected" and second["target"] == "p95"
    assert "lower rung" in second["reason"] and "mean" in second["reason"]
    assert state["best"]["iteration"] == 1


def test_the_same_run_without_the_ladder_stops_at_z_one_on_mean(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[z3(1.0, 0.5, 0.2)])
    loop, runner = _loop(tmp_path, intake, [design_step()], engine, ladder=False, max_iterations=3)
    state = loop.run()
    assert state["stop"] == "Z reached 1 on the target box" and [c.name for c in runner.calls] == ["design"]
    assert engine.tune_targets == ["mean"]


def test_the_ladder_stops_at_z_one_on_max_without_a_critic(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[1.0])
    loop, runner = _loop(tmp_path, intake, [design_step()], engine, ladder=True, max_iterations=3)
    state = loop.run()
    assert "Z reached 1" in state["stop"] and "max" in state["stop"]
    assert [c.name for c in runner.calls] == ["design"]


def test_a_state_from_before_the_ladder_resumes_without_it(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[z3(1.0, 0.5, 0.2), z3(1.0, 0.6, 0.2)])
    loop, _ = _loop(tmp_path, intake, [design_step(), critic_step()], engine, ladder=False)
    loop.state["iterations"].append(loop.iterate(1))
    old = {k: v for k, v in loop.state["config"].items() if k not in ("ladder", "parent", "baseline_only")}
    loop.state["config"] = old
    for k in ("target", "held_out_by"):
        loop.state["best"].pop(k, None)
    OR._save(loop.run_dir, loop.state)
    runner = RN.ScriptedRunner([design_step("again")], ledger=loop.ledger)
    again = OR.Loop(None, _cfg(tmp_path, ladder=True, max_iterations=3), runner=runner, engine=engine, name="t",
                    resume=True)
    assert again.cfg.ladder is False
    state = again.run()
    assert state["stop"] == "Z reached 1 on the target box" and [i["iteration"] for i in state["iterations"]] == [1]
