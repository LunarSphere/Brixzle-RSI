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

REPO = Path(__file__).resolve().parents[1]
WEDGE = (REPO / "src/brixzle/families/wedge/family.py").read_text(encoding="utf-8")
BOXES = BX.default_boxes()["placement"]
GRID = {"slice": "xz", "course": 0, "axes": ["x", "z"], "step": {"x": 1.0, "z": 5.0},
        "index_extent": {"x": [-3, 3], "z": [0, 1]}, "extent": {"x": [-3.0, 3.0], "z": [0.0, 5.0]},
        "rows": ["WSSJSSJ", "FSSSSSJ"], "fixed": {"t": 0.0}}


def counts(z, n=100):
    s = round((z + 1) / 2 * n)
    return {"Z": (s - (n - s)) / n, "S": s, "J": n - s, "fell": 0, "wrong_valley": 0, "other": 0, "N": n,
            "passed": s == n}


class FakeEngine:
    """Canned engine results. ``zs`` is Z on every box per scored design, in order; ``checks``
    whether each quick check passes (default: all pass); ``held`` held-out Z per scored design."""

    def __init__(self, tmp, zs, checks=None, held=None):
        self.tmp, self.zs, self.checks, self.held = Path(tmp), list(zs), list(checks or []), list(held or [])
        self.calls, self.n = [], 0

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

    def score(self, family, p, check):
        self.calls.append("score")
        z = self.zs.pop(0)
        d = self._dir("map")
        m = {"family": family.name, "box": BOXES["mean"].to_dict(), "cover_boxes": [b.to_dict() for b in BOXES.values()],
             "grids": [GRID], "overrides": []}
        (d / "map.json").write_text(json.dumps(m), encoding="utf-8")
        return {"z": {s: counts(z) for s in ("mean", "p95", "max")}, "statistics": ["mean", "p95", "max"]}, d

    def read_map(self, d):
        return json.loads((Path(d) / "map.json").read_text(encoding="utf-8"))

    def jam(self, family, p, map_dir):
        self.calls.append("jam")
        return {"family": family.name, "box": BOXES["mean"].to_dict(), "seconds": 1.0, "one_offs": [], "bands": [],
                "capture_limits": [], "counts": {"inside_failures": 0, "repeatable": 0, "bands": 0, "three_d": 0,
                                                 "reruns": 0, "capture_limits": 0}}

    def held_out(self, family, p):
        self.calls.append("held_out")
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

    def tune(self, family, p, fixed=()):
        self.calls.append(("tune", tuple(fixed)))
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
    return LoopConfig(runs=str(tmp_path / "runs"), results=str(tmp_path / "RSI_RESULTS"),
                      **{"max_iterations": 2, "budget_usd": 5.0, **kw})


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
    cfg = LoopConfig(max_iterations=5, patience=2)
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
