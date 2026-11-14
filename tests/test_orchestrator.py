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
        self.zs_x_only = []  # Z without Y offsets per scored design (issue #86); default: the same as zs
        self.gates = []  # (passed, overhang_n) per structure gate (issue #83); default: (True, 5)
        self.rings = []  # ring build status per call (issue #89); default: 'complete'

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
        zx = self.zs_x_only.pop(0) if self.zs_x_only else z
        zx = zx if isinstance(zx, dict) else {s: zx for s in ("mean", "p95", "max")}
        return {"z": {s: counts(z[s]) for s in ("mean", "p95", "max")},
                "z_x_only": {s: counts(zx[s]) for s in ("mean", "p95", "max")},
                "statistics": ["mean", "p95", "max"]}, d

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

    def gate(self, family, p):
        """The structure gate (#83): ``gates`` is (passed, overhang_n) per call; default (True, 5)."""
        self.calls.append("gate")
        ok, n = self.gates.pop(0) if self.gates else (True, 5)
        d = self._dir("gate")
        build = lambda passed: {"passed": passed, "status": "complete" if passed else "placement_jammed",  # noqa: E731
                                "complete": 3 if passed else 0, "n": 3, "statuses": ["complete"] * 3 if passed else
                                ["placement_jammed"] * 3, "stable_bricks": [10] * 3 if passed else [4] * 3, "total": 10,
                                "max_sag_mm": [0.1] * 3, "failed_brick": [None] * 3 if passed else ["b2_0"] * 3,
                                "seconds": [1.0] * 3, "error": None}
        over = {**build(n >= 3), "total": 12, "stable_bricks": [n] * 3, "overhang_n": n}
        g = {"passed": ok, "overhang_n": n, "overhang_floor": 3, "seconds": 2.0, "run": str(d), "repeats": 3,
             "builds": {"tower": build(True), "wall": build(ok), "bridge": build(True), "overhang": over},
             "label": "simulated, uncalibrated contact"}
        return g, d

    def ring(self, family, p):
        """The ring build (#89): ``rings`` is a status per call; default 'complete'."""
        self.calls.append("ring")
        status = self.rings.pop(0) if self.rings else "complete"
        d = self._dir("ring")
        ok = status == "complete"
        stage = {"brick": "K2_1_0_2" if not ok else "S2_1_1.5_5", "part": "K2B", "outcome": "seated" if ok else "jammed",
                 "err_mm": [0.0, 0.0, 0.0] if ok else [2.0, 0.0, 3.0], "angle_deg": 0.5, "collapsed": False,
                 "sag_mm": 0.1}
        return {"structure": "ring6x2", "status": status, "seated": 16 if ok else 15, "placed": 16, "total": 16,
                "stages": [stage], "seconds": 3.0, "run": str(d), "label": "simulated, uncalibrated contact"}, d

    def screen(self, jobs, target):
        """The cheap screen (#108): ``screens`` is {module: (S, J, N, quick check passed)}; default (40, 4, 60, True)."""
        self.calls.append(("screen", tuple(k for k, _, _ in jobs), target))
        out = {}
        for k, root, module in jobs:
            s_, j_, n_, ok = getattr(self, "screens", {}).get(module, (40, 4, 60, True))
            out[k] = {"Z": (s_ - j_) / n_, "S": s_, "J": j_, "N": n_, "seconds": 2.0,
                      "check": {"passed": ok, "seated": 36 if ok else 20, "dropped": 40, "outcomes": {"seated": 36}}}
        return out

    def probe_poses(self, map_dir, target, seed):
        """The probe's poses (#107): records which map and box they came from."""
        self.calls.append(("probe_poses", None if map_dir is None else Path(map_dir).name, target, seed))
        return [{"source": "aim", "course": 0, "x_mm": 0.0, "y_mm": 0.0, "sample": {}}]

    def probe(self, root, module, poses_path):
        self.calls.append(("probe", module))
        return {"drops": 1, "outcomes": {"seated": 1}, "parent_failures_now_seated": 0,
                "parent_seated_now_failing": 0, "seconds": 0.1, "rows": []}

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


def probing_design_step(name="probewedge", calls=2):
    """A design step that calls the probe_drops tool ``calls`` times before it writes its family."""
    inner = design_step(name)

    def step(call):
        tool = next(t for t in call.mcp_tools if t.name == "probe_drops")
        for _ in range(calls):
            tool.handler({})
        return inner(call)
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
                      **{"max_iterations": 2, "budget_usd": 5.0, "parent": None, "ladder": False, "concepts": 1,
                         "lessons_file": str(tmp_path / "lessons.jsonl"), **kw})


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


def test_a_held_out_drop_inside_the_noise_band_keeps_the_design():
    # #110: held-out counts as worse only when lower by more than one standard error of Z
    best = {"z_target": 0.5, "held_out": {"a": counts(0.4)}}
    band = OR.ZS.z_band(counts(0.4))
    kept, why = OR.accept(0.6, {"a": counts(0.38)}, best)
    assert 0.02 < band and kept and "inside the band" in why
    kept, why = OR.accept(0.6, {"a": counts(0.2)}, best)
    assert not kept and "held-out Z got worse" in why and f"band {band:.3f}" in why


def test_iteration_4_of_ab4iter2_would_now_be_kept():
    # runs/loop/ab4iter2-20261114-mean/state.json: iteration 3 was the best, iteration 4 was rejected
    # on two held-out drops at friction 0.2 (277 against 279 seated, 31 jams each, N 356)
    def c(s, j, n=356):
        return {"Z": (s - j) / n, "S": s, "J": j, "N": n}

    best = {"z_target": 0.5365, "overhang_n": 5,
            "held_out": {"seed 1, friction 0.2": c(279, 31), "seed 1, friction 0.5": c(240, 74)}}
    held = {"seed 1, friction 0.2": c(277, 31), "seed 1, friction 0.5": c(242, 69)}
    kept, why = OR.accept(0.5730, held, best, overhang=5)
    assert kept, why
    assert "-0.006" in why and "inside the band of 0.033" in why


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
    assert [a["decision"] for a in it["attempts"]] == ["failed_quick_check", "passed_structure_gate"]
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
    assert [a["decision"] for a in rec["attempts"]] == ["error", "passed_structure_gate"]
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



# --- the X-Y disc (issue #86) ---


def test_no_y_error_judges_designs_on_z_without_y_offsets(tmp_path, intake):
    """``--no-y-error`` scores as before #86: the second design is better on X alone but worse with Y."""
    def run(y_error, name):
        engine = FakeEngine(tmp_path / f"eng-{name}", zs=[0.5, 0.3], held=[0.2, 0.2, 0.2])
        engine.zs_x_only = [0.5, 0.7]
        loop, _ = _loop(tmp_path / name, intake, [design_step(), critic_step(), design_step()], engine,
                        y_error=y_error)
        return [i["decision"] for i in loop.run()["iterations"]]

    assert run(True, "y") == ["kept", "rejected"]
    assert run(False, "x") == ["kept", "kept"]
    assert LoopConfig().y_error and not OR.config_from_args(OR.parser().parse_args(["--intake", "x", "--no-y-error"])).y_error



@pytest.mark.parametrize("y_error", [True, False])
def test_the_engine_quick_check_samples_along_x_and_the_map_keeps_y(monkeypatch, tmp_path, y_error):
    """Issue #97: the gate samples along X whatever ``y_error`` is; the map still has the X-Y slice."""
    from brixzle.engine import quick_check as QC
    from brixzle.engine import report as RP
    from brixzle.engine import sim_map as SM

    calls = []
    monkeypatch.setattr(QC, "quick_check", lambda *a, **k: calls.append(("check", k.get("disc", False))) or {"passed": True})
    monkeypatch.setattr(QC, "write_check", lambda res, runs: tmp_path)

    def fake_map(*a, slices=None, **k):
        calls.append(("map", tuple(slices)))
        raise RuntimeError("stop")

    monkeypatch.setattr(SM, "sim_map", fake_map)
    engine = OR.SimEngine(LoopConfig(y_error=y_error, runs=str(tmp_path)))
    engine.quick_check(None, None)
    with pytest.raises(RuntimeError, match="stop"):
        RP.score_family(None, None, statistic="mean", runs=tmp_path, y_error=y_error)
    (check1, check2, (_, slices)) = calls
    assert check1 == check2 == ("check", False)
    assert ("xy" in slices) is y_error



# --- lessons carried between iterations and runs (issue #88) ---


LESSON = {"lesson": "A wider pitch lets the capture window cover the box.", "rules": ["DR2"], "cluster": 0}


def critic_with_lesson(call):
    r = critic_step()
    return RN.AgentResult(structured={**r.structured, "lessons": [LESSON]}, cost_usd=r.cost_usd, turns=1)


def test_a_lesson_is_written_once_the_next_design_is_scored(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.5, 0.7], held=[0.2, 0.2])
    loop, _ = _loop(tmp_path, intake, [design_step("w1"), critic_with_lesson, design_step("w2")], engine)
    state = loop.run()
    (line,) = [json.loads(x) for x in (tmp_path / "lessons.jsonl").read_text().splitlines()]
    assert line["status"] == "supported" and line["lesson"] == LESSON["lesson"]
    (ev,) = line["evidence"]
    assert ev["run"] == "t" and ev["iteration"] == 2 and ev["decision"] == "kept"
    assert ev["map_before"] == state["iterations"][0]["runs"]["map"] and ev["map_after"] == state["iterations"][1]["runs"]["map"]
    assert ev["z_before"] == pytest.approx(0.5) and ev["z_after"] == pytest.approx(0.7)


def test_both_agents_read_the_lessons_file(tmp_path, intake):
    from brixzle.loop import lessons as LS

    path = tmp_path / "lessons.jsonl"
    LS.record(path, "earlier", {"iteration": 1, "decision": "kept", "family": "f", "target": "mean",
                                "z": {"mean": {"Z": 0.5}}, "runs": {"map": "m1"}, "brief": {"lessons": [LESSON]}},
              {"iteration": 2, "decision": "kept", "family": "f", "target": "mean", "z": {"mean": {"Z": 0.6}},
               "runs": {"map": "m2"}})
    engine = FakeEngine(tmp_path / "eng", zs=[0.5])
    loop, runner = _loop(tmp_path, intake, [design_step(), critic_step()], engine, max_iterations=2)
    loop.iterate(1)
    assert all(LESSON["lesson"] in c.system for c in runner.calls) and [c.name for c in runner.calls] == ["design", "critic"]


def test_no_lessons_neither_reads_nor_writes_them(tmp_path, intake):
    from brixzle.loop import lessons as LS

    path = tmp_path / "lessons.jsonl"
    LS.record(path, "earlier", {"iteration": 1, "decision": "kept", "family": "f", "target": "mean",
                                "z": {"mean": {"Z": 0.5}}, "runs": {"map": "m1"}, "brief": {"lessons": [LESSON]}},
              {"iteration": 2, "decision": "kept", "family": "f", "target": "mean", "z": {"mean": {"Z": 0.6}},
               "runs": {"map": "m2"}})
    before = path.read_text()
    engine = FakeEngine(tmp_path / "eng", zs=[0.5, 0.7], held=[0.2, 0.2])
    loop, runner = _loop(tmp_path, intake, [design_step("w1"), critic_with_lesson, design_step("w2")], engine,
                         lessons=False)
    loop.run()
    assert path.read_text() == before and not any(LESSON["lesson"] in c.system for c in runner.calls)
    assert not OR.config_from_args(OR.parser().parse_args(["--intake", "x", "--no-lessons"])).lessons



# --- the structure gate and overhang length (issue #83) ---


def test_the_overhang_is_a_second_objective():
    best = {"z_target": 0.5, "overhang_n": 5, "held_out": {}}
    kept, why = OR.accept(0.5, {}, best, overhang=6)
    assert kept and "overhang" in why
    kept, why = OR.accept(0.6, {}, best, overhang=4)
    assert not kept and "overhang got shorter" in why
    kept, why = OR.accept(0.5, {}, best, overhang=5)
    assert not kept and "did not improve" in why and "did not grow" in why
    assert OR.accept(0.6, {}, best, overhang=5)[0]
    assert not OR.accept(0.4, {}, best, overhang=9)[0]
    # A best from before #83 has no overhang: Z alone decides.
    assert OR.accept(0.6, {}, {"z_target": 0.5, "held_out": {}}, overhang=2)[0]


def test_a_design_that_fails_the_gate_is_sent_back_and_not_scored(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.4])
    engine.gates = [(False, 2), (True, 6)]
    loop, runner = _loop(tmp_path, intake, [design_step("a"), design_step("b"), critic_step()], engine,
                         max_iterations=1)
    state = loop.run()
    it = state["iterations"][0]
    assert [a["decision"] for a in it["attempts"]] == ["failed_structure_gate", "passed_structure_gate"]
    assert [c for c in engine.calls if isinstance(c, tuple) and c[0] == "score"] == [("score", "b", False)]
    prompt = runner.calls[1].prompt
    assert "structure gate" in prompt and '"wall"' in prompt and "b2_0" in prompt
    assert it["overhang_n"] == 6 and it["gate"]["passed"] is True and it["seconds"]["gate"] >= 0
    assert state["best"]["overhang_n"] == 6
    log = [json.loads(line) for line in (loop.run_dir / "design_log.jsonl").read_text().splitlines()]
    assert [x["decision"] for x in log] == ["failed_structure_gate", "kept"]
    assert log[0]["runs"]["gate"] == it["attempts"][0]["gate"]["run"] and log[1]["overhang_n"] == 6
    text = (tmp_path / "RSI_RESULTS" / "t" / "iter-01" / "summary.md").read_text(encoding="utf-8")
    assert "Structure gate" in text and "overhang" in text


def test_three_gate_failures_end_the_iteration_with_no_design(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[])
    engine.gates = [(False, 0)] * 3
    loop, _ = _loop(tmp_path, intake, [design_step("a"), design_step("b"), design_step("c")], engine,
                    max_iterations=1)
    rec = loop.iterate(1)
    assert rec["decision"] == "no_design" and "structure gate" in rec["reason"]
    assert [a["decision"] for a in rec["attempts"]] == ["failed_structure_gate"] * 3
    assert not any(isinstance(c, tuple) and c[0] == "score" for c in engine.calls)


def test_equal_z_and_a_longer_overhang_is_kept_in_the_loop(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.5, 0.5, 0.6], held=[0.2] * 6)
    engine.gates = [(True, 5), (True, 7), (True, 6)]
    loop, _ = _loop(tmp_path, intake, [design_step(), critic_step(), design_step(), critic_step(), design_step(),
                                       critic_step()], engine, max_iterations=3)
    state = loop.run()
    its = state["iterations"]
    assert [i["decision"] for i in its] == ["kept", "kept", "rejected"]
    assert "overhang got shorter" in its[2]["reason"] and state["best"]["overhang_n"] == 7


def test_the_baseline_runs_the_gate_and_sets_the_overhang_to_beat(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.4])
    engine.gates = [(True, 5)]
    loop, _ = _loop(tmp_path, intake, [critic_step()], engine, parent="ab", max_iterations=1)
    rec = loop.iterate(0)
    assert rec["gate"]["passed"] is True and rec["overhang_n"] == 5 and loop.state["best"]["overhang_n"] == 5
    assert engine.calls.index("gate") < next(i for i, c in enumerate(engine.calls) if isinstance(c, tuple))



# --- the ring (issue #89) ---


def test_a_design_that_would_be_kept_but_fails_the_ring_is_rejected(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.5, 0.7, 0.6], held=[0.2] * 6)
    engine.rings = ["complete", "placement_jammed"]
    loop, runner = _loop(tmp_path, intake, [design_step("r1"), critic_step(), design_step("r2"), critic_step(),
                                            design_step("r3")], engine, max_iterations=3)
    loop.state["iterations"].append(loop.iterate(1))
    rec = loop.iterate(2)
    assert rec["decision"] == "failed_ring" and "ring" in rec["reason"] and "placement_jammed" in rec["reason"]
    assert loop.state["best"]["iteration"] == 1 and rec["ring"]["status"] == "placement_jammed"
    assert rec["seconds"]["ring"] >= 0
    log = [json.loads(line)["decision"] for line in (loop.run_dir / "design_log.jsonl").read_text().splitlines()]
    assert log == ["kept", "failed_ring"]
    loop.state["iterations"].append(rec)
    loop.iterate(3)
    prompt = [c for c in runner.calls if c.name == "design"][-1].prompt
    assert "ring" in prompt and "K2_1_0_2" in prompt and "jammed" in prompt


def test_a_design_that_passes_the_ring_is_kept(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.5])
    loop, _ = _loop(tmp_path, intake, [design_step(), critic_step()], engine, max_iterations=1)
    rec = loop.iterate(1)
    assert rec["decision"] == "kept" and rec["ring"]["status"] == "complete" and "ring" in engine.calls
    text = (tmp_path / "RSI_RESULTS" / "t" / "iter-01" / "summary.md").read_text(encoding="utf-8")
    assert "Ring" in text and "16 of 16" in text


def test_a_design_rejected_anyway_gets_no_ring_build(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.5, 0.3], held=[0.2] * 4)
    loop, _ = _loop(tmp_path, intake, [design_step(), critic_step(), design_step(), critic_step()], engine)
    state = loop.run()
    assert [i["decision"] for i in state["iterations"]] == ["kept", "rejected"]
    assert engine.calls.count("ring") == 1 and state["iterations"][1].get("ring") is None


def test_the_baseline_gets_a_ring_build(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.4])
    engine.rings = ["placement_jammed"]
    loop, _ = _loop(tmp_path, intake, [critic_step()], engine, parent="ab", max_iterations=1)
    rec = loop.iterate(0)
    assert rec["decision"] == "baseline" and rec["ring"]["status"] == "placement_jammed"  # recorded, not judged
    assert loop.state["best"]["iteration"] == 0
    loop.state["iterations"].append(rec)
    loop.write_run_summary()
    text = (tmp_path / "RSI_RESULTS" / "t" / "summary.md").read_text(encoding="utf-8")
    assert "| ring (#89) | placement_jammed, 15 of 16 seated |" in text


def test_no_ring_is_started_with_less_than_two_minutes_left(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.5])
    loop, _ = _loop(tmp_path, intake, [design_step(), critic_step()], engine, max_iterations=1, max_minutes=30.0)
    loop._t0 = __import__("time").time() - (30 * 60 - 60)  # one minute left
    rec = loop.iterate(1)
    assert rec["decision"] == "kept" and "ring" not in engine.calls
    assert rec["ring"]["status"] == "not_run" and "two minutes" in rec["ring"]["reason"]
    text = (tmp_path / "RSI_RESULTS" / "t" / "iter-01" / "summary.md").read_text(encoding="utf-8")
    assert "ring was not run" in text


# --- the drop probe (#107) ---


def test_the_design_agent_probes_the_parents_failures_and_the_probes_are_logged_with_their_time(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.4, 0.6], held=[0.5, 0.5])
    loop, runner = _loop(tmp_path, intake, [critic_step(), probing_design_step("probewedge", calls=2)], engine,
                         parent="ab", max_iterations=1)
    state = loop.run()
    it = state["iterations"][1]
    base_map = Path(state["iterations"][0]["runs"]["map"]).name
    assert ("probe_poses", base_map, "mean", 1) in engine.calls  # the parent's map, the target, seed*1000 + i
    assert [c for c in engine.calls if c[0] == "probe"] == [("probe", it["attempts"][0]["module"])] * 2
    att = it["attempts"][0]
    assert [p["call"] for p in att["probes"]] == [1, 2] and att["probe_s"] >= 0
    assert it["seconds"]["probe"] == pytest.approx(att["probe_s"])
    entry = [e for e in loop.log.entries() if e["iteration"] == 1][-1]
    assert entry["probes"] == 2 and entry["probe_s"] >= 0


def test_without_a_parent_the_probe_spreads_over_the_disc_and_the_cap_is_a_setting(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.6], held=[0.5])
    loop, runner = _loop(tmp_path, intake, [probing_design_step("probewedge", calls=3)], engine,
                         max_iterations=1, probe_calls=2)
    state = loop.run()
    assert ("probe_poses", None, "mean", 1) in engine.calls
    att = state["iterations"][0]["attempts"][0]
    assert len(att["probes"]) == 2  # the third call hit the cap
    assert len([c for c in engine.calls if c[0] == "probe"]) == 2


def test_the_probe_can_be_turned_off(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.6], held=[0.5])
    loop, runner = _loop(tmp_path, intake, [design_step("plain")], engine, max_iterations=1, probe_calls=0)
    loop.run()
    assert not [c for c in engine.calls if c[0] in ("probe_poses", "probe")]


# --- the cheap screen (#108) ---


def test_the_screen_maps_only_the_candidates_it_cannot_rule_out_and_logs_the_rest(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[])
    engine.screens = {"m_a": (47, 6, 60, True), "m_b": (46, 7, 60, True), "m_c": (49, 3, 60, True),
                      "m_d": (17, 19, 37, True), "m_e": (40, 4, 60, False)}
    loop, _ = _loop(tmp_path, intake, [], engine, screen_max_maps=2)
    rec = {"iteration": 1, "seconds": {}, "attempts": []}
    cands = [{"id": k, "root": str(tmp_path), "module": f"m_{k}"} for k in "abcde"]
    mapped = loop.screen_round(rec, cands, "mean")
    assert [c["id"] for c in mapped] == ["c", "a"]
    by = {c["id"]: c for c in cands}
    assert by["b"]["decision"] == "screened_out" and "cap" in by["b"]["reason"]
    assert by["d"]["decision"] == "screened_out" and "below the best" in by["d"]["reason"]
    assert by["e"]["decision"] == "failed_quick_check"
    assert ("screen", tuple("abcde"), "mean") in engine.calls
    assert rec["seconds"]["screen"] >= 0 and rec["screen"]["c"]["Z"] == pytest.approx(46 / 60)
    logged = {e["family"]: e for e in loop.log.entries()}
    assert logged["m_b"]["decision"] == "screened_out" and logged["m_b"]["screen"]["Z"] == pytest.approx(39 / 60)
    assert logged["m_e"]["decision"] == "failed_quick_check"


def test_the_round_maps_no_more_designs_than_the_clock_allows_and_says_so(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[])
    loop, _ = _loop(tmp_path, intake, [], engine, screen_max_maps=3, max_minutes=10.0, tune_seconds=60.0)
    loop._t0 = __import__("time").time() - 5 * 60  # 5 of 10 minutes gone; 1 min held for the end tune
    rec = {"iteration": 1, "seconds": {}, "attempts": []}
    cands = [{"id": k, "root": str(tmp_path), "module": f"m_{k}"} for k in "abc"]
    mapped = loop.screen_round(rec, cands, "mean", score_s=100.0)
    assert len(mapped) == 2  # 4 minutes left: two full scores of 100 s fit, a third does not
    assert any("wall-clock" in w for w in rec["warnings"])


# --- explore rounds (#106) ---

AB_SUB = (
    "from dataclasses import replace\n"
    "from brixzle.families.ab.family import FAMILY as AB\n"
    "from brixzle.families.ab.params import BrickParams\n"
    "from dataclasses import dataclass\n\n"
    "@dataclass(frozen=True)\n"
    "class P(BrickParams):\n"
    "    tooth_L: float = 7.0\n\n"
    "FAMILY = replace(AB, name='{name}', params_type=P)\n")


def ab_copy_step(name="abcopy"):
    def step(call):
        pkg = Path(call.write_root)
        (pkg / "family.py").write_text(AB_SUB.format(name=name), encoding="utf-8")
        (pkg / "design_note.json").write_text(json.dumps({"summary": "A/B again", "changes": "tooth_L",
                                                          "geometry": "Prismatic"}), encoding="utf-8")
        return RN.AgentResult(text="done", cost_usd=0.10, turns=5)
    return step


def test_the_first_iteration_explores_n_concepts_with_n_principles_and_keeps_the_best(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.3, 0.7, 0.5], held=[0.5, 0.5, 0.5])
    loop, runner = _loop(tmp_path, intake, [design_step("cx"), design_step("cx"), design_step("cx")], engine,
                         max_iterations=1, concepts=3)
    state = loop.run()
    it = state["iterations"][0]
    assert it["mode"] == "explore" and "first" in it["explore_reason"]
    design_calls = [c for c in runner.calls if c.name == "design"]
    assert len(design_calls) == 3
    principles = [c["concept"] for c in it["concepts"]]
    assert len(set(principles)) == 3 and all(p.startswith("CP") for p in principles)
    assert all(p in call.prompt for p, call in zip(sorted(principles), sorted(
        design_calls, key=lambda c: next(x for x in principles if x in c.prompt))))
    scored = [c for c in it["concepts"] if c.get("z")]
    assert len(scored) == 3
    best = max(scored, key=lambda c: c["z"]["mean"]["Z"])
    assert it["decision"] == "kept" and it["module"] == best["module"] and best["z"]["mean"]["Z"] == pytest.approx(0.7)
    others = [c for c in scored if c is not best]
    assert all(c["decision"] == "rejected" and "another concept" in c["reason"] for c in others)
    assert state["principles_tried"] == sorted(principles, key=lambda x: int(x[2:]))
    logged = [e for e in loop.log.entries() if e["iteration"] == 1]
    assert len(logged) == 3 and {e.get("concept") for e in logged} == set(principles)
    summary = (Path(loop.results) / "summary.md").read_text(encoding="utf-8")
    it_summary = (Path(loop.results) / "iter-01" / "summary.md").read_text(encoding="utf-8")
    for c in it["concepts"]:
        assert c["module"] in summary and c["concept"] in summary and c["module"] in it_summary


def test_an_explore_design_that_is_a_near_copy_of_a_b_is_refused(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.6], held=[0.5])
    loop, runner = _loop(tmp_path, intake, [ab_copy_step("abcopy"), design_step("cx")], engine,
                         max_iterations=1, concepts=2)
    state = loop.run()
    it = state["iterations"][0]
    copies = [c for c in it["concepts"] if c["decision"] == "near_copy"]
    assert len(copies) == 1 and "near copy" in copies[0]["reason"]
    assert any(e["decision"] == "near_copy" for e in loop.log.entries())


def test_one_concept_keeps_the_old_one_design_iteration(tmp_path, intake):
    engine = FakeEngine(tmp_path / "eng", zs=[0.6], held=[0.5])
    loop, _ = _loop(tmp_path, intake, [design_step("solo")], engine, max_iterations=1, concepts=1)
    it = loop.run()["iterations"][0]
    assert it.get("mode", "reshape") == "reshape" and "concepts" not in it


def _it(i, z, best_before, mode="reshape", decision="rejected", brief=None):
    n = 356
    s = round((z + 1) / 2 * n)
    return {"iteration": i, "mode": mode, "decision": decision, "brief": brief, "target": "mean",
            "z": {"mean": {"Z": z, "S": s, "J": n - s, "N": n}},
            "z_change": {"best_before": best_before, "delta": z - best_before,
                         "band": 0.033}}


def test_explore_again_on_a_stall_or_when_the_critic_asks_for_a_new_family():
    cfg = LoopConfig(concepts=3, stall_iterations=2)
    first = {"iterations": [{"iteration": 0, "decision": "baseline"}]}
    assert "first" in OR.explore_reason(first, cfg)
    explored = {"iterations": [{"iteration": 0, "decision": "baseline"}, _it(1, 0.6, 0.5, mode="explore", decision="kept")]}
    assert OR.explore_reason(explored, cfg) is None
    stalled = {"iterations": explored["iterations"] + [_it(2, 0.61, 0.6), _it(3, 0.58, 0.6)]}
    assert "noise band" in OR.explore_reason(stalled, cfg)
    moving = {"iterations": explored["iterations"] + [_it(2, 0.61, 0.6), _it(3, 0.7, 0.6, decision="kept")]}
    assert OR.explore_reason(moving, cfg) is None
    brief = {"decision": "redesign", "clusters": [{"scope": "new_family"}]}
    asked = {"iterations": explored["iterations"] + [_it(2, 0.61, 0.6, brief=brief)]}
    assert "new family" in OR.explore_reason(asked, cfg)
    assert OR.explore_reason(first, LoopConfig(concepts=1)) is None

