"""End tune (issues #14, #61): a short Py-BOBYQA run on Z, once, on the final family.

Most tests inject the score and the rule check, so they need no sim and no CAD. One test runs the
default sim score once at coarse settings. Sim results are simulated, uncalibrated contact.
"""

from dataclasses import replace
import json
import pickle

import pytest

from brixzle.engine import boxes as BX
from brixzle.engine import tune as TU
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.ab.params import BOUNDS, V0

FREE = ("U", "theta")
FIXED = tuple(k for k in BOUNDS if k not in FREE)
SETTINGS = TU.TuneSettings(fixed=FIXED, maxfun=24, parallel=1)


def bowl(p):
    """A smooth score with its best point (1.0) at U = 28, theta = 44, inside the bounds."""
    return {"score": 1.0 - ((p.U - 28.0) / 14.0) ** 2 - ((p.theta - 44.0) / 35.0) ** 2}


def no_rules(p):
    return []


class Clock:
    """A fake clock that a score function advances, so the wall-clock cap needs no waiting."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


# --- the parameter vector ---


def test_the_vector_is_the_bounded_parameters_scaled_to_the_unit_interval():
    names = TU.free_names(AB, fixed=())
    assert names == tuple(BOUNDS)
    x = TU.to_unit(AB, V0, names)
    assert len(x) == len(BOUNDS) and all(0.0 <= v <= 1.0 for v in x)
    assert x[names.index("U")] == pytest.approx((25.0 - 18.0) / (32.0 - 18.0))
    assert TU.from_unit(AB, V0, names, x) == V0


def test_fixed_parameters_never_move():
    names = TU.free_names(AB, fixed=FIXED)
    assert names == FREE
    p = TU.from_unit(AB, V0, names, [1.0, 0.0])
    assert (p.U, p.theta) == (32.0, 30.0)
    assert replace(p, U=V0.U, theta=V0.theta) == V0


@pytest.mark.parametrize("settings", [
    TU.TuneSettings(fixed=("nope",)),
    TU.TuneSettings(fixed=tuple(BOUNDS)),       # nothing left to tune
    TU.TuneSettings(maxfun=0),
    TU.TuneSettings(max_seconds=0.0),
    TU.TuneSettings(max_seconds=float("nan")),
    TU.TuneSettings(rhobeg=0.0),
    TU.TuneSettings(rhobeg=0.1, rhoend=0.2),
    TU.TuneSettings(statistic="median"),
    TU.TuneSettings(parallel=0),
    TU.TuneSettings(patience=0),
])
def test_bad_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        TU.tune(AB, V0, settings, score_fn=bowl, check_fn=no_rules)


# --- the search ---


def test_the_tuned_design_beats_the_untuned_one_on_a_smooth_score():
    res = TU.tune(AB, V0, SETTINGS, score_fn=bowl, check_fn=no_rules)
    assert res["baseline"]["score"] == pytest.approx(bowl(V0)["score"])
    assert res["tuned"]["score"] > res["baseline"]["score"] + 0.03
    assert res["improved"] and res["accepted"]
    assert res["params"] == res["tuned"]["params"]
    tuned = AB.params_type.from_dict(res["params"])
    assert all(lo <= getattr(tuned, k) <= hi for k, (lo, hi) in BOUNDS.items())
    assert all(getattr(tuned, k) == getattr(V0, k) for k in FIXED)
    assert res["solver"]["name"] == "Py-BOBYQA" and res["solver"]["npt"] == len(FREE) + 1


def test_the_tune_is_repeatable():
    a = TU.tune(AB, V0, SETTINGS, score_fn=bowl, check_fn=no_rules)
    b = TU.tune(AB, V0, SETTINGS, score_fn=bowl, check_fn=no_rules)
    assert a["params"] == b["params"]
    assert [e["x"] for e in a["evaluations"]] == [e["x"] for e in b["evaluations"]]


def test_maxfun_caps_the_scored_candidates_and_a_point_is_scored_once():
    seen = []

    def score(p):
        seen.append((p.U, p.theta))
        return bowl(p)

    res = TU.tune(AB, V0, replace(SETTINGS, maxfun=7), score_fn=score, check_fn=no_rules)
    assert len(seen) == 7 and len(set(seen)) == 7  # the baseline counts, and no point is scored twice
    assert res["sim_evaluations"] == 7 and res["stopped"] == "maxfun"
    assert res["evaluations"][0]["x"] == TU.to_unit(AB, V0, FREE)  # the untuned design comes first


def test_the_wall_clock_cap_stops_the_tune():
    clock = Clock()

    def score(p):
        clock.t += 10.0
        return bowl(p)

    res = TU.tune(AB, V0, replace(SETTINGS, max_seconds=35.0), score_fn=score, check_fn=no_rules, clock=clock)
    # After 3 scores 30 s have gone, and a 4th at about 10 s would pass 35 s, so it is not started.
    assert res["sim_evaluations"] == 3 and res["stopped"] == "max_seconds"
    assert res["timing"]["search"] == pytest.approx(30.0)
    assert res["tuned"]["score"] >= res["baseline"]["score"]


def test_unbuildable_and_rule_breaking_candidates_are_penalised_and_never_simulated():
    scored = []

    def score(p):
        scored.append(p)
        return bowl(p)

    def check(p):
        if p.U > 26.0:
            raise ValueError("cannot be built")
        return ["A:R9.mass_max"] if p.theta < 49.0 else []

    res = TU.tune(AB, V0, SETTINGS, score_fn=score, check_fn=check)
    assert all(p.U <= 26.0 and p.theta >= 49.0 for p in scored)
    by_status = {s: [e for e in res["evaluations"] if e["status"] == s] for s in ("scored", "rule", "infeasible")}
    assert by_status["rule"] and by_status["infeasible"]  # the bowl pulls the search across both limits
    assert all(e["objective"] == TU.PENALTY_INFEASIBLE and "cannot be built" in e["reason"]
               for e in by_status["infeasible"])
    assert all(e["objective"] == TU.PENALTY_RULE and "R9.mass_max" in e["reason"] for e in by_status["rule"])
    assert all(e["score"] is None for e in by_status["rule"] + by_status["infeasible"])
    assert TU.PENALTY_INFEASIBLE > TU.PENALTY_RULE > TU.PENALTY_OTHER > 1.0  # worse than any Z, which is >= -1
    tuned = AB.params_type.from_dict(res["params"])
    assert tuned.U <= 26.0 and tuned.theta >= 49.0
    assert res["sim_evaluations"] == len(scored) == len(by_status["scored"])


def test_a_baseline_that_breaks_a_rule_is_rejected():
    with pytest.raises(ValueError, match="R9.mass_max"):
        TU.tune(AB, V0, SETTINGS, score_fn=bowl, check_fn=lambda p: ["A:R9.mass_max"])


def test_a_baseline_already_at_the_best_point_is_kept():
    best = replace(V0, U=28.0, theta=44.0)
    res = TU.tune(AB, best, SETTINGS, score_fn=bowl, check_fn=no_rules)
    assert not res["improved"] and not res["accepted"]
    assert res["params"] == best.to_dict() == res["baseline"]["params"]


def test_a_score_must_be_finite():
    with pytest.raises(ValueError, match="finite"):
        TU.tune(AB, V0, SETTINGS, score_fn=lambda p: {"score": float("nan")}, check_fn=no_rules)


def test_the_time_of_the_rule_check_counts_towards_the_cap():
    clock = Clock()

    def check(p):
        clock.t += 5.0  # building the solids for the rules
        return []

    def score(p):
        clock.t += 10.0
        return bowl(p)

    res = TU.tune(AB, V0, replace(SETTINGS, max_seconds=40.0), score_fn=score, check_fn=check, clock=clock)
    # Each candidate takes 15 s. After two, 30 s have gone, and a third would pass 40 s.
    assert res["sim_evaluations"] == 2 and res["stopped"] == "max_seconds"
    assert [e["seconds"] for e in res["evaluations"]] == [15.0, 15.0]


def test_the_search_stops_when_candidates_stop_improving():
    """From the best point nothing beats the start: after the first probes, ``patience`` scored
    candidates in a row with no gain end the search."""
    best = replace(V0, U=28.0, theta=44.0)
    res = TU.tune(AB, best, replace(SETTINGS, patience=2), score_fn=bowl, check_fn=no_rules)
    assert res["stopped"] == "patience" and res["sim_evaluations"] == 1 + len(FREE) + 2
    assert not res["improved"]
    longer = TU.tune(AB, best, replace(SETTINGS, patience=20), score_fn=bowl, check_fn=no_rules)
    assert longer["sim_evaluations"] > res["sim_evaluations"] and longer["stopped"] != "patience"


def test_an_improving_search_is_not_stopped_by_patience():
    res = TU.tune(AB, V0, replace(SETTINGS, patience=3), score_fn=bowl, check_fn=no_rules)
    assert res["tuned"]["score"] > res["baseline"]["score"] + 0.03


# --- the first probes, scored together ---


def test_the_first_probes_step_up_each_parameter_or_down_at_its_upper_bound():
    assert TU.initial_probes([0.5, 0.95], 0.1) == [[0.6, 0.95], [0.5, pytest.approx(0.85)]]


def test_the_first_probes_are_scored_together_and_the_search_reuses_them():
    batches = []

    def map_fn(fn, candidates):
        batches.append(list(candidates))
        return [fn(q) for q in batches[-1]]

    seen = []

    def score(p):
        seen.append((p.U, p.theta))
        return bowl(p)

    together = TU.tune(AB, V0, replace(SETTINGS, parallel=4), score_fn=score, check_fn=no_rules, map_fn=map_fn)
    x0 = TU.to_unit(AB, V0, FREE)
    assert len(batches) == 1 and len(batches[0]) == len(FREE)
    assert [e["x"] for e in together["evaluations"][:3]] == [
        x0, [x0[0] + 0.1, x0[1]], [x0[0], pytest.approx(x0[1] + 0.1)]]
    assert len(set(seen)) == len(seen)  # the search asked for the same probes and did not score them again
    one_by_one = TU.tune(AB, V0, SETTINGS, score_fn=bowl, check_fn=no_rules)
    assert together["params"] == one_by_one["params"]
    assert [e["x"] for e in together["evaluations"]] == [e["x"] for e in one_by_one["evaluations"]]


def test_the_probes_scored_together_stay_within_maxfun():
    batches = []

    def map_fn(fn, candidates):
        batches.append(list(candidates))
        return [fn(q) for q in batches[-1]]

    res = TU.tune(AB, V0, replace(SETTINGS, parallel=4, maxfun=2), score_fn=bowl, check_fn=no_rules, map_fn=map_fn)
    assert [len(b) for b in batches] == [1] and res["sim_evaluations"] == 2 and res["stopped"] == "maxfun"


# --- held-out check ---


def test_a_tuned_design_that_is_worse_held_out_is_not_accepted():
    def held_out(p):  # a second look that dislikes anything but the baseline
        return {"score": 0.9 if p == V0 else 0.2}

    res = TU.tune(AB, V0, SETTINGS, score_fn=bowl, check_fn=no_rules, held_out_fns={"seed 1": held_out})
    assert res["improved"] and not res["accepted"]
    assert res["held_out"]["seed 1"] == {"baseline": 0.9, "tuned": 0.2, "not_worse": False}
    assert res["params"] == V0.to_dict()  # the untuned design is kept


def test_a_tuned_design_that_holds_up_held_out_is_accepted():
    res = TU.tune(AB, V0, SETTINGS, score_fn=bowl, check_fn=no_rules, held_out_fns={"seed 1": bowl})
    assert res["held_out"]["seed 1"]["not_worse"] and res["accepted"]
    assert res["params"] == res["tuned"]["params"]
    assert res["sim_evaluations"] <= SETTINGS.maxfun  # held-out scores are counted apart
    assert res["held_out_evaluations"] == 2


def test_held_out_scores_are_run_together_when_there_is_a_map():
    batches = []

    def map_fn(fn, items):
        batches.append(list(items))
        return [fn(i) for i in batches[-1]]

    res = TU.tune(AB, V0, replace(SETTINGS, parallel=4), score_fn=bowl, check_fn=no_rules, map_fn=map_fn,
                  held_out_fns={"a": bowl, "b": lambda p: {"score": 0.9 if p == V0 else 0.95}})
    assert [len(b) for b in batches] == [len(FREE), 4]  # the first probes, then 2 designs x 2 held-out scores
    assert res["held_out"]["b"] == {"baseline": 0.9, "tuned": 0.95, "not_worse": True}
    assert res["held_out"]["a"]["tuned"] == res["tuned"]["score"] and res["accepted"]


def test_held_out_is_skipped_when_the_tune_found_nothing_better():
    calls = []
    best = replace(V0, U=28.0, theta=44.0)
    res = TU.tune(AB, best, SETTINGS, score_fn=bowl, check_fn=no_rules,
                  held_out_fns={"seed 1": lambda p: calls.append(p) or bowl(p)})
    assert not calls and res["held_out"] == {} and res["held_out_evaluations"] == 0


# --- output ---


def test_the_result_is_labelled_and_written_to_a_new_run_directory(tmp_path):
    res = TU.tune(AB, V0, SETTINGS, score_fn=bowl, check_fn=no_rules)
    assert res["label"] == "simulated, uncalibrated contact"
    assert res["family"] == "ab" and res["statistic"] == "mean" and res["names"] == list(FREE)
    assert set(res["timing"]) == {"search", "held_out", "total"}
    first, second = TU.write_tune(res, runs=tmp_path), TU.write_tune(res, runs=tmp_path)
    assert first != second and first.name.startswith("tune-") and first.name.endswith("-ab-mean")
    saved = json.loads((first / "tune.json").read_text(encoding="utf-8"))
    assert saved["params"] == res["params"] and saved["evaluations"] == res["evaluations"]
    assert "tuned" in TU.summary(res) and "untuned" in TU.summary(res)


# --- the default score and rule check (sim and CAD) ---


def test_the_default_rule_check_passes_the_baseline(ab_bodies):
    assert TU.rule_violations(AB, V0) == []


def test_the_default_score_is_z_on_the_target_with_every_target_box_for_the_guard():
    """The default score is zscore.ZScore (its sim run is tested in test_zscore.py)."""
    from brixzle.engine import zscore as ZS

    boxes = BX.default_boxes()["placement"]
    score = TU.z_score(AB, boxes["p95"], workers=1)
    assert isinstance(score, ZS.ZScore) and score.aim_box == boxes["p95"]
    assert set(score.boxes) == {"mean", "p95", "max"}
    assert pickle.loads(pickle.dumps(score)) == score  # it has to reach another process
    held = TU.held_out_scores(AB, boxes["mean"], replace(SETTINGS, seed=3))
    assert sorted(held) == ["seed 4, friction 0.2", "seed 4, friction 0.5"]
    assert all(isinstance(h, ZS.HeldOutZ) and h.seed == 4 and h.box == boxes["mean"] for h in held.values())


# --- Z on the other target boxes (issue #61) ---


def bowl_with_targets(p):
    """``bowl`` on the target, and a p95 Z that falls as U rises above 25 (the baseline's U)."""
    return {"score": bowl(p)["score"], "targets": {"mean": bowl(p)["score"], "p95": 0.5 - max(0.0, p.U - 25.0) / 10,
                                                   "max": {"Z": 0.2}}}


def test_a_candidate_that_lowers_z_on_another_target_is_refused_and_never_the_result():
    res = TU.tune(AB, V0, SETTINGS, score_fn=bowl_with_targets, check_fn=no_rules)
    refused = [e for e in res["evaluations"] if e["status"] == "lowers_other"]
    assert refused and all(e["objective"] == TU.PENALTY_OTHER and "p95" in e["reason"] for e in refused)
    assert res["tuned"]["params"]["U"] <= 25.0  # the bowl's best U (28) would lower p95
    assert res["tuned"]["targets"]["p95"] >= res["baseline"]["targets"]["p95"]
    assert res["baseline"]["targets"] == {"mean": bowl(V0)["score"], "p95": 0.5, "max": 0.2}


def test_without_targets_the_guard_has_nothing_to_compare():
    res = TU.tune(AB, V0, SETTINGS, score_fn=bowl, check_fn=no_rules)
    assert not any(e["status"] == "lowers_other" for e in res["evaluations"])
