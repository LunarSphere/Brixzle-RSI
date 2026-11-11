"""Scorer and diagnoser (issue #13): report.json from the sim box search and the predictor.

Sim results are simulated, uncalibrated contact; predictor results are predicted, quasi-static 2D.
Most tests use synthetic results (no sim); one small run scores A/B V0 end to end.
"""

import json
import math

import pytest

from brixzle.engine import box_search as BS
from brixzle.engine import boxes as BX
from brixzle.engine import report as R
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.ab.params import V0


# --- synthetic results ---


def _drop(scale, point, outcome, **extra):
    return {"scale": scale, "point": point, "unit": [0.0, 0.0, 0.0], "dx": 0.0, "outcome": outcome,
            "contacts": [{"piece": 0, "other": "base", "start_s": 0.0}], **extra}


def _course(scale, evals, drops):
    return {"scale": scale, "passed": scale >= 1.0, "evaluations": [{"scale": s, "ok": ok} for s, ok in evals],
            "failure_outcomes": sorted({d["outcome"] for d in drops if d["outcome"] not in ("seated", "picked")}),
            "drops": drops, "drop_count": len(drops), "simulated_count": len(drops)}


def _sim(task, stat, scale, drops_by_course, evals=None, release_ok=True):
    box = BX.default_boxes()[task][stat]
    evals = evals or [(1.0, scale >= 1.0)]
    courses = {c: _course(scale, evals, d) for c, d in drops_by_course.items()}
    out = {"label": BS.LABEL, "task": task, "family": "ab", "family_version": "x", "params": {},
           "box": box.to_dict(), "seed": 0, "config": {}, "scale": scale, "passed": scale >= 1.0,
           "limiting_course": next(iter(courses)), "courses": courses, "seconds": 1.0}
    if task == "pickup":
        out["passed"] = scale >= 1.0 and release_ok
    return out


def _pred(task, scale):
    return {"label": "predicted, quasi-static 2D", "scale": scale, "passed": scale >= 1.0, "seconds": 0.1}


RELEASE_OK = {"passed": True, "drift_mm": 0.1, "withdrawn": True, "peak_N": 0.0, "label": BS.LABEL}


def _report(place_scale=1.2, pick_scale=1.5, release=RELEASE_OK, checks=None, stats=("mean",), tasks=BX.TASKS):
    sims, preds = {}, {}
    for stat in stats:
        sims[("placement", stat)] = _sim("placement", stat, place_scale, {
            "0": [_drop(1.0, 0, "seated"), _drop(1.5, 1, "jammed"), _drop(1.25, 1, "wrong_valley")],
            "1": [_drop(1.0, 0, "seated"), _drop(1.5, 2, "fell")]},
            evals=[(1.0, True), (2.0, False), (1.5, False), (1.25, False)])
        preds[("placement", stat)] = _pred("placement", place_scale)
        if "pickup" in tasks:
            sims[("pickup", stat)] = _sim("pickup", stat, pick_scale, {
                "pick": [_drop(1.0, 0, "picked"), _drop(2.0, 3, "missed"), _drop(2.0, 4, "off_centre")]},
                evals=[(1.0, True), (2.0, False)])
            preds[("pickup", stat)] = _pred("pickup", 1.0)
    return R.build_report(AB, V0, sims, preds, release=release if "pickup" in tasks else None,
                          checks=checks or [], timing={"total": 5.0}, files={}, seed=0, tasks=tasks)


# --- categories ---


@pytest.mark.parametrize("task, outcome, category", [
    ("placement", "jammed", "jam"), ("placement", "fell", "fall"), ("placement", "wrong_valley", "wrong_valley"),
    ("pickup", "jammed", "jam"), ("pickup", "missed", "pickup_miss"), ("pickup", "off_centre", "off_centre"),
    ("pickup", "forceful", "forceful"),
])
def test_every_failure_outcome_has_a_category(task, outcome, category):
    assert R.category(task, outcome) == category


def test_an_unknown_outcome_is_rejected():
    with pytest.raises(ValueError):
        R.category("placement", "exploded")
    with pytest.raises(ValueError):
        R.category("placement", "seated")  # a success is not a failure


# --- limiting failures ---


def test_limiting_failures_are_the_failures_at_the_first_failing_scale_above_the_score():
    course = _course(1.0, [(1.0, True), (2.0, False), (1.5, False), (1.25, False)],
                     [_drop(1.0, 0, "seated"), _drop(2.0, 1, "jammed"), _drop(1.5, 2, "jammed"),
                      _drop(1.25, 3, "wrong_valley"), _drop(1.25, 4, "seated")])
    rows = R.limiting_failures(course, "seated")
    assert [(d["scale"], d["point"], d["outcome"]) for d in rows] == [(1.25, 3, "wrong_valley")]
    assert rows[0]["contacts"]  # the trace is kept


def test_a_design_failing_at_the_aim_is_limited_at_scale_zero():
    course = _course(0.0, [(1.0, False), (0.0, False)], [_drop(1.0, 0, "jammed"), _drop(0.0, 0, "fell")])
    assert [(d["scale"], d["outcome"]) for d in R.limiting_failures(course, "seated")] == [(0.0, "fell")]


def test_a_design_capped_at_s_max_has_no_limiting_failure():
    course = _course(2.0, [(1.0, True), (2.0, True)], [_drop(1.0, 0, "seated"), _drop(2.0, 0, "seated")])
    assert R.limiting_failures(course, "seated") == []


# --- cross-check ---


@pytest.mark.parametrize("sim, pred, kind", [
    ("seated", "seated", "agree_ok"), ("jammed", "wrong_valley", "agree_failed"),
    ("seated", "jammed", "conservative"), ("fell", "seated", "optimistic"),
])
def test_cross_check_kinds(sim, pred, kind):
    assert R.agreement("placement", sim, pred) == kind


def test_pickup_kinds_use_picked_as_success():
    assert R.agreement("pickup", "picked", "off_centre") == "conservative"
    assert R.agreement("pickup", "missed", "picked") == "optimistic"


def test_only_disagreements_reach_the_report_and_classes_are_untrusted():
    checks = [
        {"task": "placement", "statistic": "mean", "course": "0", "scale": 1.0, "point": 0, "unit": [0, 0, 0],
         "sim_outcome": "seated", "pred_outcome": "seated", "kind": "agree_ok"},
        {"task": "placement", "statistic": "mean", "course": "0", "scale": 1.5, "point": 1, "unit": [1, 1, 1],
         "sim_outcome": "jammed", "pred_outcome": "seated", "kind": "optimistic"},
        {"task": "placement", "statistic": "mean", "course": "1", "scale": 1.5, "point": 2, "unit": [1, 1, 1],
         "sim_outcome": "fell", "pred_outcome": "jammed", "kind": "agree_failed"},
    ]
    rep = _report(checks=checks)
    assert [d["kind"] for d in rep["disagreements"]] == ["optimistic"]
    counts = rep["disagreement_counts"]["mean"]["placement"]
    assert counts == {"agree_ok": 1, "agree_failed": 1, "optimistic": 1, "conservative": 0}
    assert rep["class_mismatches"]["mean"]["placement"] == 1  # fell vs jammed
    assert rep["class_trusted"] is False


# --- scores ---


def test_the_target_score_is_the_smaller_of_placement_and_pickup():
    rep = _report(place_scale=1.2, pick_scale=1.5)
    s = rep["score"]["mean"]
    assert (s["placement"], s["pickup"], s["overall"]) == (1.2, 1.5, 1.2)
    assert rep["targets"]["mean"]["placement"]["scale_gap"] == pytest.approx(0.0)
    assert rep["targets"]["mean"]["pickup"]["scale_gap"] == pytest.approx(1.0 - 1.5)
    assert rep["covers_all_targets"] is True


def test_a_failed_release_fails_pickup_even_above_scale_one():
    bad = {**RELEASE_OK, "passed": False, "drift_mm": 3.0}
    rep = _report(pick_scale=1.5, release=bad)
    assert rep["targets"]["mean"]["pickup"]["sim"]["passed"] is False
    assert rep["score"]["mean"]["pickup"] == 0.0  # no box in which every trial succeeds
    assert rep["covers_all_targets"] is False
    assert rep["failure_counts"]["mean"]["pickup"]["release"] == 1
    assert any(f["category"] == "release" for f in rep["failures"])


def test_failures_are_counted_by_category_and_limiting_ones_carry_traces():
    rep = _report()
    assert rep["failure_counts"]["mean"]["placement"] == {"jam": 1, "fall": 1, "wrong_valley": 1}
    assert rep["failure_counts"]["mean"]["pickup"] == {"pickup_miss": 1, "off_centre": 1}
    lim = {(f["task"], f["course"], f["outcome"]) for f in rep["failures"]}
    assert lim == {("placement", "0", "wrong_valley"), ("placement", "1", "fell"),
                   ("pickup", "pick", "missed"), ("pickup", "pick", "off_centre")}
    assert all(f["contacts"] for f in rep["failures"])


def test_a_target_below_scale_one_does_not_cover():
    assert _report(place_scale=0.6)["covers_all_targets"] is False


# --- schema ---


def test_report_passes_its_schema_and_is_json_safe():
    rep = _report(stats=BX.STATISTICS)
    R.validate_report(rep)
    json.dumps(rep, allow_nan=False)
    assert rep["schema_version"] == R.SCHEMA_VERSION
    assert rep["labels"] == {"sim": "simulated, uncalibrated contact", "pred": "predicted, quasi-static 2D"}
    assert set(rep["targets"]) == set(BX.STATISTICS)


@pytest.mark.parametrize("break_it", [
    lambda r: r.pop("score"),
    lambda r: r["score"]["mean"].__setitem__("overall", math.nan),
    lambda r: r["failures"][0].__setitem__("category", "explosion"),
    lambda r: r["labels"].__setitem__("sim", "hardware"),
    lambda r: r["targets"]["mean"].pop("pickup"),
])
def test_schema_rejects_a_broken_report(break_it):
    rep = _report()
    break_it(rep)
    with pytest.raises(ValueError):
        R.validate_report(rep)


def test_over_budget_is_flagged():
    assert _report()["timing"]["over_budget"] is False
    rep = R.build_report(AB, V0, *_report_inputs(), release=RELEASE_OK, checks=[],
                         timing={"total": R.BUDGET_S + 1}, files={}, seed=0)
    assert rep["timing"]["over_budget"] is True


def _report_inputs():
    sims = {("placement", "mean"): _sim("placement", "mean", 1.0, {"0": [_drop(1.0, 0, "seated")]}),
            ("pickup", "mean"): _sim("pickup", "mean", 1.0, {"pick": [_drop(1.0, 0, "picked")]})}
    preds = {("placement", "mean"): _pred("placement", 1.0), ("pickup", "mean"): _pred("pickup", 1.0)}
    return sims, preds


# --- tasks: placement only by default (issue #46) ---


def test_the_loop_scores_placement_only_by_default():
    assert R.DEFAULT_TASKS == ("placement",)
    assert R.parser().parse_args([]).tasks == ["placement"]
    assert R.parser().parse_args(["--tasks", "placement", "pickup"]).tasks == ["placement", "pickup"]


def test_a_placement_only_report_passes_the_schema():
    rep = _report(place_scale=1.2, stats=BX.STATISTICS, tasks=("placement",))
    R.validate_report(rep)
    json.dumps(rep, allow_nan=False)
    assert rep["tasks"] == ["placement"]
    for stat in BX.STATISTICS:
        assert rep["score"][stat] == {"placement": 1.2, "overall": 1.2}
        assert set(rep["targets"][stat]) == {"placement"}
        assert set(rep["failure_counts"][stat]) == {"placement"}
        assert set(rep["disagreement_counts"][stat]) == {"placement"}
    assert rep["covers_all_targets"] is True
    assert all(f["task"] == "placement" for f in rep["failures"])
    assert not any("pickup predictor" in flag for flag in rep["flags"])
    assert R.NOT_SCORED in rep["flags"]  # the report says pickup was left out, and why


def test_placement_only_coverage_ignores_pickup():
    assert _report(place_scale=0.6, tasks=("placement",))["covers_all_targets"] is False
    # A pickup result that would fail the target no longer matters when pickup is not scored.
    assert _report(place_scale=1.2, pick_scale=0.0, tasks=("placement",))["covers_all_targets"] is True


def test_the_full_report_is_unchanged_apart_from_naming_its_tasks():
    rep = _report(stats=BX.STATISTICS)
    R.validate_report(rep)
    assert rep["tasks"] == ["placement", "pickup"]
    for stat in BX.STATISTICS:
        assert set(rep["score"][stat]) == {"placement", "pickup", "overall"}
        assert set(rep["targets"][stat]) == {"placement", "pickup"}
        assert "release" in rep["targets"][stat]["pickup"]
    assert any("pickup predictor" in flag for flag in rep["flags"])


def test_tasks_are_inferred_from_the_results_when_not_given():
    sims, preds = _report_inputs()
    rep = R.build_report(AB, V0, sims, preds, release=RELEASE_OK, checks=[], timing={"total": 1.0}, files={},
                         seed=0)
    assert rep["tasks"] == ["placement", "pickup"]
    place = {k: v for k, v in sims.items() if k[0] == "placement"}
    rep = R.build_report(AB, V0, place, preds, release=None, checks=[], timing={"total": 1.0}, files={}, seed=0)
    assert rep["tasks"] == ["placement"]


@pytest.mark.parametrize("tasks", [(), ("pickup", "pickup"), ("placement", "painting"), "placement"])
def test_bad_tasks_are_rejected(tasks, tmp_path):
    with pytest.raises(ValueError):
        R.score_family(AB, V0, statistics=("mean",), tasks=tasks, runs=tmp_path)
    assert list(tmp_path.iterdir()) == []  # rejected before a run directory is made


@pytest.mark.parametrize("break_it", [
    lambda r: r.pop("tasks"),
    lambda r: r.__setitem__("tasks", []),
    lambda r: r.__setitem__("tasks", ["placement", "pickup"]),  # names a task the report does not hold
    lambda r: r["score"]["mean"].__setitem__("overall", 0.5),    # not the smallest of its task scores
])
def test_schema_rejects_a_report_whose_tasks_do_not_match(break_it):
    rep = _report(tasks=("placement",))
    break_it(rep)
    with pytest.raises(ValueError):
        R.validate_report(rep)


def test_the_summary_lists_only_the_scored_tasks():
    text = R.summary(_report(tasks=("placement",)))
    assert "placement" in text and "pickup" not in text
    assert "pickup" in R.summary(_report())


# --- end to end, A/B V0 (coarse on purpose, as tests/test_box_search.py) ---


def test_box_search_can_skip_the_release_check():
    box = BX.default_boxes()["pickup"]["mean"]
    out = BS.box_search(AB, V0, box, seed=0, tol=1.0, n_interior=0, s_max=1.0, workers=1, release=False)
    assert "release" not in out


@pytest.fixture(scope="module", params=[None, BX.TASKS], ids=["default", "placement+pickup"])
def scored(request, tmp_path_factory):
    runs = tmp_path_factory.mktemp("runs")
    kw = {} if request.param is None else {"tasks": request.param}
    out = R.score_family(AB, V0, statistics=("mean",), seed=0, tol=0.5, n_interior=0, workers=1, runs=runs, **kw)
    return request.param or R.DEFAULT_TASKS, out


def test_score_family_writes_a_valid_report_with_its_full_results(scored):
    tasks, out = scored
    rep = json.loads((out / "report.json").read_text(encoding="utf-8"))
    R.validate_report(rep)
    assert rep["family"] == "ab" and rep["seed"] == 0 and set(rep["targets"]) == {"mean"}
    assert rep["tasks"] == list(tasks) and set(rep["targets"]["mean"]) == set(tasks)
    for rel in rep["files"].values():
        assert (out / rel).is_file()
    assert all(f["contacts"] for f in rep["failures"] if f["category"] != "release")
    assert {"predict", "cross_check", "total"} <= set(rep["timing"])


def test_placement_only_scoring_runs_no_pickup_stage(scored):
    """Issue #46: with pickup deferred, its predictor, sim, release check and cross-check are skipped."""
    tasks, out = scored
    rep = json.loads((out / "report.json").read_text(encoding="utf-8"))
    pickup_stages = {k for k in rep["timing"] if "pickup" in k or "release" in k}
    pickup_files = {p.name for p in out.iterdir() if "pickup" in p.name or "release" in p.name}
    if "pickup" in tasks:
        assert pickup_stages == {"sim_release", "sim_pickup_mean"}
        assert pickup_files == {"release.json", "sim-pickup-mean.json", "pred-pickup-mean.json"}
        assert any(r["task"] == "pickup" for r in json.loads((out / "cross-check.json").read_text()))
    else:
        assert pickup_stages == set() and pickup_files == set()
        assert all(r["task"] == "placement" for r in json.loads((out / "cross-check.json").read_text()))


def test_run_directories_are_never_reused(tmp_path):
    a, b = R.new_run_dir(tmp_path, "ab"), R.new_run_dir(tmp_path, "ab")
    assert a != b and a.is_dir() and b.is_dir()
