"""The whole corbel (#210): later corbel bricks in the grid, build failures replayed and reduced,
builds at a fixed 16 mm, and the second-weakest tie-break. Sim results: simulated, uncalibrated contact."""
import pytest

from brixzle.director import archive as AR
from brixzle.engine import objective as OB
from brixzle.engine import radius as RA
from brixzle.engine.sim import trials as T
from brixzle.families.interface import Placement


def _lat():
    from brixzle.families import registry
    fam = registry.get("ab")
    return fam.lattice(fam.defaults)


def test_the_back_brick_layout_lands_across_the_back_and_the_cantilevered_reach():
    assert "back_cantilever" in T.EDGE_CASES
    placed, target, base = T.edge_layout(_lat(), "back_cantilever")
    assert target == Placement(2, 0)
    assert Placement(1, -1) in placed and Placement(1, 1) in placed  # the two bricks it spans
    assert Placement(0, 0) in placed and Placement(0, -2) in placed
    assert base == (-3, 2)


def test_the_grid_scores_the_back_brick_and_the_second_reach():
    assert RA.PLACEMENTS["corbel_back"] == "back_cantilever"
    assert RA.PLACEMENTS["corbel_reach2"] == "plus_cantilever"


@pytest.mark.parametrize("brick, layout", [
    ("b0_-2", 0), ("b0_0", 0),                 # on the base
    ("b1_-1", 1),                              # the course-1 back brick: a running-bond seat
    ("b1_1", "plus_anchored"),                 # the first reach
    ("b2_0", "back_cantilever"), ("b4_2", "back_cantilever"),
    ("b2_2", "plus_cantilever"), ("b5_5", "plus_cantilever3"),  # #222: reaches 3+ replay on overhang 3
])
def test_each_corbel_brick_maps_to_the_single_brick_layout_that_replays_it(brick, layout):
    assert RA.corbel_layout(brick) == layout


def test_a_failed_build_keeps_the_failed_bricks_release_and_outcome():
    sample = {"dx": 3.0, "dy": -2.0, "yaw": 0.0, "roll": 0.0, "pitch": 0.0, "clear": 5.0, "vel": [0.0, 0.0, 0.0]}
    build = {"status": "failed", "seconds": 1.0, "stages": [
        {"brick": "b0_-2", "outcome": "seated", "collapsed": False, "sample": sample, "err_mm": [0, 0, 0]},
        {"brick": "b0_0", "outcome": "seated", "collapsed": False, "sample": sample, "err_mm": [0, 0, 0]},
        {"brick": "b1_-1", "outcome": "jammed", "collapsed": False, "sample": sample, "err_mm": [1.5, 0, 2.0]}]}
    rec = OB.build_record(build)
    assert rec["failed_brick"] == "b1_-1" and rec["failed_kind"] == "back" and rec["failed_outcome"] == "jammed"
    assert rec["failed_sample"]["dx"] == 3.0 and rec["failed_err_mm"] == [1.5, 0, 2.0]


def test_builds_run_at_a_fixed_16_mm_by_default():
    assert RA.BuildSettings().radius_mm == 16.0
    with pytest.raises(ValueError):
        RA.BuildSettings(radius_mm=-1.0)


def _rec(name, green, second, steps=3.0):
    return {"name": name, "z": 0.5, "z_rings": {"16": 0.5, "28": 0.2, "40": 0.1}, "green_mm": green,
            "second_mm": second, "overhang_steps": steps, "stack_courses": 3.0, "violations": []}


def test_a_z_tie_goes_to_the_second_weakest_placement_and_not_to_overhang():
    # #222: with Z tied, the green zones weakest first decide; the noisy builds no longer rank
    arc = AR.BandArchive()
    assert arc.offer(_rec("wellkey", 3.75, 8.75, steps=4.0))["kept"]
    res = arc.offer(_rec("foldrail", 3.75, 16.25, steps=3.0))
    assert res["kept"] and res["replaced"] == ["wellkey"]
    assert not arc.offer(_rec("x", 3.75, 16.0, steps=2.0))["kept"]
    assert not arc.offer(_rec("y", 3.75, 16.0, steps=5.0))["kept"]


@pytest.mark.slow
def test_a_build_failure_is_replayed_on_its_layout_and_its_contacts_named():
    from brixzle.families import registry

    fam = registry.get("ab")
    cfg = RA._ideal_cfg(True)
    sample = {"dx": 0.0, "dy": 6.0, "aim": 0.0, "yaw": 0.0, "roll": 0.0, "pitch": 0.0,
              "clear": cfg.error.clearance[0] + 3.0, "vel": [0.0, 0.0, 0.0]}
    out = RA.replay_failures(fam, fam.defaults, [{"build": 0, "brick": "b2_0", "kind": "back", "outcome": "jammed",
                                                  "sample": sample}], held=True)
    r = out[0]
    assert r["layout"] == "back_cantilever" and r["replay_outcome"] in ("seated", "jammed", "wrong_valley", "fell")
    assert "repeats_on_ideal_layout" in r and r["label"] == RA.LABEL
    if r["replay_outcome"] != "seated":
        assert r["sites"] and all(s["on_support"]["label"] for s in r["sites"])
