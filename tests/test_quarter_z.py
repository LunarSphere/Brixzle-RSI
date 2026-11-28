"""The four-quarter score (#222): Z = 1/4 Z_stack + 1/4 Z_overhang1 + 1/4 Z_overhang2 + 1/4 Z_overhang3,
each (S - J) / N over the placement's grid cells inside the 16 mm disc; the same four placements in
the screen. Sim results are simulated, uncalibrated contact."""
import math

import pytest

from brixzle.director import archive as AR
from brixzle.engine import gridscore as GS
from brixzle.engine import quick_check as QC
from brixzle.engine import radius as RA
from brixzle.engine.sim import trials as T
from brixzle.families.interface import Placement

FINE = GS.GridSettings().fine_mm


def _lat():
    from brixzle.families import registry
    fam = registry.get("ab")
    return fam.lattice(fam.defaults)


# --- overhang 3 ---


def test_overhang_3_drops_the_course_3_reach_on_a_back_first_corbel():
    assert "plus_cantilever3" in T.EDGE_CASES
    placed, target, base = T.edge_layout(_lat(), "plus_cantilever3")
    assert target == Placement(3, 3)
    assert placed[0] == Placement(2, 2)  # the supporter: the course-2 reach
    # courses 0-2 complete and the course-3 back brick, which holds the course-2 reach's tail down
    assert set(placed) == {Placement(0, 0), Placement(0, -2), Placement(1, -1), Placement(1, 1),
                           Placement(2, 0), Placement(2, 2), Placement(3, 1)}
    assert base == (-3, 2)


def test_the_quarters_are_the_stack_and_the_three_reaches():
    assert RA.QUARTERS == ("full", "corbel_reach", "corbel_reach2", "corbel_reach3")
    assert RA.PLACEMENTS["corbel_reach3"] == "plus_cantilever3"
    assert GS.GridSettings().placements == RA.QUARTERS


@pytest.mark.parametrize("brick, layout", [("b2_2", "plus_cantilever"), ("b3_3", "plus_cantilever3"),
                                           ("b5_5", "plus_cantilever3")])
def test_later_reaches_replay_on_the_deepest_reach_layout(brick, layout):
    assert RA.corbel_layout(brick) == layout


def test_overhang_3_seats_when_dropped_on_its_aim():
    """One free drop at the aim of fable_allround on the new layout seats (simulated, uncalibrated contact)."""
    from brixzle.engine import zscore as ZS
    from brixzle.families import registry

    fam = registry.get("fable_allround")
    cfg = RA._ideal_cfg(False)
    rig = T.DropRig(fam, fam.defaults, cfg, course="plus_cantilever3")
    row = rig.drop(ZS._sample(cfg, {"x": 0.0, "y": 0.0, "z": GS.release_z_mm(GS.GridSettings()), "t": 0.0}),
                   trace=False)
    assert row["outcome"] == "seated", row


# --- Z ---


def _search(outcome_at, **kw):
    s = GS.GridSettings(**kw)
    return GS.search(lambda pts: [outcome_at(p, x, y) for p, x, y in pts], s)


def test_z_is_the_mean_of_the_quarters_z_in_the_16_mm_disc():
    per = {"full": "seated", "corbel_reach": "jammed", "corbel_reach2": "fell", "corbel_reach3": "seated"}
    res = _search(lambda p, x, y: per[p])
    assert res["z_quarters"] == {"full": 1.0, "corbel_reach": -1.0, "corbel_reach2": 0.0, "corbel_reach3": 1.0}
    assert res["z"] == pytest.approx(0.25)
    assert res["z_rings"]["16"] == pytest.approx(0.25) and set(res["z_rings"]) == {"16", "28", "40"}


def test_z_weights_cells_by_area_with_the_same_math_as_before():
    # half the 16 mm disc jams (x > 0), the rest seats: Z near 0 on that quarter, by area
    res = _search(lambda p, x, y: "jammed" if (p == "full" and x > 0) else "seated")
    zf = res["z_quarters"]["full"]
    r = res["placements"]["full"]["rings"]["16"]
    assert zf == r["Z"] == pytest.approx((r["S"] - r["J"]) / r["N"], abs=1e-4)
    assert -0.3 < zf < 0.3
    assert res["z"] == pytest.approx((zf + 3.0) / 4, abs=1e-4)


def test_the_early_stop_still_refines_every_boundary_inside_16_mm():
    # a failure at the aim ends the green zone at 0; a jam pocket at 15 mm must still be resolved,
    # because it is inside the disc Z is counted over
    def outcome(p, x, y):
        if math.hypot(x, y) <= 1.0 or math.hypot(x - 15.0, y) <= 3.0:
            return "jammed"
        return "seated"

    res = _search(outcome, placements=("full",))
    full = res["placements"]["full"]
    assert full["green_mm"] == 0.0 and full["early_stop"] is True
    near_pocket = [c for c in full["cells"] if math.hypot(c["x_mm"] - 15.0, c["y_mm"]) <= 5.0]
    assert any(c["size_mm"] == FINE for c in near_pocket)


def test_z_ring_must_be_one_of_the_rings():
    with pytest.raises(ValueError):
        GS.GridSettings(z_ring_mm=20.0)


# --- the screen ---


def test_the_screen_drops_ten_random_poses_per_quarter_inside_16_mm():
    poses = QC.quarter_poses(seed=3)
    assert len(poses) == 40
    assert {pl: sum(p == pl for p, _, _ in poses) for pl in RA.QUARTERS} == {pl: 10 for pl in RA.QUARTERS}
    assert all(math.hypot(x, y) <= 16.0 + 1e-9 for _, x, y in poses)
    assert poses == QC.quarter_poses(seed=3) and poses != QC.quarter_poses(seed=4)
    # interleaved, so a check stopped early has judged every quarter
    assert [p for p, _, _ in poses[:4]] == list(RA.QUARTERS)


def test_the_screen_passes_at_32_seated_and_stops_at_the_9th_failure():
    poses = QC.quarter_poses(seed=0)
    bad = {k for k in range(40) if k % 5 == 0}  # 8 failures, spread over the quarters
    res = QC.quarter_screen(None, None, drop=lambda k, pl, x, y: "jammed" if k in bad else "seated")
    assert res["passed"] and res["dropped"] == 40 and res["outcomes"]["seated"] == 32
    assert res["z"] == pytest.approx((32 - 8) / 40)
    assert set(res["quarters"]) == set(RA.QUARTERS)
    bad9 = bad | {1}
    res = QC.quarter_screen(None, None, drop=lambda k, pl, x, y: "jammed" if k in bad9 else "seated")
    assert not res["passed"] and res["dropped"] == max(bad9) + 1 < 40
    assert len(poses) == 40


def test_one_dead_quarter_fails_the_screen():
    res = QC.quarter_screen(None, None, drop=lambda k, pl, x, y: "fell" if pl == "corbel_reach3" else "seated")
    assert not res["passed"]
    assert res["quarters"]["corbel_reach3"]["S"] == 0 and res["quarters"]["full"]["Z"] == 1.0


# --- the archive ---


def _rec(name, z16, z28=0.0, z40=0.0, violations=()):
    return {"name": name, "z": z16, "z_rings": {"16": z16, "28": z28, "40": z40}, "green_mm": 0.0,
            "violations": list(violations)}


def test_bands_are_named_by_z():
    assert AR.band_of(-0.2) == "below 0"
    assert AR.band_of(0.3) == "0-0.5"
    assert AR.band_of(0.9) == "0.8-0.95"
    assert AR.band_of(1.0) == "0.95+"


def test_the_archive_ranks_on_z16_then_z28_then_z40():
    arc = AR.BandArchive()
    assert arc.offer(_rec("a", 0.60, z28=0.2))["kept"]
    assert not arc.offer(_rec("b", 0.59, z28=0.1))["kept"]   # a tie on Z16, worse on Z28
    res = arc.offer(_rec("c", 0.61, z28=0.4))                  # a tie on Z16, better on Z28
    assert res["kept"] and res["replaced"] == ["a"]
    assert arc.offer(_rec("d", 0.70))["kept"]                  # clearly better Z16 in the same band
    assert arc.offer(_rec("e", 0.96))["kept"]                  # another band
    assert not arc.offer(_rec("f", 1.0, violations=["mvp_gate"]))["kept"]
    assert sorted(m["name"] for m in arc.members) == ["d", "e"]


def test_a_record_scored_before_222_is_refused_for_a_rescore():
    res = AR.BandArchive().offer({"name": "old", "green_mm": 20.0, "violations": []})
    assert not res["kept"] and "rescore" in res["reason"]


# --- verify ---


def _fakes(monkeypatch, screen_passes=True):
    from brixzle.tools import check_family as CF
    from brixzle.engine import gripper_spec as GSP
    from brixzle.engine import structure_gate as SG
    from brixzle.tools import core as TC

    monkeypatch.setattr(CF, "check_in_subprocess", lambda root, module: {"ok": True, "violations": [], "mass_g": 9.0})
    monkeypatch.setattr(TC, "load", lambda spec: type("F", (), {"defaults": None, "name": "fake"})())
    monkeypatch.setattr(GSP, "target_check", lambda fam, p: {"violations": []})
    monkeypatch.setattr(SG, "run_gate", lambda *a, **k: {"passed": True, "overhang_n": 3, "builds": [], "seconds": 1,
                                                         "run": "r"})
    monkeypatch.setattr(QC, "quarter_screen", lambda fam, p, **k: {"passed": screen_passes, "z": 0.5, "dropped": 40})
    calls = {"grid": 0, "builds": 0}
    fake = GS.search(lambda pts: ["seated" if abs(x) < 18 else "jammed" for _, x, _ in pts], GS.GridSettings())

    def grid(fam, p, s, workers=None):
        calls["grid"] += 1
        return {**fake, "drops_rows": [], "jams": None}

    def builds(*a, **k):
        calls["builds"] += 1
        return {"radius_mm": 16.0, "overhang_steps": 3.0, "stack_courses": 3.0}

    monkeypatch.setattr(GS, "grid_search", grid)
    monkeypatch.setattr(RA, "noisy_builds", builds)
    return calls, fake


def test_verify_scores_z_on_the_quarters_without_the_noisy_builds(tmp_path, monkeypatch):
    from brixzle.director import verify as VE

    calls, fake = _fakes(monkeypatch)
    m = VE.measure_radius("x:y", tmp_path / "measured")
    assert calls == {"grid": 1, "builds": 0}
    assert m["z"] == fake["z"] == 1.0 and m["z_quarters"] == fake["z_quarters"]
    assert m["z_rings"]["16"] == 1.0 and m["screen"]["passed"] and m["violations"] == []
    assert m["green_mm"] == fake["green_mm"]


def test_a_design_that_fails_the_screen_skips_the_grid(tmp_path, monkeypatch):
    from brixzle.director import verify as VE

    calls, _ = _fakes(monkeypatch, screen_passes=False)
    m = VE.measure_radius("x:y", tmp_path / "measured")
    assert calls["grid"] == 0 and "screen" in m["violations"]
    assert m["z"] == 0.5  # the screen's Z stands in, marked by the violation


def test_finalists_can_still_run_the_noisy_builds(tmp_path, monkeypatch):
    from brixzle.director import verify as VE

    calls, _ = _fakes(monkeypatch)
    m = VE.measure_radius("x:y", tmp_path / "measured", build_settings=RA.BuildSettings())
    assert calls["builds"] == 1 and m["overhang_steps"] == 3.0


def test_the_screen_refuses_a_brick_that_is_not_two_voxels_long():
    fam = type("F", (), {"name": "one", "lattice": lambda self, p: type("L", (), {"span_voxels": 1})()})()
    res = QC.quarter_screen(fam, None)
    assert not res["passed"] and "span_voxels" in res["error"] and res["dropped"] == 0
