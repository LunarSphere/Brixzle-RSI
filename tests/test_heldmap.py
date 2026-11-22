"""Held objective conventions and staged assembly (#192)."""
import json
import math
from pathlib import Path

import pytest

from brixzle.engine import structures
from brixzle.engine.sim import held
from brixzle.tools import __main__ as cli


def test_reach_is_a_strict_majority_at_each_length():
    from brixzle.engine.heldmap import reach_summary
    summary = reach_summary([1, 3, 3, 4], 4, 25)
    assert summary["reliable_steps"] == 3 and summary["reliable_reach_mm"] == 75
    assert summary["median_steps"] == 3
    assert [row["rate"] for row in summary["by_length"]] == [1, .75, .75, .25]
    half = reach_summary([0, 0, 3, 3], 3, 25)
    assert half["reliable_steps"] == 0 and not half["r4_passed"]


def test_measured_mean_samples_lie_in_box_and_radius_stress_only_replaces_xy():
    from brixzle.engine import heldmap as hm
    from brixzle.engine.boxes import default_boxes
    box = default_boxes()["placement"]["mean"].half_widths
    samples = hm.mean_samples(8, 123)
    assert samples == hm.mean_samples(8, 123) and len(set(samples)) > 1
    for sample in samples:
        assert math.hypot(sample.dx_m, sample.dy_m) * 1000 <= box["x_mm"]
        assert abs(sample.dz_m)*1000 <= box["z_mm"]
        assert abs(math.degrees(sample.dpitch_rad)) <= box["theta_deg"]
        stress = hm.disc_sample(sample, 2., 123)
        assert 0 < math.hypot(stress.dx_m, stress.dy_m) <= .002
        assert stress.dz_m == sample.dz_m and stress.dyaw_rad == sample.dyaw_rad


def test_held_z_keeps_skips_and_jams_in_denominator():
    from brixzle.engine.heldmap import case_summary
    rows = [{"outcome": o, "passed": False, "phases": {}} for o in ("seated", "jammed", None, "wrong_valley", "seated")]
    result = case_summary(rows)
    assert result["Z"] == .2 and result["N"] == 5 and result["other"] == 1
    assert result["missions_passed"] == 0


def test_config_follows_candidate_with_explicit_override(tmp_path):
    from brixzle.engine.heldmap import gripper_config
    pkg = tmp_path / "brick"
    pkg.mkdir()
    spec = f"{tmp_path}:brick"
    assert gripper_config(spec) == held.HeldConfig()
    (pkg / "gripper.json").write_text('{"tine_d_m":0.003,"wrist":{"linear_k":[1,2,3]}}')
    assert gripper_config(spec).wrist.linear_k == (1, 2, 3)
    cfg = tmp_path / "override.json"
    cfg.write_text('{"tine_d_m":0.004}')
    assert gripper_config(spec, cfg).tine_d_m == .004
    for bad in ('{"drone":{"tw_min":0}}', '{"release_at_contact":true}', '{"tine_d_m":NaN}', '{"tine_d_m":true}'):
        cfg.write_text(bad)
        with pytest.raises(ValueError):
            gripper_config(spec, cfg)


def test_cli_writes_fresh_finite_reports(tmp_path, monkeypatch, capsys):
    from brixzle.engine import heldmap as hm
    spec = f"{Path(__file__).parent / 'fixtures'}:gentle_inboard_sill"
    cfg = tmp_path / "fork.json"
    cfg.write_text('{"tine_d_m":0.003}')
    def fake(family, p, cfg, settings):
        return {"scores": {"held_full_z": cfg.tine_d_m}, "seed": settings.seed, "rows": []}
    monkeypatch.setattr(hm, "heldmap", fake)
    reports = []
    for _ in range(2):
        assert cli.main(["--runs", str(tmp_path), "heldmap", spec, "--gripper", str(cfg), "--seed", "42"]) == 0
        report = json.loads(capsys.readouterr().out)
        reports.append(Path(report["run"]) / "result.json")
        assert json.loads(reports[-1].read_text())["scores"]["held_full_z"] == .003
        assert report["seed"] == 42
    assert reports[0] != reports[1]


def test_corbel_credits_only_completed_courses_and_carries_their_states():
    from brixzle.engine.heldmap import build_corbel
    st = structures.corbel(4)
    seen = []
    class Stage:
        def __init__(self, i):
            self.i = i
        def trial(self, sample, *, pickup_sample, support_states):
            assert support_states == list(range(self.i))
            seen.append((sample, pickup_sample))
            return {"outcome": "seated", "passed": self.i != 2}
        def brick_states(self):
            return list(range(self.i + 1))
        def off_seat(self):
            return ["b0_0"] if self.i == 5 else []

    result = build_corbel(st, Stage, 123, None)
    assert result["steps"] == 1 and result["mission_steps"] == 0
    assert result["stages"][-1]["off_seat"] == ["b0_0"]
    assert len(set(seen)) == len(seen) == 6


def test_incompatible_fork_reports_an_infeasible_candidate(tmp_path):
    from brixzle.engine import heldmap as hm
    from brixzle.families.ab.family import FAMILY
    report = hm.heldmap(FAMILY, FAMILY.defaults, held.HeldConfig(tine_d_m=1.), hm.HeldMapSettings())
    assert "held_gripper_fit" in report["violations"]
    assert not report["mission_passed"] and report["fit_error"]
    json.dumps(report, allow_nan=False)


def test_report_splits_release_from_mission_and_exposes_radius_scores(monkeypatch):
    from brixzle.engine import heldmap as hm
    from brixzle.families.ab.family import FAMILY
    class Rig:
        fork = dict(tines=[[-5, 6], [5, 6]], plate=[[-7,-3,2], [7,-2,9]], drone_pos_mm=[0,0,10],
                    tine_dx_dy=0., tine_r=1., y_face=-2., y_tip=12.)
        payload = .036  # injected overweight fixture in kg, not a simulated result
        def __init__(self, *args, **kwargs):
            pass
        def trial(self, *args, **kwargs):
            phases = {key: {"passed": key != "transport", "outcome": "over_budget" if key == "transport" else "ok",
                            "wall_s": .01} for key in ("takeoff", "approach", "grip", "transport", "release")}
            return {"outcome": "seated", "passed": False, "phases": phases}
        def brick_states(self):
            return []
        def off_seat(self):
            return []
    monkeypatch.setattr(hm, "HeldRig", Rig)
    result = hm.heldmap(FAMILY, FAMILY.defaults, settings=hm.HeldMapSettings(samples=1, builds=1, reaches=3))
    assert result["cases"]["full"]["N"] == 2
    assert result["scores"]["held_full_z"] == 1 and result["scores"]["held_mission_rate"] == 0
    assert result["missions"]["phases"]["transport"] == dict(passed=0, skipped=0, failed=5, wall_s=.05)
    assert result["median_steps_by_radius"] == {str(r): 3 for r in range(6)}
    assert result["scores"]["held_median_steps_5mm"] == 3
    assert result["scores"]["held_overhang_mean_steps"] == 3
    assert result["builds"]["measured"]["r4_passed"] and not result["builds"]["measured"]["missions"]["r4_passed"]
    assert not result["mission_passed"] and "held_payload" in result["violations"]
    assert result["scores"]["held_mission_reach_mm"] == 0


@pytest.mark.parametrize("kwargs", [{"seed": -1}, {"samples": True}, {"builds": 0}, {"reaches": 2}])
def test_invalid_sampling_settings(kwargs):
    from brixzle.engine.heldmap import HeldMapSettings
    with pytest.raises(ValueError):
        HeldMapSettings(**kwargs)


def test_same_hardware_must_fit_both_courses():
    from brixzle.engine.heldmap import same_fork
    a = dict(tines=[[-5, 6], [5, 6]], plate=[[-7,-3,2], [7,-2,9]], drone_pos_mm=[0,0,10],
             tine_dx_dy=0., tine_r=1., y_face=-2., y_tip=12.)
    # Re-express one mounted fork in a translated brick frame.
    b = dict(a, tines=[[-4, 8], [6, 8]], plate=[[-6,-3,4], [8,-2,11]], drone_pos_mm=[1,0,12])
    assert same_fork(a, b)
    assert not same_fork(a, dict(b, tines=[[-4, 8], [7, 8]]))


def test_station_fit_failure_reports_an_infeasible_candidate():
    from brixzle.engine import heldmap as hm
    from brixzle.engine.sim.funnel import FunnelStation
    from brixzle.families.ab.family import FAMILY
    report = hm.heldmap(FAMILY, FAMILY.defaults, held.HeldConfig(station=FunnelStation(mouth_width_m=.001)))
    assert report["violations"] == ["held_gripper_fit"] and "mouth" in report["fit_error"]
