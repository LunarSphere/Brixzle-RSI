"""Scorer (issues #13, #61): report.json schema 4, Z per target box from the full sim map.

Most tests build a report from made-up map drops, so they need no sim. One test scores the A/B
baseline at coarse map settings. Sim results are simulated, uncalibrated contact.
"""

import json

import pytest

from brixzle.engine import boxes as BX
from brixzle.engine import report as RP
from brixzle.engine import sim_map as SM

BOXES = BX.default_boxes()["placement"]
C0 = 5.0


def _drop(outcome, x=0.0, z=1.0, t=0.0):
    return {"outcome": outcome, "dx_mm": x, "clear_mm": C0 + z, "pitch_deg": t, "slice": "xz", "course": 0}


def _map(cover=("mean", "p95", "max")):
    return {"label": SM.LABEL, "family": "ab", "family_version": "1", "params": {"U": 25.0},
            "box": BOXES["mean"].to_dict(), "cover": list(cover), "config": {"error": {"clearance": [C0, 30.0]}},
            "check": {"passed": True, "seated": 36, "dropped": 40, "seed": 0}, "drops": 5, "seconds": 1.0,
            "grids": [], "inside_failures": 0}


DROPS = [_drop("seated", x=0.0), _drop("jammed", x=10.0), _drop("seated", x=20.0), _drop("fell", x=30.0),
         _drop("wrong_valley", x=-35.0)]


def _report(cover=("mean", "p95", "max"), drops=DROPS, total=10.0):
    return RP.build_report(_map(cover), drops, BOXES, {"total": total}, map_dir="map-x-ab-mean")


def test_the_report_holds_z_per_target_with_its_counts():
    rep = _report()
    assert rep["schema_version"] == 4 and rep["label"] == "simulated, uncalibrated contact"
    assert rep["statistics"] == ["mean", "p95", "max"] and rep["statistic"] == "mean"
    assert rep["z"]["mean"] == {"Z": 0.0, "S": 1, "J": 1, "N": 2, "fell": 0, "wrong_valley": 0, "other": 0,
                                "passed": False}
    assert rep["z"]["p95"]["N"] == 3 and rep["z"]["p95"]["Z"] == pytest.approx(1 / 3)
    assert rep["z"]["max"] == {"Z": pytest.approx(1 / 5), "S": 2, "J": 1, "N": 5, "fell": 1, "wrong_valley": 1,
                               "other": 0, "passed": False}
    assert rep["covers_all_targets"] is False
    assert "Z = (S - J) / N" in rep["objective"]


def test_no_box_scale_or_predictor_term_is_left_in_the_report():
    text = json.dumps(_report())
    for gone in ("scale", "pred", "disagree", "cross_check"):
        assert gone not in text


def test_every_target_passes_only_when_every_drop_in_every_box_seats():
    rep = _report(drops=[_drop("seated", x=x) for x in range(-38, 39)])
    assert all(rep["z"][s]["Z"] == 1.0 and rep["z"][s]["passed"] for s in rep["statistics"])
    assert rep["covers_all_targets"] is True


def test_a_target_the_map_did_not_cover_is_refused():
    with pytest.raises(ValueError, match="cover"):
        _report(cover=("mean",))


def test_report_passes_its_schema_and_is_json_safe():
    rep = _report()
    RP.validate_report(rep)
    json.dumps(rep, allow_nan=False)


@pytest.mark.parametrize("break_it", [
    lambda r: r.update(schema_version=3),
    lambda r: r.pop("z"),
    lambda r: r["z"]["mean"].update(Z=1.5),
    lambda r: r["z"]["mean"].update(S=2),          # (S - J) / N no longer gives Z
    lambda r: r.update(statistics=["mean", "p95"]),  # z holds a target statistics does not name
    lambda r: r["timing"].update(total=float("nan")),
])
def test_schema_rejects_a_broken_report(break_it):
    rep = _report()
    break_it(rep)
    with pytest.raises(ValueError):
        RP.validate_report(rep)


def test_over_budget_is_flagged():
    assert _report(total=31 * 60)["timing"]["over_budget"] is True
    assert _report(total=60)["timing"]["over_budget"] is False


def test_the_report_is_written_beside_the_map_and_never_overwritten(tmp_path):
    rep = _report()
    path = RP.write_report(rep, tmp_path)
    assert path == tmp_path / "report.json"
    assert json.loads(path.read_text(encoding="utf-8")) == rep
    with pytest.raises(FileExistsError):
        RP.write_report(rep, tmp_path)


def test_the_summary_names_z_for_every_target():
    text = RP.summary(_report())
    assert "Z" in text and all(s in text for s in ("mean", "p95", "max"))


COARSE = SM.MapConfig(x_step_U=0.2, z_step_U=0.5, theta_step_deg=5.0, z_limit_U=0.5, theta_limit_deg=5.0)


def test_score_family_maps_every_target_box_and_writes_a_valid_report(tmp_path):
    from brixzle.families.ab.family import FAMILY as AB
    from brixzle.families.ab.params import V0

    rep, out = RP.score_family(AB, V0, statistic="mean", runs=tmp_path, workers=4, config=COARSE,
                               config_reason="coarse cells for a short test", allow_failed_check=True)
    RP.validate_report(rep)
    assert out.parent == tmp_path and {p.name for p in out.iterdir()} == {"map.json", "drops.json.gz", "report.json"}
    m = json.loads((out / "map.json").read_text(encoding="utf-8"))
    assert m["cover"] == ["mean", "p95", "max"]
    assert rep["map"]["dir"] == out.name and rep["check"]["seed"] == 0
    for s in rep["statistics"]:
        z = rep["z"][s]
        assert z["N"] > 0 and z["Z"] == pytest.approx((z["S"] - z["J"]) / z["N"])
    assert rep["z"]["mean"]["N"] < rep["z"]["p95"]["N"] < rep["z"]["max"]["N"]  # the boxes nest
