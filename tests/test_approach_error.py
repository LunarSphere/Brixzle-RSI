# From LunarSphere/aerial-assembly brixzle/tests/test_approach_error.py @ a649663
# Changed: new imports; the error file is read from brixzle/data instead of configs/;
# rules, fixture, CLI and optimizer tests not ported (those modules are not ported).

import json
import math

import numpy as np
import pytest

from brixzle.engine.sim import approach_error as AE
from brixzle.engine.sim import trials as T

RADII_M = [0.004, 0.010, 0.012, 0.020, 0.030]


def _metrics(tmp_path, radii=RADII_M, kind="approach-accuracy-metrics"):
    approaches = [{"trial": "trial-01", "tag_id": i, "positioning_error_m": r,
                   "positioning_error_xy_m": [r, 0.0], "control_error_m": r / 4}
                  for i, r in enumerate(radii)]
    approaches.append({"trial": "trial-01", "tag_id": 9, "positioning_error_m": None})  # missing camera
    m = {"version": 1, "kind": kind, "positioning_source": "drone_marker",
         "trials_included": ["trial-01"], "trials_excluded": [],
         "summary": {"positioning": {"overall": {
             "n": len(radii), "mean_error_m": float(np.mean(radii)),
             "rmse_m": float(np.sqrt(np.mean(np.square(radii)))), "std_error_m": float(np.std(radii)),
             "max_error_m": max(radii), "bias_xy_m": [-0.008, 0.004]}}},
         "approaches": approaches}
    path = tmp_path / "metrics.json"
    path.write_text(json.dumps(m))
    return path


def test_summary_from_metrics(tmp_path):
    s = AE.load(_metrics(tmp_path))
    assert s["kind"] == "approach-error"
    assert s["n"] == 5
    assert s["radii_mm"] == pytest.approx([4.0, 10.0, 12.0, 20.0, 30.0])
    assert s["mean_mm"] == pytest.approx(15.2)
    assert s["max_mm"] == pytest.approx(30.0)
    assert s["bias_xy_mm"] == pytest.approx([-8.0, 4.0])  # recorded, not applied
    assert s["source"]["metrics"].endswith("metrics.json")
    assert s["source"]["positioning_source"] == "drone_marker"


def test_summary_round_trips_as_config(tmp_path):
    s = AE.load(_metrics(tmp_path))
    cfg = tmp_path / "error.json"
    cfg.write_text(json.dumps(s))
    assert AE.load(cfg) == s


def test_rejects_wrong_kind_and_non_finite(tmp_path):
    with pytest.raises(ValueError):
        AE.load(_metrics(tmp_path, kind="approach-accuracy"))
    with pytest.raises(ValueError):
        AE.load(_metrics(tmp_path, radii=[0.01, float("nan")]))
    with pytest.raises(ValueError):
        AE.error_model(AE.load(_metrics(tmp_path)), "median")


def test_rejects_summary_whose_n_disagrees(tmp_path):
    s = AE.load(_metrics(tmp_path))
    path = tmp_path / "error.json"
    path.write_text(json.dumps({**s, "n": 4}))
    with pytest.raises(ValueError, match="n does not match"):
        AE.load(path)


@pytest.mark.parametrize("stat,radius", [("mean", 15.2), ("max", 30.0), ("p95", 28.0)])
def test_ring_models_sample_fixed_radius_any_direction(tmp_path, stat, radius):
    err = AE.error_model(AE.load(_metrics(tmp_path)), stat)
    assert err.radial_mode == "ring" and err.aim_bias == 0.0
    assert err.radius_mm == pytest.approx(radius)
    assert err.source["statistic"] == stat
    rng = np.random.default_rng(0)
    xy = np.array([[s["dx"], s["dy"]] for s in (err.sample(rng) for _ in range(200))])
    assert np.hypot(*xy.T) == pytest.approx(radius)
    assert abs(xy.mean(0)).max() < 0.25 * radius  # zero-mean: the measured bias is not applied
    assert (xy[:, 0] < 0).any() and (xy[:, 1] < 0).any()


def test_empirical_model_bootstraps_measured_radii(tmp_path):
    err = AE.error_model(AE.load(_metrics(tmp_path)), "empirical")
    assert err.radial_mode == "bootstrap"
    rng = np.random.default_rng(1)
    r = {round(float(np.hypot(s["dx"], s["dy"])), 6) for s in (err.sample(rng) for _ in range(100))}
    assert r <= {4.0, 10.0, 12.0, 20.0, 30.0}
    assert len(r) > 1


def test_unmeasured_terms_keep_defaults_and_are_labelled(tmp_path):
    err = AE.error_model(AE.load(_metrics(tmp_path)), "mean")
    d = T.ErrorModel()
    assert (err.sigma_yaw, err.sigma_tilt, err.clearance, err.sigma_vel) == \
        (d.sigma_yaw, d.sigma_tilt, d.clearance, d.sigma_vel)
    assert set(err.source["unmeasured"]) == {"sigma_yaw", "sigma_tilt", "clearance", "sigma_vel"}


@pytest.mark.parametrize("stat,tol", [("mean", 15.2), ("max", 30.0), ("p95", 28.0), ("empirical", None)])
def test_required_tolerance(tmp_path, stat, tol):
    s = AE.load(_metrics(tmp_path))
    expect = tol if tol is not None else float(np.percentile(s["radii_mm"], 95))
    assert AE.required_tolerance(s, stat) == pytest.approx(expect)


def test_error_model_source_is_json_safe(tmp_path):
    err = AE.error_model(AE.load(_metrics(tmp_path)), "max")
    rec = T.as_dict(T.TrialConfig(error=err))["error"]
    assert rec["radial_mode"] == "ring" and rec["radius_mm"] == pytest.approx(30.0)
    json.dumps(rec, allow_nan=False)


def test_committed_exp3_r4_error_file():
    """brixzle/data/error-exp3-r4.json holds the aerial-assembly runs/exp3-r4 positioning error."""
    s = AE.load(AE.DATA_DIR / "error-exp3-r4.json")
    assert s["source"]["run"] == "runs/exp3-r4"
    assert s["n"] == len(s["radii_mm"]) == 24
    assert s["mean_mm"] == pytest.approx(14.82, abs=0.01)
    assert s["max_mm"] == pytest.approx(38.50, abs=0.01)
    assert math.isclose(max(s["radii_mm"]), s["max_mm"])
