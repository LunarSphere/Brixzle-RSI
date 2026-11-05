"""Regression: the ported sim core reproduces aerial-assembly's A/B drops (issue #4).

The reference (tests/fixtures/ab_drop_reference.json) was recorded by
tests/fixtures/make_ab_drop_reference.py from aerial-assembly @ a649663 with the same params,
seed and config. All values are simulated, uncalibrated contact, not hardware rates.

Sampled errors come from a seeded NumPy generator and must match exactly. Outcomes must match
exactly. Settled poses may differ by floating-point noise across platforms (the reference was
recorded on macOS arm64; CI runs Linux x86_64), so they are compared with a tolerance.
"""

import json
import math
from pathlib import Path

import pytest

from brixzle.engine.sim import approach_error as AE
from brixzle.engine.sim import trials as T
from brixzle.families.ab.params import V0, BrickParams

REFERENCE = json.loads(
    (Path(__file__).parent / "fixtures" / "ab_drop_reference.json").read_text(encoding="utf-8")
)
SAMPLE_KEYS = ("dx", "dy", "aim", "yaw", "roll", "pitch", "clear", "vel")
POSE_TOL_MM = 0.5
ANGLE_TOL_DEG = 0.5


def _config(error):
    if error == "default":
        return T.TrialConfig()
    _, stat = error.split(":")
    return T.TrialConfig(error=AE.error_model(AE.load(AE.DATA_DIR / "error-exp3-r4.json"), stat))


def test_reference_is_labelled_and_sourced():
    assert REFERENCE["label"] == "simulated, uncalibrated contact"
    assert REFERENCE["source"]["commit"].startswith("a649663")
    assert REFERENCE["source"]["mujoco"] == "3.16.0"


def test_params_match_reference():
    assert BrickParams.from_dict(REFERENCE["params"]) == V0


@pytest.mark.parametrize("part", ["A", "B"])
def test_geometry_matches_reference(ab_bricks, part):
    ref, b = REFERENCE["geometry"][part], ab_bricks[part]
    assert len(b["collision"]) == ref["n_collision"]
    assert len(b["collision_bored"]) == ref["n_collision_bored"]
    assert b["mass_g"] == pytest.approx(ref["mass_g"], rel=1e-6)
    assert b["volume"] == pytest.approx(ref["volume_mm3"], rel=1e-6)
    assert list(b["com"]) == pytest.approx(ref["com_mm"], rel=1e-6, abs=1e-6)
    for row, ref_row in zip(b["inertia"], ref["inertia_g_mm2"]):
        assert list(row) == pytest.approx(ref_row, rel=1e-6, abs=1e-6)


@pytest.mark.parametrize("case", REFERENCE["cases"], ids=lambda c: c["name"])
def test_drop_reproduces_aerial_assembly(ab_bricks, case):
    cfg = _config(case["error"])
    assert json.loads(json.dumps(T.as_dict(cfg))) == case["config"]  # tuples are lists in JSON
    rows, _ = T.drop_trials(V0, ab_bricks, cfg, samples=case["samples"], seed=case["seed"],
                            part=case["part"])
    assert len(rows) == len(case["rows"])
    for i, (row, ref) in enumerate(zip(rows, case["rows"])):
        assert {k: row[k] for k in SAMPLE_KEYS} == {k: ref[k] for k in SAMPLE_KEYS}, i
        assert row["outcome"] == ref["outcome"], i
        assert row["floor"] == ref["floor"], i
        assert row["err_mm"] == pytest.approx(ref["err_mm"], abs=POSE_TOL_MM), i
        assert math.isclose(row["angle_deg"], ref["angle_deg"], abs_tol=ANGLE_TOL_DEG), i
