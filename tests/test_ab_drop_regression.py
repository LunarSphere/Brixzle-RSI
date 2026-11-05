"""Regression: the ported sim core reproduces aerial-assembly's A/B drops (issue #4).

The reference (tests/fixtures/ab_drop_reference.json) was recorded by
tests/fixtures/make_ab_drop_reference.py from aerial-assembly @ a649663 with the same params,
seed and config. All values are simulated, uncalibrated contact, not hardware rates.

Two tiers:

- On every platform, sampled errors, outcomes and floor contact must match exactly, and mass
  properties closely. This is the issue's "same outcome for the same seed and config".
- On the reference platform (macOS arm64) geometry and settled poses must match bit for bit
  (1e-6). Elsewhere they are skipped: the convex decomposition of the profile depends on the
  platform's floating point (Linux x86_64 gives part B 67 collision pieces, macOS arm64 69),
  which moves contact points and so the settled pose. The upstream code does the same.
"""

import json
import platform
from pathlib import Path

import pytest

from brixzle.engine.sim import approach_error as AE
from brixzle.engine.sim import trials as T
from brixzle.families.ab.params import V0, BrickParams

REFERENCE = json.loads(
    (Path(__file__).parent / "fixtures" / "ab_drop_reference.json").read_text(encoding="utf-8")
)
SAMPLE_KEYS = ("dx", "dy", "aim", "yaw", "roll", "pitch", "clear", "vel")
CASES = REFERENCE["cases"]
REFERENCE_PLATFORM = REFERENCE["source"]["platform"]
ON_REFERENCE_PLATFORM = (
    platform.system() == "Darwin" and platform.machine() == "arm64"
    and REFERENCE_PLATFORM.startswith("macOS") and "arm64" in REFERENCE_PLATFORM
)
exact_platform_only = pytest.mark.skipif(
    not ON_REFERENCE_PLATFORM,
    reason=f"bit-exact geometry and poses are only reproducible on {REFERENCE_PLATFORM}",
)


def _config(error):
    if error == "default":
        return T.TrialConfig()
    _, stat = error.split(":")
    return T.TrialConfig(error=AE.error_model(AE.load(AE.DATA_DIR / "error-exp3-r4.json"), stat))


@pytest.fixture(scope="module")
def drops(ab_bricks):
    """{case name: rows} for every reference case, run once."""
    out = {}
    for case in CASES:
        rows, _ = T.drop_trials(V0, ab_bricks, _config(case["error"]), samples=case["samples"],
                                seed=case["seed"], part=case["part"])
        out[case["name"]] = rows
    return out


def test_reference_is_labelled_and_sourced():
    assert REFERENCE["label"] == "simulated, uncalibrated contact"
    assert REFERENCE["source"]["commit"].startswith("a649663")
    assert REFERENCE["source"]["mujoco"] == "3.16.0"


def test_params_match_reference():
    assert BrickParams.from_dict(REFERENCE["params"]) == V0


@pytest.mark.parametrize("case", CASES, ids=lambda c: c["name"])
def test_config_matches_reference(case):
    assert json.loads(json.dumps(T.as_dict(_config(case["error"])))) == case["config"]  # tuples -> lists


@pytest.mark.parametrize("part", ["A", "B"])
def test_mass_properties_match_reference(ab_bricks, part):
    ref, b = REFERENCE["geometry"][part], ab_bricks[part]
    assert b["mass_g"] == pytest.approx(ref["mass_g"], rel=1e-6)
    assert b["volume"] == pytest.approx(ref["volume_mm3"], rel=1e-6)
    assert list(b["com"]) == pytest.approx(ref["com_mm"], rel=1e-6, abs=1e-6)
    for row, ref_row in zip(b["inertia"], ref["inertia_g_mm2"]):
        assert list(row) == pytest.approx(ref_row, rel=1e-6, abs=1e-6)


@pytest.mark.parametrize("case", CASES, ids=lambda c: c["name"])
def test_drop_outcomes_reproduce_aerial_assembly(drops, case):
    rows = drops[case["name"]]
    assert len(rows) == len(case["rows"])
    for i, (row, ref) in enumerate(zip(rows, case["rows"])):
        assert {k: row[k] for k in SAMPLE_KEYS} == {k: ref[k] for k in SAMPLE_KEYS}, i
        assert row["outcome"] == ref["outcome"], i
        assert row["floor"] == ref["floor"], i


@exact_platform_only
@pytest.mark.parametrize("part", ["A", "B"])
def test_collision_pieces_match_reference_exactly(ab_bricks, part):
    ref, b = REFERENCE["geometry"][part], ab_bricks[part]
    assert len(b["collision"]) == ref["n_collision"]
    assert len(b["collision_bored"]) == ref["n_collision_bored"]


@exact_platform_only
@pytest.mark.parametrize("case", CASES, ids=lambda c: c["name"])
def test_settled_poses_match_reference_exactly(drops, case):
    for i, (row, ref) in enumerate(zip(drops[case["name"]], case["rows"])):
        assert row["err_mm"] == pytest.approx(ref["err_mm"], abs=1e-6), i
        assert row["angle_deg"] == pytest.approx(ref["angle_deg"], abs=1e-6), i
