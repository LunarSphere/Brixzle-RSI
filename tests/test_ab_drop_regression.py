"""Regression: the ported sim core reproduces aerial-assembly's A/B drops (issue #4).

Since issue #82 a course-1 drop lands across two seated course-0 bricks (running bond), where
aerial-assembly dropped onto one. Both reference cases are course-1 drops, so their outcomes and
settled poses now come from tests/fixtures/ab_drop_running_bond.json (written by
make_ab_running_bond_reference.py from this repo's rig); the params, configs, mass properties and
sampled errors are still checked against aerial-assembly's reference. Old -> new outcomes: gauss-seed0
4 seated -> 4 seated; exp3-mean-seed0 4 jammed -> 3 jammed and 1 wrong valley (drop 0).

The reference (tests/fixtures/ab_drop_reference.json) was recorded by
tests/fixtures/make_ab_drop_reference.py from aerial-assembly @ a649663 with the same params,
seed and config. All values are simulated, uncalibrated contact, not hardware rates.

Two tiers:

- On every platform, sampled errors, outcomes and floor contact must match exactly, and mass
  properties to 1e-3 relative (CadQuery's tessellation is platform dependent too). This is the issue's "same outcome for the same seed and config".
- On the reference platform (macOS arm64) geometry, mass properties and settled poses must
  match bit for bit (1e-6 or tighter). Elsewhere they are skipped: the convex decomposition of the profile depends on the
  platform's floating point (Linux x86_64 gives part B 67 collision pieces, macOS arm64 69),
  which moves contact points and so the settled pose. The upstream code does the same.
"""

import json
import platform
from pathlib import Path

import pytest

from brixzle.engine.sim import approach_error as AE
from brixzle.engine.sim import trials as T
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.ab.params import V0, BrickParams

REFERENCE = json.loads(
    (Path(__file__).parent / "fixtures" / "ab_drop_reference.json").read_text(encoding="utf-8")
)
COURSE = {"A": 0, "B": 1}  # the reference's part B drops onto a seated A, i.e. course 1
SAMPLE_KEYS = ("dx", "dy", "aim", "yaw", "roll", "pitch", "clear", "vel")
CASES = REFERENCE["cases"]
RUNNING_BOND = {c["name"]: c["rows"] for c in json.loads(
    (Path(__file__).parent / "fixtures" / "ab_drop_running_bond.json").read_text(encoding="utf-8"))["cases"]}
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
def drops():
    """{case name: rows} for every reference case, run once."""
    out = {}
    for case in CASES:
        rows, _ = T.drop_trials(AB, V0, _config(case["error"]), samples=case["samples"],
                                seed=case["seed"], course=COURSE[case["part"]])
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


def _check_mass_properties(b, ref, rel, abs_):
    assert b.mass_g == pytest.approx(ref["mass_g"], rel=rel)
    assert b.volume_mm3 == pytest.approx(ref["volume_mm3"], rel=rel)
    assert list(b.com_mm) == pytest.approx(ref["com_mm"], rel=rel, abs=abs_)
    for row, ref_row in zip(b.inertia_g_mm2, ref["inertia_g_mm2"]):
        assert list(row) == pytest.approx(ref_row, rel=rel, abs=abs_)


@pytest.mark.parametrize("part", ["A", "B"])
def test_mass_properties_match_reference(ab_bodies, part):
    # CadQuery's tessellation differs slightly by platform (part B: 13.5321 g on Linux x86_64,
    # 13.5273 g on macOS arm64), so 1e-3 relative here and bit-exact below.
    _check_mass_properties(ab_bodies[part], REFERENCE["geometry"][part], rel=1e-3, abs_=1e-2)


@exact_platform_only
@pytest.mark.parametrize("part", ["A", "B"])
def test_mass_properties_match_reference_exactly(ab_bodies, part):
    _check_mass_properties(ab_bodies[part], REFERENCE["geometry"][part], rel=1e-9, abs_=1e-9)


@pytest.mark.parametrize("case", CASES, ids=lambda c: c["name"])
def test_drop_samples_reproduce_aerial_assembly_and_outcomes_the_running_bond(drops, case):
    rows = drops[case["name"]]
    assert len(rows) == len(case["rows"]) == len(RUNNING_BOND[case["name"]])
    for i, (row, ref, bond) in enumerate(zip(rows, case["rows"], RUNNING_BOND[case["name"]])):
        assert {k: row[k] for k in SAMPLE_KEYS} == {k: ref[k] for k in SAMPLE_KEYS}, i
        assert row["outcome"] == bond["outcome"], i
        assert row["floor"] == bond["floor"], i


@exact_platform_only
@pytest.mark.parametrize("part", ["A", "B"])
def test_collision_pieces_match_reference_exactly(ab_bodies, part):
    # n_collision_bored (pieces with the tine bores open) is recorded too, but nothing builds
    # bored collision until the pickup trial (#9).
    assert len(ab_bodies[part].collision) == REFERENCE["geometry"][part]["n_collision"]


@exact_platform_only
@pytest.mark.parametrize("case", CASES, ids=lambda c: c["name"])
def test_settled_poses_match_reference_exactly(drops, case):
    for i, (row, ref) in enumerate(zip(drops[case["name"]], RUNNING_BOND[case["name"]])):
        assert row["err_mm"] == pytest.approx(ref["err_mm"], abs=1e-6), i
        assert row["angle_deg"] == pytest.approx(ref["angle_deg"], abs=1e-6), i
