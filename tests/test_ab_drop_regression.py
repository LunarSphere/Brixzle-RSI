"""Regression: the ported sim core reproduces aerial-assembly's A/B drops (issue #4).

Since issue #82 a course-1 drop lands across two seated course-0 bricks (running bond), where
aerial-assembly dropped onto one. Both reference cases are course-1 drops, so their outcomes and
settled poses now come from tests/fixtures/ab_drop_running_bond.json (written by
make_ab_running_bond_reference.py from this repo's rig); the params, configs, mass properties and
sampled errors are still checked against aerial-assembly's reference. Old -> new outcomes: gauss-seed0
4 seated -> 4 seated; exp3-mean-seed0 4 jammed -> 3 jammed and 1 wrong valley (drop 0) on macOS arm64.
Issue #98 (canonical convex cut, multiccd off): exp3-mean-seed0 on macOS arm64 -> 2 wrong valleys, then 2
jams, which Linux x86_64 already gave; gauss-seed0 unchanged.

The reference (tests/fixtures/ab_drop_reference.json) was recorded by
tests/fixtures/make_ab_drop_reference.py from aerial-assembly @ a649663 with the same params,
seed and config. All values are simulated, uncalibrated contact, not hardware rates.

Two tiers:

- On every platform, sampled errors, outcomes and floor contact must match exactly, and mass
  properties to 1e-3 relative (CadQuery's tessellation is platform dependent too). This is the issue's "same outcome for the same seed and config".
- On the reference platform (macOS arm64) mass properties and settled poses must match bit for bit
  (1e-6 or tighter). Elsewhere they are skipped: CadQuery's mass properties differ by platform at 1e-3,
  which moves the settled pose a little. Until issue #98 the convex cut also differed by platform;
  it is now canonical, and its pieces are checked on every platform.
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
_BOND = json.loads((Path(__file__).parent / "fixtures" / "ab_drop_running_bond.json").read_text(encoding="utf-8"))
# One set of course-1 outcomes for every platform since issue #98. From #82 to #98 they were pinned per
# platform, because the convex cut differed: macOS arm64 exp3-mean-seed0 was wrong valley, then three
# jams; Linux x86_64 was two wrong valleys, then two jams, which is now the outcome everywhere.
PLATFORM = f"{platform.system()} {platform.machine()}".replace("Darwin", "macOS")
RUNNING_BOND = {c["name"]: c["rows"] for c in _BOND.get("cases", [])}  # a missing case fails its test (KeyError)
PIECES = json.loads((Path(__file__).parent / "fixtures" / "ab_pieces_reference.json").read_text(encoding="utf-8"))
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
    assert len(rows) == len(case["rows"])
    for i, (row, ref) in enumerate(zip(rows, case["rows"])):
        assert {k: row[k] for k in SAMPLE_KEYS} == {k: ref[k] for k in SAMPLE_KEYS}, i
    got = [{"outcome": r["outcome"], "floor": bool(r["floor"])} for r in rows]
    pinned = [{"outcome": b["outcome"], "floor": b["floor"]} for b in RUNNING_BOND[case["name"]]]
    assert got == pinned, f"{PLATFORM} {case['name']}: got {got}"


@pytest.mark.parametrize("part", ["A", "B"])
def test_collision_pieces_are_the_canonical_cut(ab_bodies, part):
    # Since issue #98 the cut is canonical and the same on every platform: A 30 pieces, B 36. aerial-assembly's
    # reference has 36 and 36 (REFERENCE n_collision), from where its ring happened to start.
    assert len(ab_bodies[part].collision) == len(PIECES["parts"][part])
    assert REFERENCE["geometry"][part]["n_collision"] == 36


@exact_platform_only
@pytest.mark.parametrize("case", CASES, ids=lambda c: c["name"])
def test_settled_poses_match_reference_exactly(drops, case):
    for i, (row, ref) in enumerate(zip(drops[case["name"]], RUNNING_BOND[case["name"]])):
        assert row["err_mm"] == pytest.approx(ref["err_mm"], abs=1e-6), i
        assert row["angle_deg"] == pytest.approx(ref["angle_deg"], abs=1e-6), i
