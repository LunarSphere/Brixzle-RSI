"""The staged search over the prim family (#166)."""
import json

import pytest

from brixzle.families import registry
from brixzle.search import core as SC


def test_prim_is_a_registered_family_that_passes_the_contract():
    from brixzle.families import contract as C
    fam = registry.get("prim")
    assert C.check_family(fam) == [] and fam.ring is not None


def test_sampling_is_seeded_and_inside_the_bounds():
    s = SC.SearchSettings(n=64, seed=3)
    a, b = SC.sample(s), SC.sample(s)
    assert a == b and len(a) == 64
    bounds = registry.get("prim").bounds
    assert all(bounds[k][0] <= c[k] <= bounds[k][1] for c in a for k in bounds)


def test_the_rules_stage_is_fast_and_names_its_reason():
    s = SC.SearchSettings()
    ok, why, m = SC.stage_rules({"U": 30.0, "end_gap": 0.3})
    assert ok and m["length_mm"] == 60.0
    assert SC.stage_rules({"U": 38.0, "end_gap": 0.3})[1] == "length"       # 76 mm > 70
    assert SC.stage_rules({"ramp_deg": 20.0, "end_gap": 0.3})[1].startswith("invalid")
    import time
    cands = [{"id": str(k), "params": p} for k, p in enumerate(SC.sample(SC.SearchSettings(n=256)))]
    t = time.time()
    passed, rep = SC.run_stage("rules", cands, s)
    assert rep["evals_per_s"] > 500 and rep["in"] == 256 and 0 < rep["passed"] < 256


def test_the_predictor_and_drop_stages_run_on_the_defaults():
    p = {"end_gap": 0.3}
    ok, why, m = SC.stage_predict(p)
    assert m["release_mm"] > 0 and (ok or why)
    ok, why, m = SC.stage_drops(p, workers=1)
    assert m["N"] == len(SC.key_poses()) and -1 <= m["drops_score"] <= 1


def test_a_candidate_package_loads_as_a_family(tmp_path):
    spec = SC.write_package(tmp_path, "c00007", {"U": 28.0, "end_gap": 0.3})
    from brixzle.tools import core as TC
    fam = TC.load(spec)
    assert fam.name == "c00007" and fam.defaults.U == 28.0 and fam.ring is not None
