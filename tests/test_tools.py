"""The loop's deterministic tools (#163): each gives the same answer as the path it replaces."""
import gzip
import json

import pytest

from brixzle.engine import probe as PB
from brixzle.engine import zscore as ZS
from brixzle.families import registry
from brixzle.tools import core as TC


def _drop(course, dx, outcome="seated", slice_="xz", cell=(0, 0), dy=0.0, clear=5.0):
    return {"slice": slice_, "course": course, "cell": list(cell), "dx_mm": dx, "dy_mm": dy, "clear_mm": clear,
            "pitch_deg": 0.0, "outcome": outcome, "err_mm": [0, 0, 0]}


def _map(tmp, name, drops):
    d = tmp / name
    d.mkdir()
    (d / "drops.json.gz").write_bytes(gzip.compress(json.dumps(drops).encode()))
    return d


@pytest.fixture(scope="module")
def ab():
    fam = registry.get("ab")
    return fam, fam.defaults


@pytest.fixture
def cache(tmp_path, monkeypatch):
    monkeypatch.setenv("BRIXZLE_DROP_CACHE", str(tmp_path / "drops.sqlite"))
    monkeypatch.setattr(TC, "STAGE_CACHE", tmp_path / "stages")


DROPS = [_drop(0, 0.0, cell=(0, 0)), _drop(0, 6.0, "jammed", cell=(6, 0)), _drop(1, -4.0, cell=(-4, 0))]


def test_redrop_matches_the_probe_and_reads_back_from_the_cache(ab, cache):
    fam, p = ab
    first = TC.redrop(fam, p, DROPS, workers=1)
    cfg = TC._ideal()
    poses = [PB._pose_entry(d["outcome"], d["course"], PB._map_sample(d, cfg)) for d in DROPS]
    probe = PB.run_probe(fam, p, poses, workers=1)
    assert [r["outcome"] for r in first["rows"]] == [r["outcome"] for r in probe["rows"]]
    assert first["cache"] == {"hits": 0, "misses": 3}
    again = TC.redrop(fam, p, DROPS, workers=2)
    assert again["cache"] == {"hits": 3, "misses": 0} and again["rows"] == first["rows"]
    assert first["changed"] == sum(1 for d, r in zip(DROPS, first["rows"]) if d["outcome"] != r["outcome"])


def test_contacts_name_what_holds_the_brick(ab, cache):
    fam, p = ab
    res = TC.contacts(fam, p, DROPS[:1], workers=1)
    (row,) = res["rows"]
    assert row["outcome"] == "seated" and row["contacts"]
    weight = sum(b.mass_g for b in [__import__("brixzle.families.builders", fromlist=["B"]).sim_bodies(fam, p, "A")]) * 9.81e-3
    assert sum(g["normal_N"] for g in row["contacts"]) > 0.5 * weight
    assert all({"piece", "other", "other_geom", "n", "normal_N", "normal_from_vertical_deg"} <= set(g) for g in row["contacts"])


def test_slices_count_each_slice_and_agree_with_z(tmp_path):
    drops = DROPS + [_drop(0, 2.0, "wrong_valley", slice_="xy", cell=(2, 1), dy=2.0)]
    m = _map(tmp_path, "m", drops)
    res = TC.slices(m)
    c0 = TC._ideal().error.clearance[0]
    z = ZS.z_from_drops(drops, TC.boxes(), c0)
    for name in TC.boxes():
        assert res["boxes"][name]["all"]["Z"] == pytest.approx(z[name]["Z"])
    assert res["boxes"]["mean"]["xz/c0"]["N"] == 2 and res["boxes"]["mean"]["xy/c0"]["wrong_valley"] == 1


def test_mapdiff_names_every_changed_cell(tmp_path):
    a = _map(tmp_path, "a", DROPS)
    b = _map(tmp_path, "b", [DROPS[0], dict(DROPS[1], outcome="seated"), DROPS[2]])
    res = TC.mapdiff(a, b)
    assert res["cells_compared"] == 3 and res["changed"] == 1 and res["transitions"] == {"jammed->seated": 1}
    assert res["boxes"]["mean"]["dZ"] == pytest.approx(2 / 3)
    same = TC.mapdiff(a, a)
    assert same["changed"] == 0 and same["boxes"]["mean"]["verdict"] == "same"


def test_diffdesign_lists_the_changes_between_two_branches():
    res = TC.diffdesign(registry.get("fable_twinhook4"), registry.get("fable_gentle"))
    assert {"H0", "D", "side_gap", "snug_L", "y_angle", "wall"} == set(res["params_changed"])
    assert "collision:A" in res["items_changed"] and not res["sim_geometry_same"]
    same = TC.diffdesign(registry.get("fable_gentle"), registry.get("fable_gentle"))
    assert same["params_changed"] == {} and same["items_changed"] == [] and same["sim_geometry_same"]


def test_sweep_scores_each_value_and_reports_a_refused_one(ab, cache):
    fam, p = ab
    res = TC.sweep(fam, "U", [p.U, -1.0], DROPS[:2], workers=1)
    ok, bad = res["rows"]
    assert ok["feasible"] and ok["all"]["N"] == 2 and "mean" in ok["boxes"]
    assert not bad["feasible"] and bad["reason"]


def test_a_stage_is_run_once_per_geometry(ab, cache):
    fam, p = ab
    calls = []

    def run():
        calls.append(1)
        return {"status": "complete", "run": "runs/x"}

    first = TC.cached_stage(fam, p, "ring", {"seed": 0}, run)
    second = TC.cached_stage(fam, p, "ring", {"seed": 0}, run)
    other = TC.cached_stage(fam, p, "ring", {"seed": 1}, run)
    assert len(calls) == 2 and "cached_from" not in first and second["cached_from"] == "runs/x"
    assert other["status"] == "complete"


def test_evaluate_runs_only_the_stages_asked_and_reads_the_gate_back(tmp_path, cache, monkeypatch):
    calls = []

    def fake_gate(fam, p, seed=0, workers=None, runs="runs", cache=True):
        def run():
            calls.append(1)
            return {"passed": True, "overhang_n": 9, "overhang_floor": 3, "builds": {}, "seconds": 1.0,
                    "repeats": 5, "run": "runs/gate-x"}
        return TC.cached_stage(fam, p, "gate", {"seed": seed}, run) if cache else run()

    monkeypatch.setattr(TC, "gate", fake_gate)
    a = TC.evaluate("ab", tmp_path / "a", stages=("gate",))
    b = TC.evaluate("ab", tmp_path / "b", stages=("gate",))
    assert len(calls) == 1 and set(a["seconds"]) == {"gate"} and b["gate"]["cached_from"] == "runs/gate-x"
    assert json.loads((tmp_path / "b" / "eval.json").read_text())["gate"]["overhang_n"] == 9
    with pytest.raises(ValueError):
        TC.evaluate("ab", tmp_path / "c", stages=("nope",))


def test_the_command_line_runs_a_tool_and_writes_its_result(tmp_path, cache, capsys):
    from brixzle.tools import __main__ as CLI
    m = _map(tmp_path, "m", DROPS)
    assert CLI.main(["--runs", str(tmp_path / "runs"), "slices", "--map", str(m)]) == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed["boxes"]["mean"]["all"]["N"] == 3
    (written,) = (tmp_path / "runs").glob("slices-*/result.json")
    assert json.loads(written.read_text())["boxes"] == printed["boxes"]
    assert CLI.main(["--runs", str(tmp_path / "runs"), "redrop", "ab", "--map", str(m), "--outcomes", "jammed",
                     "--workers", "1"]) == 0
    assert json.loads(capsys.readouterr().out)["drops"] == 1
