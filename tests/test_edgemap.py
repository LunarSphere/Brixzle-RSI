from dataclasses import replace
import json

from brixzle.engine import box_search as BS
from brixzle.engine import drop_cache as DC
from brixzle.engine.sim import trials as T
from brixzle.families import registry
from brixzle.tools import __main__ as CLI
from brixzle.tools import core as TC


def test_edge_grid_budget_and_height_coverage():
    grid = TC.edge_grid()
    assert {case: len(poses) for case, poses in grid.items()} == {
        "plus_bare": 87, "minus_bare": 87, "plus_anchored": 13, "plus_cantilever": 13,
    }
    bare = grid["plus_bare"]
    assert bare == grid["minus_bare"]
    assert len(set(bare)) == 87
    assert {height for _, _, height in bare} == {"low", "nominal", "high"}
    assert (3, -3, "nominal") in bare
    assert (0.5, 0, "nominal") in bare
    assert (0, -6, "nominal") in bare


def test_edge_layout_has_one_supported_and_one_exposed_voxel():
    family = registry.get("ab")
    lat = family.lattice(family.defaults)
    for case in T.EDGE_CASES:
        placed, target, base = T.edge_layout(lat, case)
        assert target.course == placed[0].course + 1
        assert abs(target.i0 - placed[0].i0) == 1
        assert len(base) >= 2


def test_edge_poses_feed_contacts_and_tipped_jams_still_count(tmp_path, monkeypatch):
    observed = []

    class FakePool:
        def __init__(self, *args, **kwargs):
            self.cache_stats = {"hits": 0, "misses": 0}

        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def imap(self, jobs):
            for case, sample, opts in jobs:
                observed.append((case, sample, opts))
                yield {"outcome": "jammed", "settled": True, "angle_deg": 11.0,
                       "err_mm": [0, 0, 0], "release_world_offset_mm": [0, 0, 0],
                       "placed_moved_mm": 0, "rest": {"pos_mm": [1, 2, 3], "contacts": []}}

    monkeypatch.setattr(BS, "DropPool", FakePool)
    monkeypatch.setattr(TC, "cache_on", lambda: None)
    assert CLI.main(["--runs", str(tmp_path), "edgemap", "ab", "--workers", "1"]) == 0
    run = next(tmp_path.glob("edgemap-*"))
    report = json.loads((run / "result.json").read_text())
    assert report["cases"]["plus_bare"]["Z"] == -1
    assert report["cases"]["plus_bare"]["tipped"] == 87
    observed.clear()
    assert CLI.main(["--runs", str(tmp_path), "contacts", "ab", "--map", str(run),
                     "--edge-case", "minus_bare", "--limit", "1"]) == 0
    assert len(observed) == 1
    case, sample, opts = observed[0]
    assert case == "minus_bare" and opts["rest"]
    result = json.loads((next(tmp_path.glob("contacts-*")) / "result.json").read_text())
    row = result["rows"][0]
    original = next(r for r in report["rows"] if r["cell"] == row["cell"])
    assert row["edge_case"] == case and original["sample"] == sample
    assert row["rest_pos_mm"] == [1, 2, 3]


def test_edge_layout_is_part_of_the_cache_key(monkeypatch):
    family = registry.get("ab")
    previous = DC.geometry_key(family, family.defaults)
    original = T.edge_layout

    def revised(lat, case):
        placed, target, base = original(lat, case)
        if case == "minus_bare":
            target = replace(target, direction=-target.direction)
        return placed, target, base

    monkeypatch.setattr(T, "edge_layout", revised)
    assert previous != DC.geometry_key(family, family.defaults)
