"""Per-iteration renders (issue #15): jam map, module, progress and summaries.

They draw from small made-up maps, so no sim runs (one test builds the wedge's solid for the 3D
view). Each test checks that a PNG was written and what text the figure carries.
"""

import pytest

from brixzle.engine import boxes as BX
from brixzle.loop import render as RD

BOXES = BX.default_boxes()["placement"]
GRID = {"slice": "xz", "course": 0, "axes": ["x", "z"], "step": {"x": 1.0, "z": 5.0},
        "index_extent": {"x": [-3, 3], "z": [0, 1]}, "extent": {"x": [-3.0, 3.0], "z": [0.0, 5.0]},
        "rows": ["WSSJSSF", "?SSSSSJ"], "fixed": {"t": 0.0}}
MAP = {"family": "toy", "box": BOXES["mean"].to_dict(), "cover_boxes": [b.to_dict() for b in BOXES.values()],
       "grids": [GRID, {**GRID, "course": 1}, {**GRID, "slice": "xt", "axes": ["x", "t"], "step": {"x": 1.0, "t": 2.0},
                                                "index_extent": {"x": [-3, 3], "t": [-1, 0]}, "fixed": {"z": 3.1}}]}
Z = {"mean": {"Z": 0.5, "S": 9, "J": 2, "fell": 1, "wrong_valley": 1, "other": 1, "N": 14}}
JAM = {"bands": [{"slice": "xz", "course": 0, "extent": {"x": [0.0, 0.0], "z": [0.0, 0.0]}}]}


def _png(path):
    data = path.read_bytes()
    return data[:8] == b"\x89PNG\r\n\x1a\n" and len(data) > 1000


def test_the_jam_map_counts_every_outcome_and_names_z_and_the_label(tmp_path):
    out = RD.jam_map(MAP, Z, tmp_path / "jam_map.png", target="mean", jam=JAM)
    assert _png(out)
    text = RD.jam_map_text(MAP, Z, target="mean")
    assert "Z mean +0.500" in text and "simulated, uncalibrated contact" in text.lower()
    legend = RD.legend_counts(GRID)
    assert legend == {"seated": 9, "jammed": 2, "fell": 1, "wrong valley": 1, "other": 1}


def test_the_colours_follow_the_drop_grid_figure():
    assert RD.COLOURS["S"].startswith("#1") and RD.NAMES == {"S": "seated", "J": "jammed", "F": "fell",
                                                            "W": "wrong valley", "?": "other"}


def test_the_module_render_is_three_orthographic_views_per_part(tmp_path):
    """The maintainer asked for three orthographic views of the latest design (issue #74)."""
    from brixzle.families import builders as B
    from brixzle.families.ab.family import FAMILY as AB
    from brixzle.families.ab.params import V0

    panels = RD.ortho_panels(AB, V0)
    assert [(q["part"], q["view"]) for q in panels] == [(part, v) for part in ("A", "B")
                                                        for v in ("front", "side", "top")]
    assert [q["axes"] for q in panels[:3]] == [("x", "z"), ("y", "z"), ("x", "y")]
    lo, hi = B.sim_bodies(AB, V0, "A").mesh.bounds
    idx = {"x": 0, "y": 1, "z": 2}
    for q in panels[:3]:
        for k, a in enumerate(q["axes"]):
            assert q["extent"][k] == pytest.approx((lo[idx[a]], hi[idx[a]]), abs=1e-6)
    assert _png(RD.module_png(AB, V0, tmp_path / "module.png"))


def test_progress_marks_kept_and_rejected_designs(tmp_path):
    history = [{"iteration": 1, "decision": "kept", "z": {"mean": Z["mean"]}},
               {"iteration": 2, "decision": "rejected", "z": {"mean": {**Z["mean"], "Z": 0.2}}},
               {"iteration": 3, "decision": "no_design", "z": None}]
    assert _png(RD.progress_png(history, tmp_path / "progress.png", "mean"))


def test_the_iteration_summary_names_z_the_brief_the_decision_and_every_run(tmp_path):
    rec = {"iteration": 2, "dir": str(tmp_path / "runs/loop/r/iter-02"), "family": "zig", "module": "rsi_r_i02_a1",
           "decision": "rejected", "reason": "Z on the target did not improve", "z": Z,
           "held_out": {"seed 1, friction 0.2": Z["mean"]}, "note": {"summary": "longer ramps", "changes": "ramp 60 deg",
                                                                     "geometry": "Prismatic"},
           "brief_in": {"decision": "redesign", "reason": "jams", "clusters": [
               {"failure": "jam band", "diagnosis": "wedge", "rules": ["DR2"], "change": "longer", "scope": "reshape",
                "expected_effect": "fewer jams"}]},
           "attempts": [{"attempt": 1, "module": "rsi_r_i02_a1", "root": "runs/loop/r/iter-02/attempt-1/pkg",
                         "decision": "passed_quick_check",
                         "quick_check": {"seated": 36, "dropped": 40, "run": "runs/check-x"}}],
           "runs": {"quick_check": "runs/check-x", "map": "runs/map-x", "jam": "runs/map-x/jam_analysis.json"},
           "brief": None, "spent_usd": 0.12, "seconds": {"map": 60.0}}
    text = RD.iteration_summary(rec, "r", "mean", "runs/loop/r/api-spend.jsonl", 0.3, 5.0)
    for want in ("Decision: rejected", "Z on the target did not improve", "+0.500", "DR2", "longer ramps",
                 "runs/check-x", "runs/map-x/report.json", "runs/map-x/jam_analysis.json", "$0.1200",
                 "simulated, uncalibrated contact", "Held-out"):
        assert want in text, want


def test_the_run_summary_lists_every_iteration_and_the_stop(tmp_path):
    state = {"run": "r", "iterations": [{"iteration": 1, "family": "zig", "decision": "kept", "z": Z}],
             "best": {"family": "zig", "iteration": 1, "z_target": 0.5, "root": "a", "module": "b", "map": "m"},
             "stop": "2 iterations run", "tune": None, "elapsed_s": 300.0}
    text = RD.run_summary(state, "mean", "spend.jsonl", 0.3, 5.0)
    assert "| 1 | zig | kept | +0.500 |" in text and "2 iterations run" in text and "$0.3000 of $5.00" in text
