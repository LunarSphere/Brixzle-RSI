"""Predictor-vs-sim validation (issue #11): a shared X, Z, theta grid, agreement and a disagreement map.

Sim results are simulated, uncalibrated contact; predictor results are predicted, quasi-static 2D.
"""

import json

import numpy as np
import pytest

from brixzle.engine import boxes as BX
from brixzle.engine import validate as V
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.ab.params import V0


# --- pure parts (no sim) ---


def test_grid_spans_the_unit_box_with_the_centre_on_it():
    g = V.grid_units(nx=5, nz=3, nt=3)
    assert g.shape == (5 * 3 * 3, 3)
    assert np.all(np.abs(g) <= 1)
    for axis, n in enumerate((5, 3, 3)):
        assert sorted(set(g[:, axis])) == pytest.approx(np.linspace(-1, 1, n))
    assert any(np.array_equal(u, [0, 0, 0]) for u in g)


def test_grid_rejects_bad_counts():
    with pytest.raises(ValueError):
        V.grid_units(nx=0, nz=3, nt=3)
    with pytest.raises(ValueError):
        V.grid_units(nx=4.5, nz=3, nt=3)


def test_one_point_axes_sit_at_zero():
    assert np.array_equal(V.grid_units(1, 1, 1), [[0.0, 0.0, 0.0]])


@pytest.mark.parametrize("sim, pred, cell", [
    ("seated", "seated", "agree_seated"),
    ("jammed", "wrong_valley", "agree_failed"),
    ("seated", "jammed", "conservative"),
    ("fell", "seated", "optimistic"),
])
def test_cell_kinds(sim, pred, cell):
    assert V.cell(sim, pred) == cell


def _row(course, x, z, a, sim, pred, pred_mu=None):
    return {"course": course, "unit": [x, z, a], "dx_mm": 10 * x, "clear_mm": 5 + z, "pitch_deg": a,
            "sim": sim, "pred": pred, "pred_sim_mu": pred_mu or pred}


def test_summary_counts_agreement_and_a_confusion_matrix():
    rows = [_row("0", -1, 0, 0, "seated", "seated"), _row("0", 0, 0, 0, "seated", "jammed"),
            _row("0", 1, 0, 0, "fell", "seated"), _row("0", 1, 1, 0, "jammed", "wrong_valley")]
    s = V.summarize(rows)
    c = s["courses"]["0"]["every_mu"]
    assert c["n"] == 4
    assert c["agreement"] == pytest.approx(0.5)
    assert c["cells"] == {"agree_seated": 1, "agree_failed": 1, "conservative": 1, "optimistic": 1}
    assert c["class_agreement"] == pytest.approx(0.25)  # only seated/seated has the same class
    assert s["overall"]["every_mu"]["n"] == 4


def test_indeterminate_counts_as_a_jam_for_the_class():
    s = V.summarize([_row("0", 0, 0, 0, "jammed", "indeterminate")])
    assert s["courses"]["0"]["every_mu"]["class_agreement"] == 1.0


def test_trusted_x_band_is_the_widest_band_about_the_aim_with_full_agreement():
    # agreement at |x| <= 0.5 for every (z, theta); a miss at x = +1
    rows = [_row("0", x, z, 0, "seated" if abs(x) < 1 else "jammed", "seated")
            for x in (-1, -0.5, 0, 0.5, 1) for z in (-1, 1)]
    band = V.summarize(rows)["courses"]["0"]["every_mu"]["trusted_unit_x"]
    assert band == pytest.approx(0.5)


def test_map_marks_each_cell():
    rows = [_row("0", x, 0, 0, s, p) for x, s, p in
            ((-1, "seated", "seated"), (0, "jammed", "jammed"), (1, "seated", "jammed"))]
    rows.append(_row("0", 2, 0, 0, "jammed", "seated"))
    text = V.ascii_map(rows, "0", "pred")
    assert "S.-+" in text


def test_grid_brackets_each_rows_capture_edge_on_both_sides():
    # seated for |x| <= 0.5 on the right, only the centre on the left
    seated = {-1: False, -0.5: False, 0: True, 0.5: True, 1: False}
    rows = [_row("0", x, 0, 0, "seated" if s else "jammed", "seated") for x, s in seated.items()]
    brackets = {(b["side"], b["model"]): (b["lo"], b["hi"]) for b in V.edge_brackets(rows)}
    assert brackets[(1, "sim")] == (0.5, 1)
    assert brackets[(-1, "sim")] == (0, 0.5)
    assert brackets[(1, "pred")] == (1, None)  # seated to the end of the grid: no edge inside it
    assert {b["course"] for b in V.edge_brackets(rows)} == {"0"}


def test_a_row_whose_centre_fails_has_no_edge():
    rows = [_row("0", x, 0, 0, "jammed", "seated") for x in (-1, 0, 1)]
    sim = [b for b in V.edge_brackets(rows) if b["model"] == "sim"]
    assert all(b["lo"] is None for b in sim)


def test_edge_bisection_closes_on_the_threshold():
    t = 0.37
    lo, hi = V.bisect_edge(lambda u: u <= t, 0.25, 0.5, steps=6)
    assert lo <= t < hi and hi - lo == pytest.approx(0.25 / 64)


def test_summary_is_json_safe():
    rows = [_row("1", 0, 0, 0, "seated", "seated")]
    json.dumps(V.summarize(rows), allow_nan=False)


# --- sim and predictor (A/B V0) ---


@pytest.fixture(scope="module")
def small(ab_bodies):
    box = BX.default_boxes()["placement"]["max"]
    return V.validate(AB, V0, box, nx=3, nz=1, nt=1, courses=(0,), workers=1, edge_steps=3)


def test_aligned_and_far_poses_are_compared_on_both_models(small):
    rows = small["rows"]
    assert len(rows) == 3
    centre = next(r for r in rows if r["unit"] == [0.0, 0.0, 0.0])
    assert centre["sim"] == "seated" and centre["pred"] == "seated"
    for r in rows:
        assert {"sim", "pred", "pred_sim_mu", "per_mu", "per_mu_reason", "sample", "dx_mm", "sim_err_mm"} <= set(r)


def test_capture_edges_are_refined_for_each_model(small):
    # A/B seats at the aim and not at +/-38.5 mm, so each side has an edge for every model
    edges = small["edges"]
    assert {(e["side"], e["model"]) for e in edges} == {(s, m) for s in (-1, 1) for m in V.MODELS}
    for e in edges:
        assert 0 <= e["lo"] < e["hi"] <= 1
        assert e["hi"] - e["lo"] == pytest.approx(1 / 8)
        assert e["lo_mm"] <= e["edge_mm"] <= e["hi_mm"]
    diffs = small["summary"]["courses"]["0"]["edges"]
    assert {d["verdict"] for d in diffs} == set(V.VERDICTS)


def test_validation_records_labels_inputs_and_timing(small):
    assert small["labels"] == {"sim": "simulated, uncalibrated contact", "pred": "predicted, quasi-static 2D"}
    assert small["sim_mu"] == pytest.approx(0.35)
    assert small["grid"] == {"nx": 3, "nz": 1, "nt": 1}
    assert small["seconds"] > 0
    json.dumps(small, allow_nan=False)


def test_cli_writes_a_new_run_with_a_report(tmp_path, ab_bodies):
    args = ["--nx", "1", "--nz", "1", "--nt", "1", "--courses", "0", "--workers", "1", "--runs", str(tmp_path)]
    V.main(args)
    V.main(args)
    runs = sorted(tmp_path.iterdir())
    assert len(runs) == 2  # never overwrites
    for run in runs:
        json.loads((run / "result.json").read_text())
        assert "predicted, quasi-static 2D" in (run / "report.md").read_text()
