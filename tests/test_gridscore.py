"""The grid self-alignment score (#208): the coarse grid, refinement, the green zone, ring Z and the maps."""
import math

import pytest

from brixzle.engine import gridscore as GS

FINE = GS.GridSettings().fine_mm


def _search(outcome_at, **kw):
    """grid search with a synthetic outcome per (placement, x, y) in place of the sim."""
    settings = GS.GridSettings(**kw)
    return GS.search(lambda pts: [outcome_at(p, x, y) for p, x, y in pts], settings)


def test_the_coarse_grid_covers_the_disc_and_refines_onto_one_lattice():
    s = GS.GridSettings()
    assert s.coarse_mm == pytest.approx(3 * s.fine_mm)
    cells = GS.coarse_cells(s)
    assert (0, 0, 3) in cells
    assert all(i % 3 == 0 and j % 3 == 0 and size == 3 for i, j, size in cells)
    # every point of the 40 mm disc is in some coarse cell, and no cell lies wholly outside it
    for x, y in [(40, 0), (0, -40), (28.2, 28.2), (-39.9, 1.0)]:
        assert any(GS.contains(c, x, y, s) for c in cells), (x, y)
    assert all(GS.distance_mm(c, s) <= s.max_mm for c in cells)
    # a coarse cell splits into the nine fine cells around its own centre (its drop is reused)
    kids = GS.children((3, -6, 3))
    assert len(kids) == 9 and (3, -6, 1) in kids


def test_everything_seating_gives_the_full_radius_and_no_refinement():
    res = _search(lambda p, x, y: "seated", placements=("full",))
    full = res["placements"]["full"]
    assert full["green_mm"] == GS.GridSettings().max_mm
    assert full["drops"] == len(GS.coarse_cells(GS.GridSettings()))
    assert full["rings"]["16"]["Z"] == 1.0 and full["rings"]["40"]["N"] > 0


def test_a_ring_of_jams_is_found_to_the_fine_cell():
    res = _search(lambda p, x, y: "jammed" if math.hypot(x, y) >= 20.0 else "seated", placements=("full",))
    full = res["placements"]["full"]
    # the first failing fine cell's near edge lies within one fine cell of 20 mm
    assert 20.0 - FINE <= full["green_mm"] <= 20.0
    # refined only along the boundary and the failures, not the seated middle
    sizes = {c["size_mm"] for c in full["cells"] if math.hypot(c["x_mm"], c["y_mm"]) < 5}
    assert sizes == {GS.GridSettings().coarse_mm}
    assert full["drops"] < 4 * len(GS.coarse_cells(GS.GridSettings()))
    assert full["rings"]["16"]["Z"] == 1.0
    assert full["rings"]["40"]["Z"] < 0  # most of the 40 mm disc jams


def test_a_jam_pocket_next_to_a_coarse_seat_is_resolved():
    # a small pocket of jams off-centre: one coarse sample lands in it, its neighbours split
    pocket = lambda p, x, y: "jammed" if math.hypot(x - 15.0, y) <= 3.0 else "seated"
    res = _search(pocket, placements=("full",))
    full = res["placements"]["full"]
    assert 12.0 - FINE <= full["green_mm"] <= 12.0 + 1e-9
    assert any(c["outcome"] == "jammed" and c["size_mm"] == FINE for c in full["cells"])


def test_the_weakest_placement_sets_the_headline_and_stops_early():
    def outcome(p, x, y):
        if p == "corbel_reach":
            return "fell" if x > 6.0 else "seated"
        return "seated"

    res = _search(outcome, placements=("full", "corbel_reach"))
    assert res["weakest"] == "corbel_reach"
    assert res["green_mm"] == res["placements"]["corbel_reach"]["green_mm"] <= 6.0 + FINE / 2
    assert res["order"][0] == "corbel_reach"
    cr = res["placements"]["corbel_reach"]
    assert cr["early_stop"] is True
    # past the early stop, failing coarse cells far out are left coarse
    assert any(c["size_mm"] == GS.GridSettings().coarse_mm and c["outcome"] == "fell" for c in cr["cells"])
    assert res["floor_passed"] is False


def test_ring_z_weights_cells_by_area_and_counts_jams_against():
    cells = {(0, 0, 3): "seated", (3, 0, 1): "jammed", (4, 0, 1): "wrong_valley"}
    z = GS.ring_z(cells, GS.GridSettings(fine_mm=2.5), (40.0,))["40"]
    assert z["N"] == 11 and z["S"] == 9 and z["J"] == 1
    assert z["Z"] == pytest.approx((9 - 1) / 11, abs=1e-4)


def test_tolerated_cells_do_not_bound_the_green_zone():
    s = GS.GridSettings()
    cells = {(0, 0, 3): "seated", (3, 0, 1): "jammed", (9, 0, 1): "jammed"}
    assert GS.green_radius(cells, s) == pytest.approx(2.5 * 2.5)
    assert GS.green_radius(cells, s, ignore={(3, 0, 1)}) == pytest.approx(8.5 * 2.5)


def test_settings_refuse_nonsense():
    with pytest.raises(ValueError):
        GS.GridSettings(fine_mm=0)
    with pytest.raises(ValueError):
        GS.GridSettings(placements=("sideways",))
    with pytest.raises(ValueError):
        GS.GridSettings(max_mm=-1)


def test_the_map_is_drawn_for_each_placement(tmp_path):
    res = _search(lambda p, x, y: "jammed" if x > 12 else "seated", placements=("full",))
    paths = GS.write_maps(res, tmp_path)
    assert [p.name for p in paths] == ["map-full.png"]
    assert paths[0].stat().st_size > 5000


# --- solver noise (maintainer, 2026-11-26): an isolated one-off does not bound the green zone ---


def _lone_jam(p, x, y):
    return "jammed" if (x, y) == (7.5, 0.0) else "seated"


def test_an_isolated_one_off_jam_is_tolerated_and_marked():
    checked = []

    def noise(pl, cells):
        checked.extend(cells)
        return {c: [{"mu": 0.2, "seed": 0, "outcome": "seated"}] * 3 for c in cells}

    s = GS.GridSettings(placements=("full",))
    res = GS.search(lambda pts: [_lone_jam(*q) for q in pts], s, noise_check=noise)
    full = res["placements"]["full"]
    assert checked == [(3, 0, 1)]
    assert full["green_mm"] == s.max_mm
    assert full["tolerated"] == [[3, 0, 1]]
    assert full["noise"]["3,0,1"]["repeatable"] is False


def test_a_repeatable_lone_jam_still_bounds_the_green_zone():
    s = GS.GridSettings(placements=("full",))
    noise = lambda pl, cells: {c: [{"mu": 0.2, "seed": k, "outcome": "jammed"} for k in range(3)] for c in cells}
    res = GS.search(lambda pts: [_lone_jam(*q) for q in pts], s, noise_check=noise)
    assert res["placements"]["full"]["green_mm"] == pytest.approx(6.25)
    assert res["placements"]["full"]["tolerated"] == []


def test_touching_failures_that_do_not_repeat_are_tolerated_too():
    # maintainer, 2026-11-26 (#218): only a repeatable failure ends the green zone, even when its
    # neighbours failed as well
    pair = lambda p, x, y: "jammed" if (x, y) in {(7.5, 0.0), (10.0, 0.0)} else "seated"
    checked = []

    def noise(pl, cells):
        checked.extend(cells)
        return {c: [{"mu": 0.2, "seed": 0, "outcome": "seated"}] for c in cells}

    res = GS.search(lambda pts: [pair(*q) for q in pts], GS.GridSettings(placements=("full",)), noise_check=noise)
    assert checked == [(3, 0, 1), (4, 0, 1)]
    assert res["placements"]["full"]["green_mm"] == 40.0
    assert res["placements"]["full"]["tolerated"] == [[3, 0, 1], [4, 0, 1]]


def test_touching_failures_that_repeat_still_end_the_green_zone():
    pair = lambda p, x, y: "jammed" if (x, y) in {(7.5, 0.0), (10.0, 0.0)} else "seated"
    noise = lambda pl, cells: {c: [{"mu": 0.2, "seed": k, "outcome": "jammed"} for k in range(3)] for c in cells}
    res = GS.search(lambda pts: [pair(*q) for q in pts], GS.GridSettings(placements=("full",)), noise_check=noise)
    assert res["placements"]["full"]["green_mm"] == pytest.approx(6.25)
    assert res["placements"]["full"]["tolerated"] == []


@pytest.mark.slow
@pytest.mark.parametrize("case", ["plus_bare", "plus_anchored"])
def test_jam_analysis_reduces_and_names_contacts_on_the_edge_layouts(case):
    from brixzle.engine import jam_analysis as JA
    from brixzle.engine import radius as RA
    from brixzle.engine import zscore as ZS
    from brixzle.families import registry

    fam = registry.get("ab")
    cfg = RA._ideal_cfg(True)
    out = JA.analyse_pose(fam, fam.defaults, case, ZS._sample(cfg, {"x": 0.0, "y": 0.0, "z": 3.0, "t": 0.0}), cfg)
    assert out["sites"], out["mechanism"]
    labels = [s["on_support"]["label"] for s in out["sites"]]
    assert all(label and "unknown" not in label for label in labels)
    assert any("(brick" in label for label in labels)  # named on a placed brick of the edge layout
