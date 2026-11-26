"""The release model is hover and drop (#218): every score releases the brick in a free drop from the
top of the drone's hover wobble."""
import pytest

from brixzle.engine import gridscore as GS
from brixzle.engine import radius as RA
from brixzle.engine import structure_gate as SG


def test_every_score_releases_with_a_free_drop():
    assert GS.GridSettings().held is False
    assert RA.BuildSettings().held is False
    assert SG.MVP.held is False


def test_the_grid_drops_from_the_top_of_the_hover_wobble():
    from brixzle.engine import boxes as BX
    from brixzle.engine.sim import trials as T

    s = GS.GridSettings()
    assert s.z_frac == 1.0
    half = BX.default_boxes()["placement"]["mean"].half_widths["z_mm"]
    # 5 mm clearance above the highest-reach release height, plus the box's full height span
    assert T.ErrorModel().clearance[0] + s.z_frac * 2 * half == pytest.approx(11.21, abs=0.01)
