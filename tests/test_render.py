"""The module picture (issue #74): three orthographic views per part, to scale. The SDK loop's other
renders went with that loop (#204, #227)."""

import pytest

from brixzle.tools import render as RD


def _png(path):
    data = path.read_bytes()
    return data[:8] == b"\x89PNG\r\n\x1a\n" and len(data) > 1000


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
