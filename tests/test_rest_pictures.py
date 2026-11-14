"""Rest pictures and contact tables for the critic (issue #112). The pick and the contact table need
no rendering; one test renders, and is skipped where MuJoCo cannot open an offscreen GL context.
Simulated, uncalibrated contact."""

import pytest

from brixzle.engine import boxes as BX
from brixzle.families.ab.family import FAMILY as AB
from brixzle.loop import rest as RS

BOX = BX.default_boxes()["placement"]["mean"]


def _drops():
    out = []
    for k in range(40):
        sl = "xy" if k % 2 else "xz"
        out.append({"slice": sl, "course": k % 2, "cell": [k, 0], "outcome": ("seated", "jammed", "wrong_valley", "fell")[k % 4],
                    "dx_mm": float(k % 9 - 4), "dy_mm": float(k % 7 - 3) if sl == "xy" else 0.0, "clear_mm": 5.0,
                    "pitch_deg": 0.0})
    return out


def test_the_picks_are_failures_inside_the_box_with_a_jam_a_wrong_valley_and_an_x_y_drop():
    picks = RS.pick_rests(_drops(), BOX, n=4, seed=3)
    assert 3 <= len(picks) <= 4 and all(p["outcome"] != "seated" for p in picks)
    outcomes = {p["outcome"] for p in picks}
    assert {"jammed", "wrong_valley"} <= outcomes and any(p["slice"] == "xy" for p in picks)
    assert RS.pick_rests(_drops(), BOX, n=4, seed=3) == picks


def test_a_rest_has_a_contact_table_naming_the_faces_on_both_bricks():
    pick = {"k": 1, "slice": "xz", "course": 1, "outcome": "jammed", "dx_mm": 10.0, "dy_mm": 0.0, "clear_mm": 5.0,
            "pitch_deg": 0.0, "pose": {"x": 10.0, "y": 0.0, "z": 0.0, "t": 0.0}}
    out = RS.rest_of(AB, AB.defaults, pick)
    assert out["contacts"] and {"brick", "on_brick", "on_support", "normal"} <= set(out["contacts"][0])
    assert out["rest_pose"]["pos_mm"] and len(out["rest_pose"]["quat"]) == 4


def _can_render():
    try:
        import mujoco
        m = mujoco.MjModel.from_xml_string("<mujoco><worldbody><geom size='1'/></worldbody></mujoco>")
        with mujoco.Renderer(m, 8, 8):
            return True
    except Exception:  # noqa: BLE001
        return False


@pytest.mark.skipif(not _can_render(), reason="MuJoCo cannot open an offscreen GL context here")
def test_rest_pictures_are_written_side_on_and_end_on(tmp_path):
    out = RS.rest_pictures(AB, AB.defaults, _drops(), BOX, tmp_path, n=3, seed=0)
    assert out["drops"] and out["seconds"] >= 0
    for d in out["drops"]:
        assert all((tmp_path / f).exists() for f in d["pictures"]) and len(d["pictures"]) == 2
    assert (tmp_path / "rest.json").exists()
