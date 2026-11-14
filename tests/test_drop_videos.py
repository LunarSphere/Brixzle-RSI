"""Drop videos and STL in the results (issue #77).

The pick, the poses, the captions and the STL need no rendering. One test records and renders a
real drop; it is skipped where MuJoCo cannot open an offscreen GL context (a headless CI runner).
Sim results are simulated, uncalibrated contact.
"""

import shutil

import numpy as np
import pytest
import trimesh

from brixzle.engine.sim import trials as T
from brixzle.loop import videos as V

C0 = 5.0
CFG = T.TrialConfig(error=T.ErrorModel(ideal=True))


def _drops(n=30):
    out = []
    for k in range(n):
        out.append({"slice": ("xz", "xt", "zt")[k % 3], "course": k % 2, "cell": [k - 15, 0], "dx_mm": float(k - 15),
                    "clear_mm": C0 + 2.0, "pitch_deg": 0.0, "outcome": "seated" if k % 4 else "jammed"})
    out.append({**out[0], "zoom": "a closer look", "dx_mm": 99.0})
    return out


def test_five_drops_are_picked_at_random_repeatably_and_never_from_a_zoom():
    a = V.pick_drops(_drops(), n=5, seed=3, clearance0=C0)
    assert len(a) == 5 and [p["k"] for p in a] == [1, 2, 3, 4, 5]
    assert a == V.pick_drops(_drops(), n=5, seed=3, clearance0=C0)
    assert a != V.pick_drops(_drops(), n=5, seed=4, clearance0=C0)
    assert len({(p["slice"], p["course"], tuple(p["cell"])) for p in a}) == 5
    assert all(p["pose"]["x"] != 99.0 for p in a)
    assert a[0]["pose"] == {"x": a[0]["dx_mm"], "y": 0.0, "z": a[0]["clear_mm"] - C0, "t": a[0]["pitch_deg"]}


def test_with_fewer_drops_than_asked_all_are_picked():
    assert len(V.pick_drops(_drops(3), n=5, seed=0, clearance0=C0)) == 3


def test_a_picked_drop_becomes_the_same_trial_sample():
    p = V.pick_drops(_drops(), n=1, seed=0, clearance0=C0)[0]
    s = V.drop_sample(p, CFG)
    assert s["dx"] - s["aim"] == pytest.approx(p["dx_mm"]) and s["clear"] == p["clear_mm"]
    assert s["pitch"] == p["pitch_deg"] and s["dy"] == s["yaw"] == s["roll"] == 0.0


def _xy_drop():
    return {"slice": "xy", "course": 0, "cell": [2, -3], "dx_mm": 2.0, "dy_mm": -3.0, "clear_mm": C0,
            "pitch_deg": 0.0, "outcome": "jammed"}


def test_an_xy_drop_keeps_its_y_offset_and_is_replayed_there():
    # Issue #103: drops on the X-Y slice (#86) lost their Y offset and crashed the jam map.
    p = V.pick_drops([_xy_drop()], n=1, seed=0, clearance0=C0)[0]
    assert p["dy_mm"] == -3.0 and p["pose"] == {"x": 2.0, "y": -3.0, "z": 0.0, "t": 0.0}
    assert V.drop_sample(p, CFG)["dy"] == -3.0
    assert "y -3.0 mm" in V.caption(p, "jammed")


def test_the_caption_names_the_pose_the_outcome_and_the_label():
    p = {"k": 2, "slice": "xt", "course": 1, "pose": {"x": -3.0, "z": 2.0, "t": 4.0}, "outcome": "jammed"}
    text = V.caption(p, "jammed")
    for want in ("drop 2", "xt", "course 1", "x -3.0 mm", "z 2.0 mm", "pitch +4.0", "jammed",
                 "simulated, uncalibrated contact"):
        assert want in text, want


def test_frames_become_a_video_file(tmp_path):
    images = [np.full((48, 64, 3), v, dtype=np.uint8) for v in range(0, 250, 25)]
    gif = V.write_video(images, tmp_path / "a.mp4", fps=30, use_ffmpeg=False)
    assert gif.suffix == ".gif" and gif.stat().st_size > 0
    if shutil.which("ffmpeg"):
        mp4 = V.write_video(images, tmp_path / "b.mp4", fps=30)
        assert mp4.suffix == ".mp4" and mp4.stat().st_size > 0


def test_the_stl_of_each_part_is_the_printed_solid(tmp_path):
    from brixzle.families import builders as B
    from brixzle.families.wedge.family import FAMILY

    paths = V.export_stl(FAMILY, FAMILY.defaults, tmp_path)
    assert [p.name for p in paths] == ["wedge-A.stl"]
    mesh = trimesh.load(paths[0])
    assert mesh.is_watertight
    assert mesh.volume == pytest.approx(B.sim_bodies(FAMILY, FAMILY.defaults, "A").mesh.volume, rel=1e-6)


def _can_render():
    try:
        import mujoco

        m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="box" size=".1 .1 .1"/></worldbody></mujoco>')
        with mujoco.Renderer(m, 32, 32) as r:
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            r.update_scene(d)
            r.render()
        return True
    except Exception:  # noqa: BLE001 - no GL context here
        return False


@pytest.mark.skipif(not _can_render(), reason="MuJoCo cannot open an offscreen GL context here")
def test_a_real_drop_is_recorded_and_rendered(tmp_path):
    from brixzle.families.wedge.family import FAMILY

    pick = {"k": 1, "slice": "xz", "course": 0, "cell": [0, 0], "dx_mm": 0.0, "clear_mm": C0, "pitch_deg": 0.0,
            "outcome": "seated", "pose": {"x": 0.0, "z": 0.0, "t": 0.0}}
    out = V.record_and_render(FAMILY, FAMILY.defaults, pick, tmp_path / "drop-1.mp4", CFG, size=(160, 120))
    assert out["rerun_outcome"] == "seated" and out["frames"] > 10
    assert (tmp_path / out["video"]).stat().st_size > 0
