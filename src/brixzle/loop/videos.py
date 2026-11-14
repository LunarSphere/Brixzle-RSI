# From LunarSphere/aerial-assembly brixzle/src/brixzle/render.py @ a649663
# Changed: the renderer (render) is kept and adapted (fixed side camera on the bricks, pulled back, a caption on every frame, slow motion with the last frame held, MP4 written by the ffmpeg binary or a GIF by Pillow, no imageio); picking drops from a map, re-recording them and exporting STL are new (issue #77); picks keep and replay the Y offset of X-Y slice drops (issue #103).
"""Drop videos and STL for the per-iteration results (issue #77).

Each iteration of the loop picks a few drops of its full sim map at random (seeded, so a run can be
repeated), drops each again with recording on, and writes ``drop-K.mp4`` with a caption giving the
pose and the outcome. The jam map marks each pick with its number K (``render.jam_map(picks=...)``),
and ``summary.md`` lists them. The design's printed solid goes beside them as one STL per part.

Videos are simulated, uncalibrated contact. A re-run drop gives the map's outcome, since a drop
depends only on its pose; ``rerun_outcome`` records it anyway.
"""

from pathlib import Path
import shutil
import subprocess

import mujoco
import numpy as np

from brixzle.engine.sim import trials as T

LABEL = "simulated, uncalibrated contact"
N_VIDEOS = 5
SIZE = (640, 480)   # width, height
RECORD_FPS = 240    # frames kept per second of sim time: a drop settles in about 0.3 s
FPS = 30            # playback, so videos run 8 times slower than the sim
HOLD_S = 0.75       # the last frame is held this long, to show where the brick came to rest


# --- which drops ---


def pick_drops(drops, n=N_VIDEOS, seed=0, clearance0=5.0):
    """``n`` distinct drops of a map (not zoom drops), picked at random with ``seed``, numbered
    1..n. Each pick keeps its slice, course, cell and outcome, and gets its pose about the aim:
    x and y (mm), z (release height above the minimum, mm) and t (pitch, deg). Y is the drop's
    ``dy_mm`` (0 off the X-Y slice, issue #103)."""
    pool = sorted((d for d in drops if not d.get("zoom")),
                  key=lambda d: (d["slice"], d["course"], tuple(d["cell"])))
    rng = np.random.default_rng(seed)
    idx = sorted(rng.choice(len(pool), size=min(n, len(pool)), replace=False).tolist()) if pool else []
    out = []
    for k, i in enumerate(idx, start=1):
        d = pool[i]
        out.append({"k": k, "slice": d["slice"], "course": d["course"], "cell": list(d["cell"]),
                    "dx_mm": float(d["dx_mm"]), "dy_mm": float(d.get("dy_mm", 0.0)), "clear_mm": float(d["clear_mm"]),
                    "pitch_deg": float(d["pitch_deg"]), "outcome": d["outcome"],
                    "pose": {"x": float(d["dx_mm"]), "y": float(d.get("dy_mm", 0.0)),
                             "z": float(d["clear_mm"]) - clearance0, "t": float(d["pitch_deg"])}})
    return out


def drop_sample(pick, cfg):
    """The trial sample (ErrorModel.sample keys) of a picked drop."""
    aim = cfg.error.aim_bias
    return {"dx": aim + pick["dx_mm"], "dy": float(pick.get("dy_mm", 0.0)), "aim": aim, "yaw": 0.0, "roll": 0.0, "pitch": pick["pitch_deg"],
            "clear": pick["clear_mm"], "vel": [0.0, 0.0, 0.0]}


def caption(pick, outcome):
    q = pick["pose"]
    return (f"drop {pick['k']}: {pick['slice']} slice, course {pick['course']}; x {q['x']:+.1f} mm, "
            f"y {q.get('y', 0.0):+.1f} mm, z {q['z']:.1f} mm, pitch {q['t']:+.1f} deg -> {outcome} ({LABEL})")


# --- rendering (from aerial-assembly) ---


def _camera(model, frames, active, azimuth=90):
    cam = mujoco.MjvCamera()
    data = mujoco.MjData(model)
    pts = []
    for q in frames[::max(1, len(frames) // 20)] + frames[-1:]:
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        pts += [data.xpos[b] for b in active]
    pts = np.array(pts)
    lo, hi = pts.min(0), pts.max(0)
    cam.lookat[:] = (lo + hi) / 2
    cam.distance = max(0.2, float(np.max(hi - lo)) * 2.6)
    cam.azimuth, cam.elevation = azimuth, -12  # 90: looking along +Y at the X-Z profile; 0: end-on, along X
    return cam


def render_frames(model, frames, size=SIZE, text=None, azimuth=90):
    """RGB images of recorded qpos frames, from the side (``azimuth`` 90) or end-on (0), with
    ``text`` drawn on each."""
    from PIL import Image, ImageDraw

    width, height = size
    data = mujoco.MjData(model)
    data.qpos[:] = frames[-1]
    mujoco.mj_kinematics(model, data)
    active = [b for b in range(1, model.nbody) if data.xpos[b][2] < 1.5]
    cam = _camera(model, frames, active, azimuth)
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]
    images = []
    with mujoco.Renderer(model, height, width) as r:
        for q in frames:
            data.qpos[:] = q
            mujoco.mj_forward(model, data)
            r.update_scene(data, cam, opt)
            img = r.render()
            if text:
                im = Image.fromarray(img)
                draw = ImageDraw.Draw(im)
                draw.rectangle((0, 0, width, 18), fill=(0, 0, 0))
                draw.text((4, 3), text, fill=(255, 255, 255))
                img = np.asarray(im)
            images.append(img)
    return images


def write_video(images, out, fps=FPS, use_ffmpeg=True):
    """Write RGB images to ``out``: an MP4 through the ffmpeg binary, or, without it, a GIF next
    to it (Pillow). Returns the path written."""
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    height, width = images[0].shape[:2]
    if use_ffmpeg and shutil.which("ffmpeg"):
        cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
               "-r", str(fps), "-i", "-", "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-pix_fmt", "yuv420p",
               "-c:v", "libx264", str(out.with_suffix(".mp4"))]
        subprocess.run(cmd, input=b"".join(np.ascontiguousarray(i, dtype=np.uint8).tobytes() for i in images),
                       check=True, capture_output=True)
        return out.with_suffix(".mp4")
    from PIL import Image

    gif = out.with_suffix(".gif")
    frames = [Image.fromarray(i) for i in images[::2]]  # half the frames keep a GIF small
    frames[0].save(gif, save_all=True, append_images=frames[1:], duration=int(2000 / fps), loop=0)
    return gif


def record_and_render(family, p, pick, out, cfg=None, size=SIZE):
    """Drop a picked pose again with recording on and write its video; returns what was written."""
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    rig = T.DropRig(family, p, cfg, course=pick["course"], record=True, record_fps=RECORD_FPS)
    row = rig.drop(drop_sample(pick, cfg), trace=False)
    images = render_frames(rig.model, row["frames"], size=size, text=caption(pick, row["outcome"]))
    images += [images[-1]] * int(HOLD_S * FPS)
    path = write_video(images, out, fps=FPS)
    return {"video": path.name, "rerun_outcome": row["outcome"], "frames": len(row["frames"])}


def drop_videos(family, p, sim_map, drops, out_dir, n=N_VIDEOS, seed=0, cfg=None):
    """Pick ``n`` drops of a map, record and render each to ``out_dir/drop-K.mp4``; returns the
    picks with their video names. A drop that cannot be rendered keeps its pick with the error."""
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    picks = pick_drops(drops, n=n, seed=seed, clearance0=float(sim_map["config"]["error"]["clearance"][0]))
    for pick in picks:
        try:
            pick.update(record_and_render(family, p, pick, Path(out_dir) / f"drop-{pick['k']}.mp4", cfg))
        except Exception as e:  # noqa: BLE001 - a missing GL context must not stop the loop
            pick["error"] = f"{type(e).__name__}: {e}"[:300]
    return picks


# --- STL ---


def export_stl(family, p, out_dir):
    """The printed solid of each part as ``<family>-<part>.stl`` (mm); returns the paths."""
    from brixzle.families import builders as B

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for part in family.parts:
        path = out_dir / f"{family.name}-{part}.stl"
        B.sim_bodies(family, p, part).mesh.export(path)
        paths.append(path)
    return paths
