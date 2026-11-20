"""Render the SO-101 picking each design from the mould station and lifting it 100 mm (#153).

    uv run python scripts/arm_video.py [--designs ab fable_twinhook4 fable_allround]

Writes runs/arm-video-<UTC time>/<design>-<part>-pick.mp4 (ffmpeg; a GIF without it).
Simulated, uncalibrated contact.
"""
import argparse
from datetime import datetime, timezone
from pathlib import Path

from brixzle.engine.sim import pickplace as PP
from brixzle.families import registry
from brixzle.loop.videos import write_video


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--designs", nargs="+", default=["ab", "fable_twinhook4", "fable_allround"])
    ap.add_argument("--runs", default="runs")
    a = ap.parse_args()
    out = Path(a.runs) / f"arm-video-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    out.mkdir(parents=True, exist_ok=False)
    for name in a.designs:
        fam = registry.get(name)
        for part in fam.parts:
            rig = PP.station_rig(fam, fam.defaults, part, visual=True)
            x, y, z, _ = rig.station
            rig.recorder = PP.Recorder(rig.model, (x * 0.5, y, z + 110.0), distance_mm=650.0, azimuth=125.0, elevation=-18.0)
            r = PP.pick(rig)
            rig.hold(0.5)
            path = write_video(rig.recorder.frames, out / f"{name}-{part}-pick.mp4", fps=30)
            rig.recorder.close()
            print(name, part, {k: round(v, 2) for k, v in r.items()}, path, flush=True)
    print(out)


if __name__ == "__main__":
    main()
