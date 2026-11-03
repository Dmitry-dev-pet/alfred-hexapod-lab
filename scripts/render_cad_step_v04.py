"""Render ONLY genuine MuJoCo joint snapshots of the first single-leg step.

The animation replays qpos from results/cad_v04_first_step_nominal.json;
it does not hallucinate a gait. The exact Freenove CAD remains externally
pinned and is not redistributed with our repository.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hexlab.cad_mujoco_v04 import DEFAULT_CACHE, build


def render(report_path: Path, video: Path, gif: Path, cache: Path, fps: int = 10, view: str = 'side'):
    import mujoco
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    from render_video import make_gif

    result = json.loads(report_path.read_text(encoding="utf-8"))
    if result.get("cad_to_vendor_PWM_signs_calibrated") is not False:
        raise ValueError("Unexpected physical calibration flag")
    if result["selected_vendor_leg"] != "fr":
        raise ValueError("Only vendor front-right experiment supported")
    trace = result["trace_50hz"]
    if len(trace) < 120 or any("qpos_snapshot" not in x for x in trace):
        raise ValueError("No true physics states available for animation")
    scene, _ = build(result["scenario"], cache=cache, offline=True)
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)

    # Mark the actual motor15/14/13 leg visibly without changing physics.
    # CAD 'fl' is vendor front-right (opposite handedness to CAD names).
    colored_bodies = {int(model.body(name).id) for name in ("coxafl", "femurfl", "tibiafl")}
    for gid in range(model.ngeom):
        if int(model.geom_bodyid[gid]) in colored_bodies and int(model.geom_group[gid]) == 2:
            model.geom_matid[gid] = -1
            model.geom_rgba[gid] = np.array([1.0, 0.29, 0.04, 1.0])

    duration = float(trace[-1]["t_s"])
    nframes = round(duration * fps)
    if nframes < 10 or nframes > 200:
        raise ValueError("Unreasonable first-step duration")
    width, height = 640, 480
    video.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s:v", f"{width}x{height}", "-r", str(fps),
        "-i", "-", "-an", "-vcodec", "libx264", "-pix_fmt", "yuv420p",
        "-preset", "fast", "-crf", "24", "-movflags", "+faststart",
        str(video),
    ]
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    views = {
        # Lower than the previous overhead view, camera toward front/near side.
        "side": {"lookat": (0., .075, .115), "distance": .76, "azimuth": 150, "elevation": -13},
        "front": {"lookat": (0., .06, .115), "distance": .84, "azimuth": 96, "elevation": -15},
    }
    if view not in views:
        raise ValueError(f"Unknown camera view: {view}")
    spec = views[view]
    cam.lookat[:] = spec["lookat"]
    cam.distance = spec["distance"]
    cam.azimuth = spec["azimuth"]
    cam.elevation = spec["elevation"]
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 17)
    except OSError:
        font = ImageFont.load_default()

    proc = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        assert proc.stdin is not None
        with mujoco.Renderer(model, height=height, width=width) as renderer:
            for i in range(nframes):
                target_t = min(duration, (i+.5) / fps)
                closest = min(trace, key=lambda row: abs(row["t_s"] - target_t))
                q = closest["qpos_snapshot"]
                if len(q) != model.nq:
                    raise ValueError(f"Wrong qpos snapshot length: {len(q)} != {model.nq}")
                data.qpos[:] = q
                data.qvel[:] = 0.
                data.time = closest["t_s"]
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=cam)
                frame = Image.fromarray(renderer.render())
                overlay = ImageDraw.Draw(frame)
                phase = closest.get("phase", "moving").upper()
                label = f"FR leg (orange)  |  {phase}  |  MuJoCo v0.4"
                overlay.rounded_rectangle((8, 8, 390, 42), radius=8, fill=(18, 26, 39))
                overlay.text((18, 14), label, font=font, fill=(255, 255, 255))
                proc.stdin.write(frame.tobytes())
    finally:
        if proc.stdin is not None:
            proc.stdin.close()
    err = proc.stderr.read().decode("utf8", "replace")
    if proc.wait() != 0:
        raise RuntimeError(f"ffmpeg step video failed: {err}")
    make_gif(video, gif, width=480, fps=fps)
    print(json.dumps({
        "source": str(report_path),
        "physics_snapshots": len(trace),
        "frames": nframes,
        "duration_s": duration,
        "gif_bytes": gif.stat().st_size,
        "mp4_bytes": video.stat().st_size,
        "replayed_actual_simulation": True,
        "real_hardware": False,
        "highlighted_vendor_leg": "FR",
        "camera_view": view,
    }, indent=2))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--report", type=Path, default=ROOT/"results/cad_v04_first_step_nominal.json")
    p.add_argument("--video", type=Path, default=ROOT/"results/first_single_leg_step.mp4")
    p.add_argument("--gif", type=Path, default=ROOT/"results/first_single_leg_step.gif")
    p.add_argument("--fps", type=int, default=10)
    p.add_argument("--view", choices=("side","front"), default="side")
    p.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    a = p.parse_args()
    render(a.report, a.video, a.gif, a.cache, a.fps, a.view)

if __name__ == "__main__":
    main()
