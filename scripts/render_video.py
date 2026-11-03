"""Headless MuJoCo recording to MP4 and preview GIF.

Example (on a Linux server with Xvfb):
  xvfb-run -a python scripts/render_video.py --seconds 6 --fps 15

Uses the same baseline actuator controller as hexlab/mujoco_experiment.py.
The robot geometry and servo limits are approximate and not calibrated.
"""
from __future__ import annotations
import argparse
import math
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from hexlab.mujoco_model import neutral_leg_angles


def control_tripod(data, t, legs=6, period=1.8):
    _, neutral_hip, neutral_knee = neutral_leg_angles()
    for leg_id in range(legs):
        base = leg_id * 3
        phase = (t / period + (leg_id % 2) * 0.5) % 1.0
        if phase < 0.5:
            theta = phase * 2 * math.pi
            yaw = -0.18 + 0.36 * (phase / 0.5)
            hip = neutral_hip + 0.17 * math.sin(theta)
            knee = neutral_knee + 0.20 * math.sin(theta)
        else:
            yaw = 0.18 - 0.36 * ((phase - 0.5) / 0.5)
            hip, knee = neutral_hip, neutral_knee
        data.ctrl[base:base + 3] = (yaw, hip, knee)


def make_video(out: Path, seconds: float, fps: int, width: int, height: int) -> None:
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(ROOT / 'models' / 'freenove_fnk0052_approx.xml'))
    data = mujoco.MjData(model)
    keyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'stand')
    mujoco.mj_resetDataKeyframe(model, data, keyid)
    mujoco.mj_forward(model, data)
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
           '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s:v', f'{width}x{height}',
           '-r', str(fps), '-i', '-', '-an', '-vcodec', 'libx264',
           '-pix_fmt', 'yuv420p', '-preset', 'fast', '-crf', '23',
           '-movflags', '+faststart', str(out)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    assert proc.stdin is not None
    try:
        with mujoco.Renderer(model, width=width, height=height) as renderer:
            n = round(seconds * fps)
            for i in range(n):
                target_t = i / fps
                while data.time < target_t:
                    control_tripod(data, data.time)
                    mujoco.mj_step(model, data)
                renderer.update_scene(data, camera='tracking')
                frame = renderer.render()
                proc.stdin.write(frame.tobytes())
                if i % max(1, fps) == 0:
                    print(f'frame {i + 1}/{n} at t={data.time:.2f}s', flush=True)
    finally:
        proc.stdin.close()
    stderr = proc.stderr.read().decode('utf8', 'replace')
    code = proc.wait()
    if code != 0:
        raise RuntimeError(f'ffmpeg exit {code}: {stderr}')


def make_gif(mp4: Path, gif: Path, width: int=480, fps: int=10) -> None:
    # Two-pass GIF encoding: more accurate colors, lower bandwidth.
    palette = gif.with_suffix('.palette.png')
    filt = f'fps={fps},scale={width}:-1:flags=lanczos'
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(mp4),
                    '-vf',f'{filt},palettegen=stats_mode=diff','-frames:v','1',str(palette)],check=True)
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(mp4),'-i',str(palette),
                    '-filter_complex',f'[0:v]{filt}[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=3',
                    '-loop','0',str(gif)],check=True)
    palette.unlink(missing_ok=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=float, default=6.0)
    p.add_argument('--fps', type=int, default=15)
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--video', type=Path, default=ROOT / 'results' / 'first_tripod_demo.mp4')
    p.add_argument('--gif', type=Path, default=ROOT / 'results' / 'first_tripod_demo.gif')
    a = p.parse_args()
    make_video(a.video, a.seconds, a.fps, a.width, a.height)
    make_gif(a.video, a.gif)
    print('VIDEO', a.video, a.video.stat().st_size, 'bytes', flush=True)
    print('GIF', a.gif, a.gif.stat().st_size, 'bytes', flush=True)