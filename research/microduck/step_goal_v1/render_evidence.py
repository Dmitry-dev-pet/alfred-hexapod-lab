"""Render exact saved states with original STL geometry, without physics reruns."""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
import argparse
import contextlib
import io
import json
from pathlib import Path
import mujoco as mj
from PIL import Image, ImageDraw, ImageFont
import tune_recovery as sim

HERE = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", type=Path, required=True)
    a = ap.parse_args()
    sim.initialize(a.workspace)
    summary = json.loads((HERE / "traces/summary.json").read_text())
    chosen = summary["selected"]
    trace = json.loads((HERE / "traces" / chosen["trace_file"]).read_text())
    sim.E.EDGE = trace["terrain"]["edge_x_m"]
    with contextlib.redirect_stdout(io.StringIO()):
        m, d, *_ = sim.BUILD(sim.IP, trace["terrain"]["height_m"])
    m.vis.global_.offwidth, m.vis.global_.offheight = 720, 480
    m.mat_reflectance[:] = 0
    step = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, "step_top")
    m.geom_rgba[step] = [.38, .60, .48, 1.]
    renderer = mj.Renderer(m, height=480, width=720)
    opt = mj.MjvOption()
    opt.geomgroup[:] = 0
    opt.geomgroup[0] = opt.geomgroup[2] = 1
    frames = trace["frames"]
    trigger = chosen["trigger_s"]
    targets = [min(frames, key=lambda f: abs(f["t"] - trigger)),
               min(frames, key=lambda f: abs(f["forward_rotation_deg"] - 180)), frames[-1]]
    canvas = Image.new("RGB", (2160, 568), "#102537")
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
    small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 19)
    labels = ["Подход", "Переворот", "Стойка наверху"]
    for i, (f, label) in enumerate(zip(targets, labels)):
        d.qpos[:] = f["qpos"]
        mj.mj_forward(m, d)
        camera = mj.MjvCamera()
        camera.lookat[:] = [float(d.qpos[0]) + .05, float(d.qpos[1]), .105]
        camera.distance, camera.azimuth, camera.elevation = .72, 120, -18
        renderer.update_scene(d, camera, scene_option=opt)
        canvas.paste(Image.fromarray(renderer.render()), (720 * i, 54))
        draw.text((720 * i + 18, 14), f"{label} · t = {f['t']:.2f} с", fill="#eef7ff", font=font)
    renderer.close()
    draw.text((18, 542), "MicroDuck · ступень 1 см · реальные состояния MuJoCo · исходная STL-геометрия · веса политик не менялись", fill="#cee0ed", font=small)
    canvas.save(HERE / "microduck-step-recovery.png")
    print("Rendered original-STL evidence triptych.")


if __name__ == "__main__":
    main()
