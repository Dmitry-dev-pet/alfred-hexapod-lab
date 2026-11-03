"""Run the approximate 18-DoF MJCF model using a locally installed MuJoCo.

This script will not claim simulation succeeded without mujoco installed.
"""
from __future__ import annotations
import argparse
import json
import math
import time
from pathlib import Path

from .mujoco_model import LEGS, write_model, neutral_leg_angles


def run(seconds: float = 10.0, mode: str = "tripod", out: Path = Path("results/mujoco_run.json")) -> dict:
    try:
        import mujoco
    except ImportError as ex:
        raise RuntimeError("MuJoCo not installed. Restore a Linux cp313 wheelhouse, then run 'pip install --no-index --find-links=wheelhouse mujoco==3.14.0'.") from ex
    model_file = Path(__file__).resolve().parents[1] / "models" / "freenove_fnk0052_approx.xml"
    if not model_file.exists():
        write_model(model_file)
    model = mujoco.MjModel.from_xml_path(str(model_file))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    steps = int(seconds / model.opt.timestep)
    log = []
    started = time.perf_counter()
    # A minimal illustrative position-control baseline (not neuroWalknet).
    # No commanded body translation: any forward motion must result from simulated contacts.
    period = 1.8
    neutral_yaw, neutral_hip, neutral_knee = neutral_leg_angles()
    for it in range(steps):
        t = it*model.opt.timestep
        for leg_id, (name,*_) in enumerate(LEGS):
            base = leg_id*3
            phase = (t / period + (leg_id % 2)*0.5) % 1.0
            if mode == "stand":
                yaw, hip, knee = neutral_yaw, neutral_hip, neutral_knee
            else:
                # First half of cycle = swing. The sine lifts the foot.
                if phase < 0.5:
                    theta = phase * 2*math.pi
                    yaw, hip, knee = (-0.18+0.36*(phase/0.5),
                                      neutral_hip+0.17*math.sin(theta),
                                      neutral_knee+0.20*math.sin(theta))
                else:
                    yaw, hip, knee = 0.18-0.36*((phase-0.5)/0.5), neutral_hip, neutral_knee
            data.ctrl[base:base+3] = (yaw,hip,knee)
        mujoco.mj_step(model,data)
        if it%25 == 0:
            contacts = [round(float(data.sensor(f"{name}_contact").data[0]),4) for name,*_ in LEGS]
            log.append({"t":round(t,4),"x":round(float(data.qpos[0]),5),"y":round(float(data.qpos[1]),5),"z":round(float(data.qpos[2]),5),"contacts":contacts})
    report = {"simulator":"MuJoCo", "mujoco_version":mujoco.__version__, "model":str(model_file),"model_approximate":True,"seconds":seconds,"mode":mode,"wall_seconds":round(time.perf_counter()-started,3),"n_steps":steps,"start_x":log[0]['x'],"end_x":log[-1]['x'],"min_height_m":min(x['z'] for x in log),"steps_without_contacts":sum(not any(x['contacts']) for x in log),"samples":log}
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    return report


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=10.0);p.add_argument('--mode',choices=['stand','tripod'],default='stand');p.add_argument('--out',type=Path,default=Path('results/mujoco_run.json'))
    a=p.parse_args()
    report=run(a.seconds,a.mode,a.out)
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))