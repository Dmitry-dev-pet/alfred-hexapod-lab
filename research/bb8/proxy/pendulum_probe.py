#!/usr/bin/env python3
"""Uncalibrated *proxy* physics test of a spherical shell with internal pendulum.

DOES NOT load or represent authentic BB-8 source CAD/STL; never publish as such.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import mujoco

XML = '''<mujoco model="bb8_pendulum_proxy_NOT_CAD">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>
  <worldbody>
    <geom name="floor" type="plane" size="4 4 .1" friction="0.8 0.005 0.0001"/>
    <body name="shell" pos="0 0 0.201">
      <freejoint name="shell_free"/>
      <geom name="sphere_shell_collision_proxy" type="sphere" size="0.2" mass="0.20" friction="0.8 0.005 0.0001" rgba="0.8 0.4 0.1 0.45"/>
      <body name="inner_pendulum" pos="0 0 0">
        <joint name="pendulum_hinge" type="hinge" axis="0 1 0" damping="0.015"/>
        <geom name="internal_mass_proxy" type="box" pos="0 0 -0.11" size="0.035 0.04 0.025" mass="0.38" contype="0" conaffinity="0" rgba="0.15 0.3 0.7 1"/>
      </body>
    </body>
  </worldbody>
  <actuator><motor name="drive_torque" joint="pendulum_hinge" gear="1" ctrllimited="true" ctrlrange="-0.12 0.12"/></actuator>
</mujoco>'''

def simulate(torque: float, *, duration=6., frame_dt=.02):
    m=mujoco.MjModel.from_xml_string(XML)
    d=mujoco.MjData(m)
    samples=[]
    next_frame=0.
    shell=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,"shell")
    end_idx=int(round(duration/m.opt.timestep))
    x0=float(d.qpos[0])
    max_tilt=0.
    contacts=0
    for i in range(end_idx+1):
        t=i*m.opt.timestep
        # pulse: 0-1.5s positive, 1.5-3s negative, 3-6 coast
        ctrl=torque if t<1.5 else (-torque if t<3.0 else 0.0)
        d.ctrl[0]=ctrl
        if t+1e-9>=next_frame:
            angle=float(d.qpos[7]); max_tilt=max(max_tilt,abs(angle))
            contacts+=int(d.ncon>0)
            samples.append({"t":round(t,6),"qpos":d.qpos.tolist(),"qvel":d.qvel.tolist(),"ctrl":[ctrl],"contact_count":d.ncon})
            next_frame+=frame_dt
        if i<end_idx:
            mujoco.mj_step(m,d)
    return {
      "model_type":"UNCALIBRATED_SPHERE_PENDULUM_PROXY_NOT_ORIGINAL_CAD",
      "model_xml_sha256":hashlib.sha256(XML.encode()).hexdigest(),
      "engine":"MuJoCo","engine_version":mujoco.__version__,
      "parameters":{"sphere_radius_m_assumed":.2,"shell_mass_kg_assumed":.2,"pendulum_mass_kg_assumed":.38,"pendulum_offset_m_assumed":.11,"hinge_torque_Nm_assumed":torque,"friction_assumed":.8,"timestep_s":m.opt.timestep,"duration_s":duration},
      "summary":{"displacement_x_m":float(d.qpos[0]-x0),"shell_final_height_m":float(d.qpos[2]),"max_abs_joint_angle_rad":float(max_tilt),"sampled_steps_with_contact":contacts,"final_qpos":d.qpos.tolist(),"n_samples":len(samples)},
      "samples":samples
    }

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--torque",type=float,default=.04)
    ap.add_argument("--out",type=Path,default=None)
    args=ap.parse_args()
    o=simulate(args.torque)
    if args.out:
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(o,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(o["summary"],indent=2))
if __name__=="__main__":main()
