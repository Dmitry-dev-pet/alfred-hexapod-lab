#!/usr/bin/env python3
"""STEP-based Mini Wheelbot V2 rollout with upstream roll standup torque sequence.

Geometry: verified frame.stp motor seats CAD Y ±32.5 mm, axes CAD X and Z;
map CAD Y-up to MuJoCo Z-up by +90deg around X.
Robot mechanics beyond those source-constrained axes remain APPROXIMATIONS:
collision box/cylinders; body COM inferred, not verified; torso inertia projection;
no validated electrical motor torque-speed curve. Never label a pass as real robot proof.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from audit_original_cad import acquire
from audit_step_axes import audit as audit_step
from simulate_mujoco_probe import compiled_visual_body, source_inertia_for_mujoco

UPSTREAM_CONFIG = {
    "source": "wheelbot/Mini-Wheelbot/wheelbot-lib/config/default.json",
    "inputs_standup_rp": [0.55, 3600, 1500],
    "inputs_standup_rn": [-0.55, 3600, 1500],
    "K_roll": [-1.3, -0.16, -0.00008, -0.0004],
    "source_control": "wheelbot-lib/include/wheelbot/controller.hpp: RollStandupController",
}
SOURCE_COMMIT = "73815e49371c5fb5f49eec59d364e4d9d1b57b10"
STEP_DT = .001
OMEGA_CAP_ASSUMED = 22.2 * 380 * 2 * math.pi / 60
CONTROLLER_DELAY_S = 1.0


def assert_source_frames(step):
    seats = step["motor_seats"]
    low = next(x for x in seats if x["axis_point_mm"][1] == -32.5)
    high = next(x for x in seats if x["axis_point_mm"][1] == 32.5)
    assert abs(low["axis_direction"][0]) == 1
    assert abs(high["axis_direction"][2]) == 1
    return low, high


def make_xml(cache: Path, params: dict, step: dict, limit_nm: float = .55,
             contact_profile: str = "default") -> str:
    low, high = assert_source_frames(step)
    assert low and high
    orig = cache / "hardware/mechanical/meshes"
    original_body = compiled_visual_body(cache)
    rotor = orig / "actuator_rotor.stl"
    inertias = source_inertia_for_mujoco(params)["projected_kg_m2"]
    # Source axes in CAD: X=low axle, Z=high axle; source CAD Y = vertical.
    # Coordinate rotation +90° about CAD X maps (x,y,z) -> (x,-z,y) in MuJoCo.
    # Two axles are at local Z=±.0325 m after conversion.
    # Body COM *inferred* from L_WB=.065m above bottom axle: at +.0325m.
    # Body geometry full box proxy: measured original STL extents / 2.
    body_size = ".043 .0515 .043"
    # Diagonal body source inertias are rotated into the new axes. Identified
    # I_By and I_Bz may use source conventions differing from CAD; unknown.
    body_ix, body_iy, body_iz = inertias[0], inertias[2], inertias[1]
    # Upper reaction wheel: axial inertia about world Y (CAD Z).
    # Lower driving wheel: axial inertia about world X (CAD X).
    axle_I = params["I_Wy_Rx"]
    side_I = params["I_Wxz_Ryz"]
    xml = f"""<mujoco model="mini-wheelbot-STEPAxes-original-roll-sequence">
  <compiler angle="radian"/>
  <option timestep="{STEP_DT}" gravity="0 0 -9.81" integrator="implicitfast" iterations="100"/>
  <asset>
    <mesh name="body_source_mesh" file="{original_body}" scale=".001 .001 .001"/>
    <mesh name="rotor_source_mesh" file="{rotor}" scale=".001 .001 .001"/>
  </asset>
  <worldbody>
    <geom name="ground" type="plane" size="3 3 .1" friction="0.9 .01 .001"/>
    <body name="body" pos="0 0 .065">
      <freejoint name="base"/>
      <inertial pos="0 0 .0325" mass="{params["m_B"]}"
                diaginertia="{body_ix} {body_iy} {body_iz}"/>
      <geom name="body_visible_original_CAD" type="mesh" mesh="body_source_mesh"
            quat=".707106781 .707106781 0 0" contype="0" conaffinity="0"
            rgba=".65 .7 .75 1"/>
      <geom name="body_collision_PROXY" type="box" size="{body_size}"
            friction="0.8 .01 .001" rgba=".45 .6 .75 .16"/>
      <body name="lower_drive" pos="0 0 -.0325">
        <joint name="drive_axis_STEPX" type="hinge" axis="1 0 0" damping=".0002"/>
        <inertial pos="0 0 0" mass="{params["m_WR"]}"
          diaginertia="{axle_I} {side_I} {side_I}"/>
        <geom name="lower_drive_coll_PROXY" type="cylinder"
              size=".032 .014" quat=".707106781 0 .707106781 0"
              friction="1 .01 .001"/>
        <geom name="lower_drive_visual_original_rotor" type="mesh"
              mesh="rotor_source_mesh" contype="0" conaffinity="0"
              rgba=".7 .6 .35 1"/>
      </body>
      <body name="upper_reaction" pos="0 0 .0325">
        <joint name="reaction_axis_STEPZ" type="hinge" axis="0 -1 0" damping=".0002"/>
        <inertial pos="0 0 0" mass="{params["m_WR"]}"
          diaginertia="{side_I} {axle_I} {side_I}"/>
        <geom name="upper_reaction_coll_PROXY" type="cylinder"
              size=".032 .014" quat=".707106781 .707106781 0 0"
              friction="1 .01 .001"/>
        <geom name="upper_reaction_visual_original_rotor" type="mesh"
              mesh="rotor_source_mesh" quat=".707106781 0 0 -.707106781"
              contype="0" conaffinity="0" rgba=".75 .62 .4 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="drive" joint="drive_axis_STEPX" gear="1"
          ctrllimited="true" ctrlrange="-{limit_nm} {limit_nm}"/>
    <motor name="reaction" joint="reaction_axis_STEPZ" gear="1"
          ctrllimited="true" ctrlrange="-{limit_nm} {limit_nm}"/>
  </actuator>
</mujoco>"""
    if contact_profile == "stiff":
        # Sensitivity study, not a calibration based on force measurements.
        for friction in ('0.9', '0.8', '1'):
            needle = f'friction="{friction} .01 .001"'
            xml = xml.replace(needle, needle + ' solref=".003 1" solimp=".97 .99 .0001"')
    elif contact_profile != "default":
        raise ValueError("Unsupported contact profile: " + contact_profile)
    return xml


def upstream_roll_input(elapsed_seconds: float, sign: int = 1) -> float:
    """Literal RollStandupController piecewise sequence: 1 s idle, 3.6 s
    negative ramp reaching -0.6*max, then +max for 1.5 s."""
    max_input = .55 * sign
    u_time_ms = (elapsed_seconds - CONTROLLER_DELAY_S) * 1000
    if u_time_ms < 0:
        return 0.
    if u_time_ms <= 3600:
        return -max_input * (u_time_ms / 3600) * .6
    if u_time_ms <= 5100:
        return max_input
    return 0.


def body_tilt(m, d, body_id):
    zaxis = d.xmat[body_id].reshape(3, 3)[:, 2]
    return math.degrees(math.acos(float(np.clip(zaxis[2], -1, 1))))


def run_one(cache, params, step, start_roll=90, mode="upstream", speed_cap=True,
            duration=8., sample_s=.05, initial_qvel=0,
            contact_profile="default"):
    xml = make_xml(cache, params, step, contact_profile=contact_profile)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    joint_reaction = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "reaction_axis_STEPZ")
    dof = int(m.jnt_dofadr[joint_reaction])
    body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "body")
    # Roll about MuJoCo Y = orthogonal upper reaction wheel shaft;
    # starts as original-positive-roll (+90 degrees) but sign convention
    # is not yet independently validated against C++ estimate axes.
    angle = math.radians(start_roll)
    d.qpos[:3] = (0., 0., .07)
    d.qpos[3:7] = (math.cos(angle/2), 0, math.sin(angle/2), 0)
    d.qvel[dof] = initial_qvel
    mujoco.mj_forward(m, d)
    n = int(round(duration / STEP_DT))
    stride = max(1, int(sample_s / STEP_DT))
    mins, max_torque, max_omega, collision_steps, max_pen_mm = 180.,0.,0.,0,0.
    time_with_upright, prev_in_upright, overspeed_hits = 0.,None,0
    frames = []
    numerical_warning = None
    last_step = 0
    for k in range(n):
        last_step = k
        tilt = body_tilt(m,d,body_id)
        mins = min(mins,tilt)
        if tilt <= 10:
            if prev_in_upright is None: prev_in_upright = float(d.time)
            time_with_upright = max(time_with_upright,float(d.time)-prev_in_upright)
        else:
            prev_in_upright = None
        omega = float(d.qvel[dof])
        max_omega = max(max_omega,abs(omega))
        if mode=="passive": torque = 0.
        elif mode=="upstream":
            torque = upstream_roll_input(float(d.time),sign=1 if start_roll>0 else -1)
        else: raise ValueError(mode)
        if speed_cap and abs(omega)>=OMEGA_CAP_ASSUMED and torque*omega>0:
            torque=0.;overspeed_hits+=1
        d.ctrl[0] = 0
        d.ctrl[1] = torque
        max_torque = max(max_torque,abs(torque))
        if d.ncon:
            collision_steps+=1
            max_pen_mm=max(max_pen_mm,1000*max(0.,-min(float(c.dist) for c in d.contact)))
        if k%stride==0 or k==n-1:
            frames.append([round(float(d.time),3),
                           *[round(float(v),6) for v in d.qpos],
                           round(tilt,3),round(omega,3),round(torque,4),d.ncon])
        mujoco.mj_step(m,d)
        warning_records = [
            {"warning_index": i, "count": int(stat.number)}
            for i, stat in enumerate(d.warning) if stat.number > 0
        ]
        if warning_records:
            numerical_warning = {
                "simulation_time_s": round((k+1)*STEP_DT,5),
                "mujoco_time_after_warning_s": float(d.time),
                "record": warning_records,
            }
            # Fail closed: MuJoCo can reset unstable qpos/qvel after warning.
            # Continuing would generate false "standing up" segments.
            break
        if not (np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()):
            numerical_warning = {
                "simulation_time_s": round((k+1)*STEP_DT,5),
                "record": [{"error": "non-finite state"}],
            }
            break
    final_tilt=body_tilt(m,d,body_id)
    observed_upright=bool(final_tilt<=10 and time_with_upright>=1)
    success=bool(observed_upright and overspeed_hits==0 and
                 numerical_warning is None and speed_cap)
    return {
       "status": ("NUMERICAL_DIVERGENCE" if numerical_warning is not None
                  else "PROVISIONAL_GEOMETRY_MOTOR_AND_CONTROLLER_MAPPING"),
       "numerical_warning": numerical_warning,
       "simulated_duration_s": round((last_step+1)*STEP_DT,5),
       "contact_profile": contact_profile,
       "motor_limits_hardware_validated": False,
       "source_joint_label_parity_validated": False,
       "start_roll_deg":start_roll,"mode":mode,"speed_cap_assumed":speed_cap,
       "min_tilt_deg":round(mins,3),"final_tilt_deg":round(final_tilt,3),
       "longest_upright_s":round(time_with_upright,3),
       "peak_reaction_speed_rad_s":round(max_omega,3),
       "peak_applied_torque_nm":round(max_torque,4),
       "speed_limit_command_blocks":overspeed_hits,
       "contact_steps":collision_steps,
       "max_contact_penetration_mm":round(max_pen_mm,3),
       "observed_upright_duration_and_final_pose": observed_upright,
       "meets_provisional_up_criterion":success,
       "frames":frames
    }


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--cache",type=Path,default=Path.home()/"mini-wheelbot-original-cad")
    parser.add_argument("--step-cache",type=Path,default=Path.home()/"mini-wheelbot-step")
    parser.add_argument("--out",type=Path,default=Path("/tmp/wheelbot-v2-step-controller"))
    parser.add_argument("--duration",type=float,default=8.)
    parser.add_argument("--contact-profile",choices=["default","stiff"],default="default")
    args=parser.parse_args()
    if not 3<=args.duration<=15: parser.error("Duration outside 3..15 s")
    original=acquire(args.cache)
    step=audit_step(args.step_cache)
    args.out.mkdir(parents=True,exist_ok=True)
    cases=[(90,"passive",True),(90,"upstream",True),(90,"upstream",False),
           (-90,"upstream",True),(-90,"upstream",False),
           (60,"upstream",True),(60,"upstream",False)]
    outcomes=[]
    for initial,mode,cap in cases:
        result=run_one(args.cache,original["params"],step,initial,mode,cap,args.duration,
                       contact_profile=args.contact_profile)
        frames=result.pop("frames")
        name=f"roll_{initial:+03d}_{mode}_{'cap' if cap else 'nocap'}"
        file=args.out/(name+".json")
        file.write_text(json.dumps({"info":result,"frames":frames},separators=(',',':'))+"\n")
        result["frames_sha256"]=hashlib.sha256(file.read_bytes()).hexdigest()
        result["frames_file"]=file.name
        outcomes.append(result)
        print(name,result["min_tilt_deg"],result["final_tilt_deg"],
              result["peak_reaction_speed_rad_s"],result["meets_provisional_up_criterion"],flush=True)
    report={
       "classification":"EXPERIMENTAL_STEP_AXIS_ORIGINAL_OPEN_LOOP_NOT_HARDWARE_VALIDATED",
       "upstream_commit":SOURCE_COMMIT,
       "contact_profile": args.contact_profile,
       "source_controller":UPSTREAM_CONFIG,
       "step_sha256":{name:info["sha256"] for name,info in step["files"].items()},
       "step_motor_seats":step["motor_seats"],
       "source_model_xml_sha256":hashlib.sha256(make_xml(args.cache,original["params"],step,
            contact_profile=args.contact_profile).encode()).hexdigest(),
       "source_cad_body_sha256":next(x["sha256"] for x in original["source_files"] if x["path"].endswith("body.stl")),
       "assumptions":[
          "lower CAD X-axis motor is driving wheel; upper CAD Z-axis is reaction",
          "CAD Y-up mapped to MJ Z-up using +90deg X rotation",
          "body COM inferred at upper axle (65mm above bottom axle) from sysid l_WB",
          "body inertias projected into feasible tensor and mapped to CAD axes",
          "body and wheels approximated as collision box/cylinders; visual CAD authentic",
          "22.2V x 380KV no-load omega cap is an unvalidated assumption",
          "upstream C++ open-loop torque copied, but source controller stage-to-balance not ported",
          "no actual assembly, motor, contact or sensor validation"
        ],
       "runs":outcomes
    }
    (args.out/"summary.json").write_text(json.dumps(report,indent=2)+"\n")
    print("REPORT",args.out/"summary.json",flush=True)


if __name__=="__main__":
    main()
