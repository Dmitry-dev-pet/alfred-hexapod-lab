"""First constrained six-leg standing experiment for CAD-derived Freenove FNK0052.

This searches for six attainable toe positions using MuJoCo's *actual* joint axes
and CAD tibia meshes, then runs a limited-torque hold test under gravity. It is
a numerical standing candidate, NOT the manufacturer's zero-angle/PWM
calibration, and NOT an experimentally validated servo controller.

The user wants experiments grounded in a realistic mass+geometry model before
biologically motivated gait experiments.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .cad_mujoco_v04 import DEFAULT_CACHE, build
from .stand_geometry_v04 import LEG_CODES, load_legs, tip_world, geom_mesh_world_vertices
from .simulation import support_margin

ROOT = Path(__file__).resolve().parents[1]
STAND_HEIGHT_M = 0.145
TARGET_RADIAL_EXTENSION_M = 0.115
FOOT_TARGET_CLEARANCE_M = 0.004
MAX_CALIBRATION_JOINT_ANGLE_RAD = 1.65


def solve_leg(model, data, mujoco, leg: dict, target: np.ndarray) -> dict:
    """Bounded damped inverse kinematics with analytic point Jacobian.

    The candidate toe is the most distal CAD tibia mesh vertex. A small angular
    regularizer discourages extreme angles and leaves better motor reserve.
    """
    from scipy.optimize import least_squares
    qadrs, dadrs = leg["qadrs"], leg["dadrs"]
    orig = np.asarray(data.qpos[qadrs]).copy()
    reg = 0.0025
    nvel = int(model.nv)

    def eval_position(x):
        data.qpos[qadrs] = x
        mujoco.mj_forward(model, data)
        return tip_world(model, data, leg)

    def fun(x):
        return np.r_[eval_position(x) - target, reg * x]

    def jac(x):
        point = eval_position(x)
        jpos = np.zeros((3, nvel))
        jrot = np.zeros((3, nvel))
        mujoco.mj_jac(model, data, jpos, jrot, point, int(leg["body"]))
        return np.vstack([jpos[:, dadrs], reg * np.eye(3)])

    seeds = [
        (0., 0., 0.),
        (0., .8, -.8),
        (0., -.8, .8),
        (0., .9, .9),
        (0., -.9, -.9),
        (.15, .5, -1.2),
        (-.15, -.5, 1.2),
    ]
    candidates = []
    for seed in seeds:
        fit = least_squares(
            fun, np.array(seed, dtype=float),
            jac=jac, bounds=(-MAX_CALIBRATION_JOINT_ANGLE_RAD,
                             MAX_CALIBRATION_JOINT_ANGLE_RAD),
            max_nfev=110, xtol=1e-9, ftol=1e-9, gtol=1e-9
        )
        point = eval_position(fit.x)
        pos_err = float(np.linalg.norm(point - target))
        candidates.append((pos_err + 0.0003 * np.linalg.norm(fit.x), pos_err, fit.x.copy()))
        if pos_err < .002:
            break
    _score, err, xbest = min(candidates, key=lambda x: x[0])
    data.qpos[qadrs] = xbest
    mujoco.mj_forward(model, data)
    answer = {
        "name": leg["suffix"],
        "joint_names": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                        for j in leg["jids"]],
        "qpos_rad": [round(float(x), 6) for x in xbest],
        "target_foot_xyz_m": [round(float(x), 6) for x in target],
        "fitted_foot_xyz_m": [round(float(x), 6) for x in tip_world(model, data, leg)],
        "position_error_mm": round(err * 1000, 3),
        "within_15mm": err < .015,
        "knee_to_candidate_tip_m": leg["knee_to_tip_m"],
    }
    return answer


def static_pose(model, data, mujoco, legs: list[dict], height: float,
                radial_extension: float) -> dict:
    """Find stance joint positions; does not move the body down to the floor."""
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link_2")
    data.qpos[2] = height
    mujoco.mj_forward(model, data)
    com0 = None
    fitted = []
    for leg in legs:
        hip = np.asarray(data.xanchor[leg["jids"][0]]).copy()
        target_xy = hip[:2] + radial_extension * leg["radial_unit_xy"]
        target = np.r_[target_xy, FOOT_TARGET_CLEARANCE_M]
        fitted.append(solve_leg(model, data, mujoco, leg, target))
    mujoco.mj_forward(model, data)
    # Evaluate REAL low point of each tibia mesh, not merely a diagnostic toe.
    minima = {}
    for leg in legs:
        xyz = geom_mesh_world_vertices(model, data, leg["geom"])
        minima[leg["suffix"]] = float(xyz[:,2].min())
    pts = [tip_world(model,data,l) for l in legs]
    com = (np.asarray(data.xipos) * np.asarray(model.body_mass)[:,None]).sum(axis=0) / model.body_mass.sum()
    margin = float(support_margin(tuple(com[:2]),[tuple(p[:2]) for p in pts]))
    base_pos = np.asarray(data.xpos[root_id])
    return {
        "root_position_xyz_m": [round(float(x), 6) for x in base_pos],
        "com_world_xyz_m": [round(float(x), 6) for x in com],
        "static_com_in_toe_polygon_margin_m": round(margin, 6),
        "min_tibia_mesh_height_m": {k:round(v,6) for k,v in minima.items()},
        "min_tibia_all_m": round(min(minima.values()),6),
        "max_tibia_all_m": round(max(minima.values()),6),
        "fit_all_legs_under_15mm": all(x["within_15mm"] for x in fitted),
        "joint_poses": fitted,
        "note": "A numerical pose based on real joint axes; toe landmarks are inferred from CAD and not verified against physical contact pads",
    }


def hold_under_gravity(model, data, mujoco, initial_qpos: np.ndarray,
                       duration: float = 2.0) -> dict:
    """PD torque hold with explicit 0.7Nm leg limits. No welded stance constraints."""
    mujoco.mj_forward(model, data)
    base = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"base_link_2")
    start_xyz = np.asarray(data.xpos[base]).copy()
    initial_rot = np.asarray(data.xmat[base]).reshape(3,3).copy()
    local_up = initial_rot.T @ np.array([0.,0.,1.])
    motor_jid = [mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,
                                  mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,
                                  int(model.actuator_trnid[a,0])))
                 for a in range(model.nu)]
    if len(motor_jid)!=20 or any(j < 0 for j in motor_jid):
        raise ValueError("Expected 20 motor-to-joint references")
    qa = np.array([model.jnt_qposadr[j] for j in motor_jid],dtype=int)
    da = np.array([model.jnt_dofadr[j] for j in motor_jid],dtype=int)
    targets = initial_qpos[qa]
    # small cold-start damping; not asserted to be native servo physics.
    kp, kd = 7.0, 0.32
    bounds = np.asarray(model.actuator_ctrlrange)
    if not np.all(bounds[:,0] < bounds[:,1]):
        raise ValueError("Torque limits missing")
    root_z = []
    tilt_deg = []
    contact_leg_count = []
    torques = []
    distances = []
    step_hz = 1.0/float(model.opt.timestep)
    n = round(duration * step_hz)
    interval = max(1,round(.02*step_hz))
    leg_collision_ids = {}
    for suffix in LEG_CODES:
        g = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,f"tibia{suffix}_collision_0")
        leg_collision_ids[g] = suffix
    floor = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"floor")
    if floor < 0 or len(leg_collision_ids)!=6:
        raise ValueError("No floor or six contact geometries")

    for k in range(n):
        torque = kp*(targets - data.qpos[qa]) - kd*data.qvel[da]
        torque = np.maximum(bounds[:,0],np.minimum(bounds[:,1],torque))
        data.ctrl[:] = torque
        mujoco.mj_step(model, data)
        if k % interval == 0 or k == n-1:
            xyz = np.asarray(data.xpos[base])
            orientation = np.asarray(data.xmat[base]).reshape(3,3)
            up = orientation @ local_up
            tilt = math.degrees(math.acos(float(np.clip(up @ np.array([0.,0.,1.]),-1,1))))
            found=set()
            for ci in range(data.ncon):
                c=data.contact[ci]
                other = int(c.geom2 if c.geom1 == floor else c.geom1
                            if c.geom2 == floor else -1)
                if other in leg_collision_ids:
                    found.add(leg_collision_ids[other])
            root_z.append(float(xyz[2]))
            tilt_deg.append(tilt)
            contact_leg_count.append(len(found))
            torques.append(float(np.max(np.abs(torque))))
            distances.append(float(np.linalg.norm(xyz[:2]-start_xyz[:2])))
            if not np.all(np.isfinite(data.qpos)):
                raise ValueError(f"Unstable nonfinite physics step {k}")

    tail = max(1,int(round(.4/(interval*model.opt.timestep))))
    tail_heights = root_z[-tail:]
    tail_contacts = contact_leg_count[-tail:]
    tail_tilts = tilt_deg[-tail:]
    tail_drift = distances[-tail:]
    stable = (min(tail_heights) > 0.08
              and max(tail_tilts) < 18
              and min(tail_contacts) >= 3
              and max(tail_drift) < .08)
    return {
      "duration_s": duration,
      "simulated_steps": n,
      "root_start_xyz_m": [round(float(x),6) for x in start_xyz],
      "root_end_xyz_m": [round(float(x),6) for x in data.xpos[base]],
      "min_root_height_m": round(min(root_z),6),
      "final_root_height_m": round(root_z[-1],6),
      "max_tilt_deg": round(max(tilt_deg),3),
      "final_tilt_deg": round(tilt_deg[-1],3),
      "last_0p4s_min_contact_legs": min(tail_contacts),
      "last_0p4s_mean_contact_legs": round(float(np.mean(tail_contacts)),3),
      "max_applied_abs_torque_nm": round(max(torques),3),
      "max_horizontal_drift_m": round(max(distances),6),
      "last_0p4s_max_tilt_deg": round(max(tail_tilts),3),
      "last_0p4s_min_root_height_m": round(min(tail_heights),6),
      "stable_by_proxy_thresholds": bool(stable),
      "physically_calibrated": False,
      "time_series_50hz": [{"t_s":round(min((i+1)*interval*model.opt.timestep,duration),3),
           "height_m":round(root_z[i],5), "tilt_deg":round(tilt_deg[i],2),
           "contact_legs":contact_leg_count[i]} for i in range(len(root_z))],
      "note":"No foot fixation, only limited PD torques; model servos, ground and CAD foot contact uncalibrated",
    }


def experiment(case: str, duration: float = 2.0, *,
               cache: Path=DEFAULT_CACHE, stand_height: float=STAND_HEIGHT_M,
               radial_extension: float=TARGET_RADIAL_EXTENSION_M) -> dict:
    import mujoco
    scene, construction = build(case, cache=cache, offline=True)
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    legs = load_legs(model,data,mujoco)
    result_pose=static_pose(model,data,mujoco,legs,stand_height,radial_extension)
    initial_qpos=np.asarray(data.qpos).copy()
    # Save candidate even if unsatisfactory: failed attempts are useful evidence.
    report = {
        "model": "Freenove CAD-derived MuJoCo v0.4",
        "scenario":case,
        "mujoco_version":mujoco.__version__,
        "mass_kg":round(float(sum(model.body_mass)),6),
        "stand_height_parameter_m":stand_height,
        "radial_extension_parameter_m":radial_extension,
        "cad_source_sha":construction["upstream_commit"],
        "motor_torque_limits_nm":{"legs":0.7,"head":0.2},
        "fit":result_pose,
        "qpos_candidate":[round(float(x),7) for x in initial_qpos],
        "physical_validity":"EXPERIMENTAL_NOT_CALIBRATED",
    }
    report["hold_test"] = hold_under_gravity(model,data,mujoco,initial_qpos,duration)
    # NOTE candidate fit and hold must both pass before claiming a stable pose.
    report["six_leg_standing_candidate_succeeded"] = (
        result_pose["fit_all_legs_under_15mm"] and
        result_pose["static_com_in_toe_polygon_margin_m"] > .02 and
        result_pose["min_tibia_all_m"] > -.015 and
        report["hold_test"]["stable_by_proxy_thresholds"]
    )
    return report


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--scenario",choices=("nominal","lower","upper"),default="nominal")
    p.add_argument("--duration",type=float,default=2.0)
    p.add_argument("--stand-height",type=float,default=STAND_HEIGHT_M)
    p.add_argument("--radial-extension",type=float,default=TARGET_RADIAL_EXTENSION_M)
    p.add_argument("--cache",type=Path,default=DEFAULT_CACHE)
    p.add_argument("--out",type=Path)
    args=p.parse_args()
    report=experiment(args.scenario,args.duration,cache=args.cache,
                      stand_height=args.stand_height,
                      radial_extension=args.radial_extension)
    if args.out:
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
    # Print concise metrics; full trajectories are in JSON receipt.
    short={k:v for k,v in report.items() if k not in ("qpos_candidate",)}
    short["hold_test"]={k:v for k,v in report["hold_test"].items() if k!="time_series_50hz"}
    print(json.dumps(short,ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
