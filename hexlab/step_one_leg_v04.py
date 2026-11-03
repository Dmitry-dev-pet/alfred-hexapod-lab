"""Single-leg lift -> forward swing -> touchdown for the CAD-derived FNK0052.

Preliminary real-body MuJoCo dynamics, *not* a physical servo calibration.
One leg moves, the other five retain their PD joint targets; no fixed feet,
kinematic body translation or artificial force is applied to the trunk.
The result is scored with observable contact loss, renewed touchdown, tip
motion, torque saturation, support contacts, COM margin and body posture.

Only original pinned CAD meshes are loaded into the ignored local cache.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from .cad_mujoco_v04 import DEFAULT_CACHE

SELECTED_LEG = "fr"
STEP_FORWARD_M = 0.028
STEP_LIFT_M = 0.026
MAX_IK_ANGLE = 1.70
SETTLE_SECONDS = 1.0
LIFT_SECONDS = 0.8
SWING_SECONDS = 0.8
LAND_SECONDS = 0.8
RECOVER_SECONDS = 1.0


def smoothstep(s: float) -> float:
    """C1-continuous interpolation with zero terminal velocity."""
    s = max(0., min(1., float(s)))
    return s * s * (3 - 2 * s)


def phases(t: float) -> tuple[str, float]:
    """Time in seconds *after* initial six-leg stabilization."""
    if t < 0:
        raise ValueError("Negative time")
    stops = (
        ("lift", LIFT_SECONDS),
        ("swing", SWING_SECONDS),
        ("lower", LAND_SECONDS),
        ("recover", RECOVER_SECONDS),
    )
    for label, duration in stops:
        if t < duration:
            return label, smoothstep(t / duration)
        t -= duration
    return "recover", 1.


def commanded_joint_pose(time_after_settle: float,
                         q_stand, q_lift, q_swing, q_land):
    """Use smooth joint-space paths between foot-space IK waypoints."""
    import numpy as np
    name, s = phases(time_after_settle)
    if name == "lift":
        return (1 - s) * q_stand + s * q_lift
    if name == "swing":
        return (1 - s) * q_lift + s * q_swing
    if name == "lower":
        return (1 - s) * q_swing + s * q_land
    return np.asarray(q_land).copy()


def forward_from_geometry(legs) -> tuple[float, float]:
    """Derived from front/back hip locations rather than assuming world axis."""
    import numpy as np
    by = {leg["suffix"]: leg for leg in legs}
    front = (by["fr"]["hip_xyz"][:2] + by["fl"]["hip_xyz"][:2]) / 2
    back = (by["br"]["hip_xyz"][:2] + by["bl"]["hip_xyz"][:2]) / 2
    result = front - back
    if np.linalg.norm(result) < .1:
        raise ValueError("Cannot identify front from CAD geometry")
    return tuple(float(x) for x in result / np.linalg.norm(result))


def solve_waypoint(model, reference, mujoco, leg, goal, initial):
    """Local 3-DOF inverse kinematics with a continuity penalty."""
    import numpy as np
    from scipy.optimize import least_squares
    from .stand_geometry_v04 import tip_world

    qa = leg["qadrs"]
    dofs = leg["dadrs"]
    original = np.array(initial, dtype=float).copy()
    goal = np.array(goal, dtype=float)
    reference.qvel[:] = 0.
    reg = .002
    def get_point(x):
        reference.qpos[qa] = x
        mujoco.mj_forward(model, reference)
        return tip_world(model, reference, leg)

    def residual(x):
        return np.r_[get_point(x) - goal, reg * (x - original)]

    def jacobian(x):
        p = get_point(x)
        jp = np.zeros((3, model.nv))
        jr = np.zeros((3, model.nv))
        mujoco.mj_jac(model, reference, jp, jr, p, int(leg["body"]))
        return np.vstack((jp[:, dofs], reg * np.eye(3)))

    bounds = (-np.ones(3) * MAX_IK_ANGLE, np.ones(3) * MAX_IK_ANGLE)
    fit = least_squares(residual, np.clip(original, *bounds), jac=jacobian,
                        bounds=bounds, max_nfev=160,
                        xtol=1e-9, ftol=1e-9, gtol=1e-9)
    error = float(np.linalg.norm(get_point(fit.x) - goal))
    return fit.x.copy(), error


def floor_contacts(model, data, mujoco) -> set[str]:
    """Report *actual* contact points, not merely low foot height."""
    from .stand_geometry_v04 import LEG_CODES
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    geoms = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                          f"tibia{suffix}_collision_0"): suffix
        for suffix in LEG_CODES
    }
    if floor < 0 or len(geoms) != 6 or min(geoms) < 0:
        raise ValueError("Missing six tibia colliders or ground")
    found = set()
    for idx in range(data.ncon):
        c = data.contact[idx]
        other = int(c.geom2 if c.geom1 == floor else
                    c.geom1 if c.geom2 == floor else -1)
        if other in geoms:
            found.add(geoms[other])
    return found


def experiment(
    scenario: str = "nominal", leg_name: str = SELECTED_LEG,
    *, cache: Path = DEFAULT_CACHE,
    step_forward: float = STEP_FORWARD_M, lift: float = STEP_LIFT_M,
) -> dict:
    import numpy as np
    import mujoco
    from .cad_mujoco_v04 import build
    from .stand_v04 import STAND_HEIGHT_M, TARGET_RADIAL_EXTENSION_M, static_pose, hold_under_gravity
    from .stand_geometry_v04 import load_legs, tip_world, geom_mesh_world_vertices
    from .simulation import support_margin
    from .freenove_servo_map_v04 import cad_joint_names

    if scenario not in ("lower", "nominal", "upper"):
        raise ValueError("Unknown mass scenario")
    if leg_name != "fr":
        raise ValueError("Only front-right first-step experiment is validated")
    if not (0.005 <= step_forward <= .05 and 0.005 <= lift <= .05):
        raise ValueError("Unreasonable step dimensions")

    source, construction = build(scenario, cache=cache, offline=True)
    model = mujoco.MjModel.from_xml_path(str(source))
    data = mujoco.MjData(model)
    legs = load_legs(model, data, mujoco)
    leg = next(x for x in legs if x["suffix"] == leg_name)
    model_joint_names = tuple(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in leg["jids"]
    )
    if model_joint_names != cad_joint_names(leg_name):
        raise AssertionError(f"Vendor/CAD joint-channel mapping differs: {model_joint_names}")
    fit = static_pose(model, data, mujoco, legs,
                      STAND_HEIGHT_M, TARGET_RADIAL_EXTENSION_M)
    initial_qpos = np.asarray(data.qpos).copy()
    settle = hold_under_gravity(model, data, mujoco, initial_qpos, SETTLE_SECONDS)
    if not settle["stable_by_proxy_thresholds"] or settle["last_0p4s_min_contact_legs"] < 6:
        raise ValueError("Six-leg standing baseline not ready for one-leg experiment")

    # Zero velocities in IK clone, but keep dynamically settled root pose.
    ref = mujoco.MjData(model)
    ref.qpos[:] = data.qpos
    ref.qvel[:] = 0.
    mujoco.mj_forward(model, ref)

    baseline_tip = tip_world(model, ref, leg).copy()
    baseline_mesh_min = float(geom_mesh_world_vertices(model, ref, leg["geom"])[:,2].min())
    forward_xy = np.array(forward_from_geometry(legs))
    start_target = np.asarray(baseline_tip).copy()
    lift_target = start_target + np.array([0., 0., lift])
    swing_target = lift_target + np.r_[forward_xy * step_forward, 0.]
    land_target = start_target + np.r_[forward_xy * step_forward, 0.]

    q0 = np.asarray(ref.qpos[leg["qadrs"]]).copy()
    q_lift, e_lift = solve_waypoint(model, ref, mujoco, leg, lift_target, q0)
    q_swing, e_swing = solve_waypoint(model, ref, mujoco, leg, swing_target, q_lift)
    q_land, e_land = solve_waypoint(model, ref, mujoco, leg, land_target, q_swing)
    # Restore saved pose for integration; calculations above changed only clone.
    mujoco.mj_forward(model, data)

    actuator_joints = [int(model.actuator_trnid[a, 0]) for a in range(model.nu)]
    qa = np.array([int(model.jnt_qposadr[j]) for j in actuator_joints], dtype=int)
    da = np.array([int(model.jnt_dofadr[j]) for j in actuator_joints], dtype=int)
    ref_q = np.asarray(data.qpos[qa]).copy()
    bounds = np.asarray(model.actuator_ctrlrange)
    moving_indices = [int(np.where(qa == adr)[0][0]) for adr in leg["qadrs"]]
    baseid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link_2")
    base_initial = np.asarray(data.xpos[baseid]).copy()
    rot_initial = np.asarray(data.xmat[baseid]).reshape(3,3).copy()
    local_up = rot_initial.T @ np.array([0., 0., 1.])
    n = round((LIFT_SECONDS + SWING_SECONDS + LAND_SECONDS + RECOVER_SECONDS) /
              model.opt.timestep)
    log = []
    peak_torque = 0.
    saturation_events = 0

    for k in range(n):
        t = k * model.opt.timestep
        name, _s = phases(t)
        ref_q[moving_indices] = commanded_joint_pose(
            t, q0, q_lift, q_swing, q_land)
        torque_unclipped = 7.0 * (ref_q - data.qpos[qa]) - .32 * data.qvel[da]
        saturation_events += int(np.any(np.abs(torque_unclipped) > bounds[:,1] + 1e-9))
        torque = np.clip(torque_unclipped, bounds[:,0], bounds[:,1])
        peak_torque = max(peak_torque, float(np.max(np.abs(torque))))
        data.ctrl[:] = torque
        mujoco.mj_step(model, data)
        if k % 10 == 0 or k == n - 1:
            if not np.all(np.isfinite(data.qpos)):
                raise ValueError(f"Numerically unstable MuJoCo step at {t:.3f}s")
            contacts = floor_contacts(model, data, mujoco)
            toe = tip_world(model, data, leg).copy()
            lifted_mesh_z = float(geom_mesh_world_vertices(
                model, data, leg["geom"])[:,2].min())
            trunk = np.asarray(data.xpos[baseid]).copy()
            orientation = np.asarray(data.xmat[baseid]).reshape(3,3)
            up = orientation @ local_up
            tilt = math.degrees(math.acos(float(np.clip(up @ [0.,0.,1.], -1,1))))
            com = (np.asarray(data.xipos) * np.asarray(model.body_mass)[:,None]
                   ).sum(axis=0) / float(np.sum(model.body_mass))
            supports = [tip_world(model, data, other)[:2]
                        for other in legs if other["suffix"] in contacts]
            margin = float(support_margin(tuple(com[:2]), [tuple(x) for x in supports]))
            log.append({
                "t_s": round(t + model.opt.timestep, 3),
                "phase": name,
                "base_xyz_m": [round(float(x),6) for x in trunk],
                "base_tilt_deg": round(tilt, 4),
                "contacts": sorted(contacts),
                "support_legs": len(contacts - {leg_name}),
                "stepping_leg_touching": leg_name in contacts,
                "support_polygon_margin_m": (round(margin, 6) if math.isfinite(margin) else None),
                "moving_toe_xyz_m": [round(float(x),6) for x in toe],
                "moving_mesh_min_z_m": round(lifted_mesh_z,6),
                "toe_forward_m": round(float(np.dot(toe[:2]-baseline_tip[:2],forward_xy)), 6),
                "max_abs_command_nm": round(float(np.max(np.abs(torque))),4),
            })

    swing_frames = [x for x in log if x["phase"] == "swing"]
    swing_off_ground = sum(not x["stepping_leg_touching"] for x in swing_frames) / max(1,len(swing_frames))
    five_support_frames = sum(x["support_legs"] >= 5 for x in swing_frames) / max(1,len(swing_frames))
    last = [x for x in log if x["phase"] == "recover"]
    land_contacts = sum(x["stepping_leg_touching"] for x in last[-15:]) / max(1,len(last[-15:]))
    max_lift = max(x["moving_mesh_min_z_m"] for x in log) - baseline_mesh_min
    max_forward = max(x["toe_forward_m"] for x in log)
    final_forward = log[-1]["toe_forward_m"]
    min_support_margin = min((x["support_polygon_margin_m"]
                              for x in swing_frames if x["support_polygon_margin_m"] is not None),
                             default=None)
    max_tilt = max(x["base_tilt_deg"] for x in log)
    max_drift = max(float(np.linalg.norm(
        np.asarray(x["base_xyz_m"][:2]) - base_initial[:2])) for x in log)
    min_height = min(x["base_xyz_m"][2] for x in log)
    criteria = {
        "ik_waypoints_each_within_5mm": max(e_lift,e_swing,e_land)<.005,
        "support_has_five_legs_at_least_80pct_swing": five_support_frames >= .8,
        "moving_leg_off_ground_at_least_70pct_swing": swing_off_ground >= .7,
        "lift_mesh_at_least_12mm": max_lift >= .012,
        "moving_toe_forward_at_least_15mm": max_forward >= .015,
        "landed_again_at_least_80pct_last_samples": land_contacts >= .8,
        "five_leg_support_margin_positive": min_support_margin is not None and min_support_margin > .015,
        "body_below_10deg_tilt": max_tilt < 10.,
        "body_above_8cm_height": min_height > .08,
        "body_drift_below_20mm": max_drift < .020,
        "peak_torque_at_or_below_assumed_0p7Nm": peak_torque <= .700001,
    }
    return {
        "test":"First single-leg step, forward +Y derived from CAD hip geometry",
        "scenario":scenario,
        "model_mass_kg":round(float(sum(model.body_mass)),6),
        "selected_leg":leg_name,
        "vendor_servo_channels_and_joint_names":list(model_joint_names),
        "authoritative_joint_PWM_mapping_verified":False,
        "cad_to_vendor_PWM_signs_calibrated":False,
        "forward_direction_xy": [round(float(x),6) for x in forward_xy],
        "step_forward_command_m":step_forward,
        "step_lift_command_m":lift,
        "cad_source_commit":construction["upstream_commit"],
        "baseline_standing_hold":{k:v for k,v in settle.items() if k not in ("time_series_50hz","final_qpos")},
        "baseline_toe_xyz_m":[round(float(x),6) for x in baseline_tip],
        "target_ik_errors_mm":{
            "lift":round(1000*e_lift,3),"swing":round(1000*e_swing,3),
            "land":round(1000*e_land,3),
        },
        "waypoint_joint_angles_rad":{
            "stand":[round(float(x),6) for x in q0],
            "lift":[round(float(x),6) for x in q_lift],
            "swing":[round(float(x),6) for x in q_swing],
            "land":[round(float(x),6) for x in q_land],
        },
        "observed":{
            "lifted_leg_no_contact_fraction_swing":round(swing_off_ground,4),
            "five_support_fraction_swing":round(five_support_frames,4),
            "landing_contact_fraction_tail":round(land_contacts,4),
            "max_measured_mesh_lift_m":round(max_lift,6),
            "maximum_toe_forward_m":round(max_forward,6),
            "final_toe_forward_m":round(final_forward,6),
            "minimum_support_polygon_margin_m":min_support_margin,
            "max_body_tilt_deg":round(max_tilt,3),
            "max_body_xy_drift_m":round(max_drift,6),
            "min_body_height_m":round(min_height,6),
            "max_abs_command_nm":round(peak_torque,4),
            "torque_saturation_steps":saturation_events,
        },
        "criteria":criteria,
        "candidate_success":bool(all(criteria.values())),
        "limitations":[
            "This uses estimated CAD inertia, not measured robot masses",
            "Motor speed/power/temperature and sensor feedback are not modelled",
            "Attribution of actuator joint to vendor PWM channel is checked; its direction/zero is not validated",
            "Farthest CAD tibia vertex is a provisional foot landmark; contact hull is an approximation",
            "PD servo torque command ±0.7 Nm is a modelling assumption, not demonstrated continuous output",
            "The test moves one foot forward, not the entire robot's body",
        ],
        "trace_50hz":log,
    }


def main() -> None:
    p=argparse.ArgumentParser()
    p.add_argument("--scenario",choices=("lower","nominal","upper"),default="nominal")
    p.add_argument("--leg",choices=("fr",),default="fr")
    p.add_argument("--lift-mm",type=float,default=STEP_LIFT_M * 1000)
    p.add_argument("--forward-mm",type=float,default=STEP_FORWARD_M * 1000)
    p.add_argument("--cache",type=Path,default=DEFAULT_CACHE)
    p.add_argument("--out",type=Path)
    a=p.parse_args()
    report=experiment(a.scenario,a.leg,cache=a.cache,
                      lift=a.lift_mm/1000.,step_forward=a.forward_mm/1000.)
    if a.out:
        a.out.parent.mkdir(parents=True,exist_ok=True)
        a.out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n")
    summary={k:v for k,v in report.items() if k!="trace_50hz"}
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
