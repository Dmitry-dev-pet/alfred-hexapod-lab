"""G0 independent MuJoCo 3.14 vs Drake 1.30 CAD30 kinematic/inertial parity audit.

Run Drake C++ probe separately; use the *same exact private model* for both
engines. No CAD geometry or raw joint poses are written into output JSON.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import math
import numpy as np
import mujoco

LEG_BODIES = ("tibiaml", "tibiamr", "tibiabl", "tibiabr", "tibiafl", "tibiafr")


def parse_drake_probe(path: Path) -> dict:
    data = {"joints": [], "actuators": [], "bodies": {}, "mrows": []}
    for line in path.read_text(encoding="utf-8").splitlines():
        items = line.split()
        if not items: continue
        k, *v = items
        if k == "meta": data["meta"] = [float(x) for x in v]
        elif k in ("com", "q0", "gravity"): data[k] = np.array([float(x) for x in v])
        elif k == "joint": data["joints"].append(v[0])
        elif k == "actuator": data["actuators"].append((v[0], v[1], float(v[2])))
        elif k == "mrow": data["mrows"].append(np.array([float(x) for x in v[1:]]))
        elif k == "bodyxyz": data["bodies"][v[0]] = np.array([float(x) for x in v[1:]])
        elif k == "geometry": data["geometry"] = [int(x) for x in v]
    if not {"meta", "com", "q0", "gravity"}.issubset(data) or not data["mrows"]:
        raise ValueError("Incomplete Drake probe output")
    data["M"] = np.stack(data.pop("mrows"))
    return data


def compare(xml: Path, probe_file: Path) -> dict:
    d = parse_drake_probe(probe_file)
    m = mujoco.MjModel.from_xml_path(str(xml))
    sim = mujoco.MjData(m)
    mujoco.mj_forward(m, sim)
    nu = m.nu
    nq, nv = m.nq, m.nv
    dims = d["meta"]
    # The quaternion comes before xyz in Drake, after xyz in MuJoCo.
    q_drake_in_mu_order = np.r_[d["q0"][4:7], d["q0"][:4], d["q0"][7:]]
    # MuJoCo freejoint angular velocity is expressed in local body coordinates;
    # Drake quaternion floating joint uses world angular velocity, then world linear.
    # Therefore the kinetic-energy metric and generalized force must be rotated.
    R = sim.xmat[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link_2")].reshape(3,3)
    T = np.eye(nv)
    T[:6, :6] = 0
    T[:3, 3:6] = R
    T[3:6, :3] = np.eye(3)
    M_mu = np.zeros((nv,nv),dtype=float)
    mujoco.mj_fullM(m,sim,M_mu)
    M_mu_without_armature = M_mu - np.diag(m.dof_armature)
    M_reframed = T @ M_mu_without_armature @ T.T
    grav_mu_world = T @ (-np.asarray(sim.qfrc_bias))
    body_errors = {}
    for name, xyz in d["bodies"].items():
        bodyid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        if bodyid < 0: raise ValueError(f"Unknown body: {name}")
        body_errors[name] = float(np.max(np.abs(xyz - sim.xpos[bodyid])))
    joint_names = [mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,i)
                   for i in range(m.njnt) if m.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE]
    actuator_names = [mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_ACTUATOR,i) for i in range(nu)]
    names = [x[0] for x in d["actuators"]]
    # The MJCF defaults for motor control/force limits are not inherited by
    # Drake 1.30 in this model (except two head motors with explicit caps).
    effort_uncapped = [x[0] for x in d["actuators"] if not math.isfinite(x[2])]
    report = {
        "schema": "cad30-drake-mujoco-G0-parity-v1",
        "source_sha256": hashlib.sha256(xml.read_bytes()).hexdigest(),
        "mujoco_version": mujoco.__version__,
        "drake_version": "1.30.0",
        "counts": {"mu_nq": nq, "drake_nq": int(dims[0]), "mu_nv": nv,
                   "drake_nv": int(dims[1]), "mu_bodies": m.nbody,
                   "drake_bodies": int(dims[2]), "mu_actuators": nu,
                   "drake_actuators": int(dims[3]), "drake_geometry_total_and_proximity": d.get("geometry")},
        "mass_kg": {"mujoco":float(np.sum(m.body_mass)), "drake":float(dims[4])},
        "max_default_q_delta": float(np.max(np.abs(q_drake_in_mu_order - m.qpos0))),
        "max_default_com_delta_m": float(np.max(np.abs(d["com"] - np.average(sim.xipos[1:],axis=0,weights=m.body_mass[1:])))),
        "joint_names_and_order_match": joint_names == d["joints"],
        "actuator_names_and_order_match": actuator_names == names,
        "max_6_tibia_body_origin_delta_m": max(body_errors.values()) if len(body_errors) == 6 else None,
        "tibia_errors_m": body_errors,
        "max_gravity_generalized_force_delta_N_or_Nm": float(np.max(np.abs(d["gravity"]-grav_mu_world))),
        "max_inertial_matrix_delta_after_removing_mu_armature": float(np.max(np.abs(d["M"]-M_reframed))),
        "max_raw_inertial_matrix_delta": float(np.max(np.abs(d["M"]-T@M_mu@T.T))),
        "mu_joint_armature": float(max(m.dof_armature)),
        "armature_joint_count": int(np.count_nonzero(m.dof_armature)),
        "drake_uncapped_actuator_names": effort_uncapped,
        "mu_contact_geom_count": int(sum(m.geom_contype>0)),
        "caveats": [
            "Kinematic/inertial parity, not contact dynamics or gait",
            "MuJoCo joint armature 0.002 kg m^2 ignored by Drake 1.30 MJCF Parser",
            "Drake parsed actuator effort limits as inf for limb motors; 0.7 Nm must be explicitly enforced",
            "Drake MJCF Parser ignores joint frictionloss, joint armature and some contact solver settings",
            "OBJ mesh format is needed for Drake 1.30; private meshes are never included in reports",
            "Servo PD gains/saturations and contact normal/friction dynamics not yet cross-validated"
        ],
    }
    report["kinematic_inertial_parity_pass"] = bool(
        nq == d["q0"].size == int(dims[0]) and nv == d["M"].shape[0] == int(dims[1])
        and nu == int(dims[3]) and m.nbody == int(dims[2])
        and abs(np.sum(m.body_mass)-dims[4])<1e-10
        and report["max_default_q_delta"] < 1e-10
        and report["max_default_com_delta_m"] < 1e-10
        and report["joint_names_and_order_match"]
        and report["actuator_names_and_order_match"]
        and report["max_6_tibia_body_origin_delta_m"] is not None
        and report["max_6_tibia_body_origin_delta_m"]<1e-10
        and report["max_gravity_generalized_force_delta_N_or_Nm"]<1e-9
        and report["max_inertial_matrix_delta_after_removing_mu_armature"]<1e-9)
    report["G0_full_contact_actuator_parity"] = "NOT_VERIFIED"
    report["IDTO_CAD30_solver_status"] = "NOT_RUN"
    return report


def main() -> None:
    ap=argparse.ArgumentParser()
    ap.add_argument("--mujoco-xml", type=Path, required=True)
    ap.add_argument("--drake-probe", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    args=ap.parse_args()
    result=compare(args.mujoco_xml,args.drake_probe)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("KINEMATIC_INERTIAL_PARITY_PASS" if result["kinematic_inertial_parity_pass"] else "PARITY_FAIL")
    print("max mass matrix delta",result["max_inertial_matrix_delta_after_removing_mu_armature"])
    if not result["kinematic_inertial_parity_pass"]: raise SystemExit(2)


if __name__ == "__main__":
    main()
