"""Validate stance-foot Jacobians and a small WBC correction on the genuine CAD30 pose B.

Uses preserved MuJoCo qpos from the archived real dynamic simulation as initial
state, performs only mj_forward / mj_integratePos kinematic probes. Does NOT
simulate locomotion or send torque commands, and does NOT redistribute forces.
The original CAD30/STL data must be restored outside GitHub.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import zipfile

import mujoco
import numpy as np

from wbc_contact_correction import micro_correct_stance

STANCE = ("ml", "mr", "bl", "br")
ALL_LEGS = ("ml", "mr", "bl", "br", "fl", "fr")
REPLAY_JSON = "results/claw_tuck_compare_2026-11-04.json"


def load_qpos(replay_zip: Path):
    with zipfile.ZipFile(replay_zip) as archive:
        data = json.loads(archive.read(REPLAY_JSON))
    frames = data["semi_tucked"]["qpos_samples_10hz"]
    frame = min(frames, key=lambda x: abs(x["t"] - 9.3))
    if abs(frame["t"] - 9.3) > 1e-3:
        raise ValueError("Expected genuine semi_tucked B pose at 9.3 seconds")
    return np.asarray(frame["qpos"], dtype=float), frame["t"]


def foot_world(data, body_id: int, local: np.ndarray) -> np.ndarray:
    return np.asarray(data.xpos[body_id]) + np.asarray(data.xmat[body_id]).reshape(3,3) @ local


def prepare(model_path: Path, replay_zip: Path):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    q, timestamp = load_qpos(replay_zip)
    if q.shape != (model.nq,):
        raise ValueError(f"qpos shape {q.shape} does not match model nq={model.nq}")
    data.qpos[:] = q
    data.qvel[:] = 0
    mujoco.mj_forward(model,data)
    # Preserved B has four genuine ground contacts (two per stance tibia);
    # front claws fl/fr do not touch the floor.
    floor = int(model.geom("floor").id)
    leg_by_geom = {int(model.geom(f"tibia{k}_collision_0").id):k for k in ALL_LEGS}
    points = {k:[] for k in ALL_LEGS}
    for i in range(data.ncon):
        c = data.contact[i]
        other = int(c.geom2) if int(c.geom1) == floor else int(c.geom1) if int(c.geom2) == floor else -1
        if other in leg_by_geom:
            points[leg_by_geom[other]].append(np.asarray(c.pos).copy())
    if set(k for k,pts in points.items() if pts) != set(STANCE):
        raise AssertionError(f"Expected four contacting legs, got { {k:len(v) for k,v in points.items()} }")
    # One representative body-local landmark per stance tibia. It is tracked
    # after perturbation independently of contact solver's point reselection.
    anchors = []
    for leg in STANCE:
        body_id = int(model.body(f"tibia{leg}").id)
        p0 = np.mean(points[leg], axis=0)
        body_rot = np.asarray(data.xmat[body_id]).reshape(3,3)
        local = body_rot.T@(p0 - np.asarray(data.xpos[body_id]))
        anchors.append((leg,body_id,local,p0))
    leg_jids = [int(model.body(f"{link}{leg}").jntadr[0]) for leg in ALL_LEGS for link in ("coxa","femur","tibia")]
    if len(set(leg_jids)) != 18:
        raise AssertionError("Unexpected number of leg joint IDs")
    dofs = np.array([int(model.jnt_dofadr[jid]) for jid in leg_jids],dtype=int)
    qaddrs = np.array([int(model.jnt_qposadr[jid]) for jid in leg_jids],dtype=int)
    if np.any(dofs < 6) or len(set(dofs)) != 18:
        raise AssertionError("Expected 18 independent leg DOFs")
    lower = np.array([model.jnt_range[jid][0] for jid in leg_jids])
    upper = np.array([model.jnt_range[jid][1] for jid in leg_jids])
    Jbase = np.zeros((12,6)); Jjoint=np.zeros((12,18))
    for i,(_,body_id,_,p0) in enumerate(anchors):
        jp = np.zeros((3,model.nv));jr=np.zeros((3,model.nv))
        mujoco.mj_jac(model,data,jp,jr,p0,body_id)
        Jbase[3*i:3*i+3] = jp[:,:6]
        Jjoint[3*i:3*i+3] = jp[:,dofs]
    return model,data,anchors,Jbase,Jjoint,leg_jids,dofs,qaddrs,lower,upper,timestamp


def integrate(model, baseline, delta):
    d=mujoco.MjData(model)
    d.qpos[:]=baseline.qpos
    mujoco.mj_integratePos(model,d.qpos,delta,1.0)
    d.qvel[:]=0
    mujoco.mj_forward(model,d)
    return d


def landmark_vector(data, anchors):
    return np.hstack([foot_world(data,body,local) for _,body,local,_ in anchors])


def run(model_path: Path, replay_zip: Path):
    m,d,anchors,jb,jq,jids,dofs,qaddrs,lo,hi,t=prepare(model_path,replay_zip)
    J=np.zeros((12,m.nv));J[:,:6]=jb;J[:,dofs]=jq
    reference=landmark_vector(d,anchors)
    # Independently perturb all 6 free-body + 18 actuated-leg DOFs.
    eps=1e-6
    err=[]; columns=[*range(6),*dofs]
    observed_j=np.zeros((12,len(columns))); analytic_j=np.zeros_like(observed_j)
    for idx,col in enumerate(columns):
        delta=np.zeros(m.nv);delta[col]=eps
        plus=integrate(m,d,delta)
        delta[col]=-eps
        minus=integrate(m,d,delta)
        fd=(landmark_vector(plus,anchors)-landmark_vector(minus,anchors))/(2*eps)
        observed_j[:,idx]=fd
        analytic_j[:,idx]=J[:,col]
        err.append(float(np.max(np.abs(fd-J[:,col]))))
    # Requested motion: 0.35 mm forward, 0.1 mm down, a small 3D rotation.
    base_delta=np.array([0.,.00035,-.0001,.0003,-.0002,.00015],dtype=float)
    correction=micro_correct_stance(
      joint_positions_rad=d.qpos[qaddrs], lower_limits_rad=lo,
      upper_limits_rad=hi, stance_jacobian_base=jb,
      stance_jacobian_joints=jq, desired_base_delta=base_delta,
      max_joint_delta_rad=.02, max_base_translation_m=.002,
      max_base_rotation_rad=.005, max_contact_linear_residual_m=2e-5)
    delta_free=np.zeros(m.nv);delta_free[:6]=base_delta
    moving_base_only=integrate(m,d,delta_free)
    drift_base=np.linalg.norm((landmark_vector(moving_base_only,anchors)-reference).reshape(-1,3),axis=1)
    if correction.approved_kinematic:
        for adr,value in zip(dofs,correction.joint_delta_rad):delta_free[adr]=value
        corrected=integrate(m,d,delta_free)
        drift_corrected=np.linalg.norm((landmark_vector(corrected,anchors)-reference).reshape(-1,3),axis=1)
    else:
        drift_corrected=np.full(4,np.nan)
    def safe(x):return round(float(x),10)
    result={
      "schema":"cad30-pose-b-jacobian-v1",
      "status":"KINEMATIC_ONLY_PASS" if correction.approved_kinematic and max(err)<2e-6 and max(drift_corrected)<2e-5 else "CHECK_FAILED",
      "environment":{"mujoco":mujoco.__version__,"model":"FNK0052 CAD30 nominal, locally restored from user-owned Google Drive archive", "nq":int(m.nq),"nv":int(m.nv),"nact":int(m.nu),"nmesh":int(m.nmesh)},
      "state":{"pose":"semi_tucked B","time_s":t,"source":"results/claw_tuck_compare_2026-11-04.json from Drive ZIP; physical MuJoCo replay", "four_stance_legs":list(STANCE),"contacts_per_leg":2,"qpos_length":len(d.qpos),"joints_outside_research_IK_1p7_rad":[{"name":m.joint(jid).name,"q_rad":round(float(d.qpos[qa]),7)} for jid,qa in zip(jids,qaddrs) if abs(d.qpos[qa])>1.7]},
      "finite_differences":{"h_m_or_rad":eps,"tested_generalized_dofs":24,"max_jacobian_error":float(f"{max(err):.8g}"),"rmse_jacobian":float(f"{np.sqrt(np.mean((observed_j-analytic_j)**2)):.8g}")},
      "micro_correction":{"requested_body_twist_6":base_delta.tolist(),"approved_kinematic":correction.approved_kinematic,"reason":correction.reason,"joint_max_abs_delta_rad":safe(max(abs(x) for x in correction.joint_delta_rad)),
        "predicted_max_contact_linear_residual_m":safe(correction.max_contact_linear_residual_m),
        "base_only_foot_drift_mm":[round(float(v)*1000,6) for v in drift_base],
        "wbc_compensated_foot_drift_mm":[round(float(v)*1000,6) for v in drift_corrected],
        "max_compensated_drift_mm":round(float(max(drift_corrected))*1000,6) if correction.approved_kinematic else None,
        "joint_deltas_rad": [round(float(x),9) for x in correction.joint_delta_rad]
      },
      "caveats":["Jacobian finite difference and corrected pose checked kinematically via mj_forward, NOT dynamic mj_step.","Ground contact geometry tracked at baseline tibia-local points; actual future load, slip and friction not certified.","Joint bounds from approximate MuJoCo model ±pi rad, NOT hardware servo limits nor conservative ±1.7-rad IK bound.","Only one small requested body correction and one preserved pose B were tested; no MPC sequence or gait."],
    }
    return result


def main():
    cli=argparse.ArgumentParser()
    cli.add_argument("--model",type=Path,required=True)
    cli.add_argument("--replay-zip",type=Path,required=True)
    cli.add_argument("--out",type=Path)
    args=cli.parse_args()
    report=run(args.model,args.replay_zip)
    result=json.dumps(report,ensure_ascii=False,indent=2)
    if args.out:
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(result+"\n",encoding='utf-8')
    print(result)
    if report['status'] != 'KINEMATIC_ONLY_PASS':raise SystemExit(1)


if __name__=="__main__":main()
