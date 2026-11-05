"""Reproduce CAD30 Drake/MuJoCo G0b contact and actuator parity on private inputs.

Does not expose any CAD mesh, joint configuration, or full physics trace in JSON.
The numerical test deliberately reports a *negative* contact-force parity result.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

LEGS = ("ml", "mr", "bl", "br", "fl", "fr")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def drake_probe(path: Path) -> dict:
    output={"actuators":{},"gaps":{},"fz":{},"damping":{}}
    for line in path.read_text(encoding="utf-8").splitlines():
        arr=line.split()
        if not arr: continue
        k, v=arr[0],arr[1:]
        if k=="meta": output['meta']=[int(x) for x in v]
        elif k=="actuator": output['actuators'][v[0]]={'effort_limit':float(v[1]),'rotor_inertia':float(v[2])}
        elif k=="joint_damping": output['damping'][v[0]]=float(v[1])
        elif k=="mass_diagonal": output['mass_diag']=np.array([float(x) for x in v])
        elif k=="ground_pair":
            matching=[leg for leg in LEGS if 'tibia'+leg in line]
            if matching: output['gaps'][matching[0]]=float(v[-1])
        elif k=="drake_fz":
            leg=v[0].replace('tibia','');
            if leg in LEGS: output['fz'][leg]=float(v[1])
        elif k=="point_contact_count":output['point_contact_count']=int(v[0])
    if not {'mass_diag','meta'}.issubset(output) or not output['actuators']:
        raise ValueError(f'Incomplete independent Drake probe: {path}')
    return output


def mujoco_snapshot(scene: Path, state: Path) -> dict:
    import mujoco
    m=mujoco.MjModel.from_xml_path(str(scene))
    data=mujoco.MjData(m)
    js=json.loads(state.read_text(encoding='utf-8'))
    data.qpos[:]=np.array(js['final_qpos'])
    data.qvel[:]=np.array(js['final_qvel'])
    goals=np.array(js.get('final_goals',js.get('final_motor_goal')))
    jidx=m.actuator_trnid[:,0]
    qj=data.qpos[m.jnt_qposadr[jidx]]
    vj=data.qvel[m.jnt_dofadr[jidx]]
    data.ctrl[:]=np.clip(7*(goals-qj)-0.32*vj,-0.7,0.7)
    mujoco.mj_forward(m,data)
    M=np.zeros((m.nv,m.nv))
    mujoco.mj_fullM(m,data,M)
    base_id=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'base_link_2')
    R=data.xmat[base_id].reshape(3,3)
    T=np.eye(m.nv)
    T[:6,:6]=0
    T[:3,3:6]=R
    T[3:6,:3]=np.eye(3)
    mass_diag=np.diag(T@M@T.T)
    gaps={leg:[] for leg in LEGS}
    for contact in data.contact:
        names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(i)) or '' for i in [contact.geom1,contact.geom2]]
        for leg in LEGS:
            if 'tibia'+leg+'_collision_0' in names and 'floor' in names:
                gaps[leg].append(float(contact.dist))
    # MuJoCo mj_step computes reaction over the step from this exact initial state.
    mujoco.mj_step(m,data)
    fz={leg:0. for leg in LEGS}
    for i,contact in enumerate(data.contact):
        nameA=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(contact.geom1)) or ''
        nameB=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(contact.geom2)) or ''
        if 'floor' not in (nameA,nameB): continue
        leg=next((leg for leg in LEGS if 'tibia'+leg+'_collision_0' in (nameA,nameB)),None)
        if leg is None: continue
        F=np.zeros(6)
        mujoco.mj_contactForce(m,data,i,F)
        # First contact frame axis is the contact normal, expressed in world frame.
        # For the floor-tibia pairs used here its world-Z component is positive.
        Fz=F[0]*contact.frame.reshape(3,3)[0,2]
        fz[leg]+=float(Fz)
    return {
        'mass_diag':mass_diag,
        'min_gaps':{leg:min(g) for leg,g in gaps.items() if g},
        'fz':fz,
        'contact_pair_count_after_2ms':int(data.ncon),
        'nq':int(m.nq),'nv':int(m.nv),'nu':int(m.nu),
        'mu_armature_max':float(np.max(m.dof_armature)),
        'mu_armature_nonzero':int(np.count_nonzero(m.dof_armature)),
        'mu_joint_frictionloss_max':float(np.max(m.dof_frictionloss)),
        'mu_joint_damping_max':float(np.max(m.dof_damping)),
        'mu_root_final_z':float(data.qpos[2]),
    }


def audit(args) -> dict:
    meta={
        'schema':'cad30-G0b-drake-vs-mujoco-20261105',
        'method':'same state, rotor-inertia-corrected Drake discrete 2ms step vs MuJoCo 2ms step with identical PD goals; geometry signed-distance query before step',
        'mujoco_version':'3.14.0', 'drake_version':'1.30.0',
        'source_checksums_sha256':{
            'private_mu_plane_xml':sha256(args.mu_plane),
            'private_drake_box_xml':sha256(args.drake_box),
            'private_six_state':sha256(args.state_six),
            'private_four_state':sha256(args.state_four),
        },'config':{'dt_s':0.002,'pd_Kp':7.,'pd_Kd':0.32,'model_joint_torque_limit_Nm':0.7,
                   'drake_discrete_contact':'point-pair default, no explicit MuJoCo solref/solimp mapping',
                   'mu_plane_vs_box_test':True}
    }
    all_results={}
    for k,state in [('six',args.state_six),('four',args.state_four)]:
        unmodified=drake_probe(args.drake_logs/f'drake_{k}_original.out')
        corrected=drake_probe(args.drake_logs/f'drake_{k}_discrete.out')
        mu_plane=mujoco_snapshot(args.mu_plane,state)
        mu_box=mujoco_snapshot(args.drake_box,state)
        assert mu_box['nq']==corrected['meta'][0]
        assert mu_box['nv']==corrected['meta'][1]
        assert mu_box['nu']==corrected['meta'][2]
        mu_gap=mu_box['min_gaps'];dr_gap=corrected['gaps']
        keys=sorted(set(mu_gap)&set(dr_gap))
        dmass_before=max(abs(unmodified['mass_diag']-mu_box['mass_diag']))
        dmass_after=max(abs(corrected['mass_diag']-mu_box['mass_diag']))
        fixed_names=set(corrected['actuators'])
        legs_all=[x for x in fixed_names if x not in ('motor0_ctrl','motor1_ctrl')]
        caps_pass=(len(legs_all)==18 and all(abs(corrected['actuators'][n]['effort_limit']-0.7)<1e-12 for n in legs_all)
                   and all(abs(corrected['actuators'][n]['rotor_inertia']-0.002)<1e-12 for n in fixed_names)
                   and all(abs(corrected['actuators'][n]['effort_limit']-0.2)<1e-12 for n in ('motor0_ctrl','motor1_ctrl')))
        orig_caps=sum(np.isfinite(a['effort_limit']) for a in unmodified['actuators'].values())
        gapdiff=max(abs(mu_gap[n]-dr_gap[n]) for n in keys)
        fza=corrected['fz'];fzb=mu_box['fz'];
        all_results[k]={
           'source_state':k+'_private_CAD30_PhysicalSnapshot',
           'mu_floor_plane':{'n_contact_pairs':mu_plane['contact_pair_count_after_2ms'],'per_leg_Fz_N':mu_plane['fz'],'sum_Fz_N':sum(mu_plane['fz'].values())},
           'mu_floor_box':{'n_contact_pairs':mu_box['contact_pair_count_after_2ms'],'per_leg_Fz_N':fzb,'sum_Fz_N':sum(fzb.values())},
           'drake_box':{'n_contact_points':corrected['point_contact_count'],'per_leg_Fz_N':fza,'sum_Fz_N':sum(fza.values())},
           'max_contact_gap_abs_delta_m':float(gapdiff), 'compared_leg_count':len(keys),
           'max_mass_diag_delta_before_correction':float(dmass_before),
           'max_mass_diag_delta_after_correction':float(dmass_after),
           'drake_original_finite_actuators':int(orig_caps),
           'drake_corrected_finite_actuators':int(sum(bool(np.isfinite(a['effort_limit'])) for a in corrected['actuators'].values())),
           'drake_repaired_actuator_metadata_pass':bool(caps_pass),
           'max_per_leg_Fz_abs_delta_mu_box_vs_drake_N':float(max(abs(fzb[n]-fza[n]) for n in LEGS)),
           'force_parity_at_same_step':False,
        }
    meta['snapshots']=all_results
    meta['verdict']={
       'contact_geometry_signed_distance_parity':all(v['max_contact_gap_abs_delta_m']<2e-8 for v in all_results.values()),
       'inertial_diagonal_and_actuator_metadata_parity_after_repair':all(v['max_mass_diag_delta_after_correction']<1e-10 and v['drake_repaired_actuator_metadata_pass'] for v in all_results.values()),
       'contact_force_parity':'FAIL',
       'drake_model_with_corrected_actuators_ran_one_2ms_step':True,
       'solver_optimizer_with_corrected_actuators':'NOT_RUN',
       'full_actuation_contact_parity':'FAIL',
       'periodic_gait':'NOT_FOUND',
    }
    meta['limitations']=[
        'MuJoCo solref/solimp/contact cone and Drake default compliant point contact are not equivalent; different stiffness/contact resolution algorithms',
        'MuJoCo mj_step contact reaction corresponds to start of a 2ms integration and Drake last-step reported contact reaction; direct force equality is not expected without contact calibration',
        'Joint effort_limit metadata in Drake MultibodyPlant does not saturate inputs by itself; optimizer and controller must explicitly enforce bounds',
        'MuJoCo frictionloss 0.015 Nm joint dry friction is not reproduced by the Drake Parser; no forced equivalence claimed',
        'No original CAD/STL/OBJ/joint-state vectors or full traces embedded; model and snapshots only owner Drive',
        'Two static checkpoints, not a dynamic walking validation; no hard-contact gait optimized',
    ]
    return meta


def main():
    a=argparse.ArgumentParser()
    a.add_argument('--mu-plane',required=True,type=Path)
    a.add_argument('--drake-box',required=True,type=Path)
    a.add_argument('--state-six',required=True,type=Path)
    a.add_argument('--state-four',required=True,type=Path)
    a.add_argument('--drake-logs',required=True,type=Path)
    a.add_argument('--out',required=True,type=Path)
    args=a.parse_args()
    output=audit(args)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({'verdict':output['verdict'],'force_sums':{k:[v['mu_floor_box']['sum_Fz_N'],v['drake_box']['sum_Fz_N']] for k,v in output['snapshots'].items()}},indent=2))

if __name__=='__main__':main()
