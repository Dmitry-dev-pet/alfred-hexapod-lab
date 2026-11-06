#!/usr/bin/env python3
"""CPU-only explicit XGO-Lite3 ONNX -> natural-joint-order -> MuJoCo A/B.

Not native mjlab: independently implements its play-mode observations and actuation
with the original STL-backed MJCF, a simple flat ground scene, without training
randomisation. Never controls hardware. All numerical physics is mj_step.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import onnxruntime as ort
import torch
from scipy.spatial.transform import Rotation

BASE = Path(os.environ.get('LITE3_RESEARCH_ROOT', str(Path(__file__).resolve().parent)))
ASSETS = BASE / 'original_assets'
SCENE = ASSETS / 'upstream/src/assets/robots/lite3/xmls/scene_lite3.xml'
ONNX = ASSETS / 'lite3-hf-policy/lite3_walk/policy.onnx'
PT = ASSETS / 'lite3-hf-policy/lite3_walk/model_1499.pt'
FEET = ('FL','FR','RL','RR')


def sha256(p: Path) -> str:
    with p.open('rb') as f:
        return hashlib.file_digest(f,'sha256').hexdigest()


def get_model() -> mujoco.MjModel:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.opt.timestep = 0.004
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.opt.solver = mujoco.mjtSolver.mjSOL_NEWTON
    model.opt.iterations = 10
    model.opt.tolerance = 1e-8
    model.opt.ccd_iterations = 50
    named = []
    for gid in range(model.ngeom):
        geom = mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,gid)
        if geom == 'floor':
            model.geom_contype[gid] = 1
            model.geom_conaffinity[gid] = 1
        elif geom in {f+'_shank_collision' for f in FEET}:
            model.geom_contype[gid] = 0
            model.geom_conaffinity[gid] = 1
            model.geom_priority[gid] = 1
            model.geom_friction[gid,0] = 0.6
            model.geom_solimp[gid,:3] = (0.9, 0.95, 0.023)
            named.append(geom)
        else:
            model.geom_contype[gid] = 0
            model.geom_conaffinity[gid] = 0
    if len(named) != 4:
        raise ValueError(f'Expected four shank contacts; got {named}')
    return model


def make_policy() -> tuple[ort.InferenceSession,list[str],dict[str,str]]:
    so = ort.SessionOptions()
    so.intra_op_num_threads = 1
    so.inter_op_num_threads = 1
    session = ort.InferenceSession(str(ONNX),sess_options=so,providers=['CPUExecutionProvider'])
    assert len(session.get_inputs()) == 1 and session.get_inputs()[0].shape == [1,53]
    assert len(session.get_outputs()) == 1 and session.get_outputs()[0].shape == [1,14]
    meta = session.get_modelmeta().custom_metadata_map
    joints = meta['joint_names'].split(',')
    assert len(joints)==len(set(joints))==14
    return session,joints,meta


def mapping(model: mujoco.MjModel,joints: list[str]):
    """Map 14 NN outputs (natural joint order) to 14 motor control indices."""
    model_order=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,i)
                 for i in range(model.njnt) if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE]
    motor_order=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,int(model.actuator_trnid[a,0])) for a in range(model.nu)]
    if joints != model_order:
        raise RuntimeError(f'Policy joint list != physics natural joint list: {joints} vs {model_order}')
    if set(joints)!=set(motor_order): raise RuntimeError('Actuator joint set != ONNX joint set')
    q_index=np.array([int(model.jnt_qposadr[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n)]) for n in joints],dtype=np.int64)
    v_index=np.array([int(model.jnt_dofadr[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n)]) for n in joints],dtype=np.int64)
    act_index=np.array([motor_order.index(n) for n in joints],dtype=np.int64)
    return q_index,v_index,act_index,model_order,motor_order


def torch_actor(obs: np.ndarray, ckpt: dict[str, Any]) -> np.ndarray:
    w=ckpt['actor_state_dict']
    z=(obs.astype(np.float64)-w['obs_normalizer._mean'].numpy().reshape(-1)) / np.maximum(w['obs_normalizer._std'].numpy().reshape(-1),1e-8)
    for i,idx in enumerate((0,2,4,6)):
        z=w[f'mlp.{idx}.weight'].numpy().astype(np.float64)@z+w[f'mlp.{idx}.bias'].numpy().astype(np.float64)
        if i<3: z=np.where(z>=0,z,np.expm1(np.maximum(z,-60.0)))
    return z.astype(np.float32)


def compare_policies(session:ort.InferenceSession,seed:int=20261106,samples:int=201) -> dict:
    ckpt=torch.load(PT,map_location='cpu',weights_only=True)
    obs=np.random.default_rng(seed).normal(0,.25,(samples,53)).astype(np.float32)
    obs[0,:]=0
    x_name=session.get_inputs()[0].name
    ort_out=np.concatenate([session.run(None,{x_name:obs[i:i+1]})[0] for i in range(samples)])
    pt_out=np.stack([torch_actor(x,ckpt) for x in obs])
    absolute=np.abs(ort_out-pt_out)
    return {
        'samples':samples,
        'seed':seed,
        'max_abs_action_difference':float(absolute.max()),
        'mean_abs_action_difference':float(absolute.mean()),
        'zero_obs_onnx_first_six':ort_out[0,:6].tolist(),
        'zero_obs_pt_first_six':pt_out[0,:6].tolist(),
        'comparison_status':('MATCH' if float(absolute.max())<1e-4 else 'MISMATCH'),
        'warning':'Publication artifacts differ; exact cause not established.'
    }


def scenario(protocol:str):
    if protocol == 'standard':
        return [(0.,2.,(0,0,0)),(2.,4.,(.2,0,0)),
                (4.,6.,(0,0,.35)),(6.,8.,(0,0,0))]
    if protocol == 'midstride':
        # Every transition intentionally falls inside the 0.4s gait phase.
        return [(0.,1.3,(0,0,0)),(1.3,3.1,(.25,0,0)),
                (3.1,4.7,(0,.13,0)),(4.7,6.3,(0,0,-.5)),
                (6.3,7.9,(-.15,0,0)),(7.9,9.5,(0,0,0))]
    raise ValueError('Unknown protocol')


def commands(t:float,protocol:str):
    for a,b,velocity in scenario(protocol):
        if a<=t<b: return np.array(velocity,dtype=np.float32)
    return np.zeros(3,dtype=np.float32)


def build_observation(d:mujoco.MjData,cmd:np.ndarray,joint_ids:np.ndarray,joint_dofs:np.ndarray,defaults:np.ndarray,prev:np.ndarray)->np.ndarray:
    mat=np.empty(9,dtype=np.float64)
    mujoco.mju_quat2Mat(mat,d.qpos[3:7])
    gravity=mat.reshape(3,3).T@np.array([0.,0.,-1.])
    phase=np.zeros(2,dtype=np.float64)
    if np.linalg.norm(cmd)>=0.1:
        fraction=(d.time%0.4)/0.4
        phase[:]=(math.sin(2*math.pi*fraction),math.cos(2*math.pi*fraction))
    obs=np.concatenate((d.sensor('imu_ang_vel').data.copy(),gravity,cmd,phase,
                        d.qpos[joint_ids]-defaults,d.qvel[joint_dofs],prev)).astype(np.float32)
    if obs.shape != (53,) or not np.isfinite(obs).all():raise RuntimeError('Invalid actor observation')
    return obs


def run_case(session:ort.InferenceSession, joints:list[str], mode:str, seed:int, seconds:float=8., sample_every:int=25,protocol:str='standard')->dict:
    m=get_model();d=mujoco.MjData(m)
    q_idx,v_idx,a_idx,model_order,motor_order=mapping(m,joints)
    meta=session.get_modelmeta().custom_metadata_map
    defaults=np.array([float(x) for x in meta['default_joint_pos'].split(',')],dtype=np.float64)
    if defaults.shape!=(14,): raise ValueError('Unexpected metadata default pose')
    if float(meta['action_scale'])!=0.25:raise ValueError('Unexpected action scale')
    rng=np.random.default_rng(seed)
    # Same deterministic perturbation for learned policy and zero reference.
    perturb=np.array([rng.uniform(-.01745,.01745),rng.uniform(-.01745,.01745),rng.uniform(-.04,.04)])
    qwxyz=Rotation.from_euler('xyz',perturb).as_quat() # xyzw
    d.qpos[3:7]=np.r_[qwxyz[3],qwxyz[:3]]
    d.qpos[2]=.10
    d.qpos[q_idx]=defaults
    d.qvel[:]=0
    d.ctrl[a_idx]=defaults
    mujoco.mj_forward(m,d)
    prev=np.zeros(14,dtype=np.float32)
    inputs=session.get_inputs()[0].name
    steps=round(seconds/m.opt.timestep)
    trace=[];sat_steps=0;max_tilt=0.;min_root=float('inf');first_fall=None;prev_toe={}
    error=np.zeros(steps,dtype=np.float64);torque_max=0.;action_max=0.;loaded_min=4
    speed_sum=0.;n_speed=0;episode_commands=[]
    for step in range(steps):
        cmd=commands(d.time,protocol)
        if step%5==0:
            obs=build_observation(d,cmd,q_idx,v_idx,defaults,prev)
            raw=(session.run(None,{inputs:obs[None,:]})[0][0] if mode=='onnx'
                 else np.zeros(14,dtype=np.float32))
            if not np.isfinite(raw).all():raise RuntimeError(f'Non-finite policy action t={d.time}')
            prev=raw.copy()
            action_max=max(action_max,float(np.max(np.abs(raw))))
            target=defaults+float(meta['action_scale'])*raw
            d.ctrl[a_idx]=np.clip(target,m.actuator_ctrlrange[a_idx,0],m.actuator_ctrlrange[a_idx,1])
        mujoco.mj_step(m,d)
        torque_max=max(torque_max,float(np.max(np.abs(d.actuator_force))))
        sat_steps+=int(np.any(np.abs(d.actuator_force) >= m.actuator_forcerange[:,1]-1e-5))
        min_root=min(min_root,float(d.qpos[2]))
        q=d.qpos[3:7];tilt=float(math.degrees(math.acos(float(np.clip(1-2*(q[1]*q[1]+q[2]*q[2]),-1,1)))))
        max_tilt=max(max_tilt,tilt)
        if first_fall is None and (tilt>60 or d.qpos[2]<.03):first_fall={'time_s':float(d.time),'tilt_deg':tilt,'root_z':float(d.qpos[2])}
        if step%sample_every==0 or step==steps-1:
            contact_load={f:0. for f in FEET};contact_counts={f:0 for f in FEET}
            for ci in range(d.ncon):
                c=d.contact[ci];force=np.zeros(6,dtype=np.float64)
                mujoco.mj_contactForce(m,d,ci,force)
                for gid in (int(c.geom1),int(c.geom2)):
                    n=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,gid)
                    if n and n.endswith('_shank_collision') and n[:2] in contact_load:
                        foot=n[:2];contact_load[foot]+=float(force[0]);contact_counts[foot]+=1
            loaded=sum(1 for f,v in contact_load.items() if v>=.2)
            loaded_min=min(loaded_min,loaded)
            episode_commands.append({'t':float(d.time),'vx':float(cmd[0]),'vy':float(cmd[1]),'wz':float(cmd[2])})
            trace.append({'t':round(float(d.time),4),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),
                          'ctrl':d.ctrl.tolist(),'prev_action_natural_joint_order':prev.tolist(),
                          'command_body':cmd.tolist(), 'contact_normal_N':contact_load,'loaded_feet':loaded,
                          'ncon':int(d.ncon),'tilt_deg':tilt})
        if not np.isfinite(d.qpos).all():raise RuntimeError(f'Diverged at {d.time}')
    def snap(t):return min(trace,key=lambda x:abs(x['t']-t))
    movement=[]
    for start,end,velocity in scenario(protocol):
        a,b=snap(start),snap(end)
        movement.append({'start_s':start,'end_s':end,'command':list(velocity),
                         'dx_m':float(b['qpos'][0]-a['qpos'][0]),
                         'dy_m':float(b['qpos'][1]-a['qpos'][1]),
                         'yaw_start_deg':float(Rotation.from_quat(np.r_[a['qpos'][4:7],a['qpos'][3]]).as_euler('xyz')[2]*180/math.pi),
                         'yaw_end_deg':float(Rotation.from_quat(np.r_[b['qpos'][4:7],b['qpos'][3]]).as_euler('xyz')[2]*180/math.pi)})
    return {
        'mode':mode,'seed':seed,'protocol':protocol,'initial_rotation_rpy_rad':perturb.tolist(),'seconds':seconds,
        'physics_hz':250,'policy_hz':50,'net_root_m':(np.array(trace[-1]['qpos'][:3])-np.array(trace[0]['qpos'][:3])).tolist(),
        'max_tilt_deg':max_tilt,'min_root_height_m':min_root,'first_fall':first_fall,
        'torque_saturation_ticks':sat_steps,'peak_abs_actuator_force':torque_max,
        'max_abs_policy_action':action_max,'min_loaded_foot_count_samples':loaded_min,
        'joint_order':joints,'actuator_order':motor_order,'nn_output_to_actuator_indices':a_idx.tolist(),
        'segments':movement,'trace':trace,
        'status':'INDEPENDENT_ONNX_MUJOCO_CPU_NOT_NATIVE_MJLAB',
        'model_variations':'Pinned original model with documented four-shank training contact config; no training rand, no latency in play mode.',
        'limitations':'No native mjlab, no hardware, one fixed command script, no encoder calibration. Contact loaded count sampled every 0.1s. Commands in robot local frame; root displacement is world.'}


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--seeds',type=int,default=1)
    ap.add_argument('--start-seed',type=int,default=20261106)
    ap.add_argument('--seconds',type=float,default=8.)
    ap.add_argument('--modes',nargs='+',default=['onnx','zero'],choices=['onnx','zero'])
    ap.add_argument('--protocol',choices=['standard','midstride'],default='standard')
    ap.add_argument('--output-dir',type=Path,default=BASE/'onnx_corrected')
    args=ap.parse_args()
    if abs(args.seconds-scenario(args.protocol)[-1][1])>1e-6:raise SystemExit('Specify --seconds equal to scenario length')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    session,joints,metadata=make_policy();m=get_model()
    q,v,a,model_order,motor_order=mapping(m,joints)
    report={'version':'Lite3 ONNX corrected joint map, CPU only','upstream_xml_blob':'9c9699c5fc3c5f4ed4c69ebf516c346ee6a57026',
            'onnx_sha256':sha256(ONNX),'pt_sha256':sha256(PT),'onnxruntime':ort.__version__,'mujoco':mujoco.__version__,
            'protocol':args.protocol,'model_joint_order':model_order,'model_actuator_order':motor_order,'policy_joint_order':joints,
            'nn_to_actuator_index':a.tolist(),'onc_vs_pt':compare_policies(session),'runs':[]}
    for mode in args.modes:
        for si in range(args.seeds):
            seed=args.start_seed+si
            out=run_case(session,joints,mode,seed,args.seconds,protocol=args.protocol)
            out_path=args.output_dir/f'{mode}_{seed}.json'
            out_path.write_text(json.dumps(out,indent=2)+'\n')
            mtr={k:v for k,v in out.items() if k!='trace'}
            report['runs'].append(mtr)
            print(mode,seed, 'dx(mm)',round(out['net_root_m'][0]*1000,2),'tilt',round(out['max_tilt_deg'],2),
                  'force_cap',out['torque_saturation_ticks'],'fall',out['first_fall'] is not None,flush=True)
    (args.output_dir/'SUMMARY.json').write_text(json.dumps(report,indent=2)+'\n')
    print('saved',args.output_dir)

if __name__=='__main__':main()