#!/usr/bin/env python3
"""Prepare offline MuJoCo/ONNX motion cycles and exact browser FK rig.

The browser will play/blend these motions kinematically; no runtime dynamics
or physical stability is implied by joystick-controlled pose blending.
"""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'microduck-work/lab/research/microduck'))
import bam_backlash_cpu as bench

RL=WORK/'microduck-work/microduck_rl-bae39f5900b318f7e87fc485c30a31213cfa14eb'
POLICY=WORK/'microduck-work/velstand.onnx'
RAW=WORK/'microduck-work/battery-final'
OUT=Path(__file__).resolve().parent
PIN=bench.UPSTREAM_COMMIT
PACK_SHA='e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b'
COMMANDS={'idle':[0,0,0],'forward':[.30,0,0],'backward':[-.25,0,0],
          'turn_left':[0,0,1.0],'turn_right':[0,0,-1.0]}


def digest(b):return hashlib.sha256(b).hexdigest()


def new_sim(ip):
    bam=ip.load_bam_model(200,7.4,None)
    return ip.load_mujoco_with_bam(str(RL/'src/mjlab_microduck/robot/microduck/scene_backlash.xml'),bam,.005,.1,6)


def canonical(q):
    q=np.array(q,dtype=float)
    r=Rotation.from_quat(q[3:7][[1,2,3,0]])
    roll,pitch,_=r.as_euler('xyz')
    root=Rotation.from_euler('xyz',[roll,pitch,0]).as_quat()
    q[:2]=0;q[3:7]=root[[3,0,1,2]]
    return q


def loop(rows,mode):
    q=np.array([canonical(r['qpos']) for r in rows])
    if mode=='idle':return {'dt':.02,'qpos':q[-50:].round(9).tolist(),'source_range':[450,500],'seam_error_rad':0}
    best=None
    # A complete short gait period must end close to its initial joint pose
    # and velocity. The loop is sourced from measured post-settling motion.
    for start in range(150,300,2):
        for length in range(20,101):
            end=start+length
            v0=(q[start+1,7:]-q[start-1,7:])/.04
            v1=(q[end+1,7:]-q[end-1,7:])/.04
            score=float(np.mean((q[end,7:]-q[start,7:])**2)+.0003*np.mean((v1-v0)**2))
            if best is None or score<best[0]:best=(score,start,end)
    _,start,end=best
    return {'dt':.02,'qpos':q[start:end].round(9).tolist(),'source_range':[start,end],
            'seam_error_rad':float(np.max(np.abs(q[end,7:]-q[start,7:])))}


def main():
    mf=json.loads((WORK/'microduck-work/source-manifest.json').read_text());mf['_path']=str(WORK/'microduck-work/source-manifest.json')
    bench.source_gate(RL,WORK/'microduck-work/bam-62bd8ce12154340be97e06f7f41a0ca8f116d967',mf)
    assert digest(POLICY.read_bytes())==bench.POLICY_SHA256
    ip=bench.upstream_inference(RL);ip.ort.InferenceSession=bench.cpu_session
    pack_path=WORK/'microduck-publish/site/dist/assets/robot-models'/f'microduck-original-{PACK_SHA}.json'
    pb=pack_path.read_bytes();assert digest(pb)==PACK_SHA
    manifest=json.loads(pb)['manifest'];visible=manifest['visible_geoms']
    with contextlib.redirect_stdout(io.StringIO()):model,_,_,_=new_sim(ip)
    rig={'schema':'microduck-kinematic-rig/v1','model_commit':PIN,'model_package_sha256':PACK_SHA,'nq':model.nq,
         'bodies':[],'joints':[],'geoms':[],'servo_qpos':{},'servo_ranges':{}}
    for i in range(model.nbody):
        rig['bodies'].append({'id':i,'parent':int(model.body_parentid[i]),'pos':model.body_pos[i].tolist(),
                             'quat':model.body_quat[i].tolist(),'joints':list(range(int(model.body_jntadr[i]),int(model.body_jntadr[i]+model.body_jntnum[i]))) if model.body_jntnum[i] else []})
    for i in range(model.njnt):
        name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,i)
        rig['joints'].append({'name':name,'type':int(model.jnt_type[i]),'qadr':int(model.jnt_qposadr[i]),
                              'axis':model.jnt_axis[i].tolist(),'anchor':model.jnt_pos[i].tolist(),
                              'ref':float(model.qpos0[model.jnt_qposadr[i]])})
        if not name.startswith('passive_') and model.jnt_type[i]==3:
            rig['servo_qpos'][name]=int(model.jnt_qposadr[i]);rig['servo_ranges'][name]=model.jnt_range[i].tolist()
    for g in visible:
        gid=g['gid'];mid=int(model.geom_dataid[gid]);assert mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_MESH,mid)==g['mesh']
        rig['geoms'].append({'gid':gid,'body':int(model.geom_bodyid[gid]),'pos':model.geom_pos[gid].tolist(),'quat':model.geom_quat[gid].tolist()})
    kit={'schema':'microduck-kinematic-motions/v1','model_commit':PIN,'policy_sha256':bench.POLICY_SHA256,
         'nq':35,'control_semantics':'KINEMATIC_PREVIEW_NOT_LIVE_PHYSICS','cycles':{},'actions':{}}
    raw={'schema':'microduck-locomotion-offline/v1','model_commit':PIN,'policy_sha256':bench.POLICY_SHA256,'commands':{}}
    fixtures=[]
    for mode,command in COMMANDS.items():
        with contextlib.redirect_stdout(io.StringIO()):
            m,d,c,_=new_sim(ip);mujoco.mj_resetDataKeyframe(m,d,mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_KEY,'STAND'))
            c.reset(d.qpos);c.last_ts=d.time;mujoco.mj_forward(m,d)
            policy=ip.PolicyInference(m,d,walking_onnx_path=str(POLICY),bam_ctrl=c,use_projected_gravity=True,new_cmd_obs=True)
            bench.install_encoder_view(policy,c,m,d);policy.set_vel_cmd(*command)
            rows=[]
            for step in range(500):
                mujoco.mj_forward(m,d);policy.apply_action(policy.infer())
                for _ in range(4):c.update();mujoco.mj_step(m,d)
                mujoco.mj_forward(m,d)
                if not np.isfinite(d.qpos).all():raise ValueError('Nonfinite locomotion')
                rows.append({'t':round((step+1)*.02,6),'qpos':d.qpos.round(9).tolist(),'qvel':d.qvel.round(9).tolist(),'ctrl':d.ctrl.round(9).tolist()})
        tilts=[np.degrees(np.arccos(np.clip(Rotation.from_quat(np.array(r['qpos'][3:7])[[1,2,3,0]]).as_matrix()[2,2],-1,1))) for r in rows[-100:]]
        displacement=float(np.linalg.norm(np.array(rows[-1]['qpos'][:2])-rows[150]['qpos'][:2]))
        yaw=np.unwrap([Rotation.from_quat(np.array(r['qpos'][3:7])[[1,2,3,0]]).as_euler('xyz')[2] for r in rows])
        yaw_change=float(yaw[-1]-yaw[150]);warnings={str(i):int(v.number) for i,v in enumerate(d.warning) if v.number}
        metrics={'command':command,'displacement_after_3s_m':displacement,'yaw_change_after_3s_rad':yaw_change,'final_2s_max_tilt_deg':float(max(tilts)),'warnings':warnings}
        raw['commands'][mode]={'metrics':metrics,'frames':rows}
        kit['cycles'][mode]={**loop(rows,mode),'metrics':metrics}
        print(mode,json.dumps({**metrics,'loop_frames':len(kit['cycles'][mode]['qpos']),'seam':kit['cycles'][mode]['seam_error_rad']}),flush=True)
        for at in [0,200,499]:
            mdata=mujoco.MjData(m);mdata.qpos[:]=rows[at]['qpos'];mujoco.mj_forward(m,mdata)
            poses=[]
            for g in visible:
                q=np.empty(4);mujoco.mju_mat2Quat(q,mdata.geom_xmat[g['gid']])
                poses.append({'p':mdata.geom_xpos[g['gid']].tolist(),'q':q.tolist()})
            fixtures.append({'qpos':rows[at]['qpos'],'poses':poses})
    # v8's sampled negative-velocity command did not produce a useful backward
    # walk. For this explicitly kinematic viewer, backward uses a declared
    # time-reversal of the measured forward cycle, not a claimed physics pass.
    forward=kit['cycles']['forward']
    kit['cycles']['backward']={**forward,'qpos':list(reversed(forward['qpos'])),
                              'derivation':'kinematic_time_reverse_of_forward',
                              'metrics':{'not_a_measured_backward_walk':True}}
    kit['neutral_qpos']=kit['cycles']['idle']['qpos'][0]
    for case in ['side90','roll180','pitch180']:
        name=f'bam_backlash-{case}-20261109.json';b=(RAW/name).read_bytes();tr=json.loads(b)
        kit['actions'][case]={'source_sha256':digest(b),'fall':[[r['t_physics']-.02,*r['qpos']] for r in tr['settling_frames']],
                              'rise':[[0,*tr['initial_after_settle']['qpos']]]+[[r['t'],*r['qpos']] for r in tr['frames']]}
        for r in [tr['settling_frames'][0],tr['frames'][150],tr['frames'][-1]]:
            d=mujoco.MjData(model);d.qpos[:]=r['qpos'];mujoco.mj_forward(model,d);poses=[]
            for g in visible:
                q=np.empty(4);mujoco.mju_mat2Quat(q,d.geom_xmat[g['gid']]);poses.append({'p':d.geom_xpos[g['gid']].tolist(),'q':q.tolist()})
            fixtures.append({'qpos':r['qpos'],'poses':poses})
    for name,obj in [('rig.json',rig),('motion-kit.json',kit),('offline-locomotion.json',raw),('fk-fixtures.json',fixtures)]:
        (OUT/name).write_text(json.dumps(obj,separators=(',',':'),allow_nan=False)+'\n')
    print('saved',[(n,(OUT/n).stat().st_size) for n in ['rig.json','motion-kit.json','offline-locomotion.json','fk-fixtures.json']],flush=True)


if __name__=='__main__':main()
