#!/usr/bin/env python3
"""Export measured Issue #62 CAD30 qpos/Fz run to existing offline viewer schemas.

Frames are strictly reconstructed with mujoco.mj_forward on actual stored
qpos/qvel sampled during free-base mj_step. No synthesized body motion.
CAD STL files remain in private owner storage; donor is only metadata for the
original external pinned 30 STL mesh URLs and scene visual geometry.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco as mj

LEG_ORDER=('ml','bl','br','mr','fr','fl')

def sha(p:Path)->str:return hashlib.sha256(p.read_bytes()).hexdigest()
def vec(x):return [round(float(t),7) for t in x]
def encode(o):return (json.dumps(o,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n').encode()

def export(source_model:Path,trace_path:Path,donor_path:Path,out_docs:Path):
    record=json.loads(trace_path.read_text())
    if record.get('status')!='PHYSICAL_PASS' or record.get('gait')!='wave' or record.get('physics')!='3.14.0':
        raise ValueError('Expected actual successful wave MuJoCo 3.14 recording')
    samples=record.get('trace')
    if not isinstance(samples,list) or len(samples)<100 or len(samples)>300:
        raise ValueError('Missing 10Hz physical sample trace')
    if not all(isinstance(s.get('qpos'),list) and isinstance(s.get('qvel'),list) for s in samples):
        raise ValueError('Missing measured joint and body states')
    ts=np.array([float(s['time_s']) for s in samples])
    if not(np.all(np.diff(ts)>0) and np.max(np.abs(np.diff(ts)[:-1]-.1))<1e-6 and 12<ts[-1]<13):
        raise ValueError('Nonmonotone/incomplete physical time samples')
    donor=json.loads(donor_path.read_text())
    model=mj.MjModel.from_xml_path(str(source_model))
    if model.nmesh!=30 or model.nq!=27 or len(donor['bodies'])!=27 or len(donor['meshes'])!=30:
        raise ValueError('Model/donor CAD30 source mismatches')
    d=mj.MjData(model)
    root=model.body('base_link_2').id
    from hexlab.stand_geometry_v04 import load_legs,tip_world
    from hexlab.step_one_leg_v04 import forward_from_geometry
    legs=load_legs(model,d,mj)
    if tuple(l['suffix'] for l in legs)!=LEG_ORDER:raise ValueError('Unexpected leg names')
    ids=[int(b['id']) for b in donor['bodies']]
    points=[];rigs=[];phases=[];support=[]
    stage_names=('forward','left','turn','stop')
    for sample in samples:
        q=np.array(sample['qpos'],float);qd=np.array(sample['qvel'],float)
        if q.shape!=(model.nq,) or qd.shape!=(model.nv,) or not np.isfinite(q).all() or not np.isfinite(qd).all():
            raise ValueError('Nonphysical qpos/qvel')
        d.qpos[:]=q;d.qvel[:]=qd;mj.mj_forward(model,d)
        frame=vec([*d.xpos[root],*d.xquat[root]])
        for leg in legs:
            for jid in leg['jids']:frame+=vec(d.xanchor[jid])
            frame+=vec(tip_world(model,d,leg))
        loads={k:float(sample['Fz_N'][k]) for k in LEG_ORDER}
        mask=sum((1<<i) for i,k in enumerate(LEG_ORDER) if loads[k]>=.2)
        frame.append(mask)
        if len(frame)!=80:raise ValueError('Bad point frame layout')
        points.append(frame)
        r=[]
        for bi in ids:r+=vec(d.xpos[bi])+vec(d.xquat[bi])
        if len(r)!=189:raise ValueError('Bad rig frame length')
        rigs.append(r)
        stage=int(sample['stage']);phases.append(stage_names[stage])
        loaded=[k for k in LEG_ORDER if loads[k]>=.2]
        if int(sample['loaded_legs'])!=len(loaded):raise ValueError('Measured Fz/load count mismatch')
        support.append({'contacts':len(loaded),'loaded_stance':loaded,'fz_N':{k:round(loads[k],3) for k in LEG_ORDER},
                        'body_tilt_deg':sample['tilt_deg'],'root_forward_mm':round(sample['root_forward_m']*1000,3),
                        'root_left_mm':round(sample['root_left_m']*1000,3),'yaw_rad':sample['yaw_rad'],
                        'stage':stage_names[stage]})
    # Use the recorded initial 3D model orientation to derive a world XY heading.
    forward_xy=list(forward_from_geometry(legs))
    if len(forward_xy)!=2:raise ValueError('No geometric heading')
    points_data={'format':'fnk0052-physical-points-v1',
       'source':'Freenove FNK0052 CAD30 · Issue62 live velocity controller · original physical 500 Hz MuJoCo recording',
       'recording_sha256':sha(trace_path),'study':'issue62-live-velocity-v1','case_id':'live-v1-slow',
       'visualization':'Recorded qpos and Fz from free-base MuJoCo; command changes at 0,3.1,6.2,9.3s. Not live joystick.',
       'units':'metres','coordinate_system':'MuJoCo Z-up','duration_s':round(ts[-1]-ts[0],3),
       'source_time_offset_s':round(float(ts[0]),4),'sample_interval_s':.1,'sample_count':len(points),
       'stride':80,'layout':'base world xyz+quat, 6 legs 3 joint anchors+toe world XYZ, actual Fz-based mask',
       'leg_order':list(LEG_ORDER),'forward_xy':vec(forward_xy),
       'summary':{'status':'PHYSICAL_PASS','diagnostic':True,'live_joystick':False,'physics':'MuJoCo 3.14.0',
                  'duration_s':12.4,'controls':['forward','left','yaw','stop'],
                  'min_loaded_leg_samples_10Hz':record['min_real_loaded_legs'],
                  'torque_saturation_steps':record['motor_saturation_steps'],
                  'max_tilt_deg':record['max_tilt_deg'],
                  'stages':record['stages'],
                  'note':'PASS only under original fixed V1 gates, observed samples every 100ms, uncalibrated model'},
       'phase_samples':phases,'support_samples':support,
       'limitations':['Recorded replay, not realtime joystick control','Model masses/friction/servos uncalibrated',
                       'Contact loads sampled 10Hz, physics integration 500Hz','Post-stop root drift persists'],
       'frames':points}
    rig=copy.deepcopy({k:v for k,v in donor.items() if k not in ('frames','duration_s','samples','sample_interval_s','sample_base0','sample_base_last','case_id','source_experiment','source_recording_sha256','source_qpos_sha256','pose_layout')})
    rig.update({'duration_s':round(float(ts[-1]-ts[0]),3),'samples':len(rigs),
                'sample_interval_s':.1,'frame_stride':189,
                'pose_layout':'each body in bodies[]: world xyz (3), quaternion wxyz (4); measured 100ms qpos',
                'source_qpos_sha256':sha(trace_path),
                'sample_base0':rigs[0][:7],'sample_base_last':rigs[-1][:7],
                'source_experiment':'https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/63',
                'source_recording_sha256':sha(trace_path),'frames':rigs,'case_id':'live-v1-slow'})
    outdir=out_docs/'hexapod'/'data';outdir.mkdir(parents=True,exist_ok=True)
    a=outdir/'live-v1-slow-points.json';b=outdir/'cad-rig-live-v1-slow.json'
    a.write_bytes(encode(points_data));b.write_bytes(encode(rig))
    return {'frames':len(points),'points':str(a),'rig':str(b),'source_sha256':sha(trace_path),
            'min_loaded':min(x['contacts'] for x in support),'statuses':record['gates']}

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--model',type=Path,required=True);ap.add_argument('--trace',type=Path,required=True)
    ap.add_argument('--donor',type=Path,required=True);ap.add_argument('--docs',type=Path,required=True)
    a=ap.parse_args()
    print(json.dumps(export(a.model,a.trace,a.donor,a.docs),ensure_ascii=False,indent=2))