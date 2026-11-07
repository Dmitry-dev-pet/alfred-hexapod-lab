#!/usr/bin/env python3
"""Export pinned Stanford pretrained 5s MuJoCo rollout for approved 3D player.

Poses are taken from measured qpos, rendered with mj_forward only. No poses
fabricated, no browser-trained policy; foot contact forces from mj_step log.
"""
import argparse, hashlib, json
from pathlib import Path
import mujoco, numpy as np
from reproduce_pupper import read_pinned_xml,load_model,FOOT_GEOMS

HERE=Path(__file__).resolve().parent
MODEL_ORIGINAL_SHA='7771e176c42b3aed1d48cc62434771b0fceb7c8c'


def export(path,output,stance_template):
    raw=path.read_bytes()
    log=json.loads(raw)
    r=log['summary']
    assert log['policy_sha256']=='ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712'
    assert log['parity']=='not yet matched against upstream C++ outputs or native MJLab'
    assert r['frames']==250 and r['physics_steps']==1250 and r['duration_s']==5
    assert len(log['frames'])==250
    source=json.loads(stance_template.read_bytes())
    assert source['schema']=='pupper-v3-mujoco-cpu-stance-replay-v1'
    model=load_model(read_pinned_xml(HERE/'upstream/pupper_v3.xml'))
    layout=source['layout']; names=layout['body_names'];sites=layout['foot_site_names']
    bidx=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n) for n in names]
    sidx=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,n) for n in sites]
    assert min(bidx)>=0 and min(sidx)>=0
    data=mujoco.MjData(model)
    outframes=[]
    for row in log['frames']:
        assert len(row['qpos'])==model.nq==19
        assert len(row['qvel'])==model.nv==18
        mujoco.mj_resetData(model,data)
        data.qpos[:]=row['qpos']
        mujoco.mj_forward(model,data)
        assert np.allclose(data.xpos[bidx[0]],row['qpos'][:3],atol=2e-8)
        pose=[]
        for body in bidx:
            pose.extend(round(float(v),8) for v in (*data.xpos[body],*data.xquat[body]))
        foot=[]
        for s in sidx: foot.extend(round(float(v),8) for v in data.site_xpos[s])
        fz=[float(row['foot_Fz'][name]) for name in FOOT_GEOMS]
        num=sum(v>.2 for v in fz)
        this=[row['t'],*pose,*foot,*fz,num]
        assert len(this)==109
        outframes.append(this)
    assert abs(outframes[-1][0]-5.)<1e-8
    doc={
      'schema':'pupper-v3-mujoco-cpu-pretrained-walk-replay-v1',
      'is_walk':True,
      'control':{
          'method':'trained Stanford default_policy.json inference via original 20x48 observation + gait reference',
          'training_performed_by_us':False,
          'policy_repo':'cs123-stanford/pupper_gait_deploy',
          'policy_commit':'70df56dd47e98195b7d51fe7ff7194a2581b0e0d',
          'policy_git_blob_sha1':'27f8a94b95c0e960b01b0cd7b88c5cd9ab249b3c',
          'policy_sha256':log['policy_sha256'],
          'in_shape':[None,960],
          'command_vx_mps':.20,
          'warmup_s':log['warmup_s'],
          'fade_s':log['fade_s'],
          'stop_from_s':4.0,
          'servo_delay_physics_steps':log['actuator_feedback_delay_steps'],
          'parity':'CPU reconstruction; C++/mjlab end-to-end numerical parity not validated',
      },
      'source':{
          'repo':'cs123-stanford/pupper-mjlab',
          'commit':'aa475e9a393a209b29fd5de7e0ee10f3de643230',
          'original_xml_git_blob_sha1':MODEL_ORIGINAL_SHA,
          'trace_sha256':hashlib.sha256(raw).hexdigest(),
          'trace_bytes':len(raw),
          'visual_source_prefix':'https://raw.githubusercontent.com/G-Levine/pupper_v3_description/1a3c8590189d3a406219d155f4177afc491a7d77/description/meshes/stl/'
      },
      'duration_s':5.,'physics_steps':1250,'frame_rate_hz':50,
      'layout':layout,
      'visual_meshes':source['visual_meshes'],
      'summary':{
         'mass_kg':3.17,'forward_m':r['forward_dx_m'],'final_base_z_m':r['final_z_m'],
         'fall_guard_steps':r['fall_guard_steps'],
         'peak_abs_servo_torque_Nm':r['peak_servo_torque_Nm'],
         'saturation_fraction':r['torque_saturation_rate'],
         'policy_iterations':r['events'],
      },
      'frames':outframes,
    }
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_bytes(json.dumps(doc,separators=(',',':'),ensure_ascii=False).encode('utf-8'))
    return doc

if __name__=='__main__':
    arg=argparse.ArgumentParser()
    arg.add_argument('--source',type=Path,required=True)
    arg.add_argument('--template',type=Path,required=True)
    arg.add_argument('--output',type=Path,required=True)
    args=arg.parse_args()
    d=export(args.source,args.output,args.template)
    print('EXPORT_OK',d['schema'],len(d['frames']),len(d['visual_meshes']),args.output.stat().st_size)