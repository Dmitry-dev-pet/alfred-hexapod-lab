"""Package a measured MuJoCo collision-proxy rollout for read-only 3D inspection.
Does not assert exact original CAD or controller equivalence; fails closed on raw evidence.
"""
from __future__ import annotations
import hashlib, json, math, argparse
from pathlib import Path

FMT='mini-wheelbot-mujoco-physical-replay-v1'
COLS=['time_s','root_xyz_m_3','root_quat_wxyz_4','drive_joint_rad','reaction_joint_rad','reaction_omega_rad_s','reaction_command_Nm']

def sha(data): return hashlib.sha256(data).hexdigest()

def export(src:Path,dst:Path):
    b=src.read_bytes(); s=json.loads(b)
    if s.get('schema')!='wheelbot-step-collision-only-raw/v1': raise ValueError('wrong trace')
    info=s['info']
    if info.get('status')!='DIAGNOSTIC_COLLISION_PROXY_NO_VISUAL_CAD' or info.get('numerical_warning') is not None:
        raise ValueError('not a valid complete numerical rollout')
    if info.get('mode')!='upstream' or info.get('meets_provisional_up_criterion') is not False:
        raise ValueError('this evidence must remain a negative control')
    rows=s['frames']; verified=s['samples']
    if len(rows)!=len(verified) or len(rows)<50: raise ValueError('missing frames')
    last=-1
    out=[]
    for row, state in zip(rows,verified):
        if len(row)!=14 or not all(type(x) in (float,int) and math.isfinite(x) for x in row):raise ValueError('invalid state shape')
        if row[0]<=last: raise ValueError('nonmonotonic time')
        last=row[0]
        if abs(row[0]-state['t'])>1e-6 or any(abs(row[i]-state['qpos'][i-1])>1e-6 for i in range(1,10)):
            raise ValueError('pose mismatch with independent raw samples')
        data=row[:10]+[row[11],row[12]]
        if len(data)!=12:raise ValueError('bad frame')
        out.append(data)
    result={'format':FMT,'title':'Mini Wheelbot · collision proxy · negative physics run',
            'robot':'Mini Wheelbot experimental collision-only model (NOT original CAD mesh)',
            'physics_engine':'MuJoCo','physics_version':'3.14.0',
            'description':'Original upstream torque pulse, proxy geometry, authentic recorded mj_step states. No original CAD/STL visual mesh.',
            'source_repo':'wheelbot/Mini-Wheelbot','source_commit':info['source_upstream'],
            'model_xml_sha256':info['model_xml_sha256'],
            'status':'failed', 'verification':{'source_trace_sha256':sha(b),'physics_steps':info['physics_steps'],
            'source_raw_schema':s['schema'],'original_cad_meshes_included':False,'hardware_calibrated':False},
            'limitations':['Collision and mass distribution uncalibrated; original CAD visual meshes omitted',
                           'Original controller closed-loop AMPC is not implemented',
                           'The model differs numerically from some previously published PR #86 traces; do not claim exact parity',
                           'No sustained 1s upright capture; real robot performance unknown'],
            'integrator_step_s':.001,'replay_hz':20,'duration_s':info['simulated_duration_s'],
            'frame_columns':COLS,'frame_count':len(out),'sample_stride':12,'frames':out,
            'result':{'initial_roll_deg':90,'end_roll_deg':info['final_tilt_deg'],
                      'minimum_tilt_deg':info['min_tilt_deg'],
                      'peak_reaction_omega_rad_s':info['peak_reaction_speed_rad_s'],
                      'longest_upright_s':info['longest_upright_s'],
                      'meets_provisional_up_criterion':False}}
    raw=(json.dumps(result,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n').encode('utf-8')
    if dst.exists() and dst.read_bytes()!=raw: raise ValueError('immutable data conflict')
    dst.write_bytes(raw)
    return {'sha256':sha(raw),'source_trace_sha256':sha(b),'size':len(raw),'frames':len(out)}

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--source',required=True,type=Path);a.add_argument('--output',required=True,type=Path)
    p=a.parse_args();print(json.dumps(export(p.source,p.output),indent=2))
