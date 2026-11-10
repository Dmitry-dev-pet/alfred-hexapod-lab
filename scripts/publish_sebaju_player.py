#!/usr/bin/env python3
"""Deterministically export verified physical SeBaJu traces to the existing Hexapod player.

The source controller and XML are intentionally NOT copied to the public viewer.
No physics, no browser code, no motion synthesis: only measured qpos and contacts.
"""
from __future__ import annotations
import argparse
import base64
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

PROJECT_ROOT=Path(__file__).resolve().parents[1]
DOCS=PROJECT_ROOT/'docs'
sys.path.insert(0,str(PROJECT_ROOT/'scripts'))
from register_viewer_replays import register

SOURCE='7bbc38f4b26b8ecce7ac0ee37e06164e851dc600'
ENGINE='3.14.0'
ORIGINAL_MODEL_BLOB='c0abe8d5f9663813e2782fbd44fb38c40d7ea048'
ORIGINAL_CONTROLLER_BLOB='42397bb5e613a12ecbe6074030bb917f3d03533a'
GEOMS=('torso_box','front_low_balance_mass','left_thigh_geom','left_shank_geom','left_wheel_geom','right_thigh_geom','right_shank_geom','right_wheel_geom')
GEOM_INDEX={name:i for i,name in enumerate(GEOMS)}
EXPERIMENTS=(
 ('sebaju-jump-v1','canonical_jump','SeBaJu · прыжок и баланс','13 с · настоящая динамика · прыжок 44 мм','passed'),
 ('sebaju-tip12-v1','canonical_tip12','SeBaJu · падение набок','18 с · импульс 12 Н·м · не встал','failed'),
 ('sebaju-tip20-long-v1','strict_tip_20Nm_30s','SeBaJu · переворот и ожидание','30 с · импульс 20 Н·м · не встал','failed'),
)

def sha(data:bytes)->str:return hashlib.sha256(data).hexdigest()
def raw_bytes(data)->bytes:return (json.dumps(data,ensure_ascii=False,sort_keys=True,separators=(',',':'))+'\n').encode('utf-8')

def check_trace(t:dict,r:dict,trial_id:str):
    assert t['schema']=='alfred-sebaju-mujoco-physical-trace/v1'
    assert t['source_commit']==SOURCE and r['source_commit']==SOURCE
    assert t['engine']=='MuJoCo' and t['engine_version']==ENGINE and r['engine']==ENGINE
    assert t['physics_dt_s']==.002 and r['physics_dt_s']==.002
    assert t['steps']==r['steps'] and len(t['frames'])==r['frames']
    assert t['model']=='SeBaJu_BOT(II).xml' and t['controller']=='SeBaJu_Ctrl_BalJumpAchieve.py'
    assert len(t['frames'])>=500 and t['steps']>=5000
    assert t['frames'][-1]['t']==t['duration_s']==r['duration_s']
    assert r['summary']['jump_detected'] and r['summary']['landing_recovery_observed']
    if trial_id!='sebaju-jump-v1':
        assert t['trial']=='tip' and r['summary']['physical_tip_observed'] is True
        assert r['summary']['self_righted'] is False
        assert any('torso_box' in f['ground_contact_geoms'] or 'left_thigh_geom' in f['ground_contact_geoms'] or 'right_thigh_geom' in f['ground_contact_geoms'] for f in t['frames'][-50:])
    else:
        assert t['trial']=='jump'
    last=-1
    for i,f in enumerate(t['frames']):
        assert f['t']>last and 0<f['t']<=r['duration_s']
        last=f['t'];q=f['qpos']
        assert len(q)==13 and all(isinstance(n,(int,float)) and math.isfinite(n) for n in q)
        assert abs(sum(x*x for x in q[3:7])-1)<.01
        assert isinstance(f['grounded'],list) and len(f['grounded'])==2
        assert len(f['wheel_vertical_normal_N'])==2
        assert all(g in GEOM_INDEX for g in f['ground_contact_geoms'])
        assert all(math.isfinite(x) and x>=0 for x in f['wheel_vertical_normal_N'])
        assert math.isfinite(f['tilt_deg'])
        assert i==0 or (f['t']-t['frames'][i-1]['t']) <= 0.021


def make_data(case_id:str, source:Path):
    trace_file=source/'physical_trace.json';receipt_file=source/'receipt.json'
    t=json.loads(trace_file.read_text());r=json.loads(receipt_file.read_text())
    check_trace(t,r,case_id)
    rows=[]
    for f in t['frames']:
        mask=sum(1<<GEOM_INDEX[name] for name in set(f['ground_contact_geoms']))
        rows.append([f['t'],*f['qpos'],f['tilt_deg'],int(f['grounded'][0]),int(f['grounded'][1]),
                     *f['wheel_vertical_normal_N'],mask,f['wheel_gap_min_m']])
    assert all(len(row)==21 for row in rows)
    full={
       'format':'sebaju-mujoco-measured-v1',
       'case_id':case_id,
       'source':'https://github.com/k3nkreate/SeBaJu_BOT_5845',
       'source_commit':SOURCE,
       'source_model_git_blob_sha1':ORIGINAL_MODEL_BLOB,
       'source_controller_git_blob_sha1':ORIGINAL_CONTROLLER_BLOB,
       'source_trace_sha256':sha(trace_file.read_bytes()),
       'source_receipt_sha256':sha(receipt_file.read_bytes()),
       'engine':'MuJoCo 3.14.0',
       'physics_dt_s':.002,
       'sampled':True,
       'duration_s':t['duration_s'],
       'frame_count':len(rows),
       'joint_order':['root_x','root_y','root_z','root_qw','root_qx','root_qy','root_qz','left_hip','left_knee','left_wheel','right_hip','right_knee','right_wheel'],
       'contact_geom_bit_order':list(GEOMS),
       'disturbance':t.get('disturbance'),
       'summary':r['summary'],
       'rows':rows,
       'row_columns':['t',*('q'+str(i) for i in range(13)),'tilt_deg','left_wheel_contact','right_wheel_contact','left_wheel_Fz_N','right_wheel_Fz_N','ground_geom_mask','wheel_gap_min_m'],
       'display_geometry':'Schematic Three.js primitives with dimensions from the upstream MuJoCo model, not original STL',
       'notes':['Original unmodified controller and real MuJoCo physics steps; no generated intermediate motion','All playback poses and contacts originate in original measured trace; schematic display is not upstream CAD','The final failed self-right classification does not prove theoretical impossibility.'],
    }
    raw=raw_bytes(full)
    pack={
        'format':'sebaju-mujoco-gzip-b64-v1',
        'case_id':case_id,
        'source_trace_sha256':full['source_trace_sha256'],
        'source_receipt_sha256':full['source_receipt_sha256'],
        'source_commit':SOURCE,
        'engine':'MuJoCo 3.14.0',
        'frame_count':len(rows),
        'duration_s':t['duration_s'],
        'raw_bytes':len(raw),'raw_sha256':sha(raw),
        'compression':'gzip','base64':base64.b64encode(gzip.compress(raw,compresslevel=9,mtime=0)).decode('ascii'),
    }
    return pack,full

def output_project(evidence:Path):
    results=[]
    for case_id, folder, title, detail, status in EXPERIMENTS:
        pack,full=make_data(case_id,evidence/folder)
        path=DOCS/'hexapod/data'/f'{case_id}.json';path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(raw_bytes(pack))
        webpage=DOCS/'hexapod'/case_id/'index.html'
        if not webpage.exists():raise FileNotFoundError(webpage)
        files=[f'hexapod/{case_id}/index.html',f'hexapod/data/{case_id}.json']
        if case_id==EXPERIMENTS[0][0]:files.insert(0,'hexapod/sebaju-player.js')
        assets=[{'path':p,'sha256':sha((DOCS/p).read_bytes()),'size':(DOCS/p).stat().st_size} for p in files]
        results.append({'id':case_id,'title':title,'detail':detail,'duration_s':full['duration_s'],'href':f'/hexapod/{case_id}/','status':status,'files':assets})
        print(json.dumps({'case_id':case_id,'source_trace_sha256':full['source_trace_sha256'],'frames':pack['frame_count'],'size':path.stat().st_size,'raw_sha256':pack['raw_sha256']}))
    return results

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--evidence',type=Path,required=True);p.add_argument('--catalog',type=Path,default=DOCS/'hexapod/library.json');p.add_argument('--output',type=Path,default=DOCS/'hexapod/library.json');a=p.parse_args()
    entries=output_project(a.evidence)
    merged=register(json.loads(a.catalog.read_text()),entries,DOCS)
    a.output.write_bytes(json.dumps(merged,ensure_ascii=False,indent=2).encode('utf8')+b'\n')
    print(json.dumps({'schema':merged['schema'],'revision':merged['revision'],'entries':len(merged['entries'])}))
