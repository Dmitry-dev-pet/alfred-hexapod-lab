#!/usr/bin/env python3
"""Generate exact full-state MuJoCo V12 physics recordings for offline Hexapod Lab replay.
Reuses the V12 physical integration; only records qpos alongside its original metrics.
Original CAD30 model required locally. DOES NOT publish CAD/STL assets.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
PHYSICS_V12=Path(__file__).resolve().with_name('reference_v12_runner.py')
TARGETS=ROOT/'research/physics_v12/data/targets_v11.json'
ARCHIVED=ROOT/'research/physics_v12/results'
OUT=ROOT/'docs/hexapod/data'
VARIANTS={
    'nominal_full':{'friction_scale':1.,'torque_limit':.7,'scene':'scene_local30_nominal.xml'},
    'upper_mass_full':{'friction_scale':1.,'torque_limit':.7,'scene':'scene_local30_upper.xml'},
    'friction_035_full':{'friction_scale':.35,'torque_limit':.7,'scene':'scene_local30_nominal.xml'},
    'torque_035_full':{'friction_scale':1.,'torque_limit':.35,'scene':'scene_local30_nominal.xml'},
}
CODE=['ml','bl','br','mr','fr','fl']
# To guarantee identical dynamics, derive a recorder from the *exact pinned V12*
# source via a controlled one-location patch that adds observation-only qpos fields.
SRC=PHYSICS_V12.read_text()
needle="return {'t_s':round(float(t),3),'root_z_m'"
assert SRC.count(needle)==1,'V12 integration source changed: audit recorder before proceeding'
patched=SRC.replace(needle,"return {'physical_qpos': [round(float(v),8) for v in d.qpos], 't_s':round(float(t),3),'root_z_m'",1)
ns={'__name__':'v12_observer','__file__':str(PHYSICS_V12)}
exec(compile(patched,str(PHYSICS_V12),'exec'),ns)
run_case=ns['run_case']

def pack_row(s):
    q=s['physical_qpos'];assert len(q)==27
    contactmask=sum((1<<i) for i,name in enumerate(CODE) if s['per_leg_Fz_N'][name]>.05)
    return [s['t_s'],*q,contactmask,s['max_contact_point_speed_mm_s'],*[s['per_leg_Fz_N'][n] for n in CODE]]

def comparable(old,new):
    if not new['samples']:return False
    if len(old['samples'])!=len(new['samples']):return False
    a,b=old['samples'][-1],new['samples'][-1]
    return all(abs(a[k]-b[k])<=.02 for k in ['root_x_m','root_y_m','root_z_m','yaw_deg']) and a['true_grounded_legs']==b['true_grounded_legs']

def run_one(variant,cases=None):
    meta=VARIANTS[variant]
    model=ROOT/'source/mjcf'/meta['scene']
    targets=json.loads(TARGETS.read_text())['cases']
    archive=ARCHIVED/variant
    names=sorted(p.stem for p in archive.glob('*.json') if p.stem!='summary')
    if cases:names=[x for x in names if x in cases]
    if not names:raise ValueError('No physics cases in '+str(archive))
    output={'format':'hexapod-actual-mujoco-v12-replay-v13','variant':variant,'original_v12_scene_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),'integration':'same mj_step as V12 free-root, positional PD 7/0.32; replay qpos measured, not V11 target','hz':50,'frame_columns':['time_s','qpos_0_26','grounded_mask','max_loaded_contact_speed_mm_s','Fz_ml','Fz_bl','Fz_br','Fz_mr','Fz_fr','Fz_fl'],'runs':{}}
    for name in names:
        old=json.loads((archive/(name+'.json')).read_text())
        result=run_case(model,json.loads((ROOT/'docs/hexapod/data/freenove-original-v7-kit.json').read_text()),json.loads((ROOT/'docs/hexapod/data/live-v4-gaits-kit.json').read_text()),targets[name],friction_scale=meta['friction_scale'],torque_limit=meta['torque_limit'])
        if not comparable(old,result):
            raise RuntimeError(f'Rerun NOT consistent with archived V12 evidence: {variant}/{name} old={old["samples"][-1]} new={result["samples"][-1]}')
        frames=[pack_row(result['settle']),*[pack_row(s) for s in result['samples']]]
        output['runs'][name]={'duration_s':old['duration_simulated_s'],'active_s':old['requested']['active_s'],'gait':old['gait'],'no_fall':old['completed'] and not old['fall'],'title':name,'mass_kg':old['mass_kg'],'friction_scale':meta['friction_scale'],'torque_limit_Nm':meta['torque_limit'],'frames':frames,
          'provenance':{'original_case_sha256':hashlib.sha256((archive/(name+'.json')).read_bytes()).hexdigest()},'summary':{'min_contacts':old['min_true_grounded_legs'],'max_torque_Nm':old['max_abs_motor_torque_Nm'],'motor_saturation':old['actuator_saturation_fraction'],'peak_slip_mm_s':old['max_load_bearing_contact_tangential_speed_mm_s']}}
        print(variant,name,len(frames),'physical replay measured',flush=True)
    return output

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--variant',choices=VARIANTS.keys(),required=True);p.add_argument('--cases',nargs='*');a=p.parse_args()
    result=run_one(a.variant,a.cases)
    target=OUT/f'physics-v12-{a.variant}.json';target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(result,separators=(',',':'),ensure_ascii=False)+'\n')
    print('WRITTEN',target,target.stat().st_size)
