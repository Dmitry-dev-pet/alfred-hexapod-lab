#!/usr/bin/env python3
"""Replay genuine CAD30 2/3/4 contact diagnostics and export immutable Three.js assets.

Physical dynamics are rerun from the same qpos/qvel initial snapshot with the
unmodified PD commands. `two_support_diagnostic.simulate(capture_qpos=True)`
collects observed positions and contact reactions on the integration trajectory.
No interpolation or schematic motion is substituted for missing qpos.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments' / 'cad30'))
from two_support_diagnostic import simulate, STANCE
from hexlab.stand_geometry_v04 import load_legs, tip_world

LEG_ORDER=('ml','bl','br','mr','fr','fl')
CASE_LIST={
    'four': {'lift':(), 'title':'4 опоры · удержание', 'detail':'Контроль · стойка без перестановки · 2,4 с'},
    'three_rear_bl': {'lift':('bl',), 'title':'3 опоры · задняя левая', 'detail':'Подъём bl подтверждён · 2,4 с'},
    'three_middle_ml': {'lift':('ml',), 'title':'3 опоры · средняя левая', 'detail':'Команда ml не сработала: разгрузилась br · 2,4 с'},
    'two_middles_ml_mr': {'lift':('ml','mr'), 'title':'2 опоры? · обе средние', 'detail':'Отрицательный опыт: две опоры не получены · 2,4 с'},
    'two_rears_bl_br': {'lift':('bl','br'), 'title':'2 опоры? · обе задние', 'detail':'Отрицательный опыт: две опоры не получены · 2,4 с'},
    'two_left_ml_bl': {'lift':('ml','bl'), 'title':'2 опоры? · левая пара', 'detail':'Отрицательный опыт: две опоры не получены · 2,4 с'},
    'two_right_mr_br': {'lift':('mr','br'), 'title':'2 опоры? · правая пара', 'detail':'Отрицательный опыт: две опоры не получены · 2,4 с'},
    'two_diag_ml_br': {'lift':('ml','br'), 'title':'2 опоры? · диагональ ml/br', 'detail':'Отрицательный опыт: две опоры не получены · 2,4 с'},
    'two_diag_mr_bl': {'lift':('mr','bl'), 'title':'2 опоры? · диагональ mr/bl', 'detail':'Отрицательный опыт: две опоры не получены · 2,4 с'},
}

def encoded(data):
    return (json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n').encode('utf-8')

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def floats(values,n=7):return [round(float(v),n) for v in values]

def export(model, results, donor, out, original, case):
    readings=results['recorded_qpos_20hz']
    assert len(readings)==49 and all(abs(float(row['t_s'])-i*.05)<1e-5 for i,row in enumerate(readings))
    md=mujoco.MjModel.from_xml_path(str(model))
    assert md.nmesh==30 and md.nq==27
    data=mujoco.MjData(md)
    legs=load_legs(md,data,mujoco)
    assert tuple(l['suffix'] for l in legs)==LEG_ORDER
    body_ids=[entry['id'] for entry in donor['bodies']]
    assert len(body_ids)==27 and donor['frame_stride']==len(body_ids)*7
    origin_sha=sha(original)
    frames=[];rig_frames=[];samples=[];phases=[]
    for row in readings:
        data.qpos[:]=row['qpos'];data.qvel[:]=0
        mujoco.mj_forward(md,data)
        base=int(md.body('base_link_2').id)
        root=floats([*data.xpos[base],*data.xquat[base]],7)
        anchors=[]
        for leg in legs:
            for jid in leg['jids']: anchors.extend(floats(data.xanchor[jid],7))
            anchors.extend(floats(tip_world(md,data,leg),7))
        mask=sum(1<<i for i,k in enumerate(LEG_ORDER) if k in row['contact_mask'])
        frames.append(root+anchors+[mask]); assert len(frames[-1])==80
        rig=[]
        for bi in body_ids:rig.extend(floats(data.xpos[bi])+floats(data.xquat[bi]))
        assert len(rig)==189
        rig_frames.append(rig)
        phases.append(row['phase'])
        samples.append({'contacts':len(row['contact_mask']),
                        'margin_mm':None,
                        'torque_nm':None,
                        'fz_N':{k:round(float(v),3) for k,v in row['fz_N'].items()},
                        'loaded':row['contact_mask']})
    info=CASE_LIST[case]
    min_contact=min(results['contact_count_full_2ms'],key=int)
    actual_min=int(min_contact)
    diagnostic={
        'format':'fnk0052-physical-points-v1',
        'source':'Re-run recorded 2.4s MuJoCo CAD30 PD contact diagnostic, PR #35',
        'recording_sha256':origin_sha,
        'visualization':'Genuine sampled qpos, not generated animation. 2-support was NOT achieved.',
        'units':'metres','coordinate_system':'MuJoCo Z-up',
        'duration_s':2.4,'sample_interval_s':0.05,'sample_count':49,'stride':80,
        'layout':'base xyz+quat, 6x[joint anchors and toe tip] xyz, actual Fz threshold mask',
        'leg_order':list(LEG_ORDER),'forward_xy':[0,1],
        'summary':{'diagnostic':True,'requested_unloads':list(info['lift']),
                   'actual_min_loaded_over_all_1200_physics_steps':actual_min,
                   'contact_count_full_2ms':results['contact_count_full_2ms'],
                   'requested_pair_matched_during_hold':results['hold_actual_contacts_match_requested_frames'] == 200,
                   'evidence_of_four_step_gait':False,
                   'model_torque_saturation_steps':results['saturated_physics_steps'],
                   'actual_forward_displacement_mm':round(results['COM_forward_displacement_mm'],3)},
        'phase_samples':phases,'support_samples':samples,
        'limitations':['Short PD test, not MPC or gait; zero 2-loaded timesteps in two-lift trials',
                       'Contacts observed every 2 ms and sampled for playback every 50 ms',
                       'Original CAD mesh binaries are not distributed',
                       'Mass/torque/friction are model parameters, not calibrated hardware'],
        'frames':frames,
    }
    cad=copy.deepcopy(donor)
    cad.update({'duration_s':2.4,'samples':49,'sample_interval_s':.05,
                'source_experiment':'https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/35',
                'source_qpos_sha256':hashlib.sha256(encoded(readings)).hexdigest(),
                'sample_base0':rig_frames[0][:7],'sample_base_last':rig_frames[-1][:7],
                'frames':rig_frames})
    out.mkdir(parents=True,exist_ok=True)
    ps=out/f'contact-{case}-points.json'
    rigp=out/f'cad-rig-contact-{case}.json'
    ps.write_bytes(encoded(diagnostic));rigp.write_bytes(encoded(cad))
    return {'case':case,'points_file':ps.name,'rig_file':rigp.name,
            'points_bytes':ps.stat().st_size,'rig_bytes':rigp.stat().st_size,
            'points_sha256':sha(ps),'rig_sha256':sha(rigp),
            'source_replay_sha256':origin_sha,'original_min_loaded':actual_min,
            'recorded_qpos_frames':len(readings)}

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--state',type=Path,required=True)
    p.add_argument('--donor',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--outdir',type=Path,required=True)
    args=p.parse_args()
    donor=json.loads(args.donor.read_text())
    old=json.loads(args.baseline.read_text())
    audit=[]
    args.outdir.mkdir(parents=True,exist_ok=True)
    for case,info in CASE_LIST.items():
        result=simulate(args.model,args.state,info['lift'],capture_qpos=True)
        original=old[case]
        # Independent equality gates against immutable previously executed experiment.
        for key in ('contact_count_full_2ms','hold_actual_contacts_match_requested_frames',
                    'saturated_physics_steps','front_pair_unexpected_ground_contact_physics_steps'):
            if result[key]!=original[key]:raise RuntimeError(f'{case}: changed baseline outcome for {key}: {result[key]} vs {original[key]}')
        assert abs(result['COM_forward_displacement_mm']-original['COM_forward_displacement_mm'])<1e-5
        assert not result['transient_two_support_seen']
        raw=args.outdir/f'raw-physics-{case}.json'
        raw.write_bytes(encoded(result))
        item=export(args.model,result,donor,args.outdir,raw,case)
        audit.append({'case':case,'requested_unloads':list(info['lift']),**item,
                      'min_loaded_during_hold':result['min_loaded_during_lift_hold'],
                      'exact_hold_match_frames':result['hold_actual_contacts_match_requested_frames']})
        print(f'{case}: 49 real qpos, {item["rig_bytes"]} CAD rig bytes, all 1200 dynamics steps match baseline',flush=True)
    (args.outdir/'replay_audit.json').write_bytes(encoded({'schema':'cad30-contact-modes-viewer-v1',
             'source_baseline_sha256':sha(args.baseline),'source_model_input':'original local CAD30 model - do not redistribute',
             'source_pose_sha256':sha(args.state),'original_donor_rig_sha256':sha(args.donor),
             'verified_cases':audit,'model_masses_uncalibrated':True}))

if __name__=='__main__': main()