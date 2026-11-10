#!/usr/bin/env python3
"""Export *recorded* CAD30 QP two-support MuJoCo data to local Three.js player.

This tool does NOT re-simulate, invent movement, distribute third-party STL,
or infer actual contacts from commanded foot poses. Inputs must contain both
real observed qpos and force/contact-pair measurements from every 2ms step.
"""
from __future__ import annotations
import argparse,copy,hashlib,json
from pathlib import Path
import mujoco as mj
import numpy as np
from hexlab.stand_geometry_v04 import load_legs,tip_world

LEG_ORDER=('ml','bl','br','mr','fr','fl')
FORMAT='fnk0052-physical-points-v1'
RIGFORMAT='fnk0052-mujoco-cad-visual-replay-v1'
EXPECTED={'nominal':'TRANSIENT_TWO_SUPPORT_AND_FOUR_SUPPORT_RECOVERED',
          'slippery':'RESEARCH_GATE_FAILED'}

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def encode(obj):return (json.dumps(obj,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n').encode()
def v(arr,n=7):return [round(float(x),n) for x in arr]

def export(model_path,record_path,donor_path,outdir,name):
    if name not in EXPECTED:raise ValueError('Use nominal or slippery only')
    record=json.loads(record_path.read_text())
    if record['schema']!='cad30_two_support_qp_then_pd_v1' or record['status']!=EXPECTED[name]:
        raise RuntimeError('Wrong physical source/outcome for viewer '+name)
    if record['is_full_walk'] or record['is_full_MPC']:
        raise RuntimeError('Cannot mislabel QP excursion as walking or MPC')
    mu=record['parameters']['mu']
    if (name=='nominal' and mu!=1.1) or (name=='slippery' and mu!=.6):
        raise RuntimeError('Unsupported friction setting')
    samples=record['qpos_samples_20ms']
    if not isinstance(samples,list) or len(samples)<180:raise RuntimeError('Insufficient physical sample coverage')
    if len(record['trace_20ms'])!=len(samples):raise RuntimeError('Missing actual contact traces')
    times=np.array([float(x['t']) for x in samples])
    if not np.allclose(np.diff(times),.02,atol=1e-7):raise RuntimeError('Not the archived 20ms sample grid')
    if abs(times[0]-.02)>1e-8:raise RuntimeError('Unrecorded starting frame should not be fabricated')
    donor=json.loads(donor_path.read_text())
    model=mj.MjModel.from_xml_path(str(model_path))
    if model.nmesh!=30 or model.nq!=27 or len(donor['bodies'])!=27 or len(donor['meshes'])!=30:
        raise RuntimeError('Wrong real CAD30 model/donor metadata')
    data=mj.MjData(model)
    legs=load_legs(model,data,mj)
    if tuple(x['suffix'] for x in legs)!=LEG_ORDER:raise RuntimeError('Leg ordering mismatch')
    body_ids=[int(b['id']) for b in donor['bodies']]
    base=int(model.body('base_link_2').id)
    points=[];rig_frames=[];phase=[];support=[]
    for row in samples:
        if len(row['qpos'])!=model.nq or not np.isfinite(row['qpos']).all():
            raise RuntimeError('Missing/corrupt actual physical joint positions')
        data.qpos[:]=row['qpos'];data.qvel[:]=row['qvel'];mj.mj_forward(model,data)
        root=[*data.xpos[base],*data.xquat[base]]
        anchors=[]
        for leg in legs:
            for jid in leg['jids']:anchors.extend(v(data.xanchor[jid]))
            anchors.extend(v(tip_world(model,data,leg)))
        mask=sum(1<<i for i,k in enumerate(LEG_ORDER) if k in row['loaded'])
        point=v(root)+anchors+[mask]
        if len(point)!=80:raise RuntimeError('Wrong points frame stride')
        points.append(point)
        rig=[]
        for bi in body_ids:rig+=v(data.xpos[bi])+v(data.xquat[bi])
        if len(rig)!=189:raise RuntimeError('Wrong CAD rig stride')
        rig_frames.append(rig)
        phase.append(row['phase'])
        fz={k:float(row['Fz_N'][k]) for k in ('ml','mr','bl','br')}
        detected=[k for k in ('ml','mr','bl','br') if fz[k]>.08]
        if set(detected)!=set(row['loaded']):raise RuntimeError('Mismatch measured forces and stored actual contact mask')
        support.append({'contacts':len(row['loaded']), 'loaded':row['loaded'], 'fz_N':fz,
            'rear_clearance_mm':row['rear_clearance_mm'],
            'genuine_two_support':bool(row['genuine_two_support']),
            'body_tilt_deg':row['body_tilt_delta_deg'],'com_y_mm':row['COM_y_mm']})
    duration=round(float(times[-1]-times[0]),6)
    contact_min=record['longest_contiguous_true_two_support_seconds']
    if name=='nominal' and (contact_min<.2 or not record['last_0p2s_all_four_Fz_ge_0p2N']):
        raise RuntimeError('False nominal success')
    if name=='slippery' and record['last_0p2s_all_four_Fz_ge_0p2N']:
        raise RuntimeError('Misclassified negative run')
    meta={
      'format':FORMAT,
      'source':'Recorded free-base MuJoCo 3.14 CAD30 whole-body QP + PD return, draft PR #39',
      'recording_sha256':sha(record_path),
      'visualization':'Measured qpos 20ms, contacts and forces are from real physics; short 2-support excursion, NOT a walking gait',
      'units':'metres', 'coordinate_system':'MuJoCo Z-up',
      'duration_s':duration, 'source_time_offset_s':round(float(times[0]),6),
      'sample_interval_s':.02,'sample_count':len(points),'stride':80,
      'layout':'base xyz+quat, 6 legs x3 joints & toe world coordinates, contact bit mask',
      'leg_order':list(LEG_ORDER),'forward_xy':[0,1],
      'summary':{'diagnostic':True,'controller':'instantaneous full-body QP / scheduled PD recovery',
          'mu':mu,'status':record['status'], 'actual_max_true_two_support_contiguous_s':contact_min,
          'last_0p2s_four_Fz_ge_0p2N':record['last_0p2s_all_four_Fz_ge_0p2N'],
          'actual_original_physics_duration_s':record['total_time_s'],
          'max_motor_tau_Nm':record['max_motor_tau_Nm'], 'was_walk':False,'was_full_MPC':False},
      'phase_samples':phase,'support_samples':support,
      'limitations':['Finite 0.12s QP phase manually switches back to PD; no automatic future step or forward foothold placement',
                     'Physical contacts measured each 2ms; downsampled 20ms for player',
                     'Real joint geometry under conditional torque/friction/mass assumptions; no hardware calibration',
                     'Original Freenove CAD binary is private; meshes from existing verified local viewer cache'],
      'frames':points,
    }
    cad=copy.deepcopy(donor)
    cad.update({'duration_s':duration,'samples':len(rig_frames),'sample_interval_s':.02,
                'source_qpos_sha256':hashlib.sha256(encode(samples)).hexdigest(),
                'source_recording_sha256':sha(record_path),
                'source_experiment':'https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/39',
                'sample_base0':rig_frames[0][:7],'sample_base_last':rig_frames[-1][:7],
                'frames':rig_frames})
    outdir.mkdir(parents=True,exist_ok=True)
    point_path=outdir/f'dynamic-{name}-points.json'
    cad_path=outdir/f'cad-rig-dynamic-{name}.json'
    point_path.write_bytes(encode(meta));cad_path.write_bytes(encode(cad))
    return {'name':name,'points':point_path.name,'rig':cad_path.name,
            'samples':len(points),'duration':duration,'points_bytes':point_path.stat().st_size,
            'rig_bytes':cad_path.stat().st_size,'points_sha256':sha(point_path),
            'rig_sha256':sha(cad_path),'source_sha256':sha(record_path)}

def main():
    p=argparse.ArgumentParser()
    for field in ('model','donor','nominal','slippery','outdir'):
        p.add_argument('--'+field,required=True,type=Path)
    args=p.parse_args()
    receipt={k:export(args.model,getattr(args,k),args.donor,args.outdir,k) for k in EXPECTED}
    (args.outdir/'export_receipt.json').write_bytes(encode(receipt))
    print(json.dumps(receipt,indent=2,ensure_ascii=False))

if __name__=='__main__':main()
