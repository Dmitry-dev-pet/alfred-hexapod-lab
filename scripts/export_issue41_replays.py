#!/usr/bin/env python3
"""Convert real Issue #41 qpos/Fz streams to offline Three.js CAD30 replay.

This is an evidence-only deterministic transform, NOT a scene animator: every
3D body pose comes from MuJoCo mj_forward over actually saved qpos. Actual Fz
and contact flags are copied from the physical solver, never inferred from a
commanded leg schedule. Original third-party STL remains external/private.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
from pathlib import Path
import mujoco as mj
import numpy as np
from hexlab.stand_geometry_v04 import load_legs,tip_world

LEG_ORDER=('ml','bl','br','mr','fr','fl')
EXPECTED={
 'rear5':('rear_5mm_mu1p1.json','RESEARCH_GATE_FAILED',1.1),
 'rear10':('rear_10mm_mu1p1.json','PARTIAL_REAR_PAIR_FORWARD_STEP_LT20MM',1.1),
 'mu08':('rear_10mm_mu0p8.json','RESEARCH_GATE_FAILED',.8),
 'mu06':('rear_10mm_mu0p6.json','RESEARCH_GATE_FAILED',.6),
 'mu03':('rear_10mm_mu0p3.json','RESEARCH_GATE_FAILED',.3),
}

def digest(p:Path):return hashlib.sha256(p.read_bytes()).hexdigest()
def encode(obj):return (json.dumps(obj,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n').encode('utf-8')
def vec(seq,n=7):return [round(float(x),n) for x in seq]

def export_one(model_path:Path,record_path:Path,donor_path:Path,outdir:Path,variant:str):
 if variant not in EXPECTED:raise ValueError('Unrecognized strict Issue41 replay variant')
 filename,expected_status,expected_mu=EXPECTED[variant]
 if record_path.name!=filename:raise ValueError('Wrong input trace for requested variant')
 r=json.loads(record_path.read_text())
 if r.get('schema')!='cad30_forward_foot_candidate_2support_v1' or r['status']!=expected_status or r['parameters']['mu']!=expected_mu:
  raise ValueError('Source trace incompatible with recorded physical variant')
 if r['is_full_walk'] or r['is_full_MPC'] or r['hardware_verified']:
  raise ValueError('Impossible/false physical claims in source trace')
 if bool(r['partial_rear_pair_step_pass'])!=(variant=='rear10'):
  raise ValueError('False partial success classification')
 if r['target_20mm_net_forward_pass'] or (r['partial_rear_pair_step_pass'] and r['net_forward_COM_mm']>=20):
  raise ValueError('Trace conflicts with documented Issue41 target acceptance')
 physical=r.get('physics_steps_2ms')
 samples=r.get('qpos_samples_20ms')
 if not isinstance(physical,list) or len(physical)<1500 or not isinstance(samples,list) or len(samples)<150:
  raise ValueError('Missing raw physical 2ms contact evidence or qpos samples')
 if len(physical)!=round(r['total_time_s']/.002):raise ValueError('Incomplete physics frames')
 ts=np.array([float(x['t']) for x in samples]);dts=np.diff(ts)
 if not np.allclose(dts,.02,rtol=0,atol=1e-7) or abs(ts[0]-.02)>1e-8:
  raise ValueError('Missing/not-monotone actual 20-ms qpos time grid')
 donor=json.loads(donor_path.read_text())
 m=mj.MjModel.from_xml_path(str(model_path))
 if (m.nmesh,m.nq)!=(30,27) or len(donor['bodies'])!=27 or len(donor['meshes'])!=30:
  raise ValueError('Original CAD30 or donor metadata not verified')
 d=mj.MjData(m);legs=load_legs(m,d,mj)
 if tuple(l['suffix'] for l in legs)!=LEG_ORDER:raise ValueError('Leg order changed')
 ids=[int(b['id']) for b in donor['bodies']]
 b=int(m.body('base_link_2').id)
 points=[];rigs=[];phases=[];support=[]
 for sample in samples:
  q=np.asarray(sample['qpos'],float);qd=np.asarray(sample['qvel'],float)
  if q.shape!=(m.nq,) or qd.shape!=(m.nv,) or not np.isfinite(q).all() or not np.isfinite(qd).all():
   raise ValueError('Invalid recorded physical qpos/qvel')
  d.qpos[:]=q;d.qvel[:]=qd;mj.mj_forward(m,d)
  pts=vec([*d.xpos[b],*d.xquat[b]])
  for leg in legs:
   for jid in leg['jids']:pts.extend(vec(d.xanchor[jid]))
   pts.extend(vec(tip_world(m,d,leg)))
  mask=sum(1<<i for i,k in enumerate(LEG_ORDER) if float(sample['Fz_N'][k])>.08)
  pts.append(mask)
  if len(pts)!=80:raise ValueError('Mismatched Freenove points frame length')
  points.append(pts)
  rr=[]
  for bi in ids:rr.extend(vec(d.xpos[bi]));rr.extend(vec(d.xquat[bi]))
  if len(rr)!=189:raise ValueError('Mismatched CAD 27-body frame length')
  rigs.append(rr)
  actual_loaded=tuple(k for k in ('ml','mr','bl','br') if float(sample['Fz_N'][k])>.08)
  if set(actual_loaded)!=set(sample['loaded']):raise ValueError('Fabricated/suspect contact mask')
  phases.append(sample['phase'])
  support.append({
   'contacts':len(actual_loaded),'loaded_stance':list(actual_loaded),
   'fz_N':{k:round(float(sample['Fz_N'][k]),5) for k in ('ml','mr','bl','br')},
   'rear_clearance_mm':sample['rear_clearance_mm'],
   'rear_forward_mm':sample['rear_forward_mm'],
   'genuine_two_support':bool(sample['genuine_two_support']),
   'body_tilt_deg':sample['body_tilt_delta_deg'],
   'com_y_mm':sample['COM_y_mm'],
  })
 duration=round(float(ts[-1]-ts[0]),6)
 meta={
  'format':'fnk0052-physical-points-v1',
  'source':'Freenove FNK0052 CAD30 · measured free-base MuJoCo 3.14 Issue41 draft PR42',
  'recording_sha256':digest(record_path),
  'study':'issue41-rear-pair-forward-partial',
  'case_id':variant,
  'visualization':'All poses are from recorded physical qpos; Fz/masks from physical contact solver; not a scripted gait animation',
  'units':'metres','coordinate_system':'MuJoCo Z-up',
  'duration_s':duration,'source_time_offset_s':round(float(ts[0]),5),
  'sample_interval_s':.02,'sample_count':len(points),'stride':80,
  'layout':'base world xyz+quat, 6 legs 3xjoint anchors+toe world XYZ, true Fz-based contact mask',
  'leg_order':list(LEG_ORDER),'forward_xy':[0,1],
  'summary':{'diagnostic':True,'was_walk':False,'was_full_MPC':False,
             'phase':'rear-pair forward experiment','case_id':variant,
             'controller':'instantaneous 2-support QP, fixed phase schedule, physical PD touchdown + angle softstop',
             'mu':expected_mu,'status':r['status'],
             'rear_advance_target_mm':r['candidate_stride_mm'],
             'actual_rear_final_forward_mm':{k:v['forward_from_initial_mm'] for k,v in r['final_rear_footholds'].items()},
             'net_forward_COM_mm':r['net_forward_COM_mm'],
             'actual_max_two_support_contiguous_s':r['longest_contiguous_true_two_support_seconds'],
             'last_0p2s_all_four_reloaded':r['last_0p2s_all_four_Fz_ge_0p2N'],
             'below_two_support_steps':r['below_two_support_steps'],
             'max_motor_tau_Nm':r['max_motor_tau_Nm'],
             'max_joint_abs_rad':r['max_joint_abs_rad']},
  'phase_samples':phases,'support_samples':support,
  'limitations':['Not a full 4-move gait, not contact-implicit/horizon MPC; COM +20mm does NOT mean PASS when contacts/torques fail',
                 'One deterministic nominal robot and fixed exploratory control parameters; no hardware calibration',
                 'MuJoCo 2ms actual contacts; 20ms downsampled visualization',
                 'Original Freenove STL/mesh files are NOT redistributed; uses local verified viewer cache'],
  'frames':points,
 }
 cad=copy.deepcopy(donor)
 cad.update({'duration_s':duration,'samples':len(rigs),'sample_interval_s':.02,
             'source_qpos_sha256':hashlib.sha256(encode(samples)).hexdigest(),
             'source_recording_sha256':digest(record_path),
             'source_experiment':'https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/42',
             'case_id':variant,
             'sample_base0':rigs[0][:7],'sample_base_last':rigs[-1][:7],
             'frames':rigs})
 outdir.mkdir(parents=True,exist_ok=True)
 p=outdir/f'issue41-{variant}-points.json';c=outdir/f'cad-rig-issue41-{variant}.json'
 p.write_bytes(encode(meta));c.write_bytes(encode(cad))
 return {'variant':variant,'status':expected_status,'points':p.name,'cad':c.name,'duration_s':duration,
         'samples':len(points),'points_sha256':digest(p),'points_bytes':p.stat().st_size,
         'cad_sha256':digest(c),'cad_bytes':c.stat().st_size,'source_sha256':digest(record_path)}

def export_all(model:Path,donor:Path,sources:Path,outdir:Path):
 receipt={}
 for variant,(filename,*_) in EXPECTED.items():
  receipt[variant]=export_one(model,sources/filename,donor,outdir,variant)
 (outdir/'issue41_export_receipt.json').write_bytes(encode(receipt))
 return receipt

if __name__=='__main__':
 p=argparse.ArgumentParser()
 for field in ('model','donor','sources','outdir'):p.add_argument('--'+field,type=Path,required=True)
 a=p.parse_args()
 print(json.dumps(export_all(a.model,a.donor,a.sources,a.outdir),ensure_ascii=False,indent=2))
