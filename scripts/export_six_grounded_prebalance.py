"""Deterministic export of the observed six-grounded-to-four-claw MuJoCo pose trace.

Input is an actual MuJoCo free-body qpos recording and its 50Hz contact log.
This exporter runs only mj_forward to recover visual CAD body poses. No
video render, fabricated root translations, or re-simulation of physics.
"""
from __future__ import annotations
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
import mujoco
from hexlab.stand_geometry_v04 import load_legs,tip_world

CODES=('ml','bl','br','mr','fr','fl')

def roundlist(vals,n=6):return [round(float(x),n) for x in vals]

def export(model_path:Path,source_path:Path,cad_manifest:Path,out_points:Path,out_rig:Path):
    source_bytes=source_path.read_bytes(); rec=json.loads(source_bytes)
    assert rec['model_friction_mu']==1.1
    assert rec['min_stance_contacts_all_steps']==4
    assert rec['min_contacts_all_steps']>=4
    assert rec['motor_saturation_fraction']==0.
    assert rec['after_unload_front_contacts']==0
    assert min(rec['high_claw_clearances_mm'].values())>0.09*1000
    frames=rec['qpos_10hz']; trace=rec['timeseries_50hz']; assert len(frames)==69 and len(trace)>200
    assert frames[0]['t']==0. and set(frames[0]['contacts'])==set(CODES)
    times=np.array([x['t'] for x in frames]); assert np.allclose(np.diff(times),.1,atol=1e-6)
    m=mujoco.MjModel.from_xml_path(str(model_path)); d=mujoco.MjData(m)
    legs=load_legs(m,d,mujoco)
    assert tuple(l['suffix'] for l in legs)==CODES and m.nmesh==30
    donor=json.loads(cad_manifest.read_text(encoding='utf-8'))
    bodies=[a['id'] for a in donor['bodies']];assert len(bodies)==27
    base_id=m.body('base_link_2').id
    packed=[];rigs=[]; phases=[]; support=[];i=0
    for frame in frames:
        t=float(frame['t'])
        qpos=np.asarray(frame['qpos'],float)
        assert len(qpos)==m.nq and np.isfinite(qpos).all()
        d.qpos[:]=qpos;d.qvel[:]=0.;mujoco.mj_forward(m,d)
        while i+1<len(trace) and abs(float(trace[i+1]['t'])-t)<abs(float(trace[i]['t'])-t):i+=1
        row=trace[i]
        contacts=set(frame['contacts'])
        assert len(contacts)>=4
        mask=sum(1<<j for j,c in enumerate(CODES) if c in contacts)
        root=[*d.xpos[base_id].tolist(),*d.xquat[base_id].tolist()]
        anchors=[]
        for leg in legs:
            for jid in leg['jids']:anchors.extend(d.xanchor[jid].tolist())
            anchors.extend(tip_world(m,d,leg).tolist())
        p=roundlist(root+anchors,5)+[mask];assert len(p)==80
        packed.append(p)
        rig=[]
        for bid in bodies:rig.extend(roundlist(d.xpos[bid],7)+roundlist(d.xquat[bid],8))
        assert len(rig)==189
        rigs.append(rig)
        phases.append(frame['phase'])
        support.append({'contacts':len(contacts),'margin_mm':row['four_margin_mm'],
                        'torque_nm':row['max_abs_tau_nm'],
                        'front_force_N':row['front_N'],
                        'four_contacts':sum(c in contacts for c in CODES[:4])})
    # Forward axis is CAD +Y. This replay has insignificant body advancement;
    # distance display is a body displacement, NOT a walking metric.
    meta={'format':'fnk0052-physical-points-v1','schema':'six-grounded-claws-prebalance-v1',
       'source':'Recorded free-body MuJoCo 3.14.0: six grounded legs to stable raised-front stance, not a gait',
       'source_sha256':hashlib.sha256(source_bytes).hexdigest(),
       'visualization':'Replay of physical qpos samples, not real-time MuJoCo',
       'units':'metres','coordinate_system':'MuJoCo Z-up','duration_s':round(float(times[-1]-times[0]),3),
       'sample_interval_s':.1,'sample_count':len(frames),'stride':80,
       'leg_order':list(CODES),'forward_xy':[0,1],
       'summary':{'starts_with_six_ground_contacts':True,'four_supports_maintained_2ms':True,
             'raised_front_pair':True,'four_leg_gait_verified':False,
             'cmd_support_shift_mm':rec['shift_cmd_mm'],
             'measured_prep_com_xy_m':rec['after_prep_com_xy_m'],
             'final_four_support_margin_mm':rec['after_unload_four_margin_mm'],
             'minimum_actual_contacts_2ms':rec['min_contacts_all_steps'],
             'front_last_Fz_N':rec['after_unload_front_load_N'],
             'front_height_clearance_mm':rec['high_claw_clearances_mm'],
             'torque_saturation_fraction':rec['motor_saturation_fraction'],
             'max_unclipped_torque_nm':rec['peak_unclipped_control_Nm'],
             'model_friction_mu':rec['model_friction_mu']},
       'phase_samples':phases,'support_samples':support,
       'limitations':['Nominal model μ1.1 only; μ0.6/.3 failed four-support continuity',
           'No four-legged walking or step; front limbs only raised from a grounded six-leg start',
           'Original third-party CAD STL fetched from pinned upstream, not bundled here',
           'No hardware servo, mass or traction calibration',
           'Contact state from original 2ms MuJoCo check, rendered poses sampled at 10Hz'],
       'frames':packed}
    rigmeta={k:v for k,v in donor.items() if k!='frames'}
    rigmeta.update({'duration_s':meta['duration_s'],'samples':len(rigs),'sample_interval_s':.1,
       'source_experiment':'https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/21',
       'source_recording_sha256':meta['source_sha256'],
       'sample_base0':rigs[0][:7],'sample_base_last':rigs[-1][:7],'frames':rigs})
    for target,obj in ((out_points,meta),(out_rig,rigmeta)):
       target.parent.mkdir(parents=True,exist_ok=True)
       target.write_text(json.dumps(obj,ensure_ascii=False,separators=(',',':'))+'\n',encoding='utf-8')
    return {'num_frames':len(frames),'duration_s':meta['duration_s'],
       'points_sha256':hashlib.sha256(out_points.read_bytes()).hexdigest(),
       'cad_rig_sha256':hashlib.sha256(out_rig.read_bytes()).hexdigest(),
       'points_size':out_points.stat().st_size,'cad_rig_size':out_rig.stat().st_size}

if __name__=='__main__':
 p=argparse.ArgumentParser()
 p.add_argument('--source',type=Path,default=Path('results/six_grounded_claws_mu1p1_2026-11-04.json'))
 p.add_argument('--model',type=Path,default=Path('source/mjcf/scene_local30_nominal.xml'))
 p.add_argument('--cad',type=Path,default=Path('docs/hexapod/data/cad-rig-60s.json'))
 p.add_argument('--points',type=Path,default=Path('docs/hexapod/data/six-to-four-7s.json'))
 p.add_argument('--rig',type=Path,default=Path('docs/hexapod/data/cad-rig-six-to-four-7s.json'))
 a=p.parse_args()
 print(json.dumps(export(a.model,a.source,a.cad,a.points,a.rig),indent=2))
