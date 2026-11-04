#!/usr/bin/env python3
"""Export observed MuJoCo qpos/contact trace for the experimental claws Three.js replay.

Uses only the actual 10 Hz recorded qpos and nearest 50 Hz observed contacts.
Never re-runs physics or fabricates missing motions. No CAD/STL files exported.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from hexlab.stand_geometry_v04 import load_legs, tip_world

LEG_ORDER=('ml','bl','br','mr','fr','fl')

def seq(vals,n=6): return [round(float(v),n) for v in vals]

def export(model_path, report_path, donor_cad_path, points_path, rig_path):
    source_bytes=report_path.read_bytes()
    report=json.loads(source_bytes)
    assert report['schema']=='FNK0052_event_claw_crawl_v1/1'
    assert report['integration_finished'] and report['seconds_simulated']==10.0
    assert not report['strict_four_leg_walk']
    samples=report['qpos_10hz']
    contacts=report['trace_50hz']
    assert len(samples)==100 and len(contacts)>400
    dt=np.diff([a['t'] for a in samples]); assert np.allclose(dt,0.1,atol=0.00001)
    model=mujoco.MjModel.from_xml_path(str(model_path))
    data=mujoco.MjData(model)
    legs=load_legs(model,data,mujoco)
    assert tuple(l['suffix'] for l in legs)==LEG_ORDER
    cad=json.loads(donor_cad_path.read_text())
    body_ids=[b['id'] for b in cad['bodies']]
    assert len(body_ids)==27 and model.nmesh==30 and model.nq==27
    base_id=model.body('base_link_2').id
    assert base_id in body_ids
    forward_xy=np.array([0.,1.])
    hip={a['suffix']:a['hip_xyz'] for a in legs}
    d=(hip['fr'][:2]+hip['fl'][:2]-hip['br'][:2]-hip['bl'][:2])/2
    if np.linalg.norm(d)>1e-8: forward_xy=d/np.linalg.norm(d)
    points=[]; rigframes=[]; phase_samples=[]; support_samples=[]
    ti=0
    for frame in samples:
        tt=float(frame['t'])
        qpos=np.asarray(frame['qpos'],float)
        assert len(qpos)==model.nq and np.isfinite(qpos).all()
        data.qpos[:]=qpos
        data.qvel[:]=0
        mujoco.mj_forward(model,data)
        while ti+1<len(contacts) and abs(contacts[ti+1]['t']-tt)<=abs(contacts[ti]['t']-tt): ti+=1
        obs=contacts[ti]
        mask=sum(1<<i for i,k in enumerate(LEG_ORDER) if k in obs['contacts'])
        root=[*data.xpos[base_id],*data.xquat[base_id]]
        anchors=[]
        for leg in legs:
            for jid in leg['jids']: anchors.extend(data.xanchor[jid].tolist())
            anchors.extend(tip_world(model,data,leg).tolist())
        points.append(seq(root+anchors,5)+[mask])
        rigline=[]
        for bi in body_ids: rigline.extend(seq(data.xpos[bi],7)+seq(data.xquat[bi],8))
        rigframes.append(rigline)
        phase_samples.append(str(frame.get('phase',obs.get('phase',''))))
        support_samples.append({'contacts':int(obs['support_count']),
                                'margin_mm':round(float(obs['support_margin_mm']),1),
                                'torque_nm':round(float(obs['max_tau_Nm']),4),
                                'tilt_deg':round(float(obs['tilt_deg']),2)})
    assert all(len(x)==80 for x in points)
    assert all(len(x)==189 for x in rigframes)
    assert all(len(f)>0 for f in phase_samples)
    assert len(points)==len(rigframes)
    meta={
      'format':'fnk0052-physical-points-v1',
      'source':'Recorded 10-second exploratory event-claw MuJoCo experiment; 100 original 10Hz qpos snapshots',
      'recording_sha256':hashlib.sha256(source_bytes).hexdigest(),
      'visualization':'Physical recorded pose replay; partial experiment, no four-legged walking claim',
      'units':'metres','coordinate_system':'MuJoCo Z-up',
      'duration_s':round(float(samples[-1]['t'])-float(samples[0]['t']),6),
      'sample_interval_s':0.1,'sample_count':len(points),'stride':80,
      'layout':'base xyz+quat (7), 6x [coxa, femur, tibia, mesh-tip] xyz (72), contact mask (1)',
      'leg_order':list(LEG_ORDER),'forward_xy':seq(forward_xy,6),
      'summary':{'full_four_leg_gait':False,'front_pair_held_clear':True,
        'verified_liftoff_touchdowns':report['verified_liftoff_touchdowns'],
        'new_leg_cycle_coverage':report['new_leg_cycle_coverage'],
        'forward_motion_4leg_interval_m':report['forward_motion_4leg_interval_m'],
        'minimum_actual_contacts_2ms':report['minimum_actual_contacts_2ms'],
        'max_torque_nm':report['max_torque_Nm'],
        'saturation_fraction':report['saturation_fraction']},
      'phase_samples':phase_samples, 'support_samples':support_samples,
      'events':[dict(t=round(float(e['t']),3),event=e['event'],leg=e.get('leg')) for e in report['events'] if e['event'] in ('claws_elevated','swing_approved','touchdown_checked')],
      'limitations':['Only 100 recorded qpos samples (10Hz) from the 10-second integration; playback ends at last sampled qpos',
        'Contact flags from nearest observed 50Hz trace, not re-inferred from zero-velocity qpos',
        'Estimated mass, friction and actuator limits are not hardware calibrated',
        'Rear leg lifts did not produce forward walking; both middle legs stayed grounded',
        'This is central quasistatic LP/QP selection, not a decentralized neural controller'],
      'frames':points,
    }
    cad.update({
      'duration_s':meta['duration_s'],'samples':len(rigframes),'sample_interval_s':0.1,
      'source_qpos_sha256':hashlib.sha256(json.dumps(samples,separators=(',',':'),sort_keys=True).encode()).hexdigest(),
      'sample_base0':rigframes[0][:7], 'sample_base_last':rigframes[-1][:7],
      'source_experiment':'https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/22',
      'source_recording_sha256':meta['recording_sha256'],
      'frames':rigframes
    })
    for path,obj in ((points_path,meta),(rig_path,cad)):
      path.parent.mkdir(parents=True,exist_ok=True)
      path.write_text(json.dumps(obj,ensure_ascii=False,separators=(',',':'))+'\n')
    return {"samples":len(samples),"duration":meta['duration_s'],"points_bytes":points_path.stat().st_size,"cad_bytes":rig_path.stat().st_size,"point_sha":hashlib.sha256(points_path.read_bytes()).hexdigest(),"rig_sha":hashlib.sha256(rig_path.read_bytes()).hexdigest()}

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--model',type=Path,default=Path('source/mjcf/scene_local30_nominal.xml'))
    p.add_argument('--recording',type=Path,default=Path('results/event_claw_crawl_mu1p1.json'))
    p.add_argument('--cad-manifest',type=Path,default=Path('docs/hexapod/data/cad-rig-60s.json'))
    p.add_argument('--points',type=Path,default=Path('docs/hexapod/data/claws-10s.json'))
    p.add_argument('--rig',type=Path,default=Path('docs/hexapod/data/cad-rig-claws-10s.json'))
    a=p.parse_args()
    print(json.dumps(export(a.model,a.recording,a.cad_manifest,a.points,a.rig),indent=2))
