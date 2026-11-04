"""Export three recorded CAD30 MuJoCo folding trials into interactive Three.js replay.

Uses only previously physically simulated qpos and mj_forward. Never moves
root/artificially alters a trajectory, creates videos, or runs a new gait.
"""
from __future__ import annotations
import json,hashlib,sys
from pathlib import Path
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from hexlab.stand_geometry_v04 import load_legs,tip_world
CODES=('ml','bl','br','mr','fr','fl')
POSES=('extended','semi_tucked','compact')

def rounded(vals,n=6):return [round(float(x),n) for x in vals]

def export(source:Path,compact:Path,model_path:Path,rig_template:Path,outdir:Path):
    records=json.loads(source.read_text(encoding='utf-8'))
    summary=json.loads(compact.read_text(encoding='utf-8'))
    assert tuple(records)==POSES and summary['recommended'] in POSES
    donor=json.loads(rig_template.read_text(encoding='utf-8'))
    assert donor['format']=='fnk0052-mujoco-cad-visual-replay-v1' and len(donor['bodies'])==27 and len(donor['meshes'])==30
    bodies=[int(b['id']) for b in donor['bodies']]
    model=mujoco.MjModel.from_xml_path(str(model_path));data=mujoco.MjData(model)
    legs=load_legs(model,data,mujoco); assert tuple(l['suffix'] for l in legs)==CODES
    assert model.nmesh==30
    base=model.body('base_link_2').id
    outdir.mkdir(parents=True,exist_ok=True)
    results={}
    baseline_before_fold=None
    for mode in POSES:
        r=records[mode]
        assert r['strict_pass'] and r['model_friction_mu']==1.1
        assert r['min_support_four_after_unload']==4 and not r['saturated_steps_fraction']
        rawframes=r['qpos_samples_10hz']; assert rawframes
        rawtrace=r['support_samples_50hz']; assert rawtrace
        frames_by_time={}
        for f in rawframes:frames_by_time[round(float(f['t']),3)]=f
        times=sorted(frames_by_time)
        assert len(times)==93 and np.allclose(np.diff(times),.1,atol=1e-6), (len(times),times[:4],times[-4:])
        early=[frames_by_time[t]['qpos'] for t in times if t<=6.8]
        if baseline_before_fold is None: baseline_before_fold=early
        else: assert np.allclose(np.asarray(early),np.asarray(baseline_before_fold),atol=1e-8)
        srcbytes=source.read_bytes()
        trace_idx=0
        pointframes=[];rigframes=[];phases=[];support=[]
        for t in times:
            snap=frames_by_time[t]
            q=np.asarray(snap['qpos'],float)
            assert q.size==model.nq and np.isfinite(q).all()
            data.qpos[:]=q;data.qvel[:]=0;mujoco.mj_forward(model,data)
            while trace_idx+1<len(rawtrace) and abs(float(rawtrace[trace_idx+1]['time_s'])-t)<=abs(float(rawtrace[trace_idx]['time_s'])-t):trace_idx+=1
            row=rawtrace[trace_idx]
            contacts=set(snap['contacts']);mask=sum((1<<i) for i,c in enumerate(CODES) if c in contacts)
            coords=[*data.xpos[base].tolist(),*data.xquat[base].tolist()]
            for leg in legs:
                for jid in leg['jids']:coords.extend(data.xanchor[jid].tolist())
                coords.extend(tip_world(model,data,leg).tolist())
            row80=rounded(coords,5)+[mask];assert len(row80)==80
            pointframes.append(row80)
            rr=[]
            for bid in bodies:rr.extend(rounded(data.xpos[bid],7)+rounded(data.xquat[bid],8))
            assert len(rr)==189
            rigframes.append(rr)
            phases.append(snap['phase'])
            support.append({'contacts':len(contacts),'four_contacts':sum(c in contacts for c in CODES[:4]),
                            'margin_mm':float(row['COM_margin_four_mm']),'front_force_N':float(row['front_force_N']),
                            'torque_nm':float(row['torque_max_nm']),'tilt_deg':float(row['tilt_deg'])})
        short=summary['comparison'][mode]
        points={
            'format':'fnk0052-physical-points-v1','schema':'three-claw-folding-postures-v1',
            'source':'Recorded 9.3-second physical MuJoCo CAD30 trial: six grounded -> prebalance -> unload front -> raised front -> fold',
            'source_sha256':hashlib.sha256(srcbytes).hexdigest(),
            'visualization':'Replay of measured qpos, not live physics; no 4-legged gait',
            'model_friction_mu':1.1,'units':'metres','coordinate_system':'MuJoCo Z-up',
            'duration_s':round(times[-1]-times[0],3),'sample_interval_s':.1,'sample_count':len(times),'stride':80,
            'leg_order':list(CODES),'forward_xy':[0,1],
            'summary':{'pose':mode,'final_support_margin_mm':short['final_COM_margin_mm'],
                       'final_COM_y_mm':short['final_COM_y_mm'],'final_front_clearance_mm':short['final_front_tibia_min_height_mm'],
                       'min_four_contacts_after_unload':short['min_four_contacts_after_front_unload'],
                       'motor_saturation_fraction':short['motor_saturation_fraction'],
                       'joint_limit_reserve_front_deg':short['final_joint_margin_to_1p7_deg'],
                       'motor_torque_peak_nm':short['peak_unclipped_motor_torque_nm'],
                       'four_leg_gait_verified':False,'first_six_legs_grounded':True,
                       'original_mode_preferred':'semi_tucked'},
            'phase_samples':phases,'support_samples':support,'frames':pointframes,
            'limitations':summary['limitations']}
        rig={k:v for k,v in donor.items() if k!='frames'}
        rig.update({'duration_s':points['duration_s'],'samples':len(rigframes),
                    'sample_interval_s':.1,'source_recording_sha256':points['source_sha256'],'frames':rigframes})
        fpoints=outdir/f'fold-{mode}.json';frig=outdir/f'cad-rig-fold-{mode}.json'
        fpoints.write_text(json.dumps(points,ensure_ascii=False,separators=(',',':'))+'\n',encoding='utf-8')
        frig.write_text(json.dumps(rig,ensure_ascii=False,separators=(',',':'))+'\n',encoding='utf-8')
        results[mode]={'points_path':str(fpoints.relative_to(ROOT)),'rig_path':str(frig.relative_to(ROOT)),
                       'frames':len(times),'points_sha256':hashlib.sha256(fpoints.read_bytes()).hexdigest(),
                       'rig_sha256':hashlib.sha256(frig.read_bytes()).hexdigest()}
    return results

if __name__=='__main__':
    result=export(ROOT/'results/claw_tuck_compare_2026-11-04.json',
                  ROOT/'results/claw_tuck_comparison_compact_2026-11-04.json',
                  ROOT/'source/mjcf/scene_local30_nominal.xml',
                  ROOT/'docs/hexapod/data/cad-rig-six-to-four-7s.json',ROOT/'docs/hexapod/data')
    print(json.dumps(result,indent=2))
