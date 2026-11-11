"""Frozen-controller validation, preserving failures and an off-control comparison."""
import argparse,hashlib,json,math
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import mujoco
from simulate import simulate,make,zero_angle,quat_pitch,load_parts,build
from imu_control import Controller

GAINS={'kp':.3,'kd':.04,'kv':1.2,'ki':.3}
PEAK=.2
CASES=[('hold','nominal',True,False),('forward','nominal',True,False),('reverse','nominal',True,False),
       ('yaw-left','nominal',True,False),('yaw-right','nominal',True,False),('push','nominal',True,False),
       ('hold','light',True,False),('hold','heavy',True,False),('hold','nominal',True,True),('hold','nominal',False,False)]

def fixture_bench(xml,profile,fit,zero,enabled=True):
    root=E.fromstring(xml);shell=root.find('.//body[@name="shell"]');shell.remove(shell.find('freejoint'))
    m=mujoco.MjModel.from_xml_string(E.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
    R=np.array([[math.cos(zero),0,math.sin(zero)],[0,1,0],[-math.sin(zero),0,math.cos(zero)]])
    q=int(m.jnt_qposadr[m.joint('chassis_free').id]);d.qpos[q:q+3]=np.array([0,0,.1001])+R@fit
    d.qpos[q+3:q+7]=[math.cos(zero/2),0,math.sin(zero/2),0]
    ctl=Controller(zero=zero,kp=GAINS['kp'],kd=GAINS['kd'],kv=0,ki=0,enabled=enabled)
    ctl.update(np.zeros(3),R.T@np.array([0,0,9.81]),np.zeros(2));pending=None;request=np.zeros(2);frames=[];maxerr=0
    for k in range(4001):
        mujoco.mj_forward(m,d)
        if k%10==0:
            observation=(d.sensor('imu_gyro').data.copy(),d.sensor('imu_accel').data.copy(),np.array([d.sensor(s+'_encoder').data[0] for s in ['left','right']]))
            received=pending if pending is not None else observation;pending=observation;request=ctl.update(*received)
        d.ctrl[:]=np.clip(request-.596/(330*35/19*math.pi/30)*d.qvel[-2:],-PEAK,PEAK) if enabled else 0.
        error=quat_pitch(d.qpos[q+3:q+7])-zero;maxerr=max(maxerr,abs(error))
        if k%20==0:frames.append({'t':k*.001,'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'ctrl':d.ctrl.tolist(),'imu_control':ctl.last.copy()})
        if k<4000:mujoco.mj_step(m,d)
        if max(d.warning.number)>0:raise RuntimeError('Fixture integration warning')
    return {'schema':'alfred-bb8-fixed-shell-bench/v3','fixed_shell':True,'hardware_calibrated':False,'controller_enabled':enabled,
            'max_pitch_error_deg':math.degrees(maxerr),'final_pitch_error_deg':math.degrees(error),
            'final_gyro_rate_rad_s':d.sensor('imu_gyro').data.tolist(),'warnings':d.warning.number.tolist(),
            'passed_fixture_only':maxerr<math.radians(8) and abs(error)<math.radians(4),'frames':frames}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);ap.add_argument('--tire',type=Path,required=True)
    ap.add_argument('--fit',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--allow-estimated-mass',action='store_true')
    a=ap.parse_args()
    if not a.allow_estimated_mass:ap.error('Hardware masses are not verified; --allow-estimated-mass required for research')
    a.out.mkdir(parents=True,exist_ok=True);parts,_=load_parts(a.source,a.tire)
    fit=np.array(json.loads(a.fit.read_text())['fit']['rigid_translation_from_source_pose_mm'])*.001
    profiles={p:build(a.source,a.tire,p) for p in ['light','nominal','heavy']};zero,_=zero_angle(profiles['nominal'],fit)
    models={};report=[];hashes={};traces={}
    for p,profile in profiles.items():
        xml,meta=make(profile,parts,fit,level=5,peak=PEAK,collision='facets');models[p]=(mujoco.MjModel.from_xml_string(xml),meta,xml)
        (a.out/(p+'-model.xml')).write_text(xml);(a.out/(p+'-mass.json')).write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n')
    for case,p,enabled,noise in CASES:
        tag=case+'-'+p+('-noise' if noise else '')+('-OFF' if not enabled else '')
        model,meta,_=models[p]
        r=simulate(model,meta,fit,profiles[p],zero,case,6.,GAINS,enabled,PEAK,noise=noise)
        s=r['summary'];s['id']=tag
        passed=(s['max_pitch_error_deg']<10 and s['max_full_tilt_deg']<15 and s['final_speed_m_s']<.01
                and s['max_contact_penetration_m']<.0005 and min(s['wheel_contact_fraction'].values())>.8)
        if case in ['forward','reverse']:
            f1=r['frames'][50];f3=r['frames'][150];delta=f3['qpos'][0]-f1['qpos'][0]
            passed &= delta>.02 if case=='forward' else delta<-.02
        if case in ['yaw-left','yaw-right']:
            passed &= s['final_yaw_rad']>.2 if case=='yaw-left' else s['final_yaw_rad']<-.2
            passed &= abs(s['final_yaw_rate_rad_s'])<.1
        s['behavior_pass']=bool(passed);s['included_in_controlled_behavior_count']=enabled
        raw=(json.dumps(r,separators=(',',':'))+'\n').encode();(a.out/(tag+'.json')).write_bytes(raw)
        hashes[tag]=hashlib.sha256(raw).hexdigest();report.append(s);traces[tag]=r
        print(json.dumps({'id':tag,'pass':bool(passed),'max_pitch':s['max_pitch_error_deg'],'stop_speed':s['final_speed_m_s'],
                          'plate21':s['CAD_shell_contact_fraction_sampled'].get('cad_c21',0.),'casters':s['upper_caster_contact_fraction_sampled']}),flush=True)
    m,meta,xml=models['nominal'];repeat=simulate(m,meta,fit,profiles['nominal'],zero,'hold',6.,GAINS,True,PEAK)
    deterministic=repeat['frames']==traces['hold-nominal']['frames']
    if not deterministic:raise RuntimeError('Nominal hold is not repeatable')
    bench=fixture_bench(xml,profiles['nominal'],fit,zero);(a.out/'fixed-shell-bench.json').write_text(json.dumps(bench,separators=(',',':'))+'\n')
    bench_off=fixture_bench(xml,profiles['nominal'],fit,zero,enabled=False);(a.out/'fixed-shell-OFF.json').write_text(json.dumps(bench_off,separators=(',',':'))+'\n')
    receipt={'schema':'alfred-bb8-IMU-battery/v3','hardware_calibrated':False,'gains_frozen':GAINS,'peak_torque_Nm_ESTIMATED':PEAK,
             'control_rate_Hz':100,'sensor_latency_s':.01,'nominal_only_tuning_then_frozen':True,
             'controlled_cases':9,'controlled_behavior_passed':sum(int(s['behavior_pass']) for s in report if s['included_in_controlled_behavior_count']),
             'off_control_comparison_cases':1,'integration_steps':74000,'exact_hold_repeat':deterministic,
             'fixture_only_pass':bench['passed_fixture_only'],'fixture_only_max_pitch_error_deg':bench['max_pitch_error_deg'],
             'fixture_off_control_max_pitch_error_deg':bench_off['max_pitch_error_deg'],
             'mass_range_without_head_kg':{p:v['total_mass_without_head_kg'] for p,v in profiles.items()},
             'criteria':{'max_pitch_error_deg':10,'max_full_tilt_deg':15,'max_final_speed_m_s':.01,'max_contact_penetration_m':.0005,'minimum_each_wheel_contact_fraction':.8},
             'trace_sha256':hashes,'xml_sha256':{p:hashlib.sha256(t[2].encode()).hexdigest() for p,t in models.items()},'cases':report,
             'limitations':['Estimated components; no measured HWeigo motor/driver curve or actual mass verification',
                            'Head and magnetic load absent; fixed-shell pass does not validate rolling BB8',
                            'Upper caster contacts and servo-plate interference must be reconciled with physical assembly',
                            'Failed counterfactuals/slow analytic-SDF probe retained as research evidence, not successful behavior']}
    (a.out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');print('RECEIPT',json.dumps({k:v for k,v in receipt.items() if k not in ['cases','trace_sha256','xml_sha256']}),flush=True)

if __name__=='__main__':main()
