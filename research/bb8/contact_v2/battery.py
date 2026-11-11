"""Declared baseline suite; retain every failed behavior and counterfactual."""
import argparse,hashlib,json,time
from pathlib import Path
from audit_geometry import load_parts
from experiment import run
import numpy as np

CASES=[('coast',.8,True,5),('pulse',.8,True,5),('brake',.8,True,5),
       ('turn',.8,True,5),('forward',.8,True,5),('pulse',0.,True,5),
       ('pulse',.8,False,5),('pulse',.8,True,4)]

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--tire',type=Path,required=True)
    p.add_argument('--geometry',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--allow-assumed-mass',action='store_true')
    a=p.parse_args()
    if not a.allow_assumed_mass:p.error('Component masses unverified; --allow-assumed-mass is required for diagnostic-only runs')
    a.out.mkdir(parents=True,exist_ok=True);parts,_=load_parts(a.source,a.tire)
    fit=np.array(json.loads(a.geometry.read_text())['fit']['rigid_translation_from_source_pose_mm'])*.001
    summaries=[];hashes={};all_traces={}
    criteria={'max_chassis_pitch_deg':20.,'max_final_speed_m_s':.01,'max_internal_penetration_m':.0005,
              'max_sampled_CAD_intrusion_m':.0005,'minimum_both_wheel_contact_fraction':.8,
              'source_masses_and_motor_calibrated':False}
    for case,mu,contact,level in CASES:
        name=f'{case}-L{level}-mu{mu:g}'+('-no-wheel-contact' if not contact else '')
        try:
            xml,trace=run(case,6.,level,.001,mu,.002,contact,parts,fit,allow_assumed_mass=True)
        except RuntimeError as error:
            failure={'id':name,'behavior_pass':False,'integration_valid':False,'error':str(error)}
            summaries.append(failure);(a.out/(name+'-INVALID.json')).write_text(json.dumps(failure,indent=2)+'\n')
            print(json.dumps(failure),flush=True);continue
        raw=(json.dumps(trace,separators=(',',':'))+'\n').encode()
        (a.out/(name+'.xml')).write_text(xml);(a.out/(name+'.json')).write_bytes(raw)
        s=trace['summary'];s['id']=name
        s['behavior_pass']=(s['max_chassis_pitch_deg']<=20 and s['final_speed_xy_m_s']<=.01
                            and s['max_any_internal_contact_penetration_m']<=.0005
                            and s['max_source_CAD_intrusion_into_wall_m_sampled']<=.0005
                            and min(s['wheel_contact_fraction'].values())>=.8
                            and s['escaped_chassis_steps']==0 and max(s['warning_counts'])==0)
        if case=='forward':s['behavior_pass'] &= s['shell_xy_after_settle_m'][0]>=.03
        hashes[name]={'trace_sha256':hashlib.sha256(raw).hexdigest(),'xml_sha256':trace['model']['xml_sha256']}
        summaries.append(s);all_traces[name]=trace
        print(json.dumps({'id':name,'pass':s['behavior_pass'],'pitch':s['max_chassis_pitch_deg'],'final_speed':s['final_speed_xy_m_s']}),flush=True)
    base=all_traces['pulse-L5-mu0.8'];repeat_xml,repeat=run('pulse',6.,5,.001,.8,.002,True,parts,fit,allow_assumed_mass=True)
    same=base['frames']==repeat['frames'] and base['model']==repeat['model']
    if not same:raise RuntimeError('Pulse repeat is not deterministic')
    comparisons={}
    for name in ['coast-L5-mu0.8','pulse-L5-mu0','pulse-L5-mu0.8-no-wheel-contact','pulse-L4-mu0.8']:
        if name not in all_traces:continue
        other=all_traces[name]
        differences=[float(np.linalg.norm(np.array(x['qpos'][:2])-np.array(y['qpos'][:2]))) for x,y in zip(base['frames'],other['frames'])]
        comparisons[name]={'max_time_aligned_shell_xy_difference_m':max(differences),'final_shell_xy_difference_m':differences[-1],
                           'max_first_1s_difference_m':max(differences[:51]),
                           'interpretation':'Diverging unstable trajectories; not a convergence or hardware validation'}
    report={'schema':'alfred-bb8-contact-battery/v2','engine':'MuJoCo 3.14.0','model_status':'UNCALIBRATED_DIAGNOSTIC',
            'cases':len(summaries),'integrations':len(summaries)+1,'steps':54000,'criteria':criteria,
            'passed_behavior_cases':sum(bool(x['behavior_pass']) for x in summaries),'repeat_frames_and_model_exact':same,
            'trace_hashes':hashes,'comparisons':comparisons,'results':summaries,
            'conclusion':'Mechanical contact baseline only; no reliable behavior or hardware capability established.'}
    (a.out/'battery_receipt.json').write_text(json.dumps(report,indent=2)+'\n')
    print('REPORT',json.dumps({k:v for k,v in report.items() if k not in ['results','trace_hashes']}),flush=True)

if __name__=='__main__':main()
