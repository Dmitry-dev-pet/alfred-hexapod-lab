#!/usr/bin/env python3
"""Independent per-2ms contact force audit of a six-leg CAD30 wave simulation."""
import argparse,json,math,os,sys
from pathlib import Path
from collections import Counter
import numpy as np
import mujoco as mj
CADROOT=Path(os.environ.get("CAD30_ROOT",str(Path.cwd()))).resolve()
sys.path.insert(0,str(CADROOT))
from locomotion_cad30 import gait_run
LEG=('ml','bl','br','mr','fr','fl')
original_step=mj.mj_step
state={'steps':0,'min_contact':6,'min_fz':6,'max_fz':0.,'consecutive':0,'max_consecutive':0,'contacts':Counter(),'forces':Counter(),'force_samples':[]}

def observe(model,data):
    original_step(model,data)
    if data.time <= 1.000001:return
    s=state;s['steps']+=1
    floor=int(model.geom('floor').id)
    leg_geom={int(model.geom(f'tibia{k}_collision_0').id):k for k in LEG}
    fz={k:0. for k in LEG}; in_contact=set()
    for ci in range(data.ncon):
        c=data.contact[ci];g1,g2=int(c.geom1),int(c.geom2)
        geom=g2 if g1==floor else g1 if g2==floor else -1
        if geom not in leg_geom:continue
        k=leg_geom[geom];in_contact.add(k)
        force=np.zeros(6);mj.mj_contactForce(model,data,ci,force)
        fz[k]+=max(0.,float(force[0]))
    c_num=len(in_contact);f_num=sum(x>=.2 for x in fz.values())
    s['contacts'][c_num]+=1;s['forces'][f_num]+=1
    s['min_contact']=min(s['min_contact'],c_num);s['min_fz']=min(s['min_fz'],f_num)
    s['max_fz']=max(s['max_fz'],max(fz.values()))
    if f_num<5:s['consecutive']+=1
    else:s['max_consecutive']=max(s['max_consecutive'],s['consecutive']);s['consecutive']=0
    if s['steps']%10==0:
        s['force_samples'].append({'t':round(float(data.time-1.),4),'fz_N':[round(fz[k],4) for k in LEG],'mask':sum((1<<i) for i,k in enumerate(LEG) if fz[k]>=.2),'n':f_num})

def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=60.);p.add_argument('--step-mm',type=float,default=20.);p.add_argument('--lift-mm',type=float,default=12.);p.add_argument('--period',type=float,default=3.0);a=p.parse_args()
    mj.mj_step=observe
    try:r=gait_run('wave','nominal',a.step_mm/1000,a.lift_mm/1000,a.period,a.seconds,frames_hz=25.)
    finally:mj.mj_step=original_step
    s=state;s['max_consecutive']=max(s['max_consecutive'],s['consecutive'])
    if s['steps'] != round(a.seconds/.002):raise RuntimeError(f"Force steps {s['steps']} not equal {a.seconds/.002}")
    meta={'schema':'cad30-wave-per-2ms-contacts/v1','model':'Freenove CAD30 nominal MuJoCo 3.14',
          'parameters':{'wave_step_mm':a.step_mm,'lift_mm':a.lift_mm,'period_s':a.period,'duration_s':a.seconds,'mu_model':1.1,'dt_s':.002,'Fz_contact_threshold_N':.2},
          'physics_steps':s['steps'],'geometric_contact_histogram':dict(s['contacts']), 'Fz_loaded_histogram':dict(s['forces']),
          'min_geom_contacts_all_2ms':s['min_contact'],'min_Fz_loaded_all_2ms':s['min_fz'],
          'fraction_below_5_loaded':sum(v for k,v in s['forces'].items() if k<5)/s['steps'],
          'fraction_below_3_loaded':sum(v for k,v in s['forces'].items() if k<3)/s['steps'],
          'max_consecutive_below_5_s':s['max_consecutive']*.002,'max_Fz_contact_N':s['max_fz'],
          'end_report':{k:v for k,v in r.items() if k not in ('trace','qpos_samples')},'force_samples_50hz':s['force_samples'],
          'status':'REAL_MUJOCO_DIAGNOSTIC_NOT_FIVE_SUPPORT_CERTIFIED',
          'caveat':'No robust claim of maintaining 5 loaded support legs; contact count from real 2ms physics and Fz >=0.2N.'}
    (CADROOT/'results'/'wave6_60s_fz_analysis.json').write_text(json.dumps(meta,ensure_ascii=False,separators=(',',':'))+'\n')
    print(json.dumps({'simulation_pass':r['finished_full_duration'],'travel_m':r['advance_m'],'steps':s['steps'],'min_Fz_loaded':s['min_fz'],'Fz_hist':dict(s['forces']),'fraction_below_5':meta['fraction_below_5_loaded'],'max_torque_Nm':r['max_abs_torque_nm'],'angle_max_rad':r['IK_max_joint_abs_rad'],'qpos_frames':len(r['qpos_samples'])},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
