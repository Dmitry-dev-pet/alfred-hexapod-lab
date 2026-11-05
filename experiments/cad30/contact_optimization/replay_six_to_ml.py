"""Independent continuous MuJoCo episode: saved six stance -> four -> optimized ML step.

Validates one middle leg step, NOT one entire 4-leg crawl, and not net travel.
First phase replays previously recorded math-only joint targets from a real
saved six-grounded state. Then optimized local stage is applied without any
physical-state reset, root teleport, fictitious frames or hardware commands.
"""
from __future__ import annotations
import argparse
import gzip
import json
from pathlib import Path
import numpy as np
import mujoco as mj

from contact_aware_shooting import (PhysicalTrial,Command,LEG_CODES,WALKING,
                                    FULL_MODEL_DT,smoothstep,sha256)


def replay_six_to_contact(trial:PhysicalTrial, six_state:Path, through_node:int=117,
                          node_s:float=.3,keep_frames:bool=False):
    m,d,floor,geom_map,ja,va,_,legs=trial.make()
    six=json.loads(Path(six_state).read_text())
    mj.mj_resetData(m,d)
    d.qpos[:]=six['final_qpos'];d.qvel[:]=six['final_qvel']
    commands=np.asarray(six['final_goals'],float)
    d.ctrl[:]=np.clip(7*(commands-d.qpos[ja])-.32*d.qvel[va],m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
    mj.mj_forward(m,d)
    original_root_y=float(d.qpos[1]);original_com_y=float(trial.model_com(m,d)[1])
    min_loaded=6;max_q=0.;max_raw=0.;peak_tau=0.;min_full_margin=10000.;first_failure=None
    frames=[];frame_count=0
    def drive(target,seconds,name,i):
        nonlocal commands,min_loaded,max_q,max_raw,peak_tau,first_failure,frame_count,min_full_margin
        prev=commands.copy();steps=round(seconds/FULL_MODEL_DT)
        for tick in range(steps):
            g=prev+smoothstep((tick+1)/steps)*(target-prev)
            raw=7*(g-d.qpos[ja])-.32*d.qvel[va]
            u=np.clip(raw,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1]);d.ctrl[:]=u;mj.mj_step(m,d)
            fz,xy,_=trial.read_contact(m,d,floor,geom_map)
            loaded=[k for k in LEG_CODES if fz[k]>.08]
            min_loaded=min(min_loaded,len(loaded));max_q=max(max_q,float(max(abs(d.qpos[ja[:18]]))))
            max_raw=max(max_raw,float(max(abs(raw[:18]))));peak_tau=max(peak_tau,float(max(abs(u[:18]))))
            frame_count+=1
            if first_failure is None:
                if len(loaded)<3:first_failure={'node':i,'phase':name,'reason':'less_than_three_loaded','time_s':float(d.time),'loaded':loaded}
                elif max_raw>.7:first_failure={'node':i,'phase':name,'reason':'torque_limit','time_s':float(d.time)}
                elif max_q>1.7:first_failure={'node':i,'phase':name,'reason':'angle_limit','time_s':float(d.time)}
            if keep_frames or frame_count%25==0:
                frames.append({'phase':name,'node':i,'time_s':float(d.time),'loaded':loaded,'Fz_N':fz,
                               'COM_xyz_m':trial.model_com(m,d).tolist(),
                               'qpos':d.qpos.tolist() if keep_frames else None,
                               'qvel':d.qvel.tolist() if keep_frames else None})
        commands=target.copy()
    drive(commands,.25,'six_initial_hold',0)
    for i in range(1,through_node+1):
        node=trial.math_nodes[i];goal=np.asarray(node['qpos'],float)[ja]
        duration=max(node_s,.3) if node['phase'].startswith(('six_support_touchdown_','touchdown_')) else node_s
        drive(goal,duration,node['phase'],i)
        if first_failure:break
    fz,xy,_=trial.read_contact(m,d,floor,geom_map)
    receipt={'status':'PASS_TO_PRE_ML' if not first_failure and i==through_node else 'FAIL',
             'node_last':i,'requested_last':through_node,'sim_time_s':float(d.time),
             'physics_steps':frame_count,'minimum_loaded_leg_count':min_loaded,'peak_leg_tau_Nm':peak_tau,
             'max_raw_leg_tau_Nm':max_raw,'max_leg_joint_rad':max_q,'first_failure':first_failure,
             'final_Fz_N':fz,'root_y_delta_mm_from_saved_six':float((d.qpos[1]-original_root_y)*1000),
             'COM_y_delta_mm_from_saved_six':float((trial.model_com(m,d)[1]-original_com_y)*1000),
             'final_qpos':d.qpos.tolist(),'final_qvel':d.qvel.tolist(),
             'measured_checkpoint_model':'original CAD30 and saved six-grounded physical Pupper baseline',
             'frames':frames if keep_frames else None}
    return (m,d),receipt


def main():
    a=argparse.ArgumentParser()
    a.add_argument('--cad-root',type=Path,required=True)
    a.add_argument('--repo-root',type=Path,default=Path(__file__).resolve().parents[3])
    a.add_argument('--six-state',type=Path,required=True)
    a.add_argument('--math-trace',type=Path,required=True)
    a.add_argument('--physical-trace',type=Path,required=True)
    a.add_argument('--optimized-summary',type=Path,required=True)
    a.add_argument('--mu',type=float,default=1.1)
    a.add_argument('--out',type=Path,required=True)
    args=a.parse_args()
    trial=PhysicalTrial(args.cad_root,args.repo_root,args.math_trace,args.physical_trace,mu=args.mu,full_log=True)
    (m,d),prelude=replay_six_to_contact(trial,args.six_state,keep_frames=True)
    frames=prelude.pop('frames') or []
    if prelude['status']=='PASS_TO_PRE_ML':
        aparam=json.loads(args.optimized_summary.read_text())['best']['command']
        after=trial.rollout(Command(**aparam),live_model_data=(m,d))
        after_frames=after.pop('_raw_log',[])
    else:after={'status':'PRELUDE_FAILED','accepted':False};after_frames=[]
    args.out.parent.mkdir(parents=True,exist_ok=True)
    raw_path=args.out.with_name(args.out.stem+'_complete_2ms.json.gz')
    with gzip.open(raw_path,'wt') as f:json.dump({'schema':'cad30-continuous-from-six-to-optimized-ml-2ms/1',
             'prefix':frames,'optimized_segment':after_frames},f)
    out={'schema':'cad30-six-to-ML-optimized-physics/1','prelude':prelude,'optimized_middle_swing':after,
         'total_status':'ONE_MIDDLE_SWING_FROM_SAVED_SIX_PASS_NOT_FULL_GAIT' if after.get('accepted') else 'FAIL',
         'raw_trace_sha256':sha256(raw_path),
         'input_sources':{'math_trace_sha256':sha256(args.math_trace),'physics_checkpoint_trace_sha256':sha256(args.physical_trace),
                          'saved_six_state_sha256':sha256(args.six_state)},
         'limits_hardware_calibrated':False,'full_four_step_gait':False,'positive_net_from_original_six_stance':False}
    args.out.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'status':out['total_status'],'prelude':{k:v for k,v in prelude.items() if k not in ('final_qpos','final_qvel')},
          'optimized':{k:v for k,v in after.items() if k not in ('_raw_log',)},'sha':out['raw_trace_sha256']},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
