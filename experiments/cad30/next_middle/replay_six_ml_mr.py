#!/usr/bin/env python3
"""Independent chain original saved six stance -> real ML -> exploratory MR.
All actions operate on SAME MuJoCo MjData; no root qpos teleport or reset after startup.
MR is not a qualified crawl step if ANY required support is unloaded.
"""
from __future__ import annotations
import json, gzip, argparse, sys, hashlib
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from mr_contact_continuation import Trial

def run(cad,repo,six,math,physics,optimized,out,raw=False,touchdown_feedback=False):
    sys.path.insert(0,str(repo/'experiments/cad30/contact_optimization'))
    from contact_aware_shooting import PhysicalTrial,Command
    from replay_six_to_ml import replay_six_to_contact
    outer=PhysicalTrial(cad,repo,math,physics,mu=1.1,full_log=False)
    (m,d),prelude=replay_six_to_contact(outer,six,through_node=117,node_s=.3,keep_frames=False)
    if prelude['status']!='PASS_TO_PRE_ML':raise RuntimeError('Prelude unsuccessful')
    base_com_y=float(outer.model_com(m,d)[1])-prelude['COM_y_delta_mm_from_saved_six']/1000
    base_root_y=float(d.qpos[1])-prelude['root_y_delta_mm_from_saved_six']/1000
    cmd=Command(**json.loads(optimized.read_text())['best']['command'])
    ml=outer.rollout(cmd,live_model_data=(m,d));assert ml['accepted'],ml
    ml_com_y=float(outer.model_com(m,d)[1]);ml_root_y=float(d.qpos[1])
    nexttrial=Trial(cad,repo,HERE/'results/after_ml_physical_checkpoint_20261105.json',mu=1.1,leg='mr')
    next_result,frames=nexttrial.run(step_mm=33,clearance_mm=18,swing_node_s=.35,
            pre_shift_xy_mm=(0.,20.),ml_servo_bias_rad=(0.,-.075,.055),hold_s=.6,
            live_model_data=(m,d,outer.last_motor_goals),save_raw=raw,
            touchdown_feedback=touchdown_feedback)
    result={'schema':'cad30-real-six-to-ml-and-mr-contiguous-v1','model_dt_s':.002,
      'prelude_status':prelude['status'],'prelude_physics_steps':prelude['physics_steps'],
      'ml_status':ml['status'],'ml_actual_world_forward_mm':ml['actual_ml_world_forward_mm'],
      'mr_status':next_result['status'],'mr_touchdown_feedback_enabled':touchdown_feedback,'mr_accepted_strict':next_result['accepted'],
      'mr_physically_air_s':next_result['zero_contact_tibia_total_s'],
      'mr_mesh_air_s':next_result['mesh_above_2mm_no_contact_sampled_s'],
      'mr_actual_toe_world_forward_mm':next_result['physical_toe_forward_mm'],
      'mr_force_loss_frames':next_result['trio_bad_force_frames'],
      'mr_longest_loss_frames':next_result['trio_max_contiguous_bad_frames'],
      'mr_peak_raw_leg_torque_Nm':next_result['max_raw_tau_Nm'],
      'mr_max_leg_q_rad':next_result['max_leg_joint_rad'],
      'mr_end_four_loaded_fraction':next_result['last_hold_fraction_four_loaded'],
      'COM_Y_mm_after_ml_from_six':(ml_com_y-base_com_y)*1000,
      'COM_Y_mm_after_mr_from_six':(float(outer.model_com(m,d)[1])-base_com_y)*1000,
      'root_Y_mm_after_ml_from_six':(ml_root_y-base_root_y)*1000,
      'root_Y_mm_after_mr_from_six':(float(d.qpos[1])-base_root_y)*1000,
      'no_root_set_or_reset_between_stages':True,
      'not_validated_full_gait':True,
      'mr_details':next_result,'warnings':['Any 2ms Fz loss invalidates original strict >=3-loaded gait criterion',
       'Model motor and contact parameters provisional; no physical Freenove experiment']}
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    if raw:
        gz=out.with_name(out.stem+'_MR_2ms.json.gz')
        with gzip.open(gz,'wt') as f:json.dump({'schema':'cad30-contiguous-simulation-mr-2ms',
          'source':'continuous from original six stance via ML', 'frames':frames},f)
        result['mr_full_trace_sha256']=hashlib.sha256(gz.read_bytes()).hexdigest()
        out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    return result

def main():
 p=argparse.ArgumentParser()
 p.add_argument('--cad-root',type=Path,required=True);p.add_argument('--repo-root',type=Path,required=True)
 p.add_argument('--six-state',type=Path,required=True);p.add_argument('--math-trace',type=Path,required=True)
 p.add_argument('--physical-trace',type=Path,required=True);p.add_argument('--optimized-ml',type=Path,required=True)
 p.add_argument('--out',type=Path,required=True);p.add_argument('--raw',action='store_true');p.add_argument('--touchdown-feedback',action='store_true')
 a=p.parse_args()
 res=run(a.cad_root,a.repo_root,a.six_state,a.math_trace,a.physical_trace,a.optimized_ml,a.out,raw=a.raw,touchdown_feedback=a.touchdown_feedback)
 print(json.dumps({k:v for k,v in res.items() if k!='mr_details'},indent=2))
if __name__=='__main__':main()
