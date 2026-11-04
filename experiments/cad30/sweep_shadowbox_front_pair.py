"""Fixed, non-post-hoc friction/mass matrix and weight-shift negative controls."""
from __future__ import annotations
import json, argparse, datetime
from pathlib import Path
from shadowbox_front_pair import experiment
MASS=('lower','nominal','upper')
MU=(1.1,0.6,0.3)
SHIFTS=(0.,.02,.03,.04)
PARAM={'lift_m':.14,'guard_forward_m':.05,'jab_forward_m':.09,'shift_m':.04}
FIELDS=('mass_scenario','mass_model_kg','friction_mu_assumed','high_guard_and_air_jabs_contact_valid',
 'high_guard_and_air_jabs_valid','torque_saturation_observed','complete',
 'min_front_mesh_clearance_during_guard_and_jabs_m','min_support_COM_margin_during_guard_and_jabs_m',
 'max_tilt_guard_and_jabs_deg','measured_forward_jab_excursion_m','max_abs_torque_nm',
 'motor_torque_saturation_fraction','max_abs_actual_joint_speed_rad_s','max_abs_command_joint_speed_rad_s',
 'end_six_contact_restore_fraction','failure_reason')

def main():
 cases=[];controls=[]
 for mass in MASS:
  for mu in MU:
   r=experiment(mass=mass,mu=mu,**PARAM)
   cases.append({k:r[k] for k in FIELDS})
   print(mass,mu,'contact',r['high_guard_and_air_jabs_contact_valid'],'strict',r['high_guard_and_air_jabs_valid'],flush=True)
 for shift in SHIFTS:
  r=experiment(lift_m=.14,guard_forward_m=.05,jab_forward_m=.09,shift_m=shift,mass='nominal',mu=1.1)
  controls.append({'shift_m':shift,**{k:r[k] for k in FIELDS}})
  print('control',shift,'contact',r['high_guard_and_air_jabs_contact_valid'],'strict',r['high_guard_and_air_jabs_valid'],flush=True)
 summary={
  'experiment':'FNK0052 high front-pair four-support air-jab, physical MuJoCo CAD30',
  'date':'2026-11-04','model':'pinned Freenove CAD30 upstream commit 5c9a84c; 30 original STL; two visual-only proxies',
  'source':'local self-contained Python 3.13 / MuJoCo 3.14.0, not remote CI',
  'physics_not_calibrated_to_hardware':True,'real_robot_tested':False,
  'motor_speed_modelled':False,'impact_target_present':False,'neural_or_autonomous_control':False,
  'conditions':{'mass_scenarios':list(MASS),'friction_mu':list(MU),'control_shifts_m':list(SHIFTS),
                'programmed_trajectory':PARAM,'duration_s':10.0,'pd_kp':7.,'pd_kd':.32,
                'motor_torque_cap_nm':.7,'imu_noise_modelled':False},
  'rows':cases,'weight_shift_controls':controls,
  'counts':{
   'matrix_conditions':len(cases),
   'contacts_balance_pass':sum(c['high_guard_and_air_jabs_contact_valid'] for c in cases),
   'contacts_balance_no_motor_saturation_pass':sum(c['high_guard_and_air_jabs_valid'] for c in cases),
   'failed_grounding_or_balance':sum(not c['high_guard_and_air_jabs_contact_valid'] for c in cases),
   'support_contact_pass_but_actuator_saturated':sum(c['high_guard_and_air_jabs_contact_valid'] and c['torque_saturation_observed'] for c in cases)},
  'caveats':['This is choreographed joint-space motion, not a learned punch, not autonomous stance adaptation',
              'All strict passes are within the approximate simulated motor torque limit but DO NOT establish real actuator feasibility',
              'No servo speed-torque curve, thermal limitations, backlash, supply sag, friction validation',
              'No object is hit; no striking or impact force has been measured']}
 out=Path(__file__).resolve().parents[2]/'results'/'shadowbox_front_pair_summary_2026-11-04.json'
 if not out.parent.exists():out=Path(__file__).resolve().parent/'results'/'shadowbox_front_pair_summary_2026-11-04.json'
 out.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
 print(out,summary['counts'])

if __name__=='__main__':main()
