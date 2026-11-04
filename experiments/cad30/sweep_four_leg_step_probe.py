"""Preselected, paired four-support step feasibility matrix; no gait success claim."""
from pathlib import Path
import json,hashlib
from probe_four_leg_rear_step import run

HERE=Path(__file__).resolve().parent
OUT=HERE/'results'/'four_leg_step_probe_2026-11-04.json'
SETUPS=[('bl','lower',1.1),('bl','nominal',1.1),('bl','upper',1.1),('bl','nominal',.6),('bl','nominal',.3),('br','nominal',1.1),('ml','nominal',1.1),('mr','nominal',1.1)]
results=[]
for leg,mass,mu in SETUPS:
  r=run(leg,step_m=.010,lift_m=.010,shift=.020,mass=mass,mu=mu)
  if 'fail' in r:
    results.append({'leg':leg,'mass':mass,'mu':mu,'IK_feasible':False,'IK_failure':r})
    continue
  phase=[x for x in r['trace'] if 3.6<=x['t']<5.20]
  raised=[x for x in phase if leg not in x['contacts']]
  before=min(r['trace'],key=lambda x:abs(x['t']-2.9))['stepping_foot_y']
  after=min(r['trace'],key=lambda x:abs(x['t']-6.3))['stepping_foot_y']
  trial={k:v for k,v in r.items() if k not in ('trace','qpos_frames')}
  trial.update({'IK_feasible':True,'swing_frames_50hz':len(raised),
      'swing_three_other_supports_fraction':sum(len(x['contacts'])==3 and not x['front_contacts'] for x in raised)/len(raised) if raised else 0,
      'swing_min_three_contact_margin_m':min((x['margin'] for x in raised if x['margin'] is not None),default=None),
      'foot_y_shift_world_m':round(after-before,5),
      'max_raised_height_world_m':max((x['step_min_z'] for x in raised),default=None),
      'rear_foot_step_qualified':leg in ('bl','br') and len(raised)>0 and all(len(x['contacts'])>=3 and not x['front_contacts'] for x in raised) and r['min_margin']>.004 and r['step_clearance_max']>.005 and r['saturation_frac']==0 and after-before>.008,
  })
  results.append(trial)
report={
 'date':'2026-11-04','experiment':'FNK0052 CAD30 front pair raised + one-step three-support quadrupedal feasibility (NOT continuous four-legged walking)',
 'provenance':'Original Freenove 30-of-32 meshes, CAD MJCF pinned upstream, source private; masses and motor assumptions from v0.4, no physical hardware',
 'physical_simulation':{'engine':'MuJoCo 3.14.0','free_base':True,'timestep_s':.002,'model_torque_limit_Nm':.7,'not_hardware_calibrated':True,
    'normal_ground_is_simulated':True,'front_leg_guard_targets':{'rise_m':.105,'forward_m':.05},
    'initial_support_rear_shift_target_m':.020,'rear_step_target_m':.010,'rear_lift_target_m':.010,'run_duration_s':7.5},
 'result':results,
 'interpretation':[
  'Both rear legs can individually take a short step with the two front legs held in the air, nominal friction, nominal model, and the three other legs grounded throughout swing; this is NOT continuous quadrupedal walking.',
  'Mid-leg lift is not a success with identical controls. At raised front pair, center-of-mass projection lies outside the three-support triangle left by a removed middle leg unless active body/foot repositioning occurs.',
  'The body is shifted backwards during preparation and is NOT shown progressing forwards continuously; no stride-cycle gait or velocity target is validated.',
  'Friction mu=0.3 is an intentional negative condition showing contact/tau failures; do not omit it.',
  'Actual motors lack calibrated speed-torque, supply sag, PWM, backlash, contacts and thermal behavior. No direct hardware transfer claimed.',
  'Next experiment should redistribute load before each leg lift with local neighbor feedback rather than scripted +20mm shift, and show repeated 4-leg locomotion with 2 front legs held raised.'
 ]}
OUT.parent.mkdir(exist_ok=True)
OUT.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('SAVED',OUT)
for x in results:
 print(x['foot'],x['mass'],x['mu'],x.get('rear_foot_step_qualified'),x.get('swing_min_three_contact_margin_m'),x.get('foot_y_shift_world_m'),x.get('saturation_frac'))
