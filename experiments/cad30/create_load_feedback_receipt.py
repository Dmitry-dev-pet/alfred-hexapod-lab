"""Compare controller variants using fixed, predeclared load-gate parameters."""
import json
from pathlib import Path
from statistics import mean
from load_feedback_ablation import run,MODES,compact,MAX_SELF_LOAD_MULTIPLIER,MIN_NEIGHBOR_LOAD_MULTIPLIER,LOAD_FILTER_TAU_S
ROOT=Path(__file__).resolve().parent
SCENARIOS=((1.1,(11,23,37,51,97)),(0.6,(11,23,37)),(0.3,(11,23)))
runs=[]
for mu,seeds in SCENARIOS:
 for seed in seeds:
  for mode in MODES:
   r=run(mode,seed,friction=mu,write=True)
   c=compact(r)
   c['friction_mu']=mu
   c['full_receipt_file']=f'load_feedback_{mode}_seed{seed}_nominal_mu{str(mu).replace(".","p")}.json'
   runs.append(c)
   print(json.dumps({k:c[k] for k in ('friction_mu','seed','mode','root_advance_cm','min_grounded_legs','max_tilt_deg','lift_count','liftoff_block_reasons')}),flush=True)
summary={}
for mu,seeds in SCENARIOS:
 sub={}
 for mode in MODES:
  rs=[x for x in runs if x['mode']==mode and x['friction_mu']==mu]
  sub[mode]={
   'mean_advance_cm':round(mean(x['root_advance_cm'] for x in rs),4),
   'mean_peak_tilt_deg':round(mean(x['max_tilt_deg'] for x in rs),4),
   'mean_fraction_below_3':round(mean(x['fraction_below_3_contacts'] for x in rs),5),
   'num_runs_with_below_3':sum(x['min_grounded_legs']<3 for x in rs),
   'min_grounded_legs':min(x['min_grounded_legs'] for x in rs),
   'mean_saturation_fraction':round(mean(x['saturation_fraction'] for x in rs),5),
   'total_lift':sum(x['lift_count'] for x in rs),
   'all_completed':all(x['finished_full_duration'] for x in rs),
  }
 summary[str(mu)]={'seeds':list(seeds),'modes':sub}
receipt={
 'schema':'fnk0052_cad30_load_feedback_ablation/1','date_utc':'2026-11-03',
 'physics':'Local MuJoCo 3.14.0, CAD30 (30 of 32 original meshes, six tibia contact hulls), free base and torque-limited PD',
 'scenario':'Nominal 1.804868kg; 8s motion after 1s stabilization; dt 0.002s; 50Hz diagnostics',
 'seed_policy':'Preselected five seeds from earlier decentralized experiment; same randomized initial conditions across variants',
 'mass_estimates_are_measured':False,'motor_model_is_calibrated':False,
 'force_sensor':'MuJoCo mj_contactForce normal forces on tibia-floor contacts; no physical load cell, no noise, EMA filter',
 'gate':{'max_self_ratio_to_ideal_leg_load':MAX_SELF_LOAD_MULTIPLIER,'min_each_neighbor_ratio_to_ideal_leg_load':MIN_NEIGHBOR_LOAD_MULTIPLIER,'ema_tau_s':LOAD_FILTER_TAU_S},
 'rows':runs,'aggregates':summary,
 'warning':'Evaluate both negative and positive outcomes; do not infer real robot transfer from model',
}
out=ROOT/'results'/'load_feedback_ablation_summary_2026-11-03.json'
out.write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
print('SUMMARY '+json.dumps(summary,ensure_ascii=False),flush=True)
