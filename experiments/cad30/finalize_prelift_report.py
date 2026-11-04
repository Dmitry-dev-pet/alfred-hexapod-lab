"""Reaggregate completed local runs; never silently rerun or select conditions."""
from pathlib import Path
import hashlib,json,statistics as st,zipfile

ROOT=Path(__file__).resolve().parents[2]; RESULTS=ROOT/'results'
TRIALS={'1.1':(11,23,37,51,97),'0.6':(11,23,37),'0.3':(11,23)}
MODES=('neighbor_force_gate_supported','neighbor_pre_lift_transfer')
import os
BASELINEZIP=Path(os.environ.get('BASELINE_ACTIVE_ZIP',ROOT/'FNK0052_active_local_support_2026-11-03.zip'))
METRICS=('root_advance_cm','min_grounded_legs_2ms','fraction_below_3_2ms',
         'fraction_zero_2ms','max_tilt_deg','peak_tau_nm','saturation_fraction',
         'lift_count','touchdown_count','mean_raw_force_at_liftoff_N',
         'mean_max_adjacent_press_at_liftoff_mm','mean_press_mm_per_leg_over_time',
         'max_active_press_mm','fraction_times_any_prelift_request',
         'finished_full_duration','liftoff_block_reasons')
rows=[]; full=[];baselinematches=[]
for m,seeds in TRIALS.items():
 for seed in seeds:
  for mode in MODES:
   fp=RESULTS/f'active_support_{mode}_seed{seed}_nominal_mu{m.replace(".","p")}_press4mm.json'
   assert fp.exists(),f'Missing result: {fp}'
   rec=json.loads(fp.read_text()); full.append(fp)
   assert rec['finished_full_duration'] and rec['duration_s']==8.0, (m,seed,mode)
   assert rec['friction_mu_assumed']==float(m) and rec['mode']==mode and rec['seed']==seed
   rows.append({'mode':mode,'mu':float(m),'seed':seed,**{k:rec[k] for k in METRICS}})
# Repro check uses the ORIGINAL archived results, not relabeled new simulation.
with zipfile.ZipFile(BASELINEZIP) as z:
 orig=json.loads(z.read('repo/results/active_local_support_paired_summary_2026-11-03.json'))
for x in orig['runs']:
 if x['mode']!=MODES[0]: continue
 rr=next(r for r in rows if r['mode']==MODES[0] and r['mu']==x['friction_mu_assumed'] and r['seed']==x['seed'])
 keys=('root_advance_cm','fraction_below_3_2ms','min_grounded_legs_2ms','max_tilt_deg','peak_tau_nm','lift_count')
 assert all(rr[k]==x[k] for k in keys),('baseline changed',rr,x)
 baselinematches.append({'seed':rr['seed'],'mu':rr['mu'],'fields':list(keys),'match':True})
assert len(baselinematches)==10 and len(full)==20
summaries={}
for mu,seeds in TRIALS.items():
 summaries[mu]={}
 for mode in MODES:
  s=[r for r in rows if r['mu']==float(mu) and r['mode']==mode]
  summaries[mu][mode]={'n':len(s),'mean_advance_cm':round(st.mean(r['root_advance_cm'] for r in s),4),
     'mean_fraction_below3_2ms':round(st.mean(r['fraction_below_3_2ms'] for r in s),6),
     'runs_below3_2ms':sum(r['min_grounded_legs_2ms']<3 for r in s),
     'runs_with_zero':sum(r['fraction_zero_2ms']>0 for r in s),
     'mean_peak_tilt_deg':round(st.mean(r['max_tilt_deg'] for r in s),4),
     'mean_peak_tau_nm':round(st.mean(r['peak_tau_nm'] for r in s),5),
     'mean_liftoffs':round(st.mean(r['lift_count'] for r in s),3),
     'mean_raw_force_at_liftoff_N':round(st.mean(r['mean_raw_force_at_liftoff_N'] for r in s),4),
     'mean_adjacent_command_press_at_liftoff_mm':round(st.mean(r['mean_max_adjacent_press_at_liftoff_mm'] for r in s),4)}
deltas={}
for mu in TRIALS:
 b=summaries[mu][MODES[0]];n=summaries[mu][MODES[1]]
 deltas[mu]={'distance_delta_percent':round(100*(n['mean_advance_cm']/b['mean_advance_cm']-1),2),
             'below3_fraction_difference':round(n['mean_fraction_below3_2ms']-b['mean_fraction_below3_2ms'],6),
             'mean_force_at_lift_delta_N':round(n['mean_raw_force_at_liftoff_N']-b['mean_raw_force_at_liftoff_N'],4)}
script=ROOT/'experiments/cad30/prelift_force_transfer.py'
report={
 'schema':'FNK0052_CAD30_prelift_A_B/1','date_utc':'2026-11-04',
 'experimental_question':'Does local force-targeted neighbor press preceding liftoff reduce <3-support frames during walking without sacrificing useful travel?',
 'preselected_physics':{'mass_case':'nominal','mass_kg_approx':1.804868,'torque_cap_Nm_assumed':0.7,'friction_mu':[1.1,0.6,0.3],
    'initial_seeds_by_mu':{mu:list(s) for mu,s in TRIALS.items()},'body_free':True,'contacts_free':True,
    'seconds_per_run':8,'joint_PD_Kp':7,'joint_PD_Kd':.32,'stride_m':.035,'lift_m':.020,'cycle_s':2.4,
    'swing_fraction':.18,'physics_step_s':.002,'contact_summaries_from_physics_every_s':.002,
    'force_ema_ms':50,'force_preparation_progress':.90,'target_unload_N':1.50,'release_threshold_filtered_N':1.65,
    'neighbor_press_cap_mm':4,'neighbor_press_rate_cap_mm_s':20},
 'old_baseline_reference_archive_sha256':hashlib.sha256(BASELINEZIP.read_bytes()).hexdigest(),
 'baseline_regression':{'matched_conditions':len(baselinematches),'tested_fields':baselinematches[0]['fields'],'all_equal':True},
 'source_sha256':hashlib.sha256(script.read_bytes()).hexdigest(),
 'full_run_sha256':{fp.name:hashlib.sha256(fp.read_bytes()).hexdigest() for fp in full},
 'runs':rows,'groups':summaries,'delta_new_minus_old':deltas,
 'interpretation':['The force before liftoff decreases, but the local pre-lift rule worsens advancement at low friction.',
  'Normal friction support-loss frame fraction decreased from 0.095% to 0.020% on 5 seeds, yet both modes had 2/5 starts with fewer than 3 contacts.',
  'At mu=0.3 both variants reached zero contacts in 2 of 2 starts; fewer unstable frames coincide with almost stopping, not robustness.',
  'The constrained prep phase can starve the gait when it cannot reach 1.65N filtered normal force.'],
 'limitations':['No measured servo torque-speed/heat/current curve','Unknown real rubber foot friction, slip and contact patch',
  'Normal forces are simulated solver measurements; no electronics or measurement delay except 50ms EMA',
  'Local pre-lift uses fixed force target carried over from static experiment; not learned or statistically tuned',
  '3-contact threshold is a proxy only; collinear supports or COM-outside polygon also unstable',
  'Scores over short simulation with small seed samples not a hardware claim']}
out=RESULTS/'prelift_force_transfer_paired_2026-11-04.json'
out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'all_baselines_matched':len(baselinematches),'n_runs':len(rows),'group_metrics':summaries,'deltas':deltas,'output':str(out)},ensure_ascii=False,indent=2))