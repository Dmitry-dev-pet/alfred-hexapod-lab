"""Fixed-seed A/B without selection; 2ms contact diagnostics; no CI compute."""
from __future__ import annotations
import json,hashlib,statistics
from pathlib import Path
from prelift_force_transfer import run

TRIALS={1.1:(11,23,37,51,97),0.6:(11,23,37),0.3:(11,23)}
MODES=('neighbor_force_gate_supported','neighbor_pre_lift_transfer')
METRICS=('root_advance_cm','min_grounded_legs_2ms','fraction_below_3_2ms','fraction_zero_2ms',
         'max_tilt_deg','peak_tau_nm','saturation_fraction','lift_count','touchdown_count',
         'mean_raw_force_at_liftoff_N','mean_max_adjacent_press_at_liftoff_mm',
         'mean_press_mm_per_leg_over_time','max_active_press_mm','fraction_times_any_prelift_request',
         'finished_full_duration','liftoff_block_reasons')

rows=[]; out=Path(__file__).resolve().parents[2]/'results';out.mkdir(exist_ok=True,parents=True)
for mu,seeds in TRIALS.items():
 for seed in seeds:
  pair=[]
  for mode in MODES:
   v=run(mode,seed,friction=mu,seconds=8.,press_mm=4.,press_rate_mm_s=20.,write=True)
   d={'mode':mode,'mu':mu,'seed':seed,**{k:v[k] for k in METRICS}}
   pair.append(d);rows.append(d)
   print(json.dumps(d,ensure_ascii=False),flush=True)
  assert all(d['finished_full_duration'] for d in pair),pair
summaries={}
for mu,seeds in TRIALS.items():
 summaries[str(mu)]={}
 for mode in MODES:
  s=[r for r in rows if r['mu']==mu and r['mode']==mode]
  summaries[str(mu)][mode]={'n':len(s),
      'mean_advance_cm':round(statistics.mean(x['root_advance_cm'] for x in s),4),
      'mean_fraction_below3_2ms':round(statistics.mean(x['fraction_below_3_2ms'] for x in s),6),
      'runs_with_below3_2ms':sum(x['min_grounded_legs_2ms']<3 for x in s),
      'runs_with_zero_contacts':sum(x['fraction_zero_2ms']>0 for x in s),
      'max_tilt_mean_deg':round(statistics.mean(x['max_tilt_deg'] for x in s),4),
      'mean_peak_tau_nm':round(statistics.mean(x['peak_tau_nm'] for x in s),4),
      'mean_liftoffs':round(statistics.mean(x['lift_count'] for x in s),3),
      'mean_force_at_lift_N':round(statistics.mean(x['mean_raw_force_at_liftoff_N'] for x in s),4),
      'mean_press_at_lift_mm':round(statistics.mean(x['mean_max_adjacent_press_at_liftoff_mm'] for x in s),4)}
raw=Path(__file__).with_name('prelift_force_transfer.py')
report={
  'schema':'fnk0052_CAD30_pre_lift_force_transfer_A_B/1',
  'date_utc':'2026-11-04',
  'hypothesis':'Use own loading signal and adjacent support preloading BEFORE local liftoff to reduce transient loss of support.',
  'protocol_predefined_parameters':{'mass_kg_nominal':1.804868,'torque_cap_Nm_assumed':0.7,
     'stance_period_s':2.4,'swing_fraction':0.18,'step_m':0.035,'lift_m':0.020,
     'force_filter_tau_ms':50,'target_unload_N':1.50,'liftoff_threshold_N_filtered':1.65,
     'prepare_stance_progress':0.90,'max_press_mm':4,'max_press_rate_mm_s':20,
     'paired_seed_plan':{str(k):list(v) for k,v in TRIALS.items()},'physics_dt_ms':2,'sampled_trace_hz':50},
  'implementation_sha256':hashlib.sha256(raw.read_bytes()).hexdigest(),
  'model':'CAD30 30 verified Freenove STL, 2 visual-only proxies, six convex hull tibia colliders, not hardware calibrated',
  'runs':rows,'metrics':summaries,
  'limitations':['Neither physics parameters nor force sensing calibrated to physical robot',
    'Position bias acts through joints and not true force controlled actuator',
    '0.7 N-m torque cap and gear speed/thermal behavior assumed',
    'Sensor feedback ideal except EMA; real latency/noise/footpads absent',
    'Proxy fall threshold not proof of actual stability',
    'No data-driven parameter tuning or learned controller',
    'Existing CAD30 assumptions reused; sample sizes exploratory']}
p=out/'prelift_force_transfer_paired_2026-11-04.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('FINAL',json.dumps(summaries,ensure_ascii=False,indent=2),flush=True)