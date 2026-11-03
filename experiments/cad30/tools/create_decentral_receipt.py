"""Deterministic summary of all local real-physics runs; records negative results."""
from __future__ import annotations
import hashlib,json,statistics
from pathlib import Path

ROOT=next((p for p in Path(__file__).resolve().parents if (p/'decentralized_ablation.py').exists()), None)
if ROOT is None: raise RuntimeError('Cannot locate experiment root')
MODE=('independent','neighbor_inhibit','neighbor_touchdown')
SEEDS=(11,23,37,51,97)

def read(mode,seed,friction=1.1,seconds=8):
    suffix='_20s' if seconds==20 else ''
    path=ROOT/'results'/f'decentral_{mode}_seed{seed}_nominal_mu{str(friction).replace(".","p")}{suffix}.json'
    x=json.loads(path.read_text())
    assert x['mode']==mode and x['seed']==seed and abs(x['duration_s']-seconds)<0.01, (path,x['duration_s'])
    assert x['mass']=='nominal' and not x['centrally_assigned_phases']
    assert x['global_support_or_one_swing_gate'] is False
    assert x['globally_sorted_candidate_list'] is False
    assert x['finished_full_duration']
    assert x['active_leg_count']==6
    return x

summary={'schema':'fnk0052_CAD30_decentralized_synchronization/1',
  'date':'2026-11-03','environment':'in-session Python 3.13 / MuJoCo 3.14.0, no GitHub Actions',
  'model':{'upstream_commit':'5c9a84c16335b30036e451c691d2c544abf4bc72',
           'original_stl_verified':30,'visual_only_missing_stl':['topplate.stl','bigservo1.stl'],
           'actual_tibia_collision_meshes':6,'estimated_robot_mass_kg':1.804868,'physical_servo_verified':False,
           'actuator_pd':{'Kp':7.,'Kd':.32,'torque_cap_nm':.7},
           'stance_duration_nominal_s':1.968,'swing_duration_nominal_s':.432,
           'commanded_step_m':.035,'commanded_lift_m':.020,
           'free_base_no_kinematic_body_motion':True},
  'logic':{
    'independent':'Only own clock and own toe-ground contact; no neighbor signals',
    'neighbor_inhibit':'Same + inhibit when adjacent leg swinging, or both adjacent legs request simultaneously (local fixed priority)',
    'neighbor_touchdown':'Same + brief readiness excitation in two adjacent legs after actual ground touchdown',
    'global_clock_or_phase_order':False,
    'global_swing_count_cap_or_support_polygon_gate':False,
    'single_process_logical_decentralization_only':True,
    'initial_stance_progress_seeded_equal_per_mode':True,
    'evidence_note':'Only inter-leg ordering is emergent from rules and physical body. Per-leg locomotion cycle is preprogrammed.'},
  'flat_ground':{'friction_mu_assumed':1.1,'duration_s':8,'seeds':SEEDS,'runs':{},'aggregate':{}},
  'friction_sensitivity':{'friction_mu_assumed':.6,'duration_s':8,'seeds':[11,37],'runs':{},'aggregate':{}},
  'longer_run':{'friction_mu_assumed':1.1,'duration_s':20,'seed':23,'runs':{}},
  'limitations':[
    'Five initial seeds are exploratory, not a representative statistical sample',
    'Walking is constructed from local stance/swing trajectories and inverse kinematics; no learning',
    'Only logical per-leg decisions are decentralized: all six executed in one Python process',
    'Motor dynamics, friction, foot contact hulls and masses not calibrated to a real FNK0052',
    'Fixed per-leg period can itself produce regular cycles; coordination timing not spontaneous from rest',
    'Spatial neighbor graph and collision-avoidance policy are hand-authored',
    'A short 8/20 second no-fall test does not prove robustness',
  ]}

def compact(x):
    fields=('mode','seed','initial_stance_progress','finished_full_duration','duration_s','root_advance_cm','min_grounded_legs','fraction_below_3_contacts','fraction_0_contacts','max_tilt_deg','peak_tau_nm','saturation_fraction','lift_count','touchdown_count','touchdown_timeouts','active_leg_count','max_parallel_swing','fraction_adjacent_swinging','fraction_two_or_more_swinging','per_leg_liftoffs')
    return {k:x[k] for k in fields}

def aggregate(data):
    fields=('root_advance_cm','fraction_below_3_contacts','max_tilt_deg','peak_tau_nm','saturation_fraction')
    return {k:round(statistics.mean(x[k] for x in data),5) for k in fields} | {
        'min_contacts_across_runs':min(x['min_grounded_legs'] for x in data),
        'runs_with_any_less_than_3_contacts':sum(x['min_grounded_legs']<3 for x in data),
        'all_6_legs_lifted_in_every_run':all(x['active_leg_count']==6 for x in data),
        'runs_completed':sum(x['finished_full_duration'] for x in data),
    }

for mode in MODE:
    nominal=[read(mode,s) for s in SEEDS]
    friction=[read(mode,s,.6) for s in (11,37)]
    summary['flat_ground']['runs'][mode]=[compact(x) for x in nominal]
    summary['flat_ground']['aggregate'][mode]=aggregate(nominal)
    summary['friction_sensitivity']['runs'][mode]=[compact(x) for x in friction]
    summary['friction_sensitivity']['aggregate'][mode]=aggregate(friction)
    summary['longer_run']['runs'][mode]=compact(read(mode,23,1.1,20))
for seed in SEEDS:
    reference=read('independent',seed)['initial_stance_progress']
    for mode in MODE:
        assert read(mode,seed)['initial_stance_progress']==reference
summary['flat_ground']['relative_gain_neighbor_vs_independent_percent']=round(100*(summary['flat_ground']['aggregate']['neighbor_inhibit']['root_advance_cm']/summary['flat_ground']['aggregate']['independent']['root_advance_cm']-1),2)
summary['hypothesis_status']={
 'H1_independent_legs_can_move_common_body':True,
 'H2_neighbor_inhibition_improves_initial_seed_average_distance_and_stance':True,
 'H3_touchdown_excitation_beneficial_beyond_inhibition':False,
 'H4_no_below_three_contacts_every_run':False,
 'generalization_to_hardware':'NOT ESTABLISHED',
}
for key in ('decentralized_ablation.py','experiments/decentralized/create_decentral_receipt.py','experiments/decentralized/plot_liftoff.py','experiments/decentralized/render_comparison.py','experiments/decentralized/test_decentralized_ablation.py'):
 p=ROOT/key
 summary.setdefault('source_sha256',{})[key]=hashlib.sha256(p.read_bytes()).hexdigest()
out=ROOT/'results'/'decentralized_ablation_summary_2026-11-03.json'
out.write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'summary':str(out),'gain_percent':summary['flat_ground']['relative_gain_neighbor_vs_independent_percent'],
 'nominal':summary['flat_ground']['aggregate'],'friction_mu06':summary['friction_sensitivity']['aggregate'],
 'long20':{k:r['root_advance_cm'] for k,r in summary['longer_run']['runs'].items()}},ensure_ascii=False,indent=2))