"""Create a small source-traceable summary of local CAD30 feedback experiments."""
from __future__ import annotations
from pathlib import Path
import json,hashlib,sys
from measure_stance_slip import inspect

ROOT=Path(__file__).resolve().parent
RES=ROOT/'results'


def sha256(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def load(name):return json.loads((RES/name).read_text())

def summary_one(doc):
    trace=doc['trace']
    bad=sum(x['n_contacts']<3 for x in trace)
    fewer5=sum(x['n_contacts']<5 for x in trace)
    x={
      'advance_cm':round(doc['advance_m']*100,3),
      'simulated_s':doc['duration_s'],
      'completed_without_fall':doc['finished_full_duration'] and not doc['fall_or_nonfinite'],
      'min_grounded_legs':doc['min_grounded_legs'],
      'fraction_below_3_contacts':round(bad/len(trace),5),
      'fraction_below_5_contacts':round(fewer5/len(trace),5),
      'mean_grounded_legs':doc['mean_grounded_legs'],
      'peak_torque_nm':doc['max_abs_torque_nm'],
      'saturation_fraction':doc['saturation_fraction'],
      'max_tilt_deg':doc.get('max_tilt_deg',doc.get('max_body_tilt_deg')),
      'stand_stance_tip_proxy_slip':inspect(doc),
    }
    if doc['gait'].startswith('contact_gated'):
        x.update({'completed_touchdowns':doc['touchdowns'],
                  'liftoff_events':doc['liftoffs'],
                  'touchdown_timeout_steps':doc['missed_touchdown_steps'],
                  'foot_extension_m':doc['emergency_foot_extension_m']})
    return x


def main():
    mvals={'lower':1.644,'nominal':1.804868,'upper':2.080}
    rc={}
    matrix=[]
    for mu in (1.1,.6,.3):
        suffix='' if mu==1.1 else '_mu'+str(mu).replace('.','p')
        one={'friction_mu_assumed':mu,'cases':{}}
        for gait in ('tripod','wave','contact'):
            name=f'cad30_gait_{gait}_nominal{suffix}.json'
            d=load(name)
            one['cases'][gait]=summary_one(d)
            rc[name]=sha256(RES/name)
        matrix.append(one)
    masses=[]
    for case,mass in mvals.items():
        name=f'cad30_gait_contact_{case}.json'
        d=load(name)
        masses.append({'mass_case':case,'mass_kg':mass,'metrics':summary_one(d)})
        rc[name]=sha256(RES/name)
    checks=[]
    for name in ('cad30_gait_contact_nominal_no_landing_target.json',
                 'cad30_gait_contact_nominal_extension0mm.json',
                 'cad30_gait_contact_nominal.json'):
        d=load(name)
        checks.append({'receipt_file':name,'advance_cm':round(100*d['advance_m'],3),
                       'touchdowns':d['touchdowns'],'timeout_sim_steps':d['missed_touchdown_steps'],
                       'explicit_landing_target_enabled':d.get('explicit_landing_ik_enabled',True),
                       'extra_extension_mm':round(d['emergency_foot_extension_m']*1000,2)})
        rc[name]=sha256(RES/name)
    dwell=[]
    for mu in (1.1,.6,.3):
        name='cad30_gait_contact_nominal'+('' if mu==1.1 else '_mu'+str(mu).replace('.','p'))+'_dwell100ms.json'
        d=load(name)
        dwell.append({'friction_mu':mu,'advance_cm':round(100*d['advance_m'],3),
                      'min_contacts':d['min_grounded_legs'],
                      'mean_contacts':d['mean_grounded_legs']})
        rc[name]=sha256(RES/name)
    report={
       'schema_version':'fnk0052-cad30-feedback-comparison/1',
       'date_utc':'2026-11-03',
       'environment':'OpenAI in-session Python 3.13 + local MuJoCo 3.14.0; no GitHub Actions used for these experiments',
       'source':'MattiaGrigoli/autonomous_hexapod',
       'upstream_mjcf_commit':'5c9a84c16335b30036e451c691d2c544abf4bc72',
       'cad_geometry_verified_STL_count':30,
       'cad_geometry_missing_visual_only':['topplate.stl','bigservo1.stl'],
       'real_tibia_ground_collision_mesh_count':6,
       'mass_scenarios_kg':mvals,
       'physics_assumptions':{'gravity_ms2':9.81,'PD_Kp':7.,'PD_Kd':.32,
           'leg_torque_limit_nm_assumed':.7,'planned_step_m':.035,
           'planned_foot_lift_m':.02,'planned_period_s':2.4,
           'pre_stand_s':1.,'motion_s':8.,
           'ground_contact_mu_varied':[1.1,.6,.3]},
       'nominal_friction_sensitivity':matrix,
       'reactive_controller_mass_sensitivity':masses,
       'reactive_landing_target_ablation':checks,
       'reactive_touchdown_dwell_100ms_ablation':dwell,
       'decision':'Touchdown IK target is essential to prevent hanging swing; 6mm extra extension is secondary. Under mu=.6 reactive reduces time with <3 contacts versus tripod, but no controller avoids all reduced-ground-support events at mu=.3. Do not claim robust insect locomotion.',
       'classification':'Contact-gated heuristic with independent leg stance clocks and neighbor inhibition/excitation plus global one-leg safety cap. NOT validated Walknet or neuroWalknet.',
       'literature':['https://pmc.ncbi.nlm.nih.gov/articles/PMC7205325/',
                     'https://pmc.ncbi.nlm.nih.gov/articles/PMC3755227/'],
       'limitations':[
          'Foot-contact support is measured in MuJoCo, never on a real robot.',
          'Tripod and wave are open-loop, contact version uses event-driven local clocks but retains global cap.',
          'At low friction touchdown sometimes coincides with all six feet momentarily off the floor.',
          'Visual 2/32 missing meshes have no contact, but no servo mass/geometry was measured.',
          'Tip proxy is a farthest CAD mesh vertex, and its movement can include rotation/rolling, not just surface slip.',
          'Torque-speed and power delivery, servo backlash, electronics, friction and real PWM joint calibration unknown.',
          'Numerical angle bound 1.7 rad has not been validated as a hardware travel limit.',
          '8 simulated seconds per experiment on flat floor is not a comprehensive stability test.'
       ],
       'receipt_sha256_by_filename':rc,
       'related_source_files':{
           name:sha256(ROOT/name)
           for name in ['local_contact_gait.py','locomotion_cad30.py','measure_stance_slip.py','tests/test_contact_feedback.py']
       },
       'drive_original_cad_bundle':'https://drive.google.com/file/d/1qN8KAgekhXm5RasMXSHNxHAsMxivpZ44/view?usp=drivesdk',
    }
    p=RES/'contact_feedback_cad30_comparison_2026-11-03.json'
    p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'file':str(p),'cases':len(rc),'bytes':p.stat().st_size,
         'three_friction_comparisons':len(matrix),'three_mass_cases':len(masses)},ensure_ascii=False,indent=2))

if __name__=='__main__':main()