"""Create compact, reproducible record of a force-transfer diagnostic sweep.

Requires --response files produced by diagnose_force_transfer.py for 1,2,4mm
and --audit file produced by audit_active_support_archive.py.  Does not run
physics by itself or include external meshes.
"""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path


def receipt(paths:dict[int,Path], audit_path:Path) ->dict:
    data={d:json.loads(path.read_text()) for d,path in paths.items()}
    audit=json.loads(audit_path.read_text())
    first=data[min(data)]
    assert set(paths)=={1,2,4}, 'Expected 1,2,4 mm dose study'
    assert all(r['robot_mass_kg']==first['robot_mass_kg'] for r in data.values())
    expected_weight=first['robot_mass_kg']*9.81
    doses=[]
    for depth,report in sorted(data.items()):
        scenarios={x['name']:x for x in report['cases']}
        baseline=scenarios['no_press']
        feedback=scenarios['feedback_neighbors_of_fl']
        others=scenarios['press_neighbors_of_fl']
        for case in report['cases']:
            assert abs(case['tail_mean_total_normal_N']-expected_weight)<.1
            assert case['max_tau_nm']<=report['torque_bound_Nm_assumed']+.00001
            assert case['tail_mean_tilt_deg']<1.0
        assert baseline['min_contacts']==6 and feedback['min_contacts']==6
        doses.append({'command_mm':depth,
            'intact_stand_FL_normal_N':baseline['tail_mean_force_by_leg_N']['fl'],
            'two_neighbor_feedforward_FL_normal_N':others['tail_mean_force_by_leg_N']['fl'],
            'two_neighbor_feedforward_min_contacts':others['min_contacts'],
            'two_neighbor_feedforward_max_tau_Nm':others['max_tau_nm'],
            'local_feedback_FL_force_target_N':1.5,
            'local_feedback_FL_normal_N':feedback['tail_mean_force_by_leg_N']['fl'],
            'local_feedback_mean_neighbor_command_mm':feedback['tail_mean_preload_mm'],
            'local_feedback_min_contacts':feedback['min_contacts'],
            'local_feedback_max_tau_Nm':feedback['max_tau_nm'],
            'local_feedback_tilt_deg':feedback['tail_mean_tilt_deg']})
    assert [x['two_neighbor_feedforward_FL_normal_N'] for x in doses]==sorted(
        [x['two_neighbor_feedforward_FL_normal_N'] for x in doses],reverse=True)
    assert abs(doses[-1]['local_feedback_FL_normal_N']-1.5)<.1
    return {'schema':'fnk0052_force_transfer_mechanism/1',
        'model':'CAD30 FNK0052 nominal, 30 genuine STL + 2 visual proxies',
        'experiment_date_utc':'2026-11-04',
        'mujoco_version':first['mujoco'],
        'mass_kg_assumed':first['robot_mass_kg'],
        'ground_mu_assumed':first['friction_mu_assumed'],
        'torque_cap_Nm_assumed':first['torque_bound_Nm_assumed'],
        'stand_duration_s_after_initial_stabilization':first['duration_s'],
        'CAD_leg_fl_vendor_label':'FR (CAD mirrored, NOT real vendor FL)',
        'initial_conditions_identical_for_all_trials':True,
        'dose_study':doses,
        'archived_dynamic_active_support_audit':audit['groups'],
        'existing_active_support_archive_sha256':audit['sha256'],
        'raw_receipts_sha256':{f'{d}mm':hashlib.sha256(p.read_bytes()).hexdigest() for d,p in paths.items()},
        'interpretation':[
            'The simulated plant mechanically redistributes loads with small local foot-target offsets.',
            'Existing gait heuristic applies commanded offsets rarely and too late; it does NOT track a force setpoint.',
            'This supports a timing/force-regulation mechanism hypothesis but does not prove it during locomotion.',
            'The standalone local feedback trial regulates FL load during 6-leg standing, not after damage.'
        ],
        'warnings':[
            'MuJoCo force reading is simulator-side, not a physical load cell.',
            'No measurements for PWM, servo torque-speed/power, or compliant rubber feet.',
            'CAD toe marker and contact hull differ from verified physical footpad.',
            'Lost contact of one unloaded leg is not necessarily a fall, but must be monitored.',
            'No real robot tests or robust sample-statistical generalization.'
        ]}


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--dir',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    out=receipt({d:a.dir/f'force_response_{d}mm.json' for d in (1,2,4)},
                a.dir/'active_support_existing_audit_2026-11-04.json')
    a.out.write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')
    for x in out['dose_study']:
        print(x)