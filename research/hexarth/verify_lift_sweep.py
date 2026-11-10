"""Independent saved-trace audit for the bounded lift experiment."""
import argparse
import json
from pathlib import Path

import numpy as np
import baseline as b


def verify(root):
    receipts=[json.loads((root/x/'lift-receipt.json').read_text()) for x in ('lift-sweep','fine-sweep','convergence')]
    certified=[];diagnostic=[];infeasible=[];total=0
    for receipt,folder in zip(receipts,('lift-sweep','fine-sweep','convergence')):
        directory=root/folder
        assert receipt['mass_kg']==2.1
        assert receipt['rated_torque_nm']==b.TORQUE
        assert receipt['joint_range_rad']==[-b.RANGE,b.RANGE]
        assert b.digest(directory/receipt['model_file'])==receipt['model_sha256']
        for case in receipt['cases']:
            key=folder+'/'+case['case']
            if case['status']=='infeasible_ik':
                assert case['error'].startswith('IK target not reached:')
                assert not (directory/(case['case']+'-trace.json')).exists()
                infeasible.append(key);continue
            path=directory/(case['case']+'-trace.json')
            trace=json.loads(path.read_text())
            replay=json.loads((directory/(case['case']+'-replay.json')).read_text())
            assert b.digest(path)==case['trace_sha256']==replay['source_trace_sha256']
            assert trace['model_sha256']==receipt['model_sha256']==replay['source_model_sha256']
            assert len(trace['frames'])==len(replay['frames'])==case['frame_count']==1200
            last=0.;max_joint=0.
            for frame,pose in zip(trace['frames'],replay['frames']):
                assert abs(frame['t']-last-.02)<1e-8;last=frame['t']
                assert pose==[frame['t']]+frame['qpos'][:7]+frame['body_poses']
                assert np.isfinite(pose).all() and np.isfinite(frame['qvel']).all()
                assert np.isfinite(frame['foot_normal_load_n']).all()
                assert np.max(np.abs(frame['ctrl']))<=b.TORQUE+1e-7
                assert np.max(np.abs(frame['target']))<=b.RANGE+1e-7
                max_joint=max(max_joint,float(np.max(np.abs(frame['qpos'][7:]))))
                assert not frame['nonfoot_ground_contact']
                for c in frame['contacts']:
                    assert np.isfinite(c['normal_world']).all() and np.isfinite(c['normal_force_n'])
            assert max_joint<b.RANGE
            m=case['metrics']
            assert m['max_ik_residual_m']<5e-6
            assert m['peak_self_penetration_m']==0
            assert not m['mujoco_warnings']
            assert case['diagnostic_stable_end']
            if m['peak_external_penetration_m']<.001 and m['peak_joint_speed_rad_s']<b.SPEED:
                certified.append(key)
            else:
                diagnostic.append(key)
            total+=len(trace['frames'])
    a=next(c for c in receipts[1]['cases'] if c['lift_mm']==22)
    z=receipts[2]['cases'][0]
    assert a['whole_robot_on_step'] and z['whole_robot_on_step']
    assert all(a['metrics']['feet_on_step_last_second']) and all(z['metrics']['feet_on_step_last_second'])
    difference=abs(a['metrics']['forward_after_settle_m']-z['metrics']['forward_after_settle_m'])
    assert difference<.001
    assert not any(c['whole_robot_on_step'] for c in receipts[0]['cases'] if c['status']=='simulated')
    assert not receipts[1]['cases'][0]['whole_robot_on_step']
    return {'schema':'hexarth-lift-audit/v1','verified_frames':total,'accepted_diagnostics':certified,
            'excluded_diagnostics':diagnostic,'infeasible_ik':infeasible,'convergence_forward_difference_m':difference,
            'all_six_feet_cross_20mm_at_lift22':True,'limits':'Contacts and poses sampled at 50 Hz; speeds tracked at every integrator step. These are model consistency checks, not hardware certification.'}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path)
    a=p.parse_args();result=verify(a.root)
    (a.root/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
