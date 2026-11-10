#!/usr/bin/env python3
"""Audit saved traces/replay provenance and bounded first-trial acceptance."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(directory):
    receipt=json.loads((directory/'receipt.json').read_text())
    assert receipt['mujoco']=='3.3.7'
    assert abs(receipt['model']['model_mass_kg']-2.1)<1e-8
    assert receipt['model']['max_fk_crosscheck_error_m']<1e-8
    total=0
    for case in receipt['cases']:
        tracepath=directory/(case['case']+'-trace.json')
        trace=json.loads(tracepath.read_text())
        replay=json.loads((directory/(case['case']+'-replay.json')).read_text())
        assert sha(tracepath)==case['trace_sha256']==replay['source_trace_sha256']
        modelname='model-flat.xml' if 'step' not in case['case'] else 'model-step-'+case['case'].split('-')[-1]+'.xml'
        assert sha(directory/modelname)==trace['model_sha256']==replay['source_model_sha256']
        assert len(trace['frames'])==len(replay['frames'])==round(case['duration_s']/.02)
        last=0.
        for frame,pose in zip(trace['frames'],replay['frames']):
            assert abs(frame['t']-last-.02)<1e-8;last=frame['t']
            assert len(frame['qpos'])==25 and len(frame['qvel'])==24 and len(frame['ctrl'])==18
            assert pose==[frame['t']]+frame['qpos'][:7]+frame['body_poses']
            assert np.isfinite(pose).all() and np.isfinite(frame['qvel']).all()
            assert max(abs(x) for x in frame['ctrl'])<=receipt['actuator']['rated_torque_nm']+1e-7
            assert not frame['nonfoot_ground_contact']
        assert case['metrics']['max_ik_residual_m']<5e-6
        assert case['metrics']['peak_self_penetration_m']<.001
        assert case['metrics']['peak_external_penetration_m']<.001
        assert case['metrics']['peak_joint_speed_rad_s']<receipt['actuator']['no_load_speed_rad_s']
        assert not case['metrics']['mujoco_warnings']
        total+=len(trace['frames'])
    byname={c['case']:c for c in receipt['cases']}
    assert byname['stand']['diagnostic_stable_end']
    assert abs(byname['stand']['metrics']['forward_after_settle_m'])<.001
    assert byname['walk-flat']['metrics']['forward_after_settle_m']>.1
    assert byname['walk-step-10mm']['whole_robot_on_step']
    assert not byname['walk-step-20mm']['whole_robot_on_step']
    convergence=json.loads((directory/'convergence/receipt.json').read_text())
    assert convergence['same_whole_step_verdict'] and convergence['forward_difference_m']<.005
    return {'verified_cases':len(byname),'verified_frames':total,'convergence_difference_m':convergence['forward_difference_m']}


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path)
    print(json.dumps(verify(ap.parse_args().directory)))
