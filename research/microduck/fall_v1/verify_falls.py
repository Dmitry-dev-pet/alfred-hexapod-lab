#!/usr/bin/env python3
"""Independent saved-state/cone/top checks and three nominal repeats."""
from pathlib import Path
import contextlib,io,json,hashlib,math
import numpy as np,mujoco as mj
from scipy.spatial.transform import Rotation
import fall_cpu as e

def main():
    folder=e.HERE/'traces';summary=json.loads((folder/'summary.json').read_text());audit=json.loads((e.HERE/'model-audit.json').read_text());ranges=np.deg2rad([j['range_deg'] for j in audit['joints']]);counts={};passed=[];maxover=0.;maxself=0.;maxenv=0.
    ip=e.e.bench.upstream_inference(e.e.RL);ip.ort.InferenceSession=e.e.bench.cpu_session
    for entry in summary['episodes']:
        path=folder/entry['trace_file'];raw=path.read_bytes();assert e.sha(path)==entry['trace_sha256'];tr=json.loads(raw);s=tr['summary'];fs=tr['frames'];assert len(fs)==500 and s=={k:entry[k] for k in s}
        assert tr['stand_policy_sha256']==e.e.bench.POLICY_SHA256 and not s['warnings'] and not s['physics_reset_detected']
        assert np.allclose(np.diff([f['t'] for f in fs]),.02,atol=1e-6)
        assert not any(82 in c['geoms'] and c['force_n']>.05 for f in fs[:50] for c in f['contacts'])
        for f in fs:
            q=np.array(f['motor_target_rad']);assert np.all(q>=ranges[:,0]+.035-1e-12) and np.all(q<=ranges[:,1]-.035+1e-12)
        first=next((f for f in fs if 1<f['t']<=2.7 and f['tilt_deg']>=65),None)
        if first:
            q=np.array(first['qpos'][3:7])[[1,2,3,0]];up=Rotation.from_quat(q).apply([0,0,1]);up=Rotation.from_euler('z',-s['initial_yaw_rad']).apply(up)
            desired={'back':np.array([-1,0,0]),'left':np.array([0,1,0]),'right':np.array([0,-1,0])}[s['direction']]
            angle=math.degrees(math.acos(np.clip(np.dot(up[:2],desired[:2])/np.linalg.norm(up[:2]),-1,1)));assert abs(angle-s['first_direction_error_deg'])<1e-5
            assert (angle<=45)==s['first_fall_in_requested_cone']
        else:assert not s['first_fall_in_requested_cone']
        last=fs[-50:];standing=all(f['tilt_deg']<20 and .09<f['trunk_z_m']<.18+s['height_m'] and f['linear_speed_mps']<.06 and f['angular_speed_rps']<.7 and all(x>.05 for x in f['foot_loads_n'].values()) for f in last)
        assert standing==s['standing_final_1s'];top=bool(s['height_m'] and all(all(x>.05 for x in f['step_loads_n'].values()) and all(b['min'][0]>.081 for b in f['foot_bounds'].values()) for f in last));assert top==s['both_feet_on_step_final_1s']
        assert s['step_goal_reached']==(top and standing);assert s['status']==('PASS' if s['fall_and_recover'] and (not s['height_m'] or top) else 'FAIL')
        maxover=max(maxover,s['max_motor_limit_overshoot_rad']);maxself=max(maxself,s['max_self_contact_penetration_m']);maxenv=max(maxenv,s['max_environment_penetration_m'])
        if s['status']=='PASS':passed.append(entry['trace_file'])
    repeats=[]
    for name in ['flat-back-4.json','flat-left-4.json','flat-right-4.json','step10-left.json']:
        old=(folder/name).read_bytes();s=json.loads(old)['summary']
        with contextlib.redirect_stdout(io.StringIO()):tr=e.run(ip,s['direction'],s['kind'],s['amplitude_rad'],s['hold_s'],s['height_m'],s['edge_x_m'])
        raw=(json.dumps(tr,separators=(',',':'),sort_keys=True,allow_nan=False)+'\n').encode();assert raw==old,name;repeats.append(name);print('Repeat exact: '+name,flush=True)
    # Independently recompute complete sole extents from compiled geometry for 1 cm success.
    tr=json.loads((folder/'step10-left.json').read_text())
    with contextlib.redirect_stdout(io.StringIO()):m,d,*_=e.setup(ip,.01,.08,'left')
    for f in tr['frames'][-50:]:
        d.qpos[:]=f['qpos'];mj.mj_forward(m,d)
        for side in ['left','right']:
            gid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,side+'_foot_collision');bounds=e.e.foot_bounds(m,d,gid)
            assert np.allclose(bounds['min'],f['foot_bounds'][side]['min'],atol=2e-8) and bounds['min'][0]>.081
    receipt={'episodes_checked':len(summary['episodes']),'flat_directed_successes':6,'flat_candidate_count':18,'step_successes':['step10-left.json'],'step_count':6,'successful_cases':passed,'byte_identical_nominal_repeats':repeats,'all_motor_targets_within_2deg_margin':True,'max_motor_limit_overshoot_deg':math.degrees(maxover),'max_sparse_self_contact_penetration_m':maxself,'max_environment_penetration_m':maxenv,'all_warnings_and_physics_resets_zero':True,'success_full_soles_recomputed':True,'robustness_to_noise_tested':False,'webgl_rendering_verified':False}
    assert len(passed)==7
    (e.HERE/'verification.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt))
if __name__=='__main__':main()
