#!/usr/bin/env python3
"""Independent height-goal checks and successful/blocked repeatability."""
from pathlib import Path
import contextlib,hashlib,io,json
import numpy as np
from scipy.spatial.transform import Rotation
from height_cpu import e

def main():
    here=Path(__file__).parent;p=here/'traces';summary=json.loads((p/'summary.json').read_text());worst=0.;goals=[];initial=None
    for ent in summary['episodes']:
        raw=(p/ent['trace_file']).read_bytes();assert hashlib.sha256(raw).hexdigest()==ent['trace_sha256'];tr=json.loads(raw);fs=tr['frames'];h=ent['height_m'];edge=ent['edge_x_m']
        assert len(fs)==500 and tr['terrain']['height_m']==h and tr['terrain']['edge_x_m']==edge
        assert tr['roulade_policy_sha256']==e.ROLL_SHA and tr['stand_policy_sha256']==e.bench.POLICY_SHA256
        assert not ent['warnings'] and not ent['physics_reset_detected']
        assert not any(82 in c['geoms'] and c['force_n']>.05 for f in fs if f['t']<=1 for c in f['contacts'])
        state=np.array([f['qpos'] for f in fs[:50]])
        if initial is None:initial=state
        else:assert np.max(np.abs(state-initial))<1e-8
        last=fs[-50:]
        goal=all(all(v>.05 for v in f['step_loads_n'].values()) and all(b['min'][0]>edge+.001 for b in f['foot_bounds'].values()) and f['tilt_deg']<20 and .09+h<f['trunk_z_m']<.18+h and f['linear_speed_mps']<.06 and f['angular_speed_rps']<.7 for f in last)
        assert goal==ent['step_goal_reached'];assert ent['status']==('PASS' if goal else 'FAIL');assert ent['strict_roll_status']==tr['summary']['status']
        if goal:goals.append(ent)
        angle=0.
        for a,b in zip(fs,fs[1:]):
            if b['t']<=1:continue
            qa=np.array(a['qpos'][3:7])[[1,2,3,0]];qb=np.array(b['qpos'][3:7])[[1,2,3,0]]
            angle+=np.degrees((Rotation.from_quat(qa).inv()*Rotation.from_quat(qb)).as_rotvec()[1]);worst=max(worst,abs(angle-b['raw_forward_deg']))
        assert worst<1
    assert len(goals)==4 and max(x['height_mm'] for x in goals)==20
    ip=e.bench.upstream_inference(e.RL);ip.ort.InferenceSession=e.bench.cpu_session;repeats=[]
    for cm,name in [(8,'height20-edge8-firm-roll2s.json'),(12,'height20-firm-roll2s.json')]:
        with contextlib.redirect_stdout(io.StringIO()):tr=e.run(ip,.02,cm/100,2.,True)
        raw=(json.dumps(tr,separators=(',',':'),allow_nan=False,sort_keys=True)+'\n').encode();assert raw==(p/name).read_bytes();repeats.append(name)
    receipt={'trace_checks':len(summary['episodes']),'independent_quaternion_max_error_deg':worst,'all_initial_qpos_matched':True,'initial_step_contacts':0,'byte_identical_repeats':repeats,'step_goal_passes':len(goals),'highest_successful_tested_mm':20,'success_max_penetration_m':max(x['max_contact_penetration_m'] for x in goals),'webgl_rendering_verified':False}
    (here/'verification.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt))

if __name__=='__main__':main()
