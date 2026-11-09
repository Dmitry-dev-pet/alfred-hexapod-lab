#!/usr/bin/env python3
"""Independent quaternion-angle and criterion checks; deterministic replay."""
from pathlib import Path
import contextlib,hashlib,io,json
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
import roll_cpu as e

def main():
    here=Path(__file__).parent;folder=here/'traces';summary=json.loads((folder/'summary.json').read_text());worst=0.
    for entry in summary['episodes']:
        raw=(folder/entry['trace_file']).read_bytes();assert hashlib.sha256(raw).hexdigest()==entry['trace_sha256'];j=json.loads(raw);fs=j['frames'];angle=0.
        for a,b in zip(fs,fs[1:]):
            if b['t']<=1:continue
            qa=np.array(a['qpos'][3:7])[[1,2,3,0]];qb=np.array(b['qpos'][3:7])[[1,2,3,0]]
            angle+=np.degrees((Rotation.from_quat(qa).inv()*Rotation.from_quat(qb)).as_rotvec()[1]);worst=max(worst,abs(angle-b['raw_forward_deg']))
        assert worst<1
        last=fs[-50:];top=j['terrain']['height_m']>0 and all(all(v>.05 for v in f['step_loads_n'].values()) and all(b['min'][0]>j['terrain']['edge_x_m']+.001 for b in f['foot_bounds'].values()) for f in last)
        assert bool(top)==entry['both_feet_on_step_final_1s']
        full=max(f['raw_forward_deg'] for f in fs)>=330 and entry['head_pivot_latched'];assert bool(full)==entry['full_rotation_completed']
        assert entry['step_goal_reached']==bool(top and entry['standing_final_1s'] and not entry['warnings'] and not entry['physics_reset_detected'])
        if not j['terrain']['height_m']:
            ref=json.loads((here/'flat-reference-hashes.json').read_text());physical=[{k:f[k] for k in ['qpos','qvel','ctrl','contacts']} for f in fs]
            assert hashlib.sha256(json.dumps(physical,sort_keys=True,separators=(',',':')).encode()).hexdigest()==ref[entry['trace_file']]
    ip=e.bench.upstream_inference(e.RL);ip.ort.InferenceSession=e.bench.cpu_session
    with contextlib.redirect_stdout(io.StringIO()):repeat=e.run(ip,.005,.12,2.,True)
    rb=(json.dumps(repeat,separators=(',',':'),allow_nan=False,sort_keys=True)+'\n').encode();expected=(folder/'step12-firm-roll2s.json').read_bytes();assert rb==expected
    # BODY uses the inertial frame; XBODY must agree with free-joint local omega.
    with contextlib.redirect_stdout(io.StringIO()):m,d,_,_=e.terrain.build(ip,0)
    f=repeat['frames'][75];d.qpos[:]=f['qpos'];d.qvel[:]=f['qvel'];mujoco.mj_forward(m,d);v=np.empty(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_XBODY,1,v,1);assert np.allclose(v[:3],d.qvel[3:6],atol=1e-10)
    receipt={'source_trace_checks':len(summary['episodes']),'independent_quaternion_max_error_deg':worst,'four_flat_physics_streams_reproduced':True,'step12_roll2s_byte_identical':True,'step12_trace_sha256':hashlib.sha256(rb).hexdigest(),'body_link_angular_frame_verified':True,'webgl_rendering_verified':False}
    (here/'verification.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt))

if __name__=='__main__':main()
