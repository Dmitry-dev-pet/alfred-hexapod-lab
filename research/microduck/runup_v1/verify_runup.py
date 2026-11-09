#!/usr/bin/env python3
from pathlib import Path
import contextlib,io,json
import numpy as np
from scipy.spatial.transform import Rotation
import runup_cpu as e
HERE=Path(__file__).resolve().parent

def main():
 j=json.loads((HERE/'traces/summary.json').read_text());assert len(j['episodes'])==12;worst=0.;loaded={};rows=[]
 for s in j['episodes']:
  path=HERE/'traces'/s['trace_file'];assert e.sha(path)==s['trace_sha256'];tr=json.loads(path.read_text());fs=tr['frames'];loaded[s['trace_file']]=tr;assert len(fs)==s['frames'] and fs[-1]['t']==s['duration_s'];assert not s['warnings'] and not s['physics_reset_detected'] and s['quality_gate']
  assert tr['roll_policy_sha256']==e.ROLL_SHA and tr['stand_policy_sha256']==e.terrain.bench.POLICY_SHA256
  angle=0.
  for a,b in zip(fs,fs[1:]):
   if s['trigger_s'] is None or b['t']<=s['trigger_s']+1e-7:continue
   qa=np.array(a['qpos'][3:7])[[1,2,3,0]];qb=np.array(b['qpos'][3:7])[[1,2,3,0]];angle+=np.degrees((Rotation.from_quat(qa).inv()*Rotation.from_quat(qb)).as_rotvec()[1]);worst=max(worst,abs(angle-b['raw_forward_deg']))
  assert worst<2
  if s['trigger_s'] is None:assert s['actual_trigger'] is None and not any(f['phase']=='roll' for f in fs)
  else:
   assert abs(s['actual_handback_s']-s['trigger_s']-(s['roll_duration_command_s']-.02))<1e-6
   before=[f for f in fs if f['phase']=='approach' and s['trigger_s']-.25<=f['t']<=s['trigger_s']+1e-7]
   if s['mode'] not in ['flat','contact']:assert s['actual_trigger']['toe_gap_m']<=float(s['mode'])/100
   if s['mode']=='contact':assert s['actual_trigger']['had_face_contact'] and s['first_face_contact_s']<=s['trigger_s']
  last=fs[-50:];goal=bool(s['height_m'] and all(all(v>.05 for v in f['step_loads_n'].values()) and all(b['min'][0]>s['edge_x_m']+.001 for b in f['foot_bounds'].values()) and f['tilt_deg']<20 and f['linear_speed_mps']<.06 and f['angular_speed_rps']<.7 for f in last));assert goal==s['step_goal_reached'];rows.append({'trace_file':s['trace_file'],'trigger_s':s['trigger_s'],'full_roll':s['full_forward_roll'],'step_goal':goal})
 a=loaded['runup-flat-moving.json'];b=loaded['runup-1-long.json'];t=b['summary']['trigger_s'];aa=[f for f in a['frames'] if f['t']<=t+1e-7];bb=[f for f in b['frames'] if f['t']<=t+1e-7];assert len(aa)==len(bb)
 for x,y in zip(aa,bb):
  for k in ['qpos','qvel','ctrl','motor_target_rad']:assert x[k]==y[k]
 ip=e.terrain.bench.upstream_inference(e.terrain.RL);ip.ort.InferenceSession=e.terrain.bench.cpu_session;repeats=[]
 for name in ['runup-flat-moving.json','runup-contact-long.json']:
  tr=loaded[name];s=tr['summary']
  with contextlib.redirect_stdout(io.StringIO()):again=e.run(ip,s['mode'],s['duration_s'],s['approach_deadline_s'],s['commanded_approach_mps'],s['height_m'],s['roll_duration_command_s'])
  raw=(json.dumps(again,separators=(',',':'),sort_keys=True,allow_nan=False)+'\n').encode();assert raw==(HERE/'traces'/name).read_bytes();repeats.append(name);print('Repeat exact: '+name,flush=True)
 receipt={'episodes_checked':12,'frames_checked':sum(s['frames'] for s in j['episodes']),'independent_quaternion_max_error_deg':worst,'matching_flat_step_approach_frames':len(aa),'matching_approach_until_s':t,'byte_identical_repeats':repeats,'flat_controls_complete_and_stable':all(s['full_forward_roll'] and s['standing_final_1s'] for s in j['episodes'] if not s['height_m']),'step_goal_successes':sum(s['step_goal_reached'] for s in j['episodes']),'triggered_step_trials':sum(s['trigger_s'] is not None and bool(s['height_m']) for s in j['episodes']),'max_body_step_penetration_m':max(s['max_body_step_penetration_m'] for s in j['episodes']),'max_all_step_penetration_m':max(s['max_step_penetration_m'] for s in j['episodes']),'max_self_contact_penetration_m':max(s['max_self_contact_penetration_m'] for s in j['episodes']),'warnings_or_resets':0,'hardware_tested':False};(HERE/'verification.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt))
if __name__=='__main__':main()
