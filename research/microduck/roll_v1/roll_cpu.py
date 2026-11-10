#!/usr/bin/env python3
"""Original roulade.onnx + official trigger/timer, without external impulses."""
import argparse,contextlib,hashlib,io,json,os,sys,math
from pathlib import Path
import mujoco
import numpy as np

WORK=Path(os.environ['MICRODUCK_WORKSPACE']).resolve() if 'MICRODUCK_WORKSPACE' in os.environ else next((p for p in Path(__file__).resolve().parents if (p/'microduck-work/source-manifest.json').is_file()),Path.cwd())
sys.path.insert(0,str(WORK/'microduck-step'))
import step_cpu as terrain
bench=terrain.bench;RL=terrain.RL;BAM=terrain.BAM
HERE=Path(__file__).resolve().parent
ROLL=Path(os.environ.get('MICRODUCK_ROLL_POLICY',str(HERE/'roulade.onnx')));STAND=WORK/'microduck-work/velstand.onnx'
ROLL_SHA='3d60da08fc13f29c1b57f41977aa898132c0d60042100149d8e775affcbca32b'
HF_COMMIT='665ae4eb20bfe13bca1ef4ea6d4bb6b4a096e448'
CDT=.02

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def run(ip,height,edge,duration,firm):
    dt=.001 if firm else .005;terrain.EDGE=edge
    with contextlib.redirect_stdout(io.StringIO()):
        m,d,c,_=terrain.build(ip,height,dt,firm)
        mujoco.mj_resetDataKeyframe(m,d,mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_KEY,'STAND'))
        sq=m.jnt_qposadr[m.actuator_trnid[:,0]];c.reset(d.qpos);c.last_ts=d.time;mujoco.mj_forward(m,d)
        policy=ip.PolicyInference(m,d,walking_onnx_path=str(STAND),roulade_onnx_path=str(ROLL),roulade_duration=duration,bam_ctrl=c,use_projected_gravity=True,new_cmd_obs=True)
        bench.install_encoder_view(policy,c,m,d);bq,_=bench.backlash_indices(m);c.q_target[:]=d.qpos[sq]+d.qpos[bq];policy.set_vel_cmd(0,0,0)
    assert policy.get_observations().shape==(61,) and m.nu==14
    floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'floor');step=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'step_top')
    feet={side:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,side+'_foot_collision') for side in ('left','right')}
    root=m.jnt_bodyid[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'trunk_base_freejoint')]
    head=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'jaw_soft');assert head>=0
    accum=frontier=raw_angle=0.;head_latched=False;peak_force=peak_torque=max_pen=0.;unsupported=active_steps=0;reset=False
    frames=[];triggered=False;handback=None;completion=None
    for k in range(500):
        t=round(k*CDT,6)
        with contextlib.redirect_stdout(io.StringIO()):
            if k==50:policy.trigger_behavior('roulade');triggered=True
            # Same pre-inference timer ordering as upstream's main loop.
            before_mode=policy.behavior_mode;policy.update_behavior(CDT)
            if before_mode and policy.behavior_mode is None:handback=t
        phase='settle' if not triggered else ('roll' if policy.behavior_mode else 'recovery')
        mujoco.mj_forward(m,d);policy.apply_action(policy.infer())
        for _ in range(round(CDT/dt)):
            before=float(d.time);c.update();mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
            reset|=abs(float(d.time)-before-dt)>1e-8
            if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():raise FloatingPointError('Nonfinite state')
            peak_torque=max(peak_torque,float(np.max(np.abs(d.actuator_force))))
            supported=False;head_contact=False
            for i in range(d.ncon):
                ct=d.contact[i];pair=(ct.geom1,ct.geom2)
                if floor not in pair and step not in pair:continue
                w=np.zeros(6);mujoco.mj_contactForce(m,d,i,w);peak_force=max(peak_force,float(np.linalg.norm(w[:3])));max_pen=max(max_pen,-float(ct.dist))
                if w[0]>.05:
                    supported=True
                    head_contact|=any(m.geom_bodyid[g]==head for g in pair)
            if triggered:
                vel=np.empty(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_XBODY,int(root),vel,1)
                q=d.xquat[root];lateral_z=abs(2*(q[2]*q[3]+q[0]*q[1]));u=float(np.clip((.866-lateral_z)/(.866-.5),0,1));flat=u*u*(3-2*u)
                raw_angle+=float(vel[1])*dt
                accum+=float(vel[1])*dt*int(supported)*flat;frontier=max(frontier,accum)
                top_down=(d.xmat[head].reshape(3,3)@np.array([.882,0,.471]))[2]<-.3
                head_latched|=bool(head_contact and math.radians(20)<accum<math.radians(170) and top_down)
                if frontier>=math.radians(330) and head_latched and completion is None:completion=float(d.time)
                if policy.behavior_mode:active_steps+=1;unsupported+=not supported
        loads={side:0. for side in feet};toploads={side:0. for side in feet};contacts=[]
        for i in range(d.ncon):
            ct=d.contact[i];pair=(ct.geom1,ct.geom2)
            if floor not in pair and step not in pair:continue
            w=np.zeros(6);mujoco.mj_contactForce(m,d,i,w)
            contacts.append({'geoms':[int(v) for v in pair],'pos':ct.pos.tolist(),'normal':ct.frame[:3].tolist(),'force_n':float(w[0]),'distance_m':float(ct.dist)})
            for side,gid in feet.items():
                if gid in pair and abs(ct.frame[2])>.85:
                    loads[side]+=max(0.,float(w[0]))
                    if step>=0 and step in pair and ct.pos[2]>=height-.001:toploads[side]+=max(0.,float(w[0]))
        frames.append({'t':round(float(d.time),6),'phase':phase,**bench.sample(m,d,0,0),'foot_loads_n':loads,'step_loads_n':toploads,
                       'foot_bounds':{side:terrain.foot_bounds(m,d,gid) for side,gid in feet.items()},'contacts':contacts,
                       'supported_forward_deg':math.degrees(accum),'max_supported_forward_deg':math.degrees(frontier),
                       'raw_forward_deg':math.degrees(raw_angle),'head_pivot_latched':bool(head_latched)})
    last=frames[-50:];warnings={str(i):int(w.number) for i,w in enumerate(d.warning) if w.number}
    standing=all(f['tilt_deg']<20 and .09+height<f['trunk_z_m']<.18+height and f['linear_speed_mps']<.06 and f['angular_speed_rps']<.7 and all(v>.05 for v in f['foot_loads_n'].values()) for f in last)
    on_top=height>0 and all(all(v>.05 for v in f['step_loads_n'].values()) and all(b['min'][0]>edge+.001 for b in f['foot_bounds'].values()) for f in last)
    roll_complete=frontier>=math.radians(330) and head_latched
    passed=bool(roll_complete and standing and (on_top or not height) and not warnings and not reset)
    summary={'height_m':height,'edge_x_m':edge,'roll_duration_command_s':duration,'actual_handback_s':handback,'contact_profile':'firm_1ms' if firm else 'upstream_5ms',
             'status':'PASS' if passed else 'FAIL','roll_complete':bool(roll_complete),'completion_time_s':completion,'supported_rotation_deg':math.degrees(frontier),
             'raw_rotation_deg':math.degrees(raw_angle),'raw_rotation_max_deg':max(f['raw_forward_deg'] for f in frames),'full_rotation_completed':bool(max(f['raw_forward_deg'] for f in frames)>=330 and head_latched),'step_goal_reached':bool(on_top and standing and not warnings and not reset),'head_pivot_latched':bool(head_latched),'standing_final_1s':bool(standing),'both_feet_on_step_final_1s':bool(on_top),
             'active_roll_unsupported_fraction':unsupported/max(1,active_steps),'final_position_m':frames[-1]['qpos'][:3],
             'max_tilt_deg':max(f['tilt_deg'] for f in frames),'final_tilt_deg':frames[-1]['tilt_deg'],'max_contact_penetration_m':max_pen,
             'peak_environment_contact_n':peak_force,'max_actuator_torque_nm':peak_torque,'warnings':warnings,'physics_reset_detected':reset,'duration_s':10,'frames':500}
    return {'schema':'microduck-roulade-physical/v1','upstream_commit':bench.UPSTREAM_COMMIT,'bam_commit':bench.BAM_COMMIT,'policy_set_commit':HF_COMMIT,
            'roulade_policy_sha256':ROLL_SHA,'stand_policy_sha256':sha(STAND),'trigger_s':1,'dt':dt,'control_dt':CDT,'contact_profile':summary['contact_profile'],
            'terrain':{'height_m':height,'edge_x_m':edge,'half_size_xy_m':[.6,.6],'center_xy_m':[edge+.6,0],'friction':[1,.005,.0001]},'summary':summary,'frames':frames}

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=HERE/'traces');p.add_argument('--flat-only',action='store_true');a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    mf=json.loads((WORK/'microduck-work/source-manifest.json').read_text());mf['_path']=str(WORK/'microduck-work/source-manifest.json');source=bench.source_gate(RL,BAM,mf)
    assert sha(ROLL)==ROLL_SHA and sha(STAND)==bench.POLICY_SHA256
    ip=bench.upstream_inference(RL);ip.ort.InferenceSession=bench.cpu_session;entries=[]
    cases=[(0,.18,d,f) for f in [False,True] for d in [1.,2.]]
    if not a.flat_only:cases +=[(.005,e,d,True) for d in [1.,2.] for e in [.08,.12,.18]]
    for h,e,d,f in cases:
        tr=run(ip,h,e,d,f);name=f"{'step'+str(round(e*100)) if h else 'flat'}-{'firm' if f else 'upstream'}-roll{round(d)}s.json";path=a.out/name;path.write_text(json.dumps(tr,separators=(',',':'),allow_nan=False,sort_keys=True)+'\n');entry={**tr['summary'],'trace_file':name,'trace_sha256':sha(path)};entries.append(entry);print(json.dumps(entry),flush=True)
    (a.out/'summary.json').write_text(json.dumps({'schema':'microduck-roulade-summary/v1','source_gate':source,'episodes':entries},indent=2)+'\n')

if __name__=='__main__':main()
