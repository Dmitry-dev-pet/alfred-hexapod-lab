#!/usr/bin/env python3
"""Physical MicroDuck v8 single-step probe; no scripted root movement."""
import argparse, contextlib, hashlib, io, json, os, sys
from pathlib import Path
import mujoco
import numpy as np

WORK=Path(os.environ['MICRODUCK_WORKSPACE']).resolve() if 'MICRODUCK_WORKSPACE' in os.environ else next((p for p in Path(__file__).resolve().parents if (p/'microduck-work/source-manifest.json').is_file()),Path.cwd())
sys.path.insert(0,str(WORK/'microduck-work/lab/research/microduck'))
import bam_backlash_cpu as bench

RL=WORK/'microduck-work/microduck_rl-bae39f5900b318f7e87fc485c30a31213cfa14eb'
BAM=WORK/'microduck-work/bam-62bd8ce12154340be97e06f7f41a0ca8f116d967'
POLICY=WORK/'microduck-work/velstand.onnx'
EDGE=.18
DT=.005
CDT=.02

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

class TerrainMuJoCo:
    def __init__(self,height,firm=False):self.height=height;self.firm=firm
    def __getattr__(self,name):
        if name!='MjSpec':return getattr(mujoco,name)
        height=self.height;firm=self.firm
        class Spec:
            @staticmethod
            def from_file(path):
                s=mujoco.MjSpec.from_file(path)
                if height:
                    # Append a static body to preserve all original geom IDs.
                    b=s.worldbody.add_body(name='step_platform',pos=[EDGE+.6,0,height/2])
                    b.add_geom(name='step_top',type=mujoco.mjtGeom.mjGEOM_BOX,
                               size=[.6,.6,height/2],rgba=[.55,.69,.43,1],
                               friction=[1,.005,.0001])
                if firm:
                    for g in s.geoms:g.solref=[.005,1]
                return s
        return Spec

def build(ip,height,dt=DT,firm=False):
    proxy=ip.mujoco;ip.mujoco=TerrainMuJoCo(height,firm)
    try:
        bam=ip.load_bam_model(200,7.4,None)
        return ip.load_mujoco_with_bam(str(RL/'src/mjlab_microduck/robot/microduck/scene_backlash.xml'),bam,dt,.1,6)
    finally:ip.mujoco=proxy

def foot_bounds(m,d,gid):
    mid=m.geom_dataid[gid];a,n=m.mesh_vertadr[mid],m.mesh_vertnum[mid]
    p=m.mesh_vert[a:a+n] @ d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid]
    return {'min':p.min(axis=0).tolist(),'max':p.max(axis=0).tolist()}

def run(ip,height,seed,firm=False):
    dt=.001 if firm else DT
    with contextlib.redirect_stdout(io.StringIO()):
        m,d,c,_=build(ip,height,dt,firm)
        mujoco.mj_resetDataKeyframe(m,d,mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_KEY,'STAND'))
        sq=m.jnt_qposadr[m.actuator_trnid[:,0]]
        rng=np.random.default_rng(seed);jitter=np.zeros(14) if seed==20261109 else rng.uniform(-.005,.005,14)
        d.qpos[sq]+=jitter;c.reset(d.qpos);c.last_ts=d.time;mujoco.mj_forward(m,d)
        policy=ip.PolicyInference(m,d,walking_onnx_path=str(POLICY),bam_ctrl=c,use_projected_gravity=True,new_cmd_obs=True)
        bench.install_encoder_view(policy,c,m,d);bq,_=bench.backlash_indices(m);c.q_target[:]=d.qpos[sq]+d.qpos[bq]
        policy.set_vel_cmd(0,0,0)
    floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'floor')
    step=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'step_top')
    feet={side:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,side+'_foot_collision') for side in ('left','right')}
    phase='settle';stop_at=None;fell_at=None;first_top=None;crossed_at=None
    frames=[];reset=False;max_pen=0.;peak_contact=0.;max_torque=0.
    commanded=None
    for k in range(750):
        t=round(k*CDT,6)
        if t>=1 and phase=='settle':phase='approach'
        if phase=='approach' and height and frames and all(f['min'][0]>EDGE+.005 for f in frames[-1]['foot_bounds'].values()) and any(frames[-1]['step_loads_n'].values()):
            phase='hold';stop_at=t;crossed_at=t
        if frames and frames[-1]['tilt_deg']>65 and fell_at is None:
            fell_at=t;phase='recovery';stop_at=t
        if not height and phase=='approach' and d.qpos[0]>.32:
            phase='hold';stop_at=t
        command=[.30,0,0] if phase=='approach' else [0,0,0]
        with contextlib.redirect_stdout(io.StringIO()):
            if command!=commanded:policy.set_vel_cmd(*command);commanded=command
        mujoco.mj_forward(m,d);policy.apply_action(policy.infer())
        for _ in range(round(CDT/dt)):
            before=float(d.time);c.update();mujoco.mj_step(m,d)
            reset|=abs(float(d.time)-before-dt)>1e-8
            if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():raise FloatingPointError('Nonfinite state')
            max_torque=max(max_torque,float(np.max(np.abs(d.actuator_force))))
            for i in range(d.ncon):
                ct=d.contact[i]
                if floor not in (ct.geom1,ct.geom2) and step not in (ct.geom1,ct.geom2):continue
                max_pen=max(max_pen,-float(ct.dist));w=np.zeros(6);mujoco.mj_contactForce(m,d,i,w);peak_contact=max(peak_contact,float(np.linalg.norm(w[:3])))
        mujoco.mj_forward(m,d)
        loads={side:0. for side in feet};contacts=[]
        for i in range(d.ncon):
            ct=d.contact[i];pair=(ct.geom1,ct.geom2)
            if floor not in pair and step not in pair:continue
            w=np.zeros(6);mujoco.mj_contactForce(m,d,i,w)
            contacts.append({'geoms':[int(v) for v in pair],'pos':ct.pos.tolist(),'normal':ct.frame[:3].tolist(),'force_n':float(w[0]),'distance_m':float(ct.dist)})
            if step>=0 and step in pair and abs(ct.frame[2])>.85 and ct.pos[2]>=height-.001:
                for side,gid in feet.items():
                    if gid in pair:loads[side]+=max(0.,float(w[0]))
        if any(v>.05 for v in loads.values()) and first_top is None:first_top=round(float(d.time),6)
        sample=bench.sample(m,d,0,0)
        frames.append({'t':round(float(d.time),6),'phase':phase,'command':command,**sample,
                       'step_loads_n':loads,'foot_bounds':{side:foot_bounds(m,d,g) for side,g in feet.items()},'contacts':contacts})
        if stop_at is not None and float(d.time)>=stop_at+(6 if fell_at is not None else 3):break
    warnings={str(i):int(w.number) for i,w in enumerate(d.warning) if w.number}
    last=frames[-50:]
    supported=height>0 and all(all(v>.05 for v in f['step_loads_n'].values()) and all(b['min'][0]>EDGE+.001 for b in f['foot_bounds'].values()) for f in last)
    quiet=all(f['tilt_deg']<20 and f['linear_speed_mps']<.06 and f['angular_speed_rps']<.7 for f in last)
    passed=bool(crossed_at is not None and supported and quiet and fell_at is None and not warnings and not reset)
    summary={'contact_profile':'firm_1ms' if firm else 'upstream_5ms','height_m':height,'seed':seed,'status':('PASS' if passed else 'FAIL') if height else 'CONTROL',
             'first_foot_top_contact_s':first_top,'fully_crossed_s':crossed_at,'stop_command_s':stop_at,'fell_s':fell_at,
             'both_feet_supported_final_1s':bool(supported),'quiet_final_1s':quiet,'final_position_m':frames[-1]['qpos'][:3],
             'final_tilt_deg':frames[-1]['tilt_deg'],'max_tilt_deg':max(f['tilt_deg'] for f in frames),
             'max_contact_penetration_m':max_pen,'peak_environment_contact_n':peak_contact,'max_actuator_torque_nm':max_torque,
             'warnings':warnings,'physics_reset_detected':reset,'duration_s':frames[-1]['t'],'frames':len(frames)}
    return {'schema':'microduck-step-physical/v1','upstream_commit':bench.UPSTREAM_COMMIT,'bam_commit':bench.BAM_COMMIT,
            'policy_sha256':sha(POLICY),'scene_sha256':sha(RL/'src/mjlab_microduck/robot/microduck/scene_backlash.xml'),
            'dt':dt,'control_dt':CDT,'contact_profile':'firm_1ms' if firm else 'upstream_5ms','contact_solref_override':[.005,1] if firm else None,'terrain':{'edge_x_m':EDGE,'height_m':height,'half_size_xy_m':[.6,.6],'center_xy_m':[EDGE+.6,0],'friction':[1,.005,.0001]},
            'initial_joint_jitter_rad':jitter.tolist(),'summary':summary,'frames':frames}

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=Path(__file__).parent/'traces');p.add_argument('--seeds',type=int,nargs='+',default=[20261109,20261110,20261111]);p.add_argument('--flat-control',action='store_true');p.add_argument('--firm-check',action='store_true');a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((WORK/'microduck-work/source-manifest.json').read_text());manifest['_path']=str(WORK/'microduck-work/source-manifest.json')
    source=bench.source_gate(RL,BAM,manifest);assert sha(POLICY)==bench.POLICY_SHA256
    ip=bench.upstream_inference(RL);ip.ort.InferenceSession=bench.cpu_session
    entries=[]
    for height,seeds in [(.005,a.seeds)]+([(0.,[20261109])] if a.flat_control else []):
        for seed in seeds:
            tr=run(ip,height,seed);name=f"{'step5mm' if height else 'flat'}-{seed}.json";path=a.out/name;path.write_text(json.dumps(tr,separators=(',',':'),allow_nan=False)+'\n');entry={**tr['summary'],'trace_file':name,'trace_sha256':sha(path)};entries.append(entry);print(json.dumps(entry),flush=True)
    if a.firm_check:
        tr=run(ip,.005,20261111,True);name='step5mm-firm-20261111.json';path=a.out/name;path.write_text(json.dumps(tr,separators=(',',':'),allow_nan=False)+'\n');entry={**tr['summary'],'trace_file':name,'trace_sha256':sha(path)};entries.append(entry);print(json.dumps(entry),flush=True)
    (a.out/'summary.json').write_text(json.dumps({'schema':'microduck-step-summary/v1','source_gate':source,'episodes':entries},indent=2)+'\n')

if __name__=='__main__':main()
