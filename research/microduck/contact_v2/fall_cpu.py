#!/usr/bin/env python3
"""Motor-only directed fall, then pinned v8 velstand recovery. No root impulse."""
import argparse,contextlib,hashlib,io,json,math,sys,os
from pathlib import Path
import mujoco as mj,numpy as np
WORK=Path(os.environ['MICRODUCK_WORKSPACE']).resolve() if 'MICRODUCK_WORKSPACE' in os.environ else next(p for p in Path(__file__).resolve().parents if (p/'microduck-work/source-manifest.json').is_file())
sys.path.insert(0,str(WORK/'microduck-step'));import step_cpu as e
HERE=Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def setup(ip,height=0.,edge=.08,direction='back'):
    e.EDGE=edge
    with contextlib.redirect_stdout(io.StringIO()):
        m,d,c,_=e.build(ip,height,.001,True)
        mj.mj_resetDataKeyframe(m,d,mj.mj_name2id(m,mj.mjtObj.mjOBJ_KEY,'STAND'))
        # Initial orientation only: desired body-relative fall points along +world X.
        yaw={'back':math.pi,'left':-math.pi/2,'right':math.pi/2}[direction] if height else 0.
        d.qpos[3:7]=[math.cos(yaw/2),0,0,math.sin(yaw/2)]
        sq=m.jnt_qposadr[m.actuator_trnid[:,0]];bq,_=e.bench.backlash_indices(m)
        c.reset(d.qpos);c.last_ts=d.time;mj.mj_forward(m,d)
        p=ip.PolicyInference(m,d,walking_onnx_path=str(e.POLICY),bam_ctrl=c,use_projected_gravity=True,new_cmd_obs=True)
        e.bench.install_encoder_view(p,c,m,d);c.q_target[:]=d.qpos[sq]+d.qpos[bq];p.set_vel_cmd(0,0,0)
    return m,d,c,p,sq,bq,yaw

def target_pose(start,direction,kind,amp):
    q=start.copy()
    if direction=='back':
        if kind=='ankle':q[4]+=amp;q[13]-=amp
        elif kind=='hip':q[2]+=amp;q[11]-=amp
        elif kind=='combined':q[2]+=amp*.6;q[11]-=amp*.6;q[4]+=amp;q[13]-=amp;q[5]-=.3
    else:
        # Construct left command; mirror using the official joint permutation/sign.
        if kind=='roll':q[1]+=amp;q[10]+=amp;q[8]+=amp*.5
        elif kind=='fold':q[2]-=abs(amp)*.5;q[3]+=abs(amp)*1.3;q[4]-=abs(amp)*.7;q[1]+=amp*.4;q[10]+=amp*.4
        elif kind=='combined':q[1]+=amp;q[10]+=amp;q[2]-=abs(amp)*.5;q[3]+=abs(amp);q[4]-=abs(amp)*.5
        if direction=='right':q=q[[9,10,11,12,13,5,6,7,8,0,1,2,3,4]]*np.array([-1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,-1,-1])
    return q

def run(ip,direction,kind,amp,hold=1.2,height=0.,edge=.08):
    m,d,c,p,sq,bq,yaw=setup(ip,height,edge,direction)
    js=m.actuator_trnid[:,0];lo=m.jnt_range[js,0]+.035;hi=m.jnt_range[js,1]-.035
    root=1;floor=0;step=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'step_top')
    feet={s:mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,s+'_foot_collision') for s in ['left','right']}
    baseRot=d.xmat[root].reshape(3,3).copy();frames=[];start=target=None;handback=None;peak=0.;peakself=0.;peakenv=0.;force=0.;torque=0.;limex=0.;reset=False;targetclip=0.;bodyground=False
    for k in range(500):
        t=k*.02;mj.mj_forward(m,d)
        if k==50:
            start=c.q_target.copy();raw=target_pose(start,direction,kind,amp);target=np.clip(raw,lo,hi);targetclip=float(np.max(np.abs(raw-target)))
        phase='settle' if k<50 else ('fall' if t<1+hold else 'recovery')
        if phase=='fall':
            u=min(1.,(t-1+.02)/.25);u=u*u*(3-2*u);q=start+(target-start)*u
            c.q_target[:]=q;p.last_action=((q-p.default_pose)/p.action_scale).astype(np.float32)
        else:
            if k>=50 and handback is None:handback=t
            action=p.infer();q=np.clip(p.default_pose+action*p.action_scale,lo,hi);c.q_target[:]=q;p.last_action=((q-p.default_pose)/p.action_scale).astype(np.float32)
        for _ in range(20):
            before=float(d.time);c.update();mj.mj_step(m,d);mj.mj_forward(m,d);reset|=abs(float(d.time)-before-.001)>1e-8
            assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
            torque=max(torque,float(abs(d.actuator_force).max()));limex=max(limex,float(np.maximum(m.jnt_range[js,0]-d.qpos[sq],d.qpos[sq]-m.jnt_range[js,1]).max()))
            up=baseRot.T@d.xmat[root].reshape(3,3)[:,2]
            component={'back':-up[0],'left':up[1],'right':-up[1]}[direction]
            lean=math.degrees(math.atan2(component,up[2]))
            if t>=1 and t<1+hold+.5:peak=max(peak,lean)
            tilt=math.degrees(math.acos(np.clip(d.xmat[root].reshape(3,3)[2,2],-1,1)))
            for i in range(d.ncon):
                ct=d.contact[i];pair=(ct.geom1,ct.geom2);w=np.zeros(6);mj.mj_contactForce(m,d,i,w)
                if floor in pair or step>=0 and step in pair:
                    peakenv=max(peakenv,-float(ct.dist));force=max(force,float(np.linalg.norm(w[:3])))
                    other=pair[1] if pair[0] in [floor,step] else pair[0]
                    if t>=1 and tilt>45 and w[0]>.05 and other not in feet.values():bodyground=True
                else:peakself=max(peakself,-float(ct.dist))
        loads={s:0. for s in feet};top={s:0. for s in feet};contacts=[]
        for i in range(d.ncon):
            ct=d.contact[i];pair=(ct.geom1,ct.geom2);w=np.zeros(6);mj.mj_contactForce(m,d,i,w)
            contacts.append({'geoms':[int(x) for x in pair],'force_n':float(w[0]),'distance_m':float(ct.dist)})
            if abs(ct.frame[2])>.85:
                for s,g in feet.items():
                    if g in pair and (floor in pair or step>=0 and step in pair):loads[s]+=max(0.,float(w[0]))
                    if g in pair and step>=0 and step in pair and ct.pos[2]>=height-.001:top[s]+=max(0.,float(w[0]))
        frames.append({'t':round(float(d.time),6),'phase':phase,**e.bench.sample(m,d,0,0),'motor_target_rad':c.q_target.tolist(),'directional_lean_deg':lean,'foot_loads_n':loads,'step_loads_n':top,'foot_bounds':{s:e.foot_bounds(m,d,g) for s,g in feet.items()},'contacts':contacts})
    last=frames[-50:];warnings={str(i):int(w.number) for i,w in enumerate(d.warning) if w.number}
    standing=all(f['tilt_deg']<20 and .09<f['trunk_z_m']<.18+height and f['linear_speed_mps']<.06 and f['angular_speed_rps']<.7 and all(x>.05 for x in f['foot_loads_n'].values()) for f in last)
    on_top=bool(height and all(all(x>.05 for x in f['step_loads_n'].values()) and all(b['min'][0]>edge+.001 for b in f['foot_bounds'].values()) for f in last))
    direction_check=check_direction(frames,direction,yaw,hold)
    fallen=bool(peak>=65 and bodyground and direction_check['first_fall_in_requested_cone']);ok=bool(fallen and standing and not warnings and not reset)
    summary={**direction_check,'direction':direction,'kind':kind,'amplitude_rad':amp,'hold_s':hold,'height_m':height,'edge_x_m':edge,'initial_yaw_rad':yaw,'status':'PASS' if ok and (not height or on_top) else 'FAIL','fall_and_recover':ok,'directed_fall':fallen,'peak_directional_lean_deg':peak,'body_environment_contact_while_tilted':bodyground,'standing_final_1s':standing,'both_feet_on_step_final_1s':on_top,'step_goal_reached':bool(on_top and standing),'actual_handback_s':handback,'final_position_m':frames[-1]['qpos'][:3],'max_tilt_deg':max(f['tilt_deg'] for f in frames),'final_tilt_deg':frames[-1]['tilt_deg'],'max_self_contact_penetration_m':peakself,'max_environment_penetration_m':peakenv,'peak_environment_force_n':force,'peak_motor_torque_nm':torque,'max_motor_limit_overshoot_rad':max(0.,limex),'authored_target_clamp_rad':targetclip,'warnings':warnings,'physics_reset_detected':reset,'duration_s':10,'frames':500}
    return {'schema':'microduck-directed-fall-physical/v1','stand_policy_sha256':sha(e.POLICY),'dt':.001,'control_dt':.02,'contact_profile':'firm_1ms','terrain':{'height_m':height,'edge_x_m':edge,'half_size_xy_m':[.6,.6],'center_xy_m':[edge+.6,0]},'summary':summary,'frames':frames}

def check_direction(frames,direction,yaw,hold):
    first=next((f for f in frames if 1<f['t']<=1+hold+.5 and f['tilt_deg']>=65),None)
    if not first:return {'first_fall_in_requested_cone':False,'first_65deg_s':None,'first_direction_error_deg':None}
    q=first['qpos'][3:7];w,x,y,z=q;up=np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
    co,si=math.cos(yaw),math.sin(yaw);up=np.array([co*up[0]+si*up[1],-si*up[0]+co*up[1],up[2]])
    comp={'back':-up[0],'left':up[1],'right':-up[1]}[direction];lateral=abs(up[1]) if direction=='back' else abs(up[0]);angle=math.degrees(math.atan2(lateral,comp))
    return {'first_fall_in_requested_cone':bool(angle<=45),'first_65deg_s':first['t'],'first_direction_error_deg':angle}

def reclassify():
    p=HERE/'traces/summary.json';summary=json.loads(p.read_text());entries=[]
    for s in summary['episodes']:
        f=HERE/'traces'/s['trace_file'];tr=json.loads(f.read_text());a=tr['summary'];a.update(check_direction(tr['frames'],a['direction'],a['initial_yaw_rad'],a['hold_s']))
        a['directed_fall']=bool(a['directed_fall'] and a['first_fall_in_requested_cone']);a['fall_and_recover']=bool(a['directed_fall'] and a['standing_final_1s'] and not a['warnings'] and not a['physics_reset_detected']);a['status']='PASS' if a['fall_and_recover'] and (not a['height_m'] or a['step_goal_reached']) else 'FAIL'
        entries.append(save(tr,s['trace_file']))
    summary['episodes']=entries;p.write_text(json.dumps(summary,indent=2)+'\n')

def save(tr,name):
    p=HERE/'traces'/name;p.write_text(json.dumps(tr,separators=(',',':'),sort_keys=True,allow_nan=False)+'\n');s={**tr['summary'],'trace_file':name,'trace_sha256':sha(p)};print(json.dumps(s),flush=True);return s

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--steps',action='store_true');a=parser.parse_args()
    mf=json.loads((WORK/'microduck-work/source-manifest.json').read_text());mf['_path']=str(WORK/'microduck-work/source-manifest.json');source=e.bench.source_gate(e.RL,e.BAM,mf);assert sha(e.POLICY)==e.bench.POLICY_SHA256
    ip=e.bench.upstream_inference(e.RL);ip.ort.InferenceSession=e.bench.cpu_session
    if a.steps:
        prev=json.loads((HERE/'traces/summary.json').read_text());entries=prev['episodes'];successful={d:next((s for s in entries if s['direction']==d and s['status']=='PASS' and not s['height_m']),None) for d in ['back','left','right']}
        for direction,s in successful.items():
            if s:
                for h in [.01,.02]:entries.append(save(run(ip,direction,s['kind'],s['amplitude_rad'],s['hold_s'],h,.08),f"step{int(h*1000)}-{direction}.json"))
    else:
        entries=[]
        cases={'back':[(k,a) for k in ['ankle','hip','combined'] for a in [-.65,.65]],'left':[(k,a) for k in ['roll','fold','combined'] for a in [-.5,.5]],'right':[(k,a) for k in ['roll','fold','combined'] for a in [-.5,.5]]}
        for direction,params in cases.items():
            for i,(kind,amp) in enumerate(params):entries.append(save(run(ip,direction,kind,amp),f'flat-{direction}-{i+1}.json'))
    (HERE/'traces/summary.json').write_text(json.dumps({'schema':'microduck-directed-fall-summary/v1','source_gate':source,'episodes':entries},indent=2)+'\n')
if __name__=='__main__':main()
