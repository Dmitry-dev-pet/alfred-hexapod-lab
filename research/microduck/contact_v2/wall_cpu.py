#!/usr/bin/env python3
"""Close-wall backward fall + stock recovery; only the obstacle placement changes."""
import argparse,contextlib,hashlib,io,json,math,sys
from pathlib import Path
import mujoco as mj,numpy as np
from measure_clearance import f,shape_data,rear_extent
WORK=f.WORK;e=f.e;setup=f.setup;target_pose=f.target_pose;check_direction=f.check_direction;sha=f.sha
HERE=Path(__file__).resolve().parent
def run(ip,direction,kind,amp,hold=1.2,height=0.,edge=.08):
    m,d,c,p,sq,bq,yaw=setup(ip,height,edge,direction)
    js=m.actuator_trnid[:,0];lo=m.jnt_range[js,0]+.035;hi=m.jnt_range[js,1]-.035
    shapes=shape_data(m);initial_wall_contacts=0;initial_wall_pen=0.;rear_contact_s=0.;vertical_rear_s=0.;rear_top_s=0.;first_rear=None;rear_peak=0.;before_gap=None
    root=1;floor=0;step=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'step_top')
    feet={s:mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,s+'_foot_collision') for s in ['left','right']}
    baseRot=d.xmat[root].reshape(3,3).copy();frames=[];start=target=None;handback=None;peak=0.;peakself=0.;peakenv=0.;force=0.;torque=0.;limex=0.;reset=False;targetclip=0.;bodyground=False
    for k in range(500):
        t=k*.02;mj.mj_forward(m,d)
        if k==50:
            before_gap=edge-rear_extent(m,d,shapes,height,True)[0]
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
            rear_this=vertical_this=top_this=False
            for i in range(d.ncon):
                ct=d.contact[i];pair=(ct.geom1,ct.geom2);w=np.zeros(6);mj.mj_contactForce(m,d,i,w)
                if floor in pair or step>=0 and step in pair:
                    peakenv=max(peakenv,-float(ct.dist));force=max(force,float(np.linalg.norm(w[:3])))
                    other=pair[1] if pair[0] in [floor,step] else pair[0]
                    if step in pair:
                        if t<1:
                            initial_wall_pen=max(initial_wall_pen,-float(ct.dist));initial_wall_contacts+=int(w[0]>.05)
                        elif m.geom_bodyid[other]==root and w[0]>.05:
                            if first_rear is None:first_rear=float(d.time)
                            rear_peak=max(rear_peak,float(w[0]))
                            # Count support only during the commanded fall and 0.5 s thereafter.
                            if t<1+hold+.5:
                                # Multiple simultaneous contacts count once per physics substep below.
                                rear_this=True
                                if abs(ct.frame[0])>.85:vertical_this=True
                                if abs(ct.frame[2])>.85 and ct.pos[2]>=height-.002:top_this=True
                    if t>=1 and tilt>45 and w[0]>.05 and other not in feet.values():bodyground=True
                else:peakself=max(peakself,-float(ct.dist))
            rear_contact_s+=.001*int(rear_this);vertical_rear_s+=.001*int(vertical_this);rear_top_s+=.001*int(top_this)
        loads={s:0. for s in feet};top={s:0. for s in feet};contacts=[]
        for i in range(d.ncon):
            ct=d.contact[i];pair=(ct.geom1,ct.geom2);w=np.zeros(6);mj.mj_contactForce(m,d,i,w)
            contacts.append({'geoms':[int(x) for x in pair],'force_n':float(w[0]),'distance_m':float(ct.dist),'pos':ct.pos.tolist(),'normal':ct.frame[:3].tolist()})
            if abs(ct.frame[2])>.85:
                for s,g in feet.items():
                    if g in pair and (floor in pair or step>=0 and step in pair):loads[s]+=max(0.,float(w[0]))
                    if g in pair and step>=0 and step in pair and ct.pos[2]>=height-.001:top[s]+=max(0.,float(w[0]))
        frames.append({'t':round(float(d.time),6),'phase':phase,**e.bench.sample(m,d,0,0),'motor_target_rad':c.q_target.tolist(),'trunk_x_m':float(d.xpos[root,0]),'directional_lean_deg':lean,'foot_loads_n':loads,'step_loads_n':top,'foot_bounds':{s:e.foot_bounds(m,d,g) for s,g in feet.items()},'contacts':contacts})
    last=frames[-50:];warnings={str(i):int(w.number) for i,w in enumerate(d.warning) if w.number}
    standing=all(f['tilt_deg']<20 and .09<f['trunk_z_m']<.18+height and f['linear_speed_mps']<.06 and f['angular_speed_rps']<.7 and all(x>.05 for x in f['foot_loads_n'].values()) for f in last)
    on_top=bool(height and all(all(x>.05 for x in f['step_loads_n'].values()) and all(b['min'][0]>edge+.001 for b in f['foot_bounds'].values()) for f in last))
    direction_check=check_direction(frames,direction,yaw,hold)
    fallen=bool(peak>=65 and bodyground and direction_check['first_fall_in_requested_cone']);ok=bool(fallen and standing and not warnings and not reset)
    contact_success=bool(peak>=10 and rear_contact_s>=.02 and initial_wall_contacts==0 and initial_wall_pen<1e-6 and not warnings and not reset)
    summary={'rear_body_contact_s':rear_contact_s,'vertical_rear_body_contact_s':vertical_rear_s,'rear_body_top_contact_s':rear_top_s,'first_rear_body_contact_s':first_rear,'peak_rear_body_contact_n':rear_peak,'back_into_obstacle':contact_success,'initial_obstacle_loaded_substeps':initial_wall_contacts,'initial_obstacle_penetration_m':initial_wall_pen,'surface_gap_before_trigger_m':before_gap,**direction_check,'direction':direction,'kind':kind,'amplitude_rad':amp,'hold_s':hold,'height_m':height,'edge_x_m':edge,'initial_yaw_rad':yaw,'status':'PASS' if (contact_success and standing if height==.25 else on_top and standing) and not warnings and not reset else 'FAIL','fall_and_recover':ok,'directed_fall':fallen,'peak_directional_lean_deg':peak,'body_environment_contact_while_tilted':bodyground,'standing_final_1s':standing,'both_feet_on_step_final_1s':on_top,'step_goal_reached':bool(on_top and standing),'actual_handback_s':handback,'final_position_m':frames[-1]['qpos'][:3],'max_tilt_deg':max(f['tilt_deg'] for f in frames),'final_tilt_deg':frames[-1]['tilt_deg'],'max_self_contact_penetration_m':peakself,'max_environment_penetration_m':peakenv,'peak_environment_force_n':force,'peak_motor_torque_nm':torque,'max_motor_limit_overshoot_rad':max(0.,limex),'authored_target_clamp_rad':targetclip,'warnings':warnings,'physics_reset_detected':reset,'duration_s':10,'frames':500}
    return {'schema':'microduck-back-to-wall-physical/v1','stand_policy_sha256':sha(e.POLICY),'dt':.001,'control_dt':.02,'contact_profile':'firm_1ms','terrain':{'height_m':height,'edge_x_m':edge,'half_size_xy_m':[.6,.6],'center_xy_m':[edge+.6,0]},'summary':summary,'frames':frames}

def save(tr,name,condition):
    tr['condition']=condition;tr['summary']['condition']=condition
    path=HERE/'traces'/name;path.write_text(json.dumps(tr,separators=(',',':'),allow_nan=False,sort_keys=True)+'\n');entry={**tr['summary'],'trace_file':name,'trace_sha256':sha(path)};print(json.dumps(entry),flush=True);return entry

def main():
    mf=json.loads((WORK/'microduck-work/source-manifest.json').read_text());mf['_path']=str(WORK/'microduck-work/source-manifest.json');source=e.bench.source_gate(e.RL,e.BAM,mf);assert sha(e.POLICY)==e.bench.POLICY_SHA256
    ip=e.bench.upstream_inference(e.RL);ip.ort.InferenceSession=e.bench.cpu_session
    layout=json.loads((HERE/'clearance-layout.json').read_text());entries=[]
    cases=[(x['height_m'],x['edge_x_m'],'close') for x in layout['entries']]+[(.01,.08,'reference'),(.02,.08,'reference')]
    for h,edge,condition in cases:
        tr=run(ip,'back','hip',.65,1.2,h,edge);name=f"{condition}-h{round(h*1000)}.json";entries.append(save(tr,name,condition))
    (HERE/'traces/summary.json').write_text(json.dumps({'schema':'microduck-back-to-wall-summary/v1','source_gate':source,'clearance_layout_sha256':sha(HERE/'clearance-layout.json'),'episodes':entries},indent=2)+'\n')
if __name__=='__main__':main()
