#!/usr/bin/env python3
"""Source-aware Stanford Pupper v3 3-axis teleop, run and jump CPU probes.

Uses the existing pinned Stanford pretrained policy + exact pinned MJCF through
infer_pupper.py. This is a CPU MuJoCo reproduction, not native Stanford MJLab
or a hardware-safe controller. Commands are interpreted as ROS cmd_vel (m/s,
rad/s) plus game-mode run/jump and desired roll/pitch pose. No fake root motion.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from infer_pupper import net, gait_ref, obs, load_policy, POLICY_SHA256
from reproduce_pupper import load_model, read_pinned_xml, contact_snapshot, FOOT_GEOMS, TORQUE_LIMIT, PUPPER_JOINTS
from history_contract import advance_history

DT = .004
POLICY_DECIMATION = 5


def wrap_to_pi(angle: float) -> float:
    wrapped = (angle + math.pi) % (2 * math.pi)
    return math.pi if wrapped == 0.0 and angle > 0.0 else wrapped - math.pi


def quaternion_yaw(q: np.ndarray) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class HeadingHold:
    """Stanford's command_filter.hpp heading hold, with pinned JSON params."""
    def __init__(self, params: dict):
        self.p = params
        self.target = 0.0
        self.prev_active = False

    def apply(self, heading: float, vx: float, vy: float, wz: float) -> float:
        active = (abs(wz) < self.p['yaw_threshold'] and
                  math.hypot(vx, vy) > self.p['walk_threshold'])
        if active and not self.prev_active:
            self.target = heading
        self.prev_active = active
        if not active:
            return wz
        return float(np.clip(self.p['kp'] * wrap_to_pi(self.target - heading),
                             -self.p['clip'], self.p['clip']))


def tilt_target(roll: float, pitch: float) -> np.ndarray:
    """cmd_pose quaternion inverse rotates world up (matches Stanford C++)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    # Rz(yaw) * Ry(pitch) * Rx(roll): transpose(R) @ world-z
    return np.array([-sp, cp * sr, cp * cr], dtype=np.float32)


def ref_with_slot(p: dict, phase_t: float, cmd: tuple, slot: float|None, hop: bool) -> np.ndarray:
    ref = gait_ref(p, phase_t, *cmd).copy()
    if slot is None:
        return ref
    j = p['jump_slot']
    active = j['hop_play_s'] if hop else j['active_s']
    elapsed = phase_t - slot
    if not (0 <= elapsed < active):
        return ref
    table = np.asarray(j['hop_table'] if hop else j['jump_table'], dtype=np.float64)
    if hop:
        f = j['hop_reach_frequency']
        offset = (slot*f-math.floor(slot*f))/f
        frac = float(np.clip((offset+elapsed)/j['hop_table_s'],0,1))
        pos = frac*(len(table)-1)
        i0 = min(max(math.floor(pos),0),len(table)-1)
        i1 = min(i0+1,len(table)-1)
    else:
        pos = float(np.clip(elapsed / j['playback_s'],0,1))*len(table)
        i0 = math.floor(pos)%len(table)
        i1 = (i0+1)%len(table)
    a = pos-math.floor(pos)
    captured=(1-a)*table[i0]+a*table[i1]
    blend=float(np.clip(min(elapsed,active-elapsed)/j['cross_fade_s'],0,1))
    return (blend*(captured-np.asarray(p['default_joint_pos']))+(1-blend)*ref).astype(np.float32)


def select_segment(schedule: list[dict], t: float) -> dict:
    i = 0
    for j,seg in enumerate(schedule):
        if t >= float(seg['t'])-1e-10:
            i=j
        else:
            break
    return schedule[i]


def validate_schedule(schedule: list[dict], duration: float) -> None:
    if not isinstance(schedule,list) or not schedule or float(schedule[0]['t'])!=0:
        raise ValueError('non-empty schedule must start at t=0')
    if duration <= 0 or duration > 120 or duration/DT != round(duration/DT):
        raise ValueError('duration must be a multiple of 4ms, <=120s')
    prior=-1
    for seg in schedule:
        t=float(seg['t'])
        if t <= prior or t>=duration or round(t/DT)*DT!=t:
            # allow small float arithmetic relative precision; below checked by tolerance.
            if not (t > prior and t < duration and math.isclose(round(t/DT)*DT,t,abs_tol=1e-10)):
                raise ValueError('segment times must increase at 4ms boundary')
        prior=t
        for name in ('vx','vy','wz','roll','pitch'):
            if not math.isfinite(float(seg.get(name,0))):
                raise ValueError('non-finite command')
        if abs(float(seg.get('roll',0)))>math.pi/4 or abs(float(seg.get('pitch',0)))>math.pi/4:
            raise ValueError('pose command over local research guard')


def rollout(p: dict, weights: list, schedule: list[dict], duration: float=9.0,
            hold_enabled: bool=True, delay: int=4, frameskip: int=5) -> dict:
    validate_schedule(schedule,duration)
    model=load_model(read_pinned_xml(ROOT/'upstream/pupper_v3.xml'))
    data=mujoco.MjData(model)
    mujoco.mj_resetData(model,data)
    data.qpos[:3]=[0,0,.18]
    data.qpos[3:7]=[1,0,0,0]
    baseline=np.asarray(p['default_joint_pos'],dtype=np.float64)
    data.qpos[7:19]=baseline
    data.ctrl[:]=baseline
    mujoco.mj_forward(model,data)
    hold=HeadingHold(p['heading_hold'])
    hist=np.zeros((20,48),dtype=np.float32)
    hist[:,5]=-1.
    last_action=np.zeros(12,dtype=np.float32)
    low=np.asarray(p['joint_lower_limits'],dtype=np.float64)
    high=np.asarray(p['joint_upper_limits'],dtype=np.float64)
    action_scale=float(p['action_scale'])
    pending=deque([baseline.copy() for _ in range(delay+1)],maxlen=delay+1)
    servo_target=baseline.copy()
    steps=int(round(duration/DT))
    base=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    contact_hist=Counter()
    frames=[]
    commands=[]
    slot_starts=[]
    slot_hop_latch={}
    prev_trigger=False
    jump_events=[]
    hold_corrections=[]
    max_torque=0.0
    saturated_steps=0
    guard_steps=0
    first_fall_t_s=None
    max_tilt=0.0
    min_z=float('inf')
    peak_positive_z=0.0
    failed=False
    for k in range(steps):
        t=k*DT
        if k%POLICY_DECIMATION==0:
            seg=select_segment(schedule,t)
            raw=[float(seg.get(x,0.)) for x in ('vx','vy','wz')]
            clipped=[float(np.clip(raw[i],*p['command_clip'][name])) for i,name in enumerate(('vx','vy','wz'))]
            run=bool(seg.get('run',False))
            cap=p['jump_slot']['run_speed_cap'] if run else p['jump_slot']['walk_speed_cap']
            clipped[0]=float(np.clip(clipped[0],-cap,cap))
            clipped[1]=float(np.clip(clipped[1],-cap,cap))
            phase_t=t-1.0
            fade=float(np.clip(phase_t,0,1.0))
            triggered=bool(seg.get('jump',False))
            if t<1.0:
                # Source's init phase (at t<1, neutral servo output and no inference).
                servo_target=baseline.copy()
                command=(0.0,0.0,0.0)
                prev_trigger=False
            else:
                if triggered and not prev_trigger and fade>=1:
                    grid=float(p['jump_slot']['grid_s'])
                    busy=float(p['jump_slot']['busy_s'])
                    start=math.ceil((phase_t+1e-6)/grid)*grid
                    while any(s<=start<s+busy for s in slot_starts):
                        start=math.ceil((max([s+busy for s in slot_starts if s<=start<s+busy])+1e-6)/grid)*grid
                    slot_starts.append(start)
                    slot_hop_latch[start]=None
                    jump_events.append({'trigger_t':round(t,5),'scheduled_t':round(start+1,5),'grid_phase_s':round(start,5)})
                prev_trigger=triggered
                wz=float(clipped[2])
                if hold_enabled:
                    wz=hold.apply(quaternion_yaw(data.qpos[3:7]),float(clipped[0]),float(clipped[1]),wz)
                command=(float(clipped[0]),float(clipped[1]),wz)
                if abs(wz-clipped[2])>1e-7:
                    hold_corrections.append({'t':round(t,5),'wz':round(wz,6)})
                active_slot=None
                active_hop=False
                for s in slot_starts:
                    if phase_t>=s:
                        if slot_hop_latch[s] is None:
                            slot_hop_latch[s]=(command[0]>=float(p['jump_slot']['hop_speed']))
                        is_hop=slot_hop_latch[s]
                        max_window=(p['jump_slot']['hop_play_s'] if is_hop else p['jump_slot']['active_s'])
                        if phase_t-s<max_window:
                            active_slot=s
                            active_hop=is_hop
                frame=obs(data,model,last_action,command,phase_t,p)
                frame[9:12]=tilt_target(float(seg.get('roll',0)),float(seg.get('pitch',0)))
                frame[36:48]=ref_with_slot(p,phase_t,command,active_slot,active_hop)
                hist[0]=frame
                act=net(weights,hist.flatten())
                scaled=(fade*act).astype(np.float32)
                servo_target=np.clip(baseline+(scaled*action_scale),low,high)
                last_action=scaled
                advance_history(hist,last_action)
                commands.append({'t':round(t,4),'requested':raw,'emitted':list(command),
                                 'run':run,'jump':triggered,'slot_start':active_slot,
                                 'hop':active_hop,'roll':float(seg.get('roll',0)),
                                 'pitch':float(seg.get('pitch',0))})
            if not np.isfinite(servo_target).all():
                raise ValueError('non-finite servo target')
        pending.append(servo_target.copy())
        data.ctrl[:]=pending[0]
        mujoco.mj_step(model,data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            failed=True
            break
        torque=float(np.max(np.abs(data.actuator_force)))
        max_torque=max(max_torque,torque)
        saturated_steps+=int(torque>=TORQUE_LIMIT-1e-5)
        tilt=math.degrees(math.acos(float(np.clip(data.xmat[base].reshape(3,3)[2,2],-1,1))))
        z=float(data.qpos[2])
        min_z=min(min_z,z)
        peak_positive_z=max(peak_positive_z,z)
        max_tilt=max(max_tilt,tilt)
        fell=(tilt>45.0 or z<0.05)
        guard_steps+=int(fell)
        if fell and first_fall_t_s is None:
            first_fall_t_s=round(float(data.time),6)
        if (k+1)%frameskip==0 or k+1==steps:
            loads=contact_snapshot(model,data)
            n=int(sum(bool(v>0.2) for v in loads.values()))
            contact_hist[n]+=1
            frames.append({'t':round(float(data.time),4), 'qpos':data.qpos.round(8).tolist(),
                'foot_Fz':{a:round(v,3) for a,v in loads.items()},
                'feet_loaded':n,'tilt_deg':round(tilt,3),
                'max_torque_Nm':round(torque,4),
                'cmd':list(command)})
    if not frames:
        raise RuntimeError('no saved frames')
    heading=[quaternion_yaw(np.asarray(row['qpos'][3:7])) for row in frames]
    unwrapped=np.unwrap(np.asarray(heading))
    jump_time=[x['scheduled_t'] for x in jump_events]
    initial=np.asarray(frames[0]['qpos'][:3]);final=np.asarray(frames[-1]['qpos'][:3])
    results={
       'duration_s':duration,'executed_steps':k+1,'frames':len(frames),'complete':(k+1)==steps and not failed,
       'world_dx_m':float(round(final[0]-initial[0],6)),
       'world_dy_m':float(round(final[1]-initial[1],6)),
       'world_final_z_m':float(round(final[2],6)),
       'net_yaw_deg':float(round(math.degrees(unwrapped[-1]-unwrapped[0]),4)),
       'min_z_m':float(round(min_z,6)),'max_z_m':float(round(peak_positive_z,6)),
       'max_tilt_deg':float(round(max_tilt,4)),'fall_guard_steps':guard_steps,
       'first_fall_t_s':first_fall_t_s,
       'peak_torque_Nm':float(round(max_torque,6)),
       'saturation_fraction':float(round(saturated_steps/(k+1),6)),
       'foot_load_histogram':{str(n):int(contact_hist[n]) for n in range(5)},
       'heading_hold_correction_ticks':len(hold_corrections),
       'jump_scheduled_count':len(jump_events),
       'jump_events':jump_events,
       'hop_active_ticks':sum(bool(c['hop']) for c in commands),
       'status':'FALL' if guard_steps>0 else ('INCOMPLETE' if failed else 'NO_FALL_GUARD'),
    }
    return {'schema':'alfred-pupper-v3-full-teleop-physical/v1',
           'policy_sha256':POLICY_SHA256,'runtime':{'mujoco':mujoco.__version__,'numpy':np.__version__},
           'source':'Stanford pinned deploy JSON, independent controller recreation (not C++/MJLab parity)',
           'servo_delay_physics_steps':delay,'heading_hold':hold_enabled,
           'schedule':schedule,'summary':results,'frames':frames,'policy_commands':commands}


def build_scenarios():
    def case(name, vx=0.,vy=0.,wz=0., duration=9.,run=False):
        return {'name':name,'duration':duration,'schedule':[
            {'t':0}, {'t':1,'vx':vx,'vy':vy,'wz':wz,'run':run},
            {'t':duration-1.0} ]}
    cases=[case('stand',duration=9.),case('forward',vx=.2),
           case('reverse',vx=-.2), case('strafe_left',vy=.15),
           case('strafe_right',vy=-.15),case('yaw_ccw',wz=.45),case('yaw_cw',wz=-.45),
           case('diagonal',vx=.16,vy=.12),case('arc_left',vx=.16,wz=.35),
           case('arc_right',vx=.16,wz=-.35),case('mixed_3axis',vx=.16,vy=.1,wz=.25),
           case('run_forward',vx=.65,run=True,duration=10.)]
    cases.append({'name':'realtime_switch','duration':20.,'schedule':[
        {'t':0}, {'t':1,'vx':.2}, {'t':4,'vy':.15},{'t':7,'wz':.45},
        {'t':10,'vx':-.2},{'t':13,'vx':.14,'vy':-.12,'wz':-.25},
        {'t':16,'vx':.08,'wz':.15},{'t':19}]})
    cases.append({'name':'pose_roll_pitch','duration':11.,'schedule':[
        {'t':0}, {'t':1,'roll':.12}, {'t':4,'pitch':.12},
        {'t':7,'vx':.12,'roll':.08,'pitch':-.08}, {'t':10}]})
    cases.append({'name':'jump_stand','duration':9.,'schedule':[
        {'t':0},{'t':1},{'t':3,'jump':True},{'t':3.20},{'t':8}]})
    cases.append({'name':'hop_running','duration':12.,'schedule':[
        {'t':0},{'t':1,'vx':.7,'run':True},
        {'t':4.12,'vx':.7,'run':True,'jump':True},
        {'t':4.40,'vx':.7,'run':True},{'t':11}]})
    return cases


def build_extended_scenarios():
    """Core scenarios + axis/speed sweeps, rapid joystick and heading-hold A/B."""
    cases=build_scenarios()
    for val in [-.15,.15,-.25,.25,-.35,.35]:
        label=f"yaw_{val:+.2f}".replace('+','p').replace('-','m').replace('.','_')
        cases.append({'name':label,'duration':8.,'schedule':[
            {'t':0},{'t':1,'wz':val},{'t':7}]})
    for val in [-.08,-.12,-.16,-.20]:
        label=f"reverse_{val:+.2f}".replace('+','p').replace('-','m').replace('.','_')
        cases.append({'name':label,'duration':8.,'schedule':[
            {'t':0},{'t':1,'vx':val},{'t':7}]})
    for val in [.50,.52,.55,.60]:
        label=f"run_{val:.2f}".replace('.','_')
        cases.append({'name':label,'duration':8.,'schedule':[
            {'t':0},{'t':1,'vx':val,'run':True},{'t':7}]})
    for val in [.51,.55]:
        label=f"hop_early_{val:.2f}".replace('.','_')
        cases.append({'name':label,'duration':8.,'schedule':[
            {'t':0},{'t':1,'vx':val,'run':True},
            {'t':2,'vx':val,'run':True,'jump':True},
            {'t':2.20,'vx':val,'run':True},{'t':7}]})
    rapid=[{'t':0},{'t':1,'vx':.15}]
    shifts=[{'vy':.12},{'wz':.2},{'vx':-.12},{'vy':-.12},{'wz':-.2},
            {'vx':.15,'vy':.08},{'vx':.1,'wz':.15},{'wz':0},
            {'vx':.15},{'vy':.12},{'vx':-.12,'vy':-.08,'wz':-.12},{'vx':0}]
    for i,command in enumerate(shifts):
        rapid.append({'t':round(1.52+i*.52,3),**command})
    rapid.append({'t':9})
    cases.append({'name':'rapid_joystick','duration':10.,'schedule':rapid})
    cases.append({'name':'clip_limits','duration':5.,'schedule':[
        {'t':0},{'t':1,'vx':3.,'vy':1.,'wz':3.},{'t':2}]})
    for original in ('reverse','forward','realtime_switch','yaw_ccw','jump_stand'):
        found=next(c for c in cases if c['name']==original)
        cases.append({**found,'name':original+'_no_hold','hold_enabled':False})
    return cases


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--policy',type=Path,default=ROOT/'default_policy.json')
    ap.add_argument('--output-dir',type=Path,default=ROOT/'full_command_results')
    ap.add_argument('--case',default='all')
    ap.add_argument('--suite',choices=['core','extended'],default='core')
    ap.add_argument('--no-heading-hold',action='store_true')
    args=ap.parse_args()
    p,w=load_policy(args.policy)
    cases=build_extended_scenarios() if args.suite=='extended' else build_scenarios()
    if args.case!='all':
        cases=[x for x in cases if x['name']==args.case]
        if not cases:raise SystemExit(f'unknown scenario {args.case}')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    receipts=[]
    for case in cases:
        v=rollout(p,w,case['schedule'],case['duration'],
                  hold_enabled=not args.no_heading_hold and case.get('hold_enabled',True))
        name=case['name']+('_no_hold' if args.no_heading_hold and case.get('hold_enabled',True) else '')
        f=args.output_dir/f'{name}.json'
        f.write_text(json.dumps(v,separators=(',',':'))+'\n')
        digest=hashlib.sha256(f.read_bytes()).hexdigest()
        receipts.append({'scenario':name,'file':f.name,'sha256':digest,'summary':v['summary']})
        print(name,json.dumps(v['summary'],ensure_ascii=False))
    index={'schema':'alfred-pupper-full-command-suite/v1','origin':'owner pinned source',
            'simulator':'MuJoCo CPU only (no RL training, no hardware)',
            'command_contract':'vx,vy m/s; wz rad/s; run, jump, roll and pitch in radians',
            'scenario_count':len(receipts),'scenarios':receipts}
    indexfile=args.output_dir/'index.json'
    indexfile.write_text(json.dumps(index,ensure_ascii=False,indent=2)+'\n')
    print('INDEX',indexfile)

if __name__=='__main__': main()
