"""CPU diagnostics of source jump commands with explicitly unknown timing/zero.

Only command sequencing is ported exactly. The v3 mass/torque/servo assumptions
are frozen; a sensitivity result is not an original robot reproduction.
"""
from __future__ import annotations
import argparse, collections, hashlib, json, math
from pathlib import Path
from .feedback_model import make_model
from .linkage_model import load_evidence, set_configuration
from .source_jump import JumpCommands, cad_targets, PERIODS_MS, ANCHORS, FIRMWARE_BLOB

HERE=Path(__file__).resolve().parent
PROTOCOL=HERE/'source_jump_protocol.json'

class JumpGate:
    """All-body contact loss, upward COM takeoff and clean stable landing."""
    def __init__(self,dt,command_s=1.5):
        self.dt=dt;self.command_s=command_s;self.before=collections.deque(maxlen=round(.1/dt))
        self.pre_supported=None;self.flight=None;self.flights=[];self.landing=False;self.clean=False;self.stable=0.;self.best=0.

    def update(self,*,t,ground,nonwheel,both_wheels,upright,clearance,com_vz):
        support=both_wheels and not nonwheel and upright
        if t<self.command_s-1e-10:self.before.append(support);return
        if self.pre_supported is None:self.pre_supported=len(self.before)==self.before.maxlen and all(self.before)
        air=not ground and clearance>.0001
        if air:
            if self.flight is None:
                self.flight={'start_s':t-self.dt,'upward_takeoff':com_vz>.05,
                             'pre_supported':bool(self.pre_supported),'takeoff_upright':upright,
                             'max_min_wheel_clearance_m':clearance,'takeoff_com_vz_m_s':com_vz}
            self.flight['max_min_wheel_clearance_m']=max(self.flight['max_min_wheel_clearance_m'],clearance)
            self.stable=0.
        elif self.flight is not None:
            f=self.flight;f['duration_s']=t-self.dt-f['start_s']
            f['qualified']=bool(f['upward_takeoff'] and f['pre_supported'] and f['takeoff_upright'] and f['duration_s']>=.02-1e-10 and f['max_min_wheel_clearance_m']>=.001)
            self.flights.append(f);self.landing=f['qualified'];self.clean=not nonwheel;self.flight=None;self.stable=0.
        if self.landing:
            self.clean&=not nonwheel
            if self.clean and support:self.stable+=self.dt;self.best=max(self.best,self.stable)
            else:self.stable=0.

    def result(self,t):
        flights=[dict(f) for f in self.flights]
        if self.flight is not None:
            f=dict(self.flight);f.update(duration_s=t-f['start_s'],qualified=False,unfinished=True);flights.append(f)
        return {'pre_jump_supported_100ms':bool(self.pre_supported),'flights':flights,
                'qualified_flight':any(f['qualified'] for f in flights),
                'stable_landing_s':self.best,'jump_pass':any(f['qualified'] for f in flights) and self.best>=2-1e-9}

def run(out,*,loop_ms=2,anchor_q=0.,jump=True):
    import mujoco
    import numpy as np
    if loop_ms not in PERIODS_MS or anchor_q not in ANCHORS:raise ValueError('Undeclared protocol setting')
    protocol=json.loads(PROTOCOL.read_text());dt=protocol['physics_timestep_s'];duration=protocol['duration_s']
    if mujoco.__version__!='3.14.0':raise ValueError('MuJoCo version mismatch')
    xml=make_model('nominal',timestep=dt);m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m)
    jid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)
    aid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,n)
    bid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n)
    sid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,n)
    gid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n)
    torso=bid('torso');floor=gid('floor');wheels={gid(s+'_wheel_geom') for s in ('left','right')}
    wheel_bodies=[bid(s+'_wheel') for s in ('left','right')]
    bodies=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,b) for b in range(1,m.nbody)]
    joints={mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,j):int(m.jnt_qposadr[j]) for j in range(m.njnt)}
    points=load_evidence()['sides']['cad_z_positive']['points_xy_mm']
    if anchor_q:
        set_configuration(m,d,points,anchor_q)
        # Nonzero anchor sensitivity starts with both wheel bottoms on the
        # same floor, rather than a drop that pretends to be a jump.
        d.qpos[2]+=.0185-min(float(d.xpos[b,2]) for b in wheel_bodies)
    # Preserve the exact frozen v3 initial state for the zero-anchor control.
    mujoco.mj_forward(m,d)
    fsm=JumpCommands();goals=cad_targets(fsm.positions,anchor_q_rad=anchor_q)
    events=[];frames=[];force=np.zeros(6);wheel_torque=0.;last_speed=0.;xref=0.;latched=False
    gate=JumpGate(dt);normal_steps=0;nonwheel_steps=0;max_pitch=max_roll=max_x=max_qvel=max_pen=max_closure=0.
    max_clear=-float('inf');peak_normal=0.;disabled=cuts=saturated_hip=0;peak_hip=peak_wheel=0.
    command_step=round(protocol['source_jump_command_s']/dt);period=loop_ms;steps=round(duration/dt)
    initial_com=None;peak_com_rise=0.;takeoff_reference=None
    max_hip_after=[-float('inf'),-float('inf')];hip_at_return=None;positive_hip_work=0.;peak_hip_power=0.
    for step in range(steps):
        t=float(d.time)
        if step%period==0:
            R=d.xmat[torso].reshape(3,3);pitch=math.atan2(-R[2,0],R[2,2]);roll=math.atan2(R[2,1],R[2,2]);gyro=float(d.sensor('imu_gyro').data[1])
            rate=sum(float(d.qvel[m.jnt_dofadr[jid('wheel_'+s)]]) for s in ('left','right'))/2
            odom=.0185*sum(float(d.qpos[m.jnt_qposadr[jid('wheel_'+s)]]) for s in ('left','right'))/2
            # Exact order and dimensionally matched source speed gate: source
            # speed is the sum of the two direction-normalized shaft rates.
            disable=abs(2*rate-last_speed)>10 or abs(2*rate)>50 or fsm.flag!=0
            last_speed=2*rate
            if disable:xref=odom
            raw=.15*pitch+.05*gyro+(0. if disable else .008*rate+.3*(odom-xref))
            latched|=max(abs(pitch),abs(roll))>math.radians(35)
            wheel_torque=0. if latched else float(np.clip(raw,-.03,.03))
            disabled+=int(disable)*period;cuts+=int(latched)*period
            packet=fsm.tick(previous_stop=jump and step==command_step,jump=jump and step>=command_step)
            goals=cad_targets(packet['positions'],anchor_q_rad=anchor_q)
            for event in packet['writes']:
                events.append({'t':t,'event':event,'flag_before':packet['flag_before'],'flag_after':packet['flag_after'],'positions':packet['positions'],'cad_targets_rad':goals[:]})
                if event=='return':hip_at_return=[float(d.qpos[m.jnt_qposadr[jid('hip_'+s)]]) for s in ('left','right')]
        for i,s in enumerate(('left','right')):
            j=jid('hip_'+s);q=float(d.qpos[m.jnt_qposadr[j]]);v=float(d.qvel[m.jnt_dofadr[j]])
            raw=3*(goals[i]-q)-.045*v
            limit=.14709975*(max(0,1-abs(v)/11.6355) if raw*v>0 else 1)
            d.ctrl[aid('a_hip_'+s)]=np.clip(raw,-limit,limit);saturated_hip+=int(abs(raw)>limit+1e-12)
            d.ctrl[aid('a_wheel_'+s)]=wheel_torque
        peak_hip=max(peak_hip,max(abs(float(d.ctrl[aid('a_hip_'+s)])) for s in ('left','right')));peak_wheel=max(peak_wheel,abs(wheel_torque))
        if step>=command_step:
            powers=[float(d.ctrl[aid('a_hip_'+s)])*float(d.qvel[m.jnt_dofadr[jid('hip_'+s)]]) for s in ('left','right')]
            positive_hip_work+=sum(max(0,p) for p in powers)*dt;peak_hip_power=max(peak_hip_power,sum(max(0,p) for p in powers))
        d.xfrc_applied[:]=0
        mujoco.mj_step(m,d);mujoco.mj_forward(m,d);mujoco.mj_subtreeVel(m,d)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():raise RuntimeError('Nonfinite physical state')
        R=d.xmat[torso].reshape(3,3);pitch=float(math.atan2(-R[2,0],R[2,2]));roll=float(math.atan2(R[2,1],R[2,2]))
        max_pitch=max(max_pitch,abs(pitch));max_roll=max(max_roll,abs(roll));max_x=max(max_x,abs(float(d.qpos[0])));max_qvel=max(max_qvel,float(max(abs(d.qvel))))
        closure=max(float(np.linalg.norm(d.site_xpos[sid(s+'_closure_calf')]-d.site_xpos[sid(s+'_closure_rod')])) for s in ('left','right'));max_closure=max(max_closure,closure)
        contacts=[];touching=set();normal=0.;nonwheel=False;pen=0.
        for ci in range(d.ncon):
            c=d.contact[ci]
            if floor not in (int(c.geom1),int(c.geom2)):continue
            other=int(c.geom2) if int(c.geom1)==floor else int(c.geom1)
            touching.add(other);mujoco.mj_contactForce(m,d,ci,force);normal+=max(0,float(force[0]));nonwheel|=other not in wheels;pen=max(pen,max(0,-float(c.dist)))
            contacts.append(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,other))
        # Support of a tilted finite cylinder along world Z, not its center
        # minus radius (which would incorrectly treat sideways wheels).
        clear=[]
        for s in ('left','right'):
            g=gid(s+'_wheel_geom');axis=d.geom_xmat[g].reshape(3,3)[:,2];az=min(1.,abs(float(axis[2])))
            extent=float(m.geom_size[g,1])*az+float(m.geom_size[g,0])*math.sqrt(max(0,1-az*az));clear.append(float(d.geom_xpos[g,2])-extent)
        clearance=min(clear);max_clear=max(max_clear,clearance);max_pen=max(max_pen,pen);peak_normal=max(peak_normal,normal);nonwheel_steps+=int(nonwheel);normal_steps+=int(bool(contacts))
        upright=max(abs(pitch),abs(roll))<math.radians(20) and not nonwheel
        com=float(d.subtree_com[torso,2]);com_vz=float(d.subtree_linvel[torso,2])
        if initial_com is None:initial_com=com
        if step==command_step:takeoff_reference=com
        if step>=command_step:
            for i,s in enumerate(('left','right')):max_hip_after[i]=max(max_hip_after[i],float(d.qpos[m.jnt_qposadr[jid('hip_'+s)]]))
        if takeoff_reference is not None:peak_com_rise=max(peak_com_rise,com-takeoff_reference)
        gate.update(t=float(d.time),ground=bool(contacts),nonwheel=nonwheel,both_wheels=wheels.issubset(touching),upright=upright,clearance=clearance,com_vz=com_vz)
        if step%20==0:
            frames.append({'t':float(d.time),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'ctrl':d.ctrl.tolist(),
                'body_poses':[[*d.xpos[bid(n)].tolist(),*d.xquat[bid(n)].tolist()] for n in bodies],
                'ground_contact':bool(contacts),'nonwheel_ground_contact':nonwheel,'contact_geoms':contacts,
                'ground_normal_force_n':normal,'pitch_rad':pitch,'roll_rad':roll,'closure_error_m':closure,
                'source_jump_flag':fsm.flag,'servo_command_positions':list(fsm.positions),'cad_targets_rad':goals[:],
                'wheel_clearance_m':clear,'com_z_m':com,'com_vz_m_s':com_vz})
    outcome=gate.result(float(d.time))
    if not jump:outcome['jump_pass']=False
    summary={'schema':'en01-fourbar-diagnostic/v1','experiment':'source_jump_v4','mode':'jump' if jump else 'stand',
        'controller_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'protocol_sha256':hashlib.sha256(PROTOCOL.read_bytes()).hexdigest(),
        'mujoco_version':mujoco.__version__,'duration_s':duration,'timestep_s':dt,'physics_steps':steps,'frame_count':len(frames),
        'total_mass_kg':float(sum(m.body_mass)),'mass_status':'UNMEASURED_v0_aggregate_redistributed',
        'hip_continuous_ceiling_nm':.14709975,'wheel_torque_nm':'ASSUMED_0.030','spring_model':'OMITTED_UNCONFIRMED',
        'model_profile':'nominal','model_sha256':hashlib.sha256(xml.encode()).hexdigest(),'geometry_source_blob':load_evidence()['source']['git_blob_sha1'],
        'source_firmware_blob':FIRMWARE_BLOB,'source_loop_period_ms_ASSUMED':loop_ms,'cad_anchor_q_at_height38_rad_ASSUMED':anchor_q,
        'source_jump_enabled':jump,'command_events':events,'source_jump_exactness':'command state machine only; MCU timing, absolute zero, servo transient dynamics and voltage-to-torque unknown',
        'max_closure_error_m':max_closure,'max_abs_pitch_deg':math.degrees(max_pitch),'max_abs_roll_deg':math.degrees(max_roll),
        'max_abs_x_m':max_x,'final_x_m':float(d.qpos[0]),'nonwheel_ground_steps':nonwheel_steps,'ground_contact_steps':normal_steps,
        'max_floor_penetration_m':max_pen,'max_abs_qvel':max_qvel,'max_min_wheel_clearance_m':max_clear,'peak_com_rise_after_command_m':peak_com_rise,
        'peak_ground_normal_force_n':peak_normal,'hip_saturated_fraction':saturated_hip/(2*steps),'peak_hip_torque_nm':peak_hip,'peak_wheel_torque_nm':peak_wheel,
        'max_hip_after_command_rad':max_hip_after,'hip_angles_at_return_rad':hip_at_return,'positive_hip_work_after_command_j':positive_hip_work,'peak_hip_mechanical_power_w':peak_hip_power,
        'disabled_distance_steps':disabled,'safety_wheel_cut_steps':cuts,'upright_6s':nonwheel_steps==0 and max(max_pitch,max_roll)<math.radians(20),
        'status':'REVIEW_ONLY_UNCALIBRATED_SOURCE_COMMAND_ADAPTATION',**outcome}
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    payload=json.dumps({'summary':summary,'body_names':bodies,'joint_qpos_addresses':joints,'frames':frames},allow_nan=False,separators=(',',':'))+'\n'
    if out.exists() and out.read_text()!=payload:raise ValueError('Immutable trace conflict')
    out.write_text(payload);return {**summary,'trace_sha256':hashlib.sha256(payload.encode()).hexdigest()}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--loop-ms',type=int,choices=PERIODS_MS,default=2);p.add_argument('--anchor-q',type=float,choices=ANCHORS,default=0);p.add_argument('--baseline',action='store_true')
    a=p.parse_args();print(json.dumps(run(a.out,loop_ms=a.loop_ms,anchor_q=a.anchor_q,jump=not a.baseline),indent=2))

if __name__=='__main__':main()
