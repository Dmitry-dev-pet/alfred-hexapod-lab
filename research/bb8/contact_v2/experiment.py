"""Reproducible contact diagnostic. All forces act at wheels, not at shell."""
from __future__ import annotations
import argparse, hashlib, json, math, time
from pathlib import Path
import numpy as np
import mujoco
from model import make_model

def quat_angle(q):
    w,x,y,z=q
    return math.atan2(2*(w*y+x*z),1-2*(y*y+x*x))

def run(case='pulse',duration=6.,level=3,dt=.001,friction=.8,wall=.003,contact=True,parts=None,fit_translation=None,allow_assumed_mass=False):
    xml,meta=make_model(level=level,timestep=dt,friction=friction,wall=wall,wheel_contact=contact,parts=parts,fit_translation=fit_translation,allow_assumed_mass=allow_assumed_mass)
    model=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(model)
    shell=model.body('shell').id;chassis=model.body('chassis').id
    shell_q=model.jnt_qposadr[model.joint('shell_free').id]
    chassis_q=model.jnt_qposadr[model.joint('chassis_free').id]
    wheel_vel=[model.jnt_dofadr[model.joint(s+'_axle').id] for s in ('left','right')]
    # Bounded first experiment deliberately has no invented calibrated balance controller.
    steps=round(duration/dt);frames=[];counts={'left':0,'right':0,'floor':0}
    max_pen=0.;max_force=0.;max_torque=0.;escapes=0;contact_steps=0
    cavity_steps=0;side_steps={s:0 for s in ('left','right')};max_chassis_angle=0.;max_cad_radius=0.;max_any_pen=0.;max_internal_force=0.
    names=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) for i in range(model.ngeom)]
    t0=time.monotonic();force=np.zeros(6)
    after_origin=None;peak_distance=0.;peak_speed=0.;max_energy=0.;max_axle_rpm=0.;velocity_integral=0.
    for k in range(steps+1):
        t=k*dt
        torque=0. if t<1. or t>=4. else (.006 if t<2.5 else -.006)
        if case=='coast':torque=0.
        ctrl=np.array([torque,torque])
        if case=='turn':ctrl[1]*=-1
        if case=='brake':
            if 2.5<=t<4.:ctrl=np.clip(-.002*d.qvel[wheel_vel],-.02,.02)
            elif t>=4.:ctrl[:]=0
        if case in {'hold','forward','reverse','yaw-left','yaw-right'}:
            pitch=quat_angle(d.qpos[chassis_q+3:chassis_q+7])
            cq=model.jnt_dofadr[model.joint('chassis_free').id]
            mat=d.xmat[chassis].reshape(3,3)
            forward=float(d.qvel[:3]@mat[:,0])
            desired=(.03 if case=='forward' else -.03 if case=='reverse' else 0.) if 1<=t<3 else 0.
            error=desired-forward
            velocity_integral=float(np.clip(velocity_integral+error*dt,-.12,.12))
            reference=float(np.clip(-.7*error-.2*velocity_integral,-.12,.12))
            common=.18*(pitch-reference)+.02*d.qvel[cq+4]
            yaw_goal=(.2 if case=='yaw-left' else -.2 if case=='yaw-right' else 0.) if 1<=t<3 else 0.
            differential=.01*(yaw_goal-d.qvel[cq+5])
            ctrl=np.clip([common+differential,common-differential],-.02,.02)
        demand=ctrl.copy()
        # Explicit hypothetical DC motor back-EMF curve (20mNm, 200rpm).
        powered=case in {'hold','forward','reverse','yaw-left','yaw-right'} or (case!='coast' and 1<=t<4)
        ctrl=np.clip(demand-.02/(200*math.pi/30)*d.qvel[wheel_vel],-.02,.02) if powered else np.zeros(2)
        d.ctrl[:]=ctrl
        mujoco.mj_forward(model,d)
        max_torque=max(max_torque,float(abs(d.actuator_force).max()))
        present=set();floor=False;local_pen=0.;local_force=0.;nwheel=0
        for ci in range(d.ncon):
            c=d.contact[ci];a=names[c.geom1];b=names[c.geom2]
            if {a,b}=={'floor','shell_outer'}:floor=True
            if a.startswith('cavity') or b.startswith('cavity'):
                max_any_pen=max(max_any_pen,-float(c.dist));mujoco.mj_contactForce(model,d,ci,force)
                max_internal_force=max(max_internal_force,float(np.linalg.norm(force[:3])))
            for side in ('left','right'):
                if (a.startswith('cavity') and b.startswith(side+'_ring')) or (b.startswith('cavity') and a.startswith(side+'_ring')):
                    present.add(side);nwheel+=1;local_pen=max(local_pen,-float(c.dist))
                    mujoco.mj_contactForce(model,d,ci,force);local_force=max(local_force,float(np.linalg.norm(force[:3])))
        if present:cavity_steps+=1
        for side in present:side_steps[side]+=1
        if floor:counts['floor']+=1
        max_pen=max(max_pen,local_pen);max_force=max(max_force,local_force)
        rel=d.xpos[chassis]-d.xpos[shell]
        escaped=np.linalg.norm(rel)>R_ESCAPE
        escapes+=int(escaped)
        q=d.qpos[chassis_q:chassis_q+7]
        angle=quat_angle(q[3:]);max_chassis_angle=max(max_chassis_angle,abs(angle))
        if k==round(1/dt):after_origin=d.qpos[shell_q:shell_q+3].copy()
        if after_origin is not None:peak_distance=max(peak_distance,float(np.linalg.norm(d.qpos[:2]-after_origin[:2])))
        speed=float(np.linalg.norm(d.qvel[:2]));peak_speed=max(peak_speed,speed)
        max_axle_rpm=max(max_axle_rpm,float(np.max(np.abs(d.qvel[wheel_vel]))*30/math.pi))
        if k%max(1,round(.02/dt))==0:
            if parts:
                for cid,points in parts.items():
                    group=next((s for s,ids in {'left':{'c07','c09','c11','c28','c12','c46'},'right':{'c08','c10','c16','c29','c13','c47'}}.items() if cid in ids),'chassis')
                    body=model.body(group+'_wheel' if group!='chassis' else group).id
                    local=points*.001
                    if group!='chassis':
                        axle=np.array([-.00313716624742838,0.,-.03922508767425143])
                        from model import RINGS
                        axle[1]=sum(p[1] for p in RINGS[group])/2
                        local=local-axle
                    world=local@d.xmat[body].reshape(3,3).T+d.xpos[body]
                    max_cad_radius=max(max_cad_radius,float(np.linalg.norm(world-d.xpos[shell],axis=1).max()))
            frames.append({'t':round(t,8),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),
                           'ctrl':ctrl.tolist(),'actuator_force':d.actuator_force.tolist(),
                           'wheel_contact_sides':sorted(present),'floor_contact':floor,'wheel_contacts':nwheel,
                           'max_wheel_penetration_m':local_pen,'max_contact_force_N':local_force,
                           'chassis_pitch_rad':angle,'chassis_center_from_shell_m':rel.tolist()})
        if k<steps:mujoco.mj_step(model,d)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():raise RuntimeError('Non-finite integration')
        if max(d.warning.number)>0 or abs(d.time-min(k+1,steps)*dt)>1e-7:
            raise RuntimeError('Engine warning/reset or time drift; trace is invalid')
    summary={'case':case,'steps':steps,'runtime_s':time.monotonic()-t0,'sampled_frames':len(frames),
             'final_shell_xyz_m':d.qpos[:3].tolist(),'shell_xy_after_settle_m':(d.qpos[:2]-after_origin[:2]).tolist(),
             'peak_xy_displacement_after_settle_m':peak_distance,'peak_speed_xy_m_s':peak_speed,
             'final_speed_xy_m_s':float(np.linalg.norm(d.qvel[:2])),
             'max_chassis_pitch_deg':math.degrees(max_chassis_angle),'max_axle_rpm':max_axle_rpm,
             'wheel_contact_fraction':{s:side_steps[s]/(steps+1) for s in side_steps},
             'floor_contact_fraction':counts['floor']/(steps+1),'max_wheel_penetration_m':max_pen,
             'max_single_contact_force_N':max_force,'max_abs_actuator_torque_Nm':max_torque,
             'max_any_internal_contact_penetration_m':max_any_pen,'max_any_internal_single_contact_force_N':max_internal_force,
             'max_source_CAD_radius_from_shell_center_m_sampled':max_cad_radius,
             'max_source_CAD_intrusion_into_wall_m_sampled':max(0.,max_cad_radius-(.1-wall)),
             'escaped_chassis_steps':escapes,'warning_counts':d.warning.number.tolist(),
             'final_chassis_from_shell_m':rel.tolist()}
    return xml,{'schema':'alfred-bb8-contact-diagnostic/v2','engine':'MuJoCo','engine_version':mujoco.__version__,
                'case':case,'model':meta,'duration_s':duration,'timestep_s':dt,'summary':summary,'frames':frames}

R_ESCAPE=.13

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--case',default='pulse')
    ap.add_argument('--level',type=int,default=3);ap.add_argument('--seconds',type=float,default=6.)
    ap.add_argument('--dt',type=float,default=.001);ap.add_argument('--friction',type=float,default=.8)
    ap.add_argument('--wall',type=float,default=.003);ap.add_argument('--no-contact',action='store_true')
    ap.add_argument('--source',type=Path);ap.add_argument('--tire',type=Path);ap.add_argument('--fit',type=Path)
    ap.add_argument('--allow-assumed-mass',action='store_true')
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    parts=fit=None
    if a.source:
        from audit_geometry import load_parts
        parts,_=load_parts(a.source,a.tire)
        fit=np.array(json.loads(a.fit.read_text())['fit']['rigid_translation_from_source_pose_mm'])*.001
    xml,r=run(a.case,a.seconds,a.level,a.dt,a.friction,a.wall,not a.no_contact,parts,fit,allow_assumed_mass=a.allow_assumed_mass)
    name=a.case+f'-L{a.level}-mu{a.friction:g}-wall{a.wall:g}'+('-no-contact' if a.no_contact else '')
    (a.out/(name+'.xml')).write_text(xml);(a.out/(name+'.json')).write_text(json.dumps(r,separators=(',',':'))+'\n')
    print(json.dumps(r['summary'],indent=2))

if __name__=='__main__':main()
