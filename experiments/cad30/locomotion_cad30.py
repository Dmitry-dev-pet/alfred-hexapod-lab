"""Local tripod/wave gait experiment on 30-of-32 original Freenove CAD mesh model.

All 20 joint transforms, 18 leg actuators, group masses and 6 real tibia
contact meshes come from pinned upstream MJCF. Two missing visual meshes are
box proxies only. All outputs observed from free-base MuJoCo physics; no
externally translated body, no welded feet, no preanimated positions.
"""
from __future__ import annotations
from pathlib import Path
import json, math, sys
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from hexlab.stand_v04 import static_pose,hold_under_gravity
from hexlab.stand_geometry_v04 import load_legs,tip_world
from hexlab.step_one_leg_v04 import smoothstep,forward_from_geometry,solve_waypoint,floor_contacts
CODES=('ml','bl','br','mr','fr','fl')

def get_trajectory(model,orig,legs,forward,step,lift,duty,samples=36):
    """Build body-relative joint cyclic target table with off-line 3DOF IK."""
    ref=mujoco.MjData(model)
    period_phase=np.linspace(0,1,samples,endpoint=False)
    qplans={}
    errors={}
    for leg in legs:
        qadr=leg['qadrs']
        ref.qpos[:]=orig;ref.qvel[:]=0;mujoco.mj_forward(model,ref)
        toe0=tip_world(model,ref,leg).copy()
        qa0=orig[qadr].copy()
        row=[];rowerr=[]
        for phi in period_phase:
            if phi<duty:
                f=phi/duty; horiz=smoothstep(f)-.5; vertical=math.sin(math.pi*f)
            else:
                f=(phi-duty)/(1.-duty);horiz=.5-f;vertical=0.
            target=toe0+np.array([step*horiz*forward[0],step*horiz*forward[1],lift*vertical])
            # IK solved independently from fixed nominal standing base, so
            # target is relative to the body and does not prescribe body motion.
            ref.qpos[:]=orig
            q,err=solve_waypoint(model,ref,mujoco,leg,target,qa0)
            row.append(q); rowerr.append(float(err))
        qplans[leg['suffix']]=np.array(row)
        errors[leg['suffix']]=float(np.max(rowerr))
    return qplans,errors



def receipt_filename(gait: str, mass: str, *, friction: float = 1.1,
                     step: float = .035, lift: float = .020,
                     period: float = 2.4, seconds: float = 8.0) -> str:
    """Parameter-qualified JSON name; do not overwrite a different experiment.

    Keep historical default receipt names stable for existing evidence, but
    include every varied physical/control input in non-default filenames.
    """
    if gait not in ('tripod', 'wave') or mass not in ('lower', 'nominal', 'upper'):
        raise ValueError('Unknown gait or mass scenario')
    def tag(value: float, *, digits: int = 5) -> str:
        val=f'{value:.{digits}f}'.rstrip('0').rstrip('.')
        return val.replace('.', 'p')
    name=f'cad30_gait_{gait}_{mass}'
    for key,value,base,scale in (
        ('mu',friction,1.1,1),
        ('stepmm',step,.035,1000),
        ('liftmm',lift,.020,1000),
        ('periods',period,2.4,1),
        ('durations',seconds,8.,1),
    ):
        if abs(value-base)>1.e-9:
            name+=f'_{key}{tag(value*scale)}'
    return name+'.json'

def gait_run(gait='tripod',mass='nominal',step=.035,lift=.020,period=2.4,seconds=8.0,*,frames_hz=9.0,friction=1.1):
    assert gait in ('tripod','wave')
    path=ROOT/'source'/'mjcf'/f'scene_local30_{mass}.xml'
    model=mujoco.MjModel.from_xml_path(str(path))
    if not (.1<=friction<=2.0):raise ValueError('Outside tested friction sensitivity range')
    model.geom_friction[model.geom('floor').id,0]=friction
    for code in CODES:
        model.geom_friction[model.geom(f'tibia{code}_collision_0').id,0]=friction
    data=mujoco.MjData(model)
    legs=load_legs(model,data,mujoco)
    fit=static_pose(model,data,mujoco,legs,.145,.15)
    pose=data.qpos.copy()
    pre=hold_under_gravity(model,data,mujoco,pose,1.0)
    assert pre['stable_by_proxy_thresholds'],pre
    initial=data.qpos.copy()
    pose0=data.xpos[model.body('base_link_2').id].copy()
    forward=np.array(forward_from_geometry(legs))
    duty=.30 if gait=='tripod' else .17
    plan,errors=get_trajectory(model,initial,legs,forward,step,lift,duty)
    # Geometry limits are independent of real servo shaft travel/PWM.
    qmax=max(abs(float(v)) for p in plan.values() for v in p.flatten())
    motor_jids=np.array(model.actuator_trnid[:,0],dtype=int)
    qadrs=np.array([model.jnt_qposadr[j] for j in motor_jids],dtype=int)
    dadrs=np.array([model.jnt_dofadr[j] for j in motor_jids],dtype=int)
    q_ref=data.qpos[qadrs].copy()
    actuator_index_by_joint={int(j):i for i,j in enumerate(motor_jids)}
    moving={leg['suffix']:[actuator_index_by_joint[j] for j in leg['jids']] for leg in legs}
    base=model.body('base_link_2').id
    start_data_time=float(data.time)  # after one second of standing stabilization
    init_rot=data.xmat[base].reshape(3,3).copy();local_up=init_rot.T@np.array([0.,0.,1.])
    phases={c:(i%2)*.5 if gait=='tripod' else i/6 for i,c in enumerate(CODES)}
    tibias={c:model.geom('tibia'+c+'_collision_0').id for c in CODES}
    prev_frame=0.
    n=round(seconds/model.opt.timestep)
    results=[];state=[];saturation=0;max_tau=0.;fall=False
    sample_interval=max(1,int(round(.02/model.opt.timestep)))
    frame_interval=max(1,int(round((1/frames_hz)/model.opt.timestep)))
    bounds=np.array(model.actuator_ctrlrange)
    for k in range(n):
        t=k*model.opt.timestep
        ramp=smoothstep(min(1.,t/period))
        qtar=q_ref.copy()
        for c,leg in zip(CODES,legs):
            phase=(t/period +phases[c])%1
            qtable=plan[c]
            idx=phase*len(qtable)
            lo=int(math.floor(idx))%len(qtable);hi=(lo+1)%len(qtable)
            frac=idx-math.floor(idx)
            qa=(1-frac)*qtable[lo]+frac*qtable[hi]
            ii=moving[c]
            qtar[ii]=q_ref[ii]+ramp*(qa-q_ref[ii])
        tau_unc=7.0*(qtar-data.qpos[qadrs])-.32*data.qvel[dadrs]
        saturation+=int(np.any(tau_unc>bounds[:,1]) or np.any(tau_unc<bounds[:,0]))
        tau=np.clip(tau_unc,bounds[:,0],bounds[:,1])
        max_tau=max(max_tau,float(np.max(np.abs(tau))))
        data.ctrl[:]=tau
        mujoco.mj_step(model,data)
        if k%sample_interval==0 or k==n-1:
            root=data.xpos[base].copy()
            rot=data.xmat[base].reshape(3,3)
            tilt=math.degrees(math.acos(float(np.clip((rot@local_up)[2],-1,1))))
            contacts=floor_contacts(model,data,mujoco)
            advance=float((root[:2]-pose0[:2])@forward)
            results.append({'t':round(float(data.time-start_data_time),4),'advance_m':round(advance,6),'base_z_m':round(float(root[2]),6),'tilt_deg':round(tilt,3),'contacts':sorted(contacts),'n_contacts':len(contacts),'max_torque_nm':round(float(np.max(np.abs(tau))),4)})
            if root[2]<.075 or tilt>50 or not np.all(np.isfinite(data.qpos)):
                fall=True;break
        if k%frame_interval==0 or k==n-1:
            state.append({'t':round(float(data.time-start_data_time),4),'qpos':np.round(data.qpos,8).tolist()})
    end=results[-1]
    report={
      'gait':gait,'scenario':mass,'initial_stand_verified':True,'duration_s':round(float(data.time-start_data_time),4),
      'ground_tibia_sliding_friction_mu_assumed':friction,
      'finished_full_duration':len(results)>0 and not fall and (k+1)==n,
      'model_source':'MattiaGrigoli original MJCF at 5c9a84c; 30 of 32 real STL; 2 visual-only box proxies',
      'physics':'MuJoCo '+mujoco.__version__,'robot_mass_kg':round(float(model.body_mass.sum()),6),
      'actual_tibia_collision_meshes':6,'verified_original_meshes':30,'unverified_visual_meshes':['topplate.stl','bigservo1.stl'],
      'step_length_command_m':step,'lift_command_m':lift,'period_s':period,'swing_duty':duty,
      'IK_max_error_mm':round(max(errors.values())*1000,3),'IK_max_joint_abs_rad':round(qmax,5),
      'advance_m':end['advance_m'],'final_height_m':end['base_z_m'],
      'min_body_height_m':min(x['base_z_m'] for x in results),'max_tilt_deg':max(x['tilt_deg'] for x in results),
      'max_abs_torque_nm':round(max_tau,5),'saturation_fraction':round(saturation/max(1,n),5),
      'min_grounded_legs':min(x['n_contacts'] for x in results),'mean_grounded_legs':round(float(np.mean([x['n_contacts'] for x in results])),3),
      'fall_or_nonfinite':fall,'body_was_actuated':False,'feet_were_pinned':False,
      'vendor_PWM_calibrated':False,'torque_speed_calibrated':False,
      'trace':results,'qpos_samples':state,
      'limitations':['Only 30 of 32 visual STL downloaded; 19 visual proxy instances','Masses and motor/contact properties are estimates','No servo PWM/shaft angle calibration','Fixed-phase open-loop locomotion; not yet local insect leg coordination']
    }
    out=ROOT/'results'/receipt_filename(
        gait,mass,friction=friction,step=step,lift=lift,period=period,seconds=seconds)
    out.write_text(json.dumps(report,indent=2)+'\n')
    return report

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser()
    p.add_argument('--gait',choices=['tripod','wave'],default='tripod');p.add_argument('--mass',choices=['lower','nominal','upper'],default='nominal')
    p.add_argument('--step',type=float,default=.035);p.add_argument('--lift',type=float,default=.020);p.add_argument('--period',type=float,default=2.4);p.add_argument('--seconds',type=float,default=8.);p.add_argument('--friction',type=float,default=1.1)
    a=p.parse_args()
    r=gait_run(a.gait,a.mass,a.step,a.lift,a.period,a.seconds,friction=a.friction)
    print(json.dumps({k:v for k,v in r.items() if k not in ('trace','qpos_samples')},indent=2))
