"""First shadow-boxing *air jab* test on the Freenove FNK0052 CAD30 model.

- Both front legs rise high while four others support the freely moving body.
- Left and right front legs take turns extending forward then retracting in air.
- All joint torques are calculated by MuJoCo with a finite PD actuator cap.
- No projectile, target object, external impact, body weld, driven root, or posed-only animation.
- This is a preprogrammed trajectory with a diagnostic safety abort, NOT an
  autonomous distributed neural controller and NOT calibrated RC servo hardware.
"""
from __future__ import annotations
import json, argparse, math, sys
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial import ConvexHull
HERE=Path(__file__).resolve().parent
ROOT=HERE if (HERE/'source/mjcf').is_dir() else HERE.parents[1]
sys.path.insert(0,str(ROOT))
from hexlab.stand_geometry_v04 import load_legs,tip_world,geom_mesh_world_vertices
from hexlab.stand_v04 import static_pose,hold_under_gravity
from hexlab.step_one_leg_v04 import solve_waypoint,smoothstep
from front_pair_trick import FRONT,SUPPORT,contact_points,margin

# Transition timestamps are after the one-second initial six-leg stabilization.
STAGES=(
  (0.00, 'six-leg stand'),
  (2.00, 'weight shift completed'),
  (3.10, 'raise both front legs'),
  (3.55, 'high guard'),
  (3.87, 'left jab'),
  (4.20, 'left retract'),
  (4.45, 'guard'),
  (4.77, 'right jab'),
  (5.10, 'right retract'),
  (5.55, 'guard hold'),
  (6.50, 'lower front legs'),
  (9.00, 'restore six-leg stance'),
  (10.00, 'stand recovery dwell'),
)
TOTAL=STAGES[-1][0]

def interpolate_keys(t:float, keyframes:list[tuple[float,np.ndarray]]) -> np.ndarray:
    if t<=keyframes[0][0]: return np.array(keyframes[0][1])
    for (a,qa),(b,qb) in zip(keyframes[:-1],keyframes[1:]):
        if t<=b:
            s=smoothstep((t-a)/(b-a))
            return qa+(qb-qa)*s
    return np.array(keyframes[-1][1])

def experiment(*,lift_m=.105, guard_forward_m=.050, jab_forward_m=.105,
               shift_m=.040, mass='nominal',mu=1.1,
               frame_hz=12,record_frames=False, save_path:Path|None=None) -> dict:
    if not (.045<=lift_m<=.17 and .025<=guard_forward_m<=.095 and .05<=jab_forward_m<=.15): raise ValueError('target size')
    if not(0. <= shift_m<=.065) :raise ValueError('control duration')
    if not(.2<=mu<=2.):raise ValueError('friction')
    if mass not in ['lower','nominal','upper']:raise ValueError('mass')
    model=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf'/f'scene_local30_{mass}.xml'))
    model.geom_friction[model.geom('floor').id,0]=mu
    for code in FRONT+SUPPORT: model.geom_friction[model.geom('tibia'+code+'_collision_0').id,0]=mu
    data=mujoco.MjData(model);legs=load_legs(model,data,mujoco);lookup={l['suffix']:l for l in legs}
    static_pose(model,data,mujoco,legs,.145,.150)
    standing=hold_under_gravity(model,data,mujoco,data.qpos.copy(),1.)
    if not standing['stable_by_proxy_thresholds'] or standing['last_0p4s_min_contact_legs']<6:raise RuntimeError('initial stand unstable')
    qstart=data.qpos.copy();root=model.body('base_link_2').id
    initial_rot=data.xmat[root].reshape(3,3).copy();up0=initial_rot.T@np.array([0.,0.,1.])
    root_start=data.xpos[root].copy()
    initial_COM=data.subtree_com[root].copy()
    # Preserve 20 total actuators, including two for the head.
    jids=np.array(model.actuator_trnid[:,0],dtype=int)
    qadrs=np.array([model.jnt_qposadr[j] for j in jids],dtype=int)
    dadrs=np.array([model.jnt_dofadr[j] for j in jids],dtype=int)
    ref0=data.qpos[qadrs].copy()
    idx={c:np.array([np.where(qadrs==addr)[0][0] for addr in lookup[c]['qadrs']]) for c in FRONT+SUPPORT}
    planned={}; errors={}; baseline_foot={}
    ref=mujoco.MjData(model)
    for code in FRONT+SUPPORT:
        l=lookup[code];ref.qpos[:]=qstart;ref.qvel[:]=0;mujoco.mj_forward(model,ref)
        tip=tip_world(model,ref,l).copy();baseline_foot[code]=tip.tolist()
        if code in SUPPORT: goals={'support':tip+np.array([0.,shift_m,0.])}
        else:goals={'guard':tip+np.array([0.,guard_forward_m,lift_m]),
                    'jab':tip+np.array([0.,jab_forward_m,lift_m])}
        planned[code]={};errors[code]={}
        for name,goal in goals.items():
            q,err=solve_waypoint(model,ref,mujoco,l,goal,qstart[l['qadrs']])
            planned[code][name]=q
            errors[code][name]=round(float(err),7)
    maxerr=max(v for z in errors.values() for v in z.values())
    if maxerr>.005:raise RuntimeError(f'IK residual too high {errors}')
    # Note front legs are symmetric and the high air jab does not touch any target.
    # Prepare each leg's position controller path through the same scripted times.
    keyed={}
    for code in SUPPORT:
        original=ref0[idx[code]]
        forward=planned[code]['support']
        keyed[code]=[(0.,original),(2.,forward),(6.50,forward),(9.00,original),(10.00,original)]
    for code in FRONT:
        original=ref0[idx[code]];g=planned[code]['guard'];jab=planned[code]['jab']
        left=(code=='fl')
        keyed[code]=[(0.,original),(2.,original),(3.10,g),(3.55,g),
                     (3.87,jab if left else g),(4.20,g),(4.45,g),
                     (4.77,jab if not left else g),(5.10,g),(5.55,g),
                     (6.50,original),(9.00,original),(10.00,original)]
    dt=float(model.opt.timestep);N=round(TOTAL/dt)
    rows=[];frames=[];max_tau=0.;sat_count=0;max_qvel=0.;max_target_qvel=0.;max_tilt=0.;fail_reason=None
    fr_period=max(1,int(round(1./frame_hz/dt)));trace_period=max(1,int(round(.02/dt)))
    prev_target=ref0.copy();initial_baseheight=float(data.xpos[root][2]);bound=model.actuator_ctrlrange.copy()
    time0=float(data.time)
    for k in range(N):
        t=k*dt
        target=ref0.copy()
        for code in FRONT+SUPPORT: target[idx[code]]=interpolate_keys(t,keyed[code])
        max_target_qvel=max(max_target_qvel,float(np.max(np.abs((target-prev_target)/dt))))
        prev_target=target.copy()
        unc=7.*(target-data.qpos[qadrs])-.32*data.qvel[dadrs]
        sat_count+=int(np.any(unc>bound[:,1]+1e-9) or np.any(unc<bound[:,0]-1e-9))
        tau=np.clip(unc,bound[:,0],bound[:,1]);data.ctrl[:]=tau
        max_tau=max(max_tau,float(np.max(np.abs(tau))))
        max_qvel=max(max_qvel,float(np.max(np.abs(data.qvel[dadrs]))))
        mujoco.mj_step(model,data)
        if k%trace_period==0 or k==N-1:
            pos=data.xpos[root].copy();rot=data.xmat[root].reshape(3,3)
            tilt=math.degrees(math.acos(float(np.clip((rot@up0)[2],-1,1))))
            cp=contact_points(model,data)
            on_sup=[c for c in SUPPORT if cp[c]];on_front=[c for c in FRONT if cp[c]]
            meshmin={c:float(geom_mesh_world_vertices(model,data,lookup[c]['geom'])[:,2].min()) for c in FRONT}
            com=data.subtree_com[root].copy()
            supp_pts=[p for c in SUPPORT for p in cp[c]]
            mar=margin(com[:2],supp_pts) if len(supp_pts)>=3 else None
            # phase-specific real joint pose and toe world coordinates
            tipstate={c:tip_world(model,data,lookup[c]).tolist() for c in FRONT}
            row={'t_s':round(t+dt,3),'base_xyz_m':np.round(pos,5).tolist(),
                 'center_of_mass_xyz_m':np.round(com,5).tolist(),
                 'tilt_deg':round(tilt,3),'support_contacts':on_sup,'front_contacts':on_front,
                 'support_margin_m':None if mar is None else round(mar,5),
                 'front_mesh_min_z_m':{c:round(v,5) for c,v in meshmin.items()},
                 'front_tip_xyz_m':{c:[round(x,5) for x in tipstate[c]] for c in FRONT},
                 'tau_abs_max_nm':round(float(np.max(abs(tau))),5)}
            rows.append(row)
            max_tilt=max(max_tilt,tilt)
            if not np.all(np.isfinite(data.qpos)):fail_reason='nonfinite_physics';break
            if pos[2]<.07 or tilt>35:fail_reason='tip_or_drop';break
        if record_frames and (k%fr_period==0 or k==N-1):
            frames.append({'t':round(t+dt,4),'qpos':np.round(data.qpos,8).tolist()})
    # Evaluate throughout guard, punch and recovery, using true free-base MuJoCo state.
    analyzed=[r for r in rows if 3.25<=r['t_s']<5.45]
    hold=[r for r in rows if 5.17<=r['t_s']<5.50]
    six_end=[r for r in rows if 9.50<=r['t_s']<10.0]
    valid= bool(analyzed) and all(len(r['support_contacts'])==4 and not r['front_contacts'] and
             min(r['front_mesh_min_z_m'].values())>.045 and r['support_margin_m'] is not None and r['support_margin_m']>.002 and r['tilt_deg']<10 for r in analyzed)
    valid= valid and not fail_reason and len(rows)>N*dt/trace_period/dt*0.9
    # We should not claim success if the leg joints don't actually extend during punches.
    leftp=[r for r in analyzed if 3.79<=r['t_s']<=3.94]
    rightp=[r for r in analyzed if 4.69<=r['t_s']<=4.84]
    # A more interpretable measure: forward difference between jab apex and guard before it.
    def fore(t,code):
        return min(rows,key=lambda x:abs(x['t_s']-t))['front_tip_xyz_m'][code][1]
    jab_excursions={'fl':round(fore(3.87,'fl')-fore(3.53,'fl'),5),
                    'fr':round(fore(4.77,'fr')-fore(4.45,'fr'),5)}
    success=valid and min(jab_excursions.values())>.012 and all(len(r['support_contacts'])==4 and not r['front_contacts'] for r in hold)
    peakabsangles=max(float(np.max(np.abs(data.qpos[qadrs]))),max(float(np.max(abs(x))) for key in keyed.values() for _,x in key))
    report={
      'label':'High-front-leg air jab (shadowboxing), simulated only; no physical impact',
      'model':'Freenove FNK0052 CAD30, 30 of 32 original CAD meshes, source 5c9a84c',
      'mass_scenario':mass,'mass_model_kg':round(float(model.body_mass.sum()),6),'friction_mu_assumed':mu,
      'commands':{'weight_shift_target_m':shift_m,'front_guard_target_vertical_m':lift_m,'front_guard_target_forward_m':guard_forward_m,
                  'jab_tip_target_forward_m':jab_forward_m,'jab_tip_target_delta_m':jab_forward_m-guard_forward_m,
                  'jacobian_ik_max_error_m':round(maxerr,6),'scripted_transitions_s':STAGES},
      'physics':{'dt_s':dt,'free_root':True,'joint_torque_limit_nm':round(float(bound[:,1].max()),3),
                 'estimated_not_hardware_calibrated':True,'motor_speed_limit_modeled':False,'has_any_impact_target':False},
      'complete':fail_reason is None and k==N-1,
      'high_guard_and_air_jabs_contact_valid':bool(success),
      'high_guard_and_air_jabs_valid':bool(success and sat_count==0),
      'torque_saturation_observed':bool(sat_count>0),
      'front_contact_free_guard_and_jabs':bool(analyzed) and all(not x['front_contacts'] for x in analyzed),
      'four_supports_during_guard_and_jabs':bool(analyzed) and all(len(x['support_contacts'])==4 for x in analyzed),
      'min_front_mesh_clearance_during_guard_and_jabs_m':round(min(min(x['front_mesh_min_z_m'].values()) for x in analyzed),5) if analyzed else None,
      'min_support_COM_margin_during_guard_and_jabs_m':round(min((x['support_margin_m'] if x['support_margin_m'] is not None else -1.) for x in analyzed),5) if analyzed else None,
      'max_tilt_guard_and_jabs_deg':round(max(x['tilt_deg'] for x in analyzed),3) if analyzed else None,
      'measured_forward_jab_excursion_m':jab_excursions,
      'max_abs_torque_nm':round(max_tau,5),'motor_torque_saturation_fraction':round(sat_count/N,5),
      'max_abs_actual_joint_speed_rad_s':round(max_qvel,3),'max_abs_command_joint_speed_rad_s':round(max_target_qvel,3),
      'max_abs_command_joint_angle_rad':round(peakabsangles,4),
      'initial_COM_xy_m':np.round(initial_COM[:2],5).tolist(),
      'end_six_contact_restore_fraction':round(sum(len(x['support_contacts'])+len(x['front_contacts'])==6 for x in six_end)/len(six_end),3) if six_end else None,
      'failure_reason':fail_reason,'trace':rows,'qpos_frames':frames,
      'limitations':['This is choreographed joint-space motion with a fixed 40-mm balance shift; NOT autonomous distributed control',
        'Freenove RC servo speed/torque-vs-speed/backlash, PWM limits and friction are unmeasured',
        'No target object is hit; no collision/impact forces inferred; high foot motion is not proof of safe real robot',
        'CAD collision uses six tibia convex mesh hulls; two missing STL replaced only visually',
        'No randomized starting poses or perturbations; limited sample window']}
    if save_path is not None:
        save_path.parent.mkdir(parents=True,exist_ok=True)
        save_path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report

if __name__=='__main__':
  p=argparse.ArgumentParser()
  p.add_argument('--lift',type=float,default=.105)
  p.add_argument('--guard-forward',type=float,default=.050)
  p.add_argument('--jab-forward',type=float,default=.105)
  p.add_argument('--shift',type=float,default=.040)
  p.add_argument('--mass',choices=['lower','nominal','upper'],default='nominal')
  p.add_argument('--mu',type=float,default=1.1)
  p.add_argument('--record',action='store_true')
  p.add_argument('--out',type=Path)
  args=p.parse_args()
  r=experiment(lift_m=args.lift,guard_forward_m=args.guard_forward,
               jab_forward_m=args.jab_forward,shift_m=args.shift,mass=args.mass,mu=args.mu,
               record_frames=args.record,save_path=args.out)
  print(json.dumps({k:v for k,v in r.items() if k not in ('trace','qpos_frames')},ensure_ascii=False,indent=2))
