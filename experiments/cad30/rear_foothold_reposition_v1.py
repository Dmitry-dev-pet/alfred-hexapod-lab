"""Freenove CAD30: physical rear-foot relocation before a prospective middle-foot liftoff.

Supports a *scripted* rear-foot relocation, with an independent sensor-only
middle-leg readiness gate and a COM/triangle oracle used for evaluation ONLY.
No learned control, no four-legged gait, no hardware calibration.
"""
from pathlib import Path
import json, math, time
import numpy as np
import mujoco
from scipy.spatial import cKDTree
from reflex_unload_middle import contact_normal_forces,ready_for_swing
from front_pair_trick import FRONT,SUPPORT,contact_points,margin
from hexlab.stand_geometry_v04 import load_legs,tip_world,geom_mesh_world_vertices
from hexlab.stand_v04 import static_pose,hold_under_gravity
from hexlab.step_one_leg_v04 import solve_waypoint,smoothstep
HERE=Path(__file__).resolve().parent
ROOT=HERE if (HERE/'source/mjcf').is_dir() else HERE.parents[1]

def run(dx=.04,dy=.20,mass='nominal',mu=1.1,record=False,rear_leg='br',tend=9.0):
 if rear_leg not in ('bl','br'):raise ValueError('rear_leg')
 candidate='ml' if rear_leg=='bl' else 'mr'
 m=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf'/f'scene_local30_{mass}.xml'))
 m.geom_friction[m.geom('floor').id,0]=mu
 for c in SUPPORT+FRONT:m.geom_friction[m.geom('tibia'+c+'_collision_0').id,0]=mu
 d=mujoco.MjData(m);legs=load_legs(m,d,mujoco);L={a['suffix']:a for a in legs};static_pose(m,d,mujoco,legs,.145,.15)
 s=hold_under_gravity(m,d,mujoco,d.qpos.copy(),1.)
 if not s['stable_by_proxy_thresholds']:return {'fail':'bad_stand'}
 q0=d.qpos.copy();j=m.actuator_trnid[:,0];qa=m.jnt_qposadr[j];da=m.jnt_dofadr[j];init=d.qpos[qa].copy()
 ix={c:np.array([np.where(qa==p)[0][0] for p in L[c]['qadrs']]) for c in SUPPORT+FRONT}
 r=mujoco.MjData(m);r.qpos[:]=q0;r.qvel[:]=0;mujoco.mj_forward(m,r)
 targets={}
 for c in SUPPORT+FRONT:
  qref=q0[L[c]['qadrs']];tip=tip_world(m,r,L[c]).copy()
  off=np.array([0,.05,.105]) if c in FRONT else np.array([0,.04,0])
  q,e=solve_waypoint(m,r,mujoco,L[c],tip+off,qref)
  targets[c]=q
  if e>.005:return {'fail':'bad_ik_baseline','leg':c,'error':e}
 q_raise0=init.copy();
 for c in FRONT+SUPPORT:q_raise0[ix[c]]=targets[c]
 root=m.body('base_link_2').id;up0=d.xmat[root].reshape(3,3).T@np.array([0.,0.,1.])
 peak=0.;sat=0;rows=[];frames=[];t0=d.time;dt=m.opt.timestep;first_moved=False;calculated={};prevphase='';sensor_ready=0;max_sensor_ready=0;gate_releases=0
 tstart=time.monotonic()
 for k in range(round(tend/dt)):
  t=k*dt;cmd=init.copy()
  if t<2:
   a=smoothstep(min(1,t/2));
   for c in SUPPORT:cmd[ix[c]]=init[ix[c]]+a*(targets[c]-init[ix[c]])
   for c in FRONT:cmd[ix[c]]=init[ix[c]]
  else:
   a=smoothstep(min(1,(t-2)/1.2))
   for c in SUPPORT:cmd[ix[c]]=targets[c]
   for c in FRONT:cmd[ix[c]]=init[ix[c]]+a*(targets[c]-init[ix[c]])
  if t>=3.5:
   if not first_moved:
    first_moved=True
    p=tip_world(m,d,L[rear_leg]).copy();qcurrent=d.qpos[L[rear_leg]['qadrs']].copy()
    ref=mujoco.MjData(m);ref.qpos[:]=d.qpos;ref.qvel[:]=0;mujoco.mj_forward(m,ref)
    wps=[('lift',np.array([dx,dy*.20,.020])),('swing',np.array([dx,dy,.025])),('land',np.array([dx,dy,-.002]))]
    for label,off in wps:
     q,e=solve_waypoint(m,ref,mujoco,L[rear_leg],p+off,qcurrent);calculated[label]={'q':q.tolist(),'err':e,'off':off.tolist()}
    calculated['init']={'q':d.qpos[L[rear_leg]['qadrs']].tolist(),'tip':p.tolist()}
   if t<4.2:scale=smoothstep((t-3.5)/.7);ql=(1-scale)*np.array(calculated['init']['q'])+scale*np.array(calculated['lift']['q'])
   elif t<5.3:scale=smoothstep((t-4.2)/1.1);ql=(1-scale)*np.array(calculated['lift']['q'])+scale*np.array(calculated['swing']['q'])
   elif t<6.1:scale=smoothstep((t-5.3)/.8);ql=(1-scale)*np.array(calculated['swing']['q'])+scale*np.array(calculated['land']['q'])
   else:ql=np.array(calculated['land']['q'])
   cmd[ix[rear_leg]]=ql
  unc=7*(cmd-d.qpos[qa])-.32*d.qvel[da]
  sat+=int(np.any((unc>m.actuator_ctrlrange[:,1])|(unc<m.actuator_ctrlrange[:,0])))
  d.ctrl[:]=np.clip(unc,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1]);peak=max(peak,float(np.max(np.abs(d.ctrl))))
  mujoco.mj_step(m,d)
  if k%10==0 or k==round(tend/dt)-1:
   cp=contact_points(m,d);supp={c:bool(cp[c]) for c in SUPPORT};fr={c:bool(cp[c]) for c in FRONT}
   com=d.subtree_com[root][:2];leg_3=[p for c in SUPPORT if c!=candidate for p in cp[c]]
   mar=margin(com,leg_3) if len(leg_3)>=3 else None
   all_points=[p for c in SUPPORT for p in cp[c]]
   mar_active=margin(com,all_points) if len(all_points)>=3 else None
   z=float(np.min(geom_mesh_world_vertices(m,d,L[rear_leg]['geom'])[:,2]));forces=contact_normal_forces(m,d);rot=d.xmat[root].reshape(3,3);tilt=math.degrees(math.acos(float(np.clip((rot@up0)[2],-1,1))));pos=d.xpos[root]
   if t>=6.4:
    other=[c for c in SUPPORT if c!=candidate]
    currently_ready=ready_for_swing(forces[candidate], [forces[c] for c in other], [supp[c] for c in other], list(fr.values()), tilt,
                                    candidate_contact=supp[candidate], gyro_speed_rad_s=float(np.linalg.norm(d.qvel[3:6])))
    sensor_ready=sensor_ready+dt*10 if currently_ready else 0.
    max_sensor_ready=max(sensor_ready,max_sensor_ready)
    # This is a diagnosis of whether a hypothetical sensor gate WOULD release.
    gate_releases+=int(sensor_ready>=.15 and sensor_ready-dt*10<.15)
   rows.append({'t':round(t,3),'leg_support':supp,'front_support':fr,'margin_candidate_m':None if mar is None else round(mar,5),'margin_active_m':None if mar_active is None else round(mar_active,5),'rear_min_z_m':round(z,5),'rear_xy_m':np.round(tip_world(m,d,L[rear_leg])[:2],4).tolist(),'com_xy_m':np.round(com,4).tolist(),'normal_forces_N':{c:round(float(forces[c]),3) for c in SUPPORT},'tilt_deg':round(tilt,2),'base_y_m':round(float(pos[1]),4),'torque_peak_nm':round(float(np.max(abs(d.ctrl))),4)})
   if pos[2]<.07 or tilt>35 or not np.all(np.isfinite(d.qpos)):
    return {'fail':'fall','t':t,'peak':peak,'sat_fraction':sat/(k+1),'ik':calculated,'trace':rows if record else []}
  if record and k%50==0:frames.append({'t':round(t,3),'qpos':np.round(d.qpos,7).tolist()})
 elapsed=time.monotonic()-tstart
 hold=[v for v in rows if 6.5<=v['t']<8.9]
 legmove=[v for v in rows if 3.6<v['t']<5.5]
 swing=[v for v in rows if 3.6<=v['t']<=5.5]
 # Mesh vertex proximity only at final steady state, not a complete self-collision test.
 mesh_vertex_distance_m=float(cKDTree(geom_mesh_world_vertices(m,d,L[rear_leg]['geom'])).query(
     geom_mesh_world_vertices(m,d,L[candidate]['geom']),k=1)[0].min())
 foot_tip_distance_m=float(np.linalg.norm(tip_world(m,d,L[rear_leg])[:2]-tip_world(m,d,L[candidate])[:2]))
 criterion=(all(not any(x['front_support'].values()) for x in swing+hold)
            and all(sum(x['leg_support'].values())>=3 for x in swing)
            and all(sum(x['leg_support'][c] for c in SUPPORT if c!=rear_leg)>=3 for x in swing if not x['leg_support'][rear_leg])
            and all(x['margin_active_m'] is not None and x['margin_active_m']>0 for x in swing)
            and all(x['leg_support'][rear_leg] for x in hold)
            and all(x['margin_candidate_m'] is not None and x['margin_candidate_m']>.005 for x in hold)
            and max(x['rear_min_z_m'] for x in legmove)>.005
            and mesh_vertex_distance_m>.015 and sat==0)
 return {'fail':None,'dx':dx,'dy':dy,'rear_leg':rear_leg,'candidate':candidate,'mass':mass,'mu':mu,'selected_after_exploratory_search':True,'not_reactive_foothold_selection':True,'no_com_in_controller':True,'safe_rear_reposition':bool(criterion),'min_rear_mesh_vertex_distance_to_middle_m':round(mesh_vertex_distance_m,5),'rear_middle_tip_xy_distance_m':round(foot_tip_distance_m,5),'min_rear_swing_support_legs':min((sum(x['leg_support'][c] for c in SUPPORT if c!=rear_leg) for x in swing if not x['leg_support'][rear_leg]),default=-1),'min_support_legs_rear_phase':min((sum(x['leg_support'].values()) for x in swing),default=-1),'min_margin_rear_phase_m':min((x['margin_active_m'] for x in swing if x['margin_active_m'] is not None),default=None),'max_sensor_gate_dwell_s':round(float(max_sensor_ready),3),'gate_release_count':gate_releases,'post_middle_load_N':round(float(np.mean([x['normal_forces_N'][candidate] for x in hold])),3),'peak_torque_nm':peak,'saturation_fraction':sat/(k+1),'ik_errors_m':{x:round(calculated[x]['err'],5) for x in ('lift','swing','land')},'rear_max_clearance_m':max(x['rear_min_z_m'] for x in legmove),'rear_min_clearance_m':min(x['rear_min_z_m'] for x in legmove),'post_reposition_contact_frac':float(np.mean([x['leg_support'][rear_leg] for x in hold])),'front_clear_frac':float(np.mean([not any(x['front_support'].values()) for x in hold])),'min_other_three_contacts':min(sum(x['leg_support'][c] for c in SUPPORT if c!=candidate) for x in hold),'margin_last_m':hold[-1]['margin_candidate_m'],'margin_max_m':max((x['margin_candidate_m'] or -1) for x in hold),'com_last':hold[-1]['com_xy_m'],'rear_tip_last':hold[-1]['rear_xy_m'],'time_run_s':round(elapsed,1),'trace':rows if record else [],'qpos_frames':frames if record else []}
if __name__=='__main__':
 import sys
 dx=float(sys.argv[1]) if len(sys.argv)>1 else -.04
 dy=float(sys.argv[2]) if len(sys.argv)>2 else .12
 print(json.dumps(run(dx=dx,dy=dy),indent=2))
