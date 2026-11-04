"""Diagnostic front-pair lift on the pinned Freenove FNK0052 CAD30 MuJoCo model.

This is a *preprogrammed* balance-preparation comparison, not a neural controller.
The two front legs (CAD fl/fr, confirmed by front-most hip Y) are lifted while
mid/rear legs support the body. An optional pre-shift of mid/rear foot targets
in the body reference frame makes the chassis translate backwards in physics.
No body welds, kinematic root commands, frozen feet or artificial contact.
"""
from __future__ import annotations
import json, math, sys
from pathlib import Path
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent
ROOT=HERE if (HERE/"source/mjcf").is_dir() else HERE.parents[1]
sys.path.insert(0,str(ROOT))
from hexlab.stand_geometry_v04 import load_legs,tip_world,geom_mesh_world_vertices
from hexlab.stand_v04 import static_pose,hold_under_gravity
from hexlab.step_one_leg_v04 import solve_waypoint,smoothstep,floor_contacts
from scipy.spatial import ConvexHull

FRONT=('fr','fl')
SUPPORT=('ml','bl','br','mr')

def margin(point, pts):
    pts=np.asarray(pts, dtype=float)
    if len(pts)<3: return None
    hull=ConvexHull(pts)
    v=pts[hull.vertices]
    # Convex hull polygon orientation CCW. Signed distance to each edge.
    edges=np.roll(v,-1,axis=0)-v
    cross=np.array([(q[0]-p[0])*(point[1]-p[1])-(q[1]-p[1])*(point[0]-p[0]) for p,q in zip(v,np.roll(v,-1,axis=0))])
    dist=cross/np.linalg.norm(edges,axis=1)
    if np.all(dist>=0): return float(np.min(dist))
    d=[]
    for p,e in zip(v,edges):
        a=np.clip(np.dot(point-p,e)/np.dot(e,e),0,1)
        d.append(np.linalg.norm(point-(p+a*e)))
    return -float(np.min(d))

def contact_points(model,data):
    floor=model.geom('floor').id
    names={model.geom('tibia'+s+'_collision_0').id:s for s in FRONT+SUPPORT}
    points={s:[] for s in FRONT+SUPPORT}
    for i in range(data.ncon):
        con=data.contact[i]
        if con.geom1==floor: other=int(con.geom2)
        elif con.geom2==floor: other=int(con.geom1)
        else: continue
        if other in names:points[names[other]].append(np.array(con.pos[:2]))
    return points

def experiment(shift_m=0.0,lift_m=0.035,scenario='nominal',fps=10,save_frames=False, friction=None):
    assert scenario in ('nominal','lower','upper')
    assert 0<=shift_m<=0.065 and 0.005<=lift_m<=0.065
    model=mujoco.MjModel.from_xml_path(str(ROOT/'source'/'mjcf'/f'scene_local30_{scenario}.xml'))
    if friction is not None:
        gid=model.geom('floor').id
        model.geom_friction[gid,0]=friction
        for s in FRONT+SUPPORT:
            model.geom_friction[model.geom('tibia'+s+'_collision_0').id,0]=friction
    data=mujoco.MjData(model)
    legs=load_legs(model,data,mujoco)
    named={leg['suffix']:leg for leg in legs}
    # The physically frontmost two legs are fl/fr regardless of mirrored OEM/CAD labels.
    ys={code:float(named[code]['hip_xyz'][1]) for code in named}
    assert sorted(ys, key=lambda k:ys[k])[-2:] == list(FRONT) or set(sorted(ys, key=lambda k:ys[k])[-2:])==set(FRONT),ys
    static_pose(model,data,mujoco,legs,.145,.150)
    standing=hold_under_gravity(model,data,mujoco,data.qpos.copy(),1.0)
    if not standing['stable_by_proxy_thresholds'] or standing['last_0p4s_min_contact_legs']<6:
        raise AssertionError('stand baseline not validated')
    qstart=data.qpos.copy()
    root=model.body('base_link_2').id
    start_pos=data.xpos[root].copy()
    start_rot=data.xmat[root].reshape(3,3).copy()
    up0=start_rot.T@np.array([0.,0.,1.])
    base_com=data.subtree_com[root].copy()
    planned={}
    errors={}
    for c in FRONT+SUPPORT:
        l=named[c]
        ref=mujoco.MjData(model); ref.qpos[:]=qstart; ref.qvel[:]=0; mujoco.mj_forward(model,ref)
        toe=tip_world(model,ref,l).copy()
        if c in FRONT:delta=np.array([0.,0.,lift_m])
        else:delta=np.array([0.,shift_m,0.])
        q,err=solve_waypoint(model,ref,mujoco,l,toe+delta,qstart[l['qadrs']].copy())
        planned[c]=q; errors[c]=float(err)
    if max(errors.values())>.008:raise AssertionError(f'IK too poor {errors}')
    # Actuator map includes two servos for head. All torques are bounded.
    actuated_jid=np.asarray(model.actuator_trnid[:,0], dtype=int)
    qadrs=np.array([model.jnt_qposadr[j] for j in actuated_jid],dtype=int)
    dadrs=np.array([model.jnt_dofadr[j] for j in actuated_jid],dtype=int)
    ref0=data.qpos[qadrs].copy()
    target_index={c:np.array([np.where(qadrs==qa)[0][0] for qa in named[c]['qadrs']]) for c in FRONT+SUPPORT}
    lo,hi=model.actuator_ctrlrange[:,0],model.actuator_ctrlrange[:,1]
    # Programmed 2s reposition, 1s lift, 1s hold, 1s lower, 1s recover.
    total=6.0
    num=int(round(total/model.opt.timestep))
    rows=[]; states=[]
    sat=0; max_tau=0.; time_at_target=0
    fail=False
    for k in range(num):
        t=k*model.opt.timestep
        if t<2.: s=smoothstep(t/2);lift=0.
        elif t<3.: s=1.;lift=smoothstep(t-2.)
        elif t<4.: s=1.;lift=1.
        elif t<5.: s=1.;lift=1.-smoothstep(t-4.)
        else:s=1.-smoothstep(t-5.);lift=0.
        target=ref0.copy()
        for c in SUPPORT:ii=target_index[c];target[ii]=ref0[ii]+s*(planned[c]-ref0[ii])
        for c in FRONT:ii=target_index[c];target[ii]=ref0[ii]+lift*(planned[c]-ref0[ii])
        unc=7.*(target-data.qpos[qadrs])-.32*data.qvel[dadrs]
        sat+=int(np.any(unc>hi+1e-9) or np.any(unc<lo-1e-9))
        tau=np.clip(unc,lo,hi);max_tau=max(max_tau,float(np.max(abs(tau))))
        data.ctrl[:]=tau
        mujoco.mj_step(model,data)
        if k%10==0 or k==num-1:
            pos=data.xpos[root]; rot=data.xmat[root].reshape(3,3)
            tilt=math.degrees(math.acos(float(np.clip((rot@up0)[2],-1,1))))
            pts=contact_points(model,data)
            support_contact=[c for c in SUPPORT if len(pts[c])>0]
            front_contact=[c for c in FRONT if len(pts[c])>0]
            footmin={c:float(geom_mesh_world_vertices(model,data,named[c]['geom'])[:,2].min()) for c in FRONT}
            com=data.subtree_com[root].copy()
            supported=[p for c in SUPPORT for p in pts[c]]
            mar=margin(com[:2],supported) if len(supported)>=3 else None
            row={
                't_s':round(t+.002,3), 'base_xyz_m':np.round(pos,6).tolist(),
                'com_xy_m':np.round(com[:2],6).tolist(),
                'tilt_deg':round(tilt,3),'support_contact':support_contact,
                'front_contact':front_contact,
                'front_mesh_min_z_m':{c:round(z,5) for c,z in footmin.items()},
                'support_contact_margin_m':None if mar is None else round(mar,5),
                'max_torque_nm':round(float(np.max(abs(tau))),5),
            }
            rows.append(row)
            if pos[2]<.075 or tilt>50 or not np.all(np.isfinite(data.qpos)):
                fail=True; break
        if save_frames and (k%(round(1/fps/model.opt.timestep))==0 or k==num-1):
            states.append({'t':round(t+.002,3),'qpos':np.round(data.qpos,8).tolist()})
    hold=[row for row in rows if 3.25<=row['t_s']<3.95]
    success=(not fail and len(hold)>0 and
             all(len(x['support_contact'])==4 and len(x['front_contact'])==0 and
                 min(x['front_mesh_min_z_m'].values())>.004 and
                 x['support_contact_margin_m'] is not None and x['support_contact_margin_m']>0
                 and x['tilt_deg']<10. for x in hold))
    report={
      'model':'FNK0052 CAD30 original mesh pinned 5c9a84c, free base',
      'scenario':scenario,'friction_assumed':friction,'lift_m':lift_m,
      'support_foot_target_forward_shift_m':shift_m,
      'method':'preprogrammed support-foot IK repositioning, NOT autonomous or neural',
      'motor_torque_cap_nm':float(max(hi)), 'motor_is_calibrated':False,
      'mass_kg':round(float(model.body_mass.sum()),6),
      'ik_err_max_mm':round(1000*max(errors.values()),3),
      'init_com_xy_m':np.round(base_com[:2],6).tolist(),
      'actual_duration_s':round(float(data.time-1.),3),
      'complete':not fail and k==num-1,
      'front_pair_clear_four_leg_balance_hold':success,
      'hold_contact_rate_4_support':round(float(np.mean([len(x['support_contact'])==4 for x in hold])),4) if hold else None,
      'hold_front_zero_contact_rate':round(float(np.mean([len(x['front_contact'])==0 for x in hold])),4) if hold else None,
      'hold_front_min_mesh_height_m':round(min(min(x['front_mesh_min_z_m'].values()) for x in hold),5) if hold else None,
      'hold_min_margin_m':round(min(x['support_contact_margin_m'] for x in hold if x['support_contact_margin_m'] is not None),5) if hold and all(x['support_contact_margin_m'] is not None for x in hold) else None,
      'hold_max_tilt_deg':round(max(x['tilt_deg'] for x in hold),3) if hold else None,
      'hold_com_y_mean_m':round(float(np.mean([x['com_xy_m'][1] for x in hold])),5) if hold else None,
      'hold_root_y_mean_m':round(float(np.mean([x['base_xyz_m'][1] for x in hold])),5) if hold else None,
      'max_tau_nm':round(max_tau,5),'saturation_step_fraction':round(sat/max(1,num),5),
      'failed':fail,'traces':rows,'qpos_frames':states,
      'warnings':['No real torque-speed model or PWM calibration','Static programmed shift, not adaptive neural feedback','Four-leg contact is proxy; margins from MuJoCo contact positions only','Visual proxy missing 2 CAD parts'],
    }
    return report

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser()
    p.add_argument('--shift',type=float,default=0.)
    p.add_argument('--lift',type=float,default=.035)
    p.add_argument('--scenario',choices=['nominal','lower','upper'],default='nominal')
    p.add_argument('--mu',type=float,default=1.1)
    p.add_argument('--frames',action='store_true')
    p.add_argument('--out',type=Path)
    a=p.parse_args()
    r=experiment(a.shift,a.lift,scenario=a.scenario,friction=a.mu,save_frames=a.frames)
    if a.out: a.out.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ('traces','qpos_frames')},ensure_ascii=False,indent=2))
