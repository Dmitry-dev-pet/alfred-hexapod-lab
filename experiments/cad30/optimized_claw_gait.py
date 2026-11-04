"""Mathematics-informed front-claw support planning on the original CAD30 FNK0052.

All choices of lifting 1/2 front legs and any additional stance-leg swing
are checked with the current model's COM, contact locations, friction cone,
free-body gravity wrench, joint Jacobians and actuator torque assumptions.
Force tracking is a bounded heuristic around a centralized QP/LP oracle; it
DOES NOT guarantee global dynamic optimality or real-world servo safety.

Original third-party STL meshes remain in a separate private archive.
"""
from __future__ import annotations
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import numpy as np
import mujoco
from scipy.optimize import minimize
from scipy.spatial import ConvexHull

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(HERE))
from support_wrench_qp import StandData, support_margin_xy, minimize_motor_fraction, build_constraints
from hexlab.stand_v04 import static_pose, hold_under_gravity
from hexlab.stand_geometry_v04 import load_legs, tip_world, geom_mesh_world_vertices
from hexlab.step_one_leg_v04 import smoothstep, solve_waypoint, floor_contacts

FRONT=('fl','fr')  # physically frontmost CAD legs, vendor labels are mirrored
ALL=('ml','bl','br','mr','fr','fl')
STABILITY_MARGIN=0.016
MU_DEFAULT=1.1


def optimal_com_target(com_xy:np.ndarray, foot_points:np.ndarray, margin_m:float=STABILITY_MARGIN)->np.ndarray:
    """Closest possible COM projection within an eroded support polygon.

    ConvexHull edges are normalized; the exact solution of min ||target-com||²
    subject to signed half-plane distances >= margin is a convex QP in 2D.
    """
    h=ConvexHull(np.asarray(foot_points,float))
    A=h.equations[:,:2];b=h.equations[:,2]
    z=np.asarray(com_xy,float)
    if np.all(A@z+b <= -margin_m+1e-10):return z.copy()
    out=minimize(lambda x:.5*float(np.sum((x-z)**2)),z,
                 jac=lambda x:x-z, method='SLSQP',
                 constraints={'type':'ineq','fun':lambda x:-(A@x+b+margin_m),
                              'jac':lambda x:-A},
                 options={'maxiter':150,'ftol':1e-13})
    if np.max(A@out.x+b+margin_m)>1e-7:
        raise ValueError('Support polygon too small for specified COM margin')
    return out.x.copy()


def physical_contact_forces(model,data):
    """Return aggregate per-leg force / normal-force-weighted contact location."""
    ground=model.geom('floor').id
    inv={model.geom(f'tibia{c}_collision_0').id:c for c in ALL}
    forces={c:np.zeros(3) for c in ALL}; weighted={c:np.zeros(3) for c in ALL};weight={c:0. for c in ALL}
    w=np.zeros(6)
    for i in range(data.ncon):
        contact=data.contact[i];a,b=int(contact.geom1),int(contact.geom2)
        gid=b if a==ground else a if b==ground else -1
        if gid not in inv:continue
        c=inv[gid];mujoco.mj_contactForce(model,data,i,w)
        world=contact.frame.reshape(3,3).T@w[:3]
        forces[c]+=world
        fz=max(0.,float(world[2]));weighted[c]+=contact.pos*fz;weight[c]+=fz
    points={c:(weighted[c]/weight[c] if weight[c]>1e-8 else None) for c in ALL}
    return forces,points


def balance_forces_qp(stand, disabled, mu, original_lp):
    """Secondary convex QP: flatter normal loads within a motor safety budget.

    A feasible max-torque LP is the mandatory first stage. Never sacrifice
    wrench/friction/actuator constraints to improve load uniformity.
    """
    t=float(original_lp['min_motor_fraction'])
    reserve_cap=min(.92,max(.75,t+.1))
    p=build_constraints(stand,disabled,mu,torque_cap=False,friction_approx='inner32')
    n=len(stand.codes)
    B=np.vstack((p['Aub'],-stand.G,stand.G))
    bb=np.r_[p['bub'],reserve_cap*stand.tau_bound-stand.h,
             reserve_cap*stand.tau_bound+stand.h]
    x0=np.asarray(original_lp['force_solution_world_N'],float).reshape(-1)
    selected=np.array([c not in disabled for c in stand.codes],dtype=bool)
    uniform=float(stand.model_mass_kg*(-stand.model.opt.gravity[2])/np.sum(selected))
    prior=stand.measured.reshape(n,3)
    def cost(x):
        f=x.reshape(n,3)
        penalty=np.sum((f[selected,2]-uniform)**2)
        penalty+=.08*np.sum(f[selected,:2]**2)
        penalty+=.005*np.sum((f[selected,2]-prior[selected,2])**2)
        return float(penalty)
    result=minimize(cost,x0,method='SLSQP',bounds=p['bounds'],
                    constraints=[{'type':'eq','fun':lambda x:p['Aeq']@x-p['beq'],
                                 'jac':lambda x:p['Aeq']},
                                 {'type':'ineq','fun':lambda x:bb-B@x,
                                  'jac':lambda x:-B}],
                    options={'ftol':1e-9,'maxiter':140})
    selected_force=result.x if result.success else x0
    eq=np.max(np.abs(p['Aeq']@selected_force-p['beq']))
    ineq=max(0.,float(np.max(B@selected_force-bb)))
    if eq>1e-5 or ineq>1e-6:selected_force=x0
    f=selected_force.reshape(n,3)
    ratio=float(np.max(abs(stand.h-stand.G@selected_force)/stand.tau_bound))
    return {'forces_world_N':{c:np.round(f[i],4).tolist() for i,c in enumerate(stand.codes)},
            'actual_max_torque_fraction':round(ratio,5),
            'target_vertical_force_std_N':round(float(np.std(f[selected,2])),4),
            'target_vertical_force_mean_N':round(float(np.mean(f[selected,2])),4),
            'used_balanced_slsqp':bool(result.success and eq<=1e-5 and ineq<=1e-6),
            'required_min_motor_fraction':t,'motor_fraction_budget':reserve_cap}

def force_oracle(model,data,legs,disabled,mu,require_present=True):
    """At actual qpos, minimize maximum motor fraction for static contacts.

    This is a *quasistatic* oracle; inertia is not included in the force targets.
    Solver response is evaluated against *actual dynamic* contact samples.
    """
    disabled=tuple(sorted(set(disabled)))
    com=(np.asarray(data.xipos)*np.asarray(model.body_mass)[:,None]).sum(axis=0)/model.body_mass.sum()
    forces,points=physical_contact_forces(model,data)
    present=[c for c in ALL if points[c] is not None]
    if require_present and any(c not in present for c in ALL if c not in disabled):
        return {'feasible':False,'reason':'one or more required contacts absent','present':present}
    if any(c not in disabled and c not in present for c in ALL):
        return {'feasible':False,'reason':'missing stance contact','present':present}
    p=np.array([points[c] if points[c] is not None else tip_world(model,data,next(x for x in legs if x['suffix']==c)) for c in ALL])
    margin=support_margin_xy(np.array([p[i,:2] for i,c in enumerate(ALL) if c not in disabled]),com[:2])
    if margin<=0:return {'feasible':False,'reason':'COM outside remaining support polygon','margin_m':margin,'present':present}
    # Copy kinematics with zero speed/accel to evaluate gravity-only joint wrench.
    d2=mujoco.MjData(model);d2.qpos[:]=data.qpos;d2.qvel[:]=0.;d2.qacc[:]=0.
    mujoco.mj_forward(model,d2)
    h=np.zeros(model.nv);mujoco.mj_rne(model,d2,0,h)
    ids=np.asarray(model.actuator_trnid[:,0],dtype=int)
    dof=np.asarray(model.jnt_dofadr[ids],dtype=int)
    G=np.zeros((model.nu,18))
    for i,c in enumerate(ALL):
        leg=next(l for l in legs if l['suffix']==c)
        jp=np.zeros((3,model.nv));jr=np.zeros((3,model.nv))
        mujoco.mj_jac(model,d2,jp,jr,p[i],int(leg['body']))
        G[:,3*i:3*i+3]=jp[:,dof].T
    s=StandData(model=model,data=d2,codes=ALL,positions=p,measured=np.array([forces[c] for c in ALL]),
                com=com,model_mass_kg=float(model.body_mass.sum()),h=h[dof],
                tau_bound=model.actuator_ctrlrange[:,1].copy(),G=G,
                nominal_tau=np.zeros(model.nu),initial_torque_residual=0.,original_stand_proxy={})
    x=minimize_motor_fraction(s,disabled,mu)
    if not x.get('feasible_for_some_motor_scale'):
        return {'feasible':False,'reason':'LP cannot balance all wrench constraints',
                'margin_m':margin,'present':present}
    tau=float(x['min_motor_fraction'])
    balanced=balance_forces_qp(s,disabled,mu,x) if tau<.95 else None
    return {'feasible':tau<.95 and margin>0.004, 'margin_m':round(float(margin),7),
            'min_motor_fraction':tau, 'present':present,
            'fz_targets_N':{c:float(balanced['forces_world_N'][c][2]) if balanced else float(x['force_solution_world_N'][i][2])
                            for i,c in enumerate(ALL)},
            'force_balancing':balanced,
            'fz_measured_N':{c:float(forces[c][2]) for c in ALL},
            'lost':list(disabled), 'reason':'ok' if tau<.95 and margin>0.004 else 'torque / geometric reserve too low'}


def choose_one_or_two(target:int,foot_xy:dict,com_xy:np.ndarray,mu:float):
    """Select front-claw set by geometric feasibility and required COM motion.

    One front claw needs no COM shift; two need explicit backwards shift.
    Caller subsequently checks full wrench/actuator optimization on measured pose.
    """
    if target not in (1,2):raise ValueError('target number of claws must be one or two')
    candidates=[('fl',),('fr',)] if target==1 else [('fl','fr')]
    out=[]
    for names in candidates:
        pts=np.array([foot_xy[c][:2] for c in ALL if c not in names])
        desired=optimal_com_target(com_xy,pts,margin_m=STABILITY_MARGIN)
        margin=support_margin_xy(pts,desired)
        out.append({'claws':list(names),'com_target':desired.tolist(),
                    'com_shift_m':(desired-com_xy).tolist(),'planned_support_margin_m':float(margin)})
    return min(out,key=lambda x:(np.linalg.norm(x['com_shift_m']),x['claws']))


def init_model(mu:float=MU_DEFAULT):
    model=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'))
    model.geom_friction[model.geom('floor').id,0]=mu
    for c in ALL:model.geom_friction[model.geom(f'tibia{c}_collision_0').id,0]=mu
    data=mujoco.MjData(model);legs=load_legs(model,data,mujoco)
    static_pose(model,data,mujoco,legs,.145,.15)
    stand=hold_under_gravity(model,data,mujoco,data.qpos.copy(),1.)
    if not stand['stable_by_proxy_thresholds'] or stand['last_0p4s_min_contact_legs']<6:
        raise RuntimeError('initial settled six-legged stand failed')
    return model,data,legs


def actuator_mapping(model,data,legs):
    ids=np.asarray(model.actuator_trnid[:,0],dtype=int)
    qa=np.asarray(model.jnt_qposadr[ids],dtype=int)
    da=np.asarray(model.jnt_dofadr[ids],dtype=int)
    m={l['suffix']:np.asarray([int(np.where(qa==a)[0][0]) for a in l['qadrs']],dtype=int) for l in legs}
    return qa,da,m


def shift_and_lift_plans(model,data,legs,needed_shift_m,claws,raise_m):
    """Precompute IK from the settled pose; shift magnitude is optimized, not a preset 40mm."""
    qa,da,idx=actuator_mapping(model,data,legs)
    first=data.qpos.copy();base=first[qa].copy()
    qshift={};qlift={}
    for l in legs:
        c=l['suffix'];d=mujoco.MjData(model)
        d.qpos[:]=first;d.qvel[:]=0.;mujoco.mj_forward(model,d)
        tip=tip_world(model,d,l).copy()
        if c not in claws:
            dest=tip+np.array([0.,needed_shift_m,0.])
            q,err=solve_waypoint(model,d,mujoco,l,dest,first[l['qadrs']].copy())
            if err>.005:raise RuntimeError(f'Support foot IK {c} error {err:.5f}')
            qshift[c]=q
        else:
            dest=tip+np.array([0.,0.,raise_m])
            q,err=solve_waypoint(model,d,mujoco,l,dest,first[l['qadrs']].copy())
            if err>.005:raise RuntimeError(f'Claw IK {c} error {err:.5f}')
            qlift[c]=q
    return qa,da,idx,base,qshift,qlift


def cartesian_shift_paths(model,data,legs,claws,shift_m,nsamples=21):
    """Solve foot IK at equal Cartesian increments, holding requested toe Z.

    Direct joint interpolation can unexpectedly lift a supporting tibia in the
    middle of the return motion; that would invalidate a 4/5 contact plan.
    """
    base=data.qpos.copy();paths={};errors={}
    for l in legs:
        c=l['suffix']
        if c in claws:continue
        ref=mujoco.MjData(model);ref.qpos[:]=base;ref.qvel[:]=0.;mujoco.mj_forward(model,ref)
        toe0=tip_world(model,ref,l).copy();previous=base[l['qadrs']].copy()
        table=[];errs=[]
        for a in np.linspace(0,1,nsamples):
            # Fresh kinematic pose for each inverse-Jacobian trial.
            ref.qpos[:]=base;ref.qvel[:]=0.;mujoco.mj_forward(model,ref)
            q,err=solve_waypoint(model,ref,mujoco,l,toe0+np.array([0.,shift_m*a,0.]),previous)
            if err>.005:raise RuntimeError(f'Cartesian support path IK {c} at {a:.3f}: err {err:.5f}')
            table.append(q.copy());errs.append(float(err));previous=q.copy()
        paths[c]=np.array(table);errors[c]=max(errs)
    return paths,errors


def sample_ik_path(path,progress):
    x=float(np.clip(progress,0.,1.))*(len(path)-1)
    j=min(int(x),len(path)-2);a=x-j
    return path[j]*(1.-a)+path[j+1]*a

def step_leg_plan(model,data,legs,code,forward_m=0.018,lift_m=.017):
    leg=next(l for l in legs if l['suffix']==code)
    ref=mujoco.MjData(model);ref.qpos[:]=data.qpos;ref.qvel[:]=0.;mujoco.mj_forward(model,ref)
    tip=tip_world(model,ref,leg)
    # +Y is front in the source CAD model.
    goal=tip+np.array([0.,forward_m,lift_m])
    q,err=solve_waypoint(model,ref,mujoco,leg,goal,data.qpos[leg['qadrs']].copy())
    return q,float(err)


def simulate(claw_count:int=2,mu:float=MU_DEFAULT,do_step:bool=True,
             use_force_feedback:bool=True,seconds:float=12.5,seed:int=0,
             record_frames:bool=True,support_shift_override:float|None=None):
    """Physical MuJoCo rollout: optimize support COM, lift claws, optionally step.

    If a planned stance is not feasible, lift/swing is vetoed. There are no
    kinematic translations of the body and no pinned toe positions.
    """
    model,data,legs=init_model(mu)
    z_sensitivities=prepare_sensitivities(model,data,legs)
    root=int(model.body('base_link_2').id)
    com0=(np.asarray(data.xipos)*np.asarray(model.body_mass)[:,None]).sum(axis=0)/model.body_mass.sum()
    measured,foot=physical_contact_forces(model,data)
    chosen=choose_one_or_two(claw_count,foot,com0[:2],mu)
    claws=tuple(chosen['claws'])
    # Body shifts opposite to support-foot IK command; past measured 0.85 ratio.
    # The output of the convex projection, not an arbitrary 40-mm preset,
    # determines the needed target; the actual physics must verify it.
    nominal_shift=max(0.,float(-chosen['com_shift_m'][1])/.85)
    nominal_shift=min(.061,nominal_shift)
    if support_shift_override is not None:
        nominal_shift=float(support_shift_override)
    qa,da,idx,qbase,qshift,qlift=shift_and_lift_plans(model,data,legs,nominal_shift,claws,.036)
    support_paths,path_errors=cartesian_shift_paths(model,data,legs,claws,nominal_shift)
    bounds=model.actuator_ctrlrange.copy();dt=float(model.opt.timestep)
    start_com=com0.copy();orientation=data.xmat[root].reshape(3,3).copy()
    up0=orientation.T@np.array([0.,0.,1.]);traces=[];frames=[];events=[]
    selected_step=None;stepq=None;step_startq=None;step_contacted=False
    recovered_front_contacts=False;recovery_approved=False
    recovery_shift_progress=1.; lost_recovery_support_steps=0
    lift_enabled=False;abort=False;max_tau=0.;sat=0;num=round(seconds/dt)
    force_offset={c:0. for c in ALL};target_Fz={c:0. for c in ALL}
    last_oracle=None;oracle_checks=0;accepted_checks=0;accepted_hold=0
    for k in range(num):
        t=k*dt
        # Reassess structural feasibility before front lift, at physical settled pose.
        if k == round(2.05/dt):
            w=force_oracle(model,data,legs,claws,mu)
            oracle_checks+=1;last_oracle=w
            lift_enabled=bool(w['feasible'])
            events.append({'t':round(t,3),'event':'front_lift_checked','allowed':lift_enabled,'oracle':w})
        # During flight, choose a feasible single support-leg swing by enumerating
        # the actual physical remaining-support possibilities.
        if k==round(3.55/dt) and lift_enabled and do_step:
            options=[]
            for c in ('bl','br','ml','mr'):
                if c in claws:continue
                w=force_oracle(model,data,legs,(*claws,c),mu)
                if w['feasible'] and w['margin_m']>=.010 and w['min_motor_fraction']<=.80:
                    options.append((float(w['margin_m'])-.015*float(w['min_motor_fraction']),c,w))
            options.sort(reverse=True)
            if options:
                attempted=[]
                for _,candidate,oracle in options:
                    for step_length,step_height in ((.018,.017),(.012,.012),(.008,.009),(-.008,.009),(-.015,.012),(-.025,.015),(0.,.015),(0.,.008)):
                        qtry,err=step_leg_plan(model,data,legs,candidate,
                                               forward_m=step_length,lift_m=step_height)
                        attempted.append([candidate,step_length,step_height,round(float(err),5)])
                        if err<=.005:
                            selected_step=candidate;stepq=qtry
                            step_startq=data.qpos[np.array(next(l for l in legs if l['suffix']==candidate)['qadrs'])].copy()
                            events.append({'t':round(t,3),'event':'stance_swing_selected','leg':selected_step,
                                           'IK_error_m':err,'step_forward_m':step_length,
                                           'step_lift_m':step_height,'oracle':oracle})
                            break
                    if selected_step:break
                if selected_step is None:
                    events.append({'t':round(t,3),'event':'stance_swing_veto',
                                   'reason':'candidate stance-leg swing IK unreachable','attempted':attempted})
            else:
                events.append({'t':round(t,3),'event':'stance_swing_veto','reason':'no three-support feasible candidate'})
        # Live local force correction is bounded and based on the optimized
        # force distribution; QP is NOT used as a hidden root-pose actuator.
        if lift_enabled and t>=3.25 and t<5.15 and k % round(.10/dt)==0:
            disabled=list(claws)
            if selected_step and 3.68<=t<4.32:disabled.append(selected_step)
            w=force_oracle(model,data,legs,disabled,mu)
            oracle_checks+=1
            if w['feasible']:
                accepted_checks+=1;last_oracle=w;target_Fz=w['fz_targets_N']
            elif (w.get('margin_m') or 0)<-.003:
                abort=True;events.append({'t':round(t,3),'event':'abort_static_margin','oracle':w})
        base=qbase.copy()
        # Preparation and body re-centering: all remaining legs move support
        # targets together; physical body COM then responds via contacts.
        if t<2.0: support_progress=smoothstep(min(1.,t/2.0))
        elif t<6.5: support_progress=1.0
        else:
            # Never move the COM forwards until *both* claws are back on the
            # floor; doing both simultaneously caused negative support margin.
            contacts_now=floor_contacts(model,data,mujoco)
            if set(claws).issubset(contacts_now):recovered_front_contacts=True
            if recovered_front_contacts:recovery_approved=True
            # Vary the return pace using *current* support status, not a
            # clock-only trajectory. Freeze/retreat when a support detaches.
            present_now=floor_contacts(model,data,mujoco)
            if recovery_approved and len(present_now)==6:
                recovery_shift_progress=max(0.,recovery_shift_progress-dt/3.)
            elif recovery_approved and len(present_now)==5:
                lost_recovery_support_steps+=1
                recovery_shift_progress=max(0.,recovery_shift_progress-dt/9.)
            else:
                lost_recovery_support_steps+=1
                recovery_shift_progress=min(1.,recovery_shift_progress+dt/3.)
            support_progress=recovery_shift_progress
        for c,q in qshift.items():
            # Path synthesized in task-space so the supporting toe should not
            # leave the ground due to a curved joint-space interpolation.
            base[idx[c]]=sample_ik_path(support_paths[c],support_progress)
        # Claws are lifted AFTER oracle approves the support set.
        if lift_enabled and not abort:
            if t<2.15:alpha=0.
            elif t<3.25:alpha=smoothstep((t-2.15)/1.1)
            elif t<5.25:alpha=1.
            elif t<6.25:alpha=1.-smoothstep(t-5.25)
            else:alpha=0.
            for c,q in qlift.items():base[idx[c]]=qbase[idx[c]]+alpha*(q-qbase[idx[c]])
        # A single support-leg step occurs only when the three-support oracle
        # considers the remaining legs capable of bearing the weight.
        if selected_step and not abort and 3.65<=t<4.55:
            a=(smoothstep((t-3.65)/.35) if t<4.0 else
               (1.0 if t<4.15 else 1.-smoothstep((t-4.15)/.4)))
            base[idx[selected_step]]=step_startq+a*(stepq-step_startq)
        # Apply small foot-z offsets in the supporting leg positions, to track
        # the continuously solved min-max-torque supporting load allocation.
        if use_force_feedback and lift_enabled and not abort and 3.25<t<5.1:
            contact_f,_=physical_contact_forces(model,data)
            for c in ALL:
                if c in claws or (selected_step==c and 3.65<=t<4.55):
                    force_offset[c]=0.;continue
                if float(target_Fz.get(c,0))>0 and float(contact_f[c][2])>0.15:
                    error=float(target_Fz[c]-contact_f[c][2])
                    # Downward foot command increases this leg's force, generally;
                    # motor/ground nonlinearity makes this only a feedback heuristic.
                    force_offset[c]=float(np.clip(force_offset[c]+.009*error*dt,-.0018,.0018))
                else:force_offset[c]*=.98
        else:
            for c in ALL:force_offset[c]*=.97
        if recovery_approved and t>=6.5:
            current_contacts=floor_contacts(model,data,mujoco)
            for c in ALL:
                if c not in current_contacts:
                    force_offset[c]=float(np.clip(force_offset[c]+.014*dt,0.,.006))
                else:
                    force_offset[c]=float(np.clip(force_offset[c]-.006*dt,0.,.006))
        # Approximate z correction by local IK derivative at the standing pose.
        # Derivative solve is precomputed separately below using finite Jacobians.
        if (use_force_feedback or recovery_approved) and lift_enabled and not abort:
            for c in ALL:
                if c in claws or (c==selected_step and 3.65<=t<4.55):continue
                dq=z_sensitivities[c]
                base[idx[c]]+=dq*force_offset[c]
        torque=np.clip(7.*(base-data.qpos[qa])-.32*data.qvel[da],bounds[:,0],bounds[:,1])
        sat+=int(np.any(np.abs(7.*(base-data.qpos[qa])-.32*data.qvel[da])>bounds[:,1]+1e-9))
        max_tau=max(max_tau,float(np.max(np.abs(torque))))
        data.ctrl[:]=torque
        mujoco.mj_step(model,data)
        if k%10==0 or k==num-1:
            cm=(np.asarray(data.xipos)*np.asarray(model.body_mass)[:,None]).sum(axis=0)/model.body_mass.sum()
            f,p=physical_contact_forces(model,data)
            present=[c for c in ALL if p[c] is not None]
            cm_margin=support_margin_xy(np.array([p[c][:2] for c in present]),cm[:2]) if len(present)>=3 else -1.
            tilt=math.degrees(math.acos(float(np.clip((data.xmat[root].reshape(3,3)@up0)[2],-1.,1.))))
            heights={c:float(geom_mesh_world_vertices(model,data,next(l for l in legs if l['suffix']==c)['geom'])[:,2].min()) for c in claws}
            tr={'t':round(t+dt,3),'com_xyz':np.round(cm,6).tolist(),'actual_supports':present,
                'n_contacts':len(present),'support_margin_mm':round(cm_margin*1000.,3),
                'claw_clearance_m':{c:round(z,5) for c,z in heights.items()},
                'tilt_deg':round(tilt,4),'max_abs_tau_nm':round(float(np.max(abs(torque))),5),
                'motor_fraction':round(float(np.max(abs(torque)/bounds[:,1])),4),
                'normal_forces_N':{c:round(float(f[c][2]),3) for c in ALL},
                'force_z_offsets_mm':{c:round(1000*force_offset[c],3) for c in ALL}}
            traces.append(tr)
            # If the actual COM leaves the observed support polygon, this
            # attempt is no longer safe. Command the claws to recover and
            # reject the outcome (even if the static oracle was feasible).
            if t>3.20 and len(present)>=3 and cm_margin < -.002 and not abort:
                abort=True
                events.append({'t':round(t+dt,3),'event':'abort_negative_dynamic_support_margin',
                               'margin_mm':round(cm_margin*1000,3)})
            if tilt>35 or cm[2]<.06 or not np.all(np.isfinite(data.qpos)):
                abort=True;events.append({'t':round(t+dt,3),'event':'physics_abort','tilt':tilt,'z':cm[2]});break
        if record_frames and k%round(.10/dt)==0:
            frames.append({'t':round(t+dt,3),'qpos':np.round(data.qpos,7).tolist()})
    win=[r for r in traces if 3.35<=r['t']<5.05]
    no_claw_contact=bool(win) and all(all(c not in r['actual_supports'] for c in claws) for r in win)
    if not lift_enabled or any(r['support_margin_mm']<0 for r in win):
        no_claw_contact=False
    late=[r for r in traces if r['t']>=3.25]
    all_phase_margin_ok=bool(late) and all(r['support_margin_mm']>=0 for r in late)
    all_phase_at_least_three=bool(late) and all(r['n_contacts']>=3 for r in late)
    final_contacts=traces[-1]['actual_supports'] if traces else []
    complete_round_trip=(no_claw_contact and not abort and all_phase_margin_ok and
                         all_phase_at_least_three and recovery_approved and
                         recovery_shift_progress<.01 and set(final_contacts)==set(ALL))
    return {'schema':'FNK0052_math_planned_claw_balance/1','claw_count_requested':claw_count,
            'claws':list(claws),'mu':mu,'model_mass_kg':float(model.body_mass.sum()),
            'static_balance_target':chosen,'planned_support_shift_m':round(nominal_shift,5),
            'max_taskspace_support_ik_error_mm':round(1000*max(path_errors.values() or [0]),3),
            'model_actuator_cap_is_assumed':True,'math_assisted_centralized':True,
            'lift_enabled_by_static_oracle':lift_enabled,'selected_stance_step':selected_step,
            'posture_reach_success':no_claw_contact,
            'full_cycle_clean_support':complete_round_trip,
            'min_support_margin_mm_after_lift':round(min((r['support_margin_mm'] for r in late), default=-1000.),3),
            'min_contact_count_after_lift':min((r['n_contacts'] for r in late),default=0),
            'abort':abort,'duration_s':round(float(data.time-1.),3),
            'front_contacts_restored_before_recenter':recovery_approved,
            'recovery_shift_fraction_remaining':round(recovery_shift_progress,4),
            'recovery_steps_with_missing_contacts':lost_recovery_support_steps,
            'peak_tau_nm':round(max_tau,5),'motor_saturation_step_fraction':round(sat/max(1,k+1),5),
            'oracle_checks':oracle_checks,'oracle_accepted':accepted_checks,
            'hold_min_n_contacts':min([x['n_contacts'] for x in win],default=None),
            'hold_min_support_margin_mm':min([x['support_margin_mm'] for x in win],default=None),
            'hold_max_tilt_deg':max([x['tilt_deg'] for x in win],default=None),
            'events':events,'traces_50hz':traces,'qpos_frames':frames,
            'limitations':['Friction/torque/mass uncalibrated; model-only simulation',
              'Optimizer re-plans at 10Hz; local foot servo force tracking is heuristic',
              'QP is quasi-static, not a full optimal dynamic MPC controller',
              'Claw actions and supporting leg swing are research maneuvers, not robust periodic gait']}


def ik_reachable_support_shift(model,data,legs,claws,minimum:float)->float:
    """Bisection for largest bounded-Jacobian IK foot shift. No free-root command."""
    lo=max(0.,float(minimum));hi=.061
    # This is only an IK candidate check; physical loads still tested by MuJoCo.
    for _ in range(9):
        mid=(lo+hi)/2.
        try:
            shift_and_lift_plans(model,data,legs,mid,claws,.036)
        except RuntimeError:
            hi=mid
        else:
            lo=mid
    return max(float(minimum),lo-.0008)


def plan_shift_with_model(claw_count:int,mu:float,verbose:bool=False)->dict:
    """Use short real-physics candidate rollouts, then choose Pareto compromise.

    Desired support set is given; balance candidates are math+IK-constrained,
    the oracle measures max torque utilization, load standard deviation and
    geometric support margin in simulated pre-lift free-body dynamics.
    There is NO manual 40mm balance shift.
    """
    model,data,legs=init_model(mu)
    forces,pts=physical_contact_forces(model,data)
    com=(np.asarray(data.xipos)*np.asarray(model.body_mass)[:,None]).sum(axis=0)/model.body_mass.sum()
    shape=choose_one_or_two(claw_count,pts,com[:2],mu)
    needed=max(0.,-shape['com_shift_m'][1]/.85)
    if claw_count==1:
        return {'shift_m':0.,'planned_claws':shape['claws'],'candidate_receipts':[],
                'method':'one claw: already deep inside 5-support polygon; avoid unnecessary COM displacement'}
    max_shift=ik_reachable_support_shift(model,data,legs,shape['claws'],needed)
    candidates=np.linspace(needed,max_shift,5)
    checks=[]
    for shift in candidates:
        try:
            result=simulate(claw_count,mu,False,False,seconds=2.12,record_frames=False,
                            support_shift_override=float(shift))
            status=next((e['oracle'] for e in result['events'] if e['event']=='front_lift_checked'),{})
            balance=status.get('force_balancing') or {}
            frac=status.get('min_motor_fraction',1.5)
            spread=balance.get('target_vertical_force_std_N',10.)
            mar=status.get('margin_m',-1.)
            # Every candidate must have a positive reserve and nonnegative
            # support wedge; a score is not an unsafe-configuration override.
            feasible=bool(status.get('feasible')) and mar>=.010 and frac<=.85
            # Minimize Fz disparity (scaled by weight), motor demand and
            # proximity to the support polygon edge simultaneously.
            score=float(.60*spread/4.426+.32*frac+.08*(.030/max(.005,mar)))
            if not feasible:score=float('inf')
            checks.append({'shift_m':round(float(shift),6),'feasible':feasible,
                           'predicted_post_lift_load_std_N':spread,
                           'predicted_motor_fraction':frac,
                           'predicted_support_margin_mm':round(1000*mar,3),
                           'score':None if not np.isfinite(score) else round(score,6)})
        except (RuntimeError,ValueError) as exc:
            checks.append({'shift_m':round(float(shift),6),'feasible':False,
                           'error':str(exc),'score':None})
    feasible=[x for x in checks if x['feasible']]
    if not feasible:
        return {'shift_m':None,'planned_claws':shape['claws'],
                'candidate_receipts':checks,'method':'no feasible model-based support target'}
    selected=min(feasible,key=lambda x:(x['score'],x['shift_m']))
    return {'shift_m':selected['shift_m'],'planned_claws':shape['claws'],
            'candidate_receipts':checks,
            'minimum_shift_from_support_polygon_m':round(float(needed),6),
            'max_shift_from_CAD_IK_m':round(float(max_shift),6),
            'method':'5 candidate physical prep rollouts; LP motor-reserve plus QP contact-load spread and COM support margin'}

def prepare_sensitivities(model,data,legs):
    """Joint q delta per 1 metre of tip Z motion, using least-squares Jacobian."""
    out={}
    for l in legs:
        jp=np.zeros((3,model.nv));jr=np.zeros((3,model.nv))
        p=tip_world(model,data,l)
        mujoco.mj_jac(model,data,jp,jr,p,int(l['body']))
        local=jp[:,l['dadrs']]
        out[l['suffix']]=np.linalg.lstsq(local,np.array([0.,0.,-1.]),rcond=1e-8)[0]
    return out


_PLAN_CACHE={}

def run_trial(claws:int=2,mu:float=1.1,do_step:bool=True,use_force_feedback:bool=True,
              plan_optimized:bool=True):
    # The planner is always a separate physics experiment, not an invisible
    # kinematic movement of the root during the subsequent rollout.
    if plan_optimized and (claws,float(mu)) not in _PLAN_CACHE:
        _PLAN_CACHE[(claws,float(mu))]=plan_shift_with_model(claws,mu)
    planner=_PLAN_CACHE[(claws,float(mu))] if plan_optimized else None
    if planner and planner['shift_m'] is None:
        return {'schema':'FNK0052_math_planned_claw_balance/1',
                'claw_count_requested':claws,'lift_enabled_by_static_oracle':False,
                'planning':planner,'reason':'no safe model-based support candidate'}
    result=simulate(claws,mu,do_step,use_force_feedback,
                    support_shift_override=planner['shift_m'] if planner else None)
    result['planning']=planner
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--claws',choices=('1','2','auto'),default='auto')
    parser.add_argument('--mu',type=float,default=1.1)
    parser.add_argument('--no-step',action='store_true')
    parser.add_argument('--no-plan',action='store_true')
    parser.add_argument('--priority',choices=('mobility','claws'),default='mobility',
                        help='In auto mode, prefer a verified stance-leg step or two elevated claws')
    parser.add_argument('--no-force-feedback',action='store_true')
    parser.add_argument('--out',type=Path)
    args=parser.parse_args()
    if args.claws=='auto':
        attempts=[]
        result=None
        options=([(2,True),(1,True),(2,False),(1,False)] if args.priority=='mobility'
                 else [(2,True),(2,False),(1,True),(1,False)])
        if args.no_step:options=[(2,False),(1,False)]
        for candidate,step in options:
            trial=run_trial(candidate,args.mu,step,not args.no_force_feedback,not args.no_plan)
            attempts.append({'claw_count':candidate,'stance_step':step,
                             'lift_allowed':trial.get('lift_enabled_by_static_oracle',False),
                             'physical_hold_success':trial.get('posture_reach_success',False),
                             'complete_cycle_with_nonnegative_margin':trial.get('full_cycle_clean_support',False),
                             'aborted':trial.get('abort',False),
                             'planning':trial.get('planning')})
            result=trial
            if trial.get('full_cycle_clean_support',False):break
        result['automatic_fallback_trials']=attempts
        result['automatic_mode_priority']=args.priority
        result['automatic_motion_approved']=bool(result.get('full_cycle_clean_support'))
    else:
        result=run_trial(int(args.claws),args.mu,not args.no_step,
                         not args.no_force_feedback,not args.no_plan)
    if args.out:
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('traces_50hz','qpos_frames')},ensure_ascii=False,indent=2))