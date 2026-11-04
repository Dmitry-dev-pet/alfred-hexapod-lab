"""Math-first preflight of one 4-leg step on archived CAD30 + Mink.

Not a walking controller. Each step candidate gets 24 *virtual* kinematic
waypoints, checks nominal joint limits, support geometry and gravity-only
wrench/actuator constraints. It does not teleport a body or step a physics
simulator. Autonomy is vetoed until an entire 4-leg lookahead is validated.
Original Freenove mesh binaries must remain on private Google Drive.
"""
from __future__ import annotations
import argparse, json, math
from dataclasses import dataclass
from pathlib import Path

import mink
import mujoco as mj
import numpy as np
from scipy.optimize import linprog, minimize
from scipy.spatial import ConvexHull

STANCE=("ml","mr","bl","br")
FRONT=("fl","fr")
PREFIX="planning_contact_"

@dataclass
class Context:
    model: mj.MjModel
    qpos: np.ndarray
    qvel: np.ndarray
    foot_world: dict
    joint_qpos_addrs: np.ndarray
    joint_dof_addrs: np.ndarray
    limits: list
    freeze: list
    safe_snapshot_source: str


def load_context(model_xml: Path, saved_state: Path, max_joint_rad: float=1.7) -> Context:
    """Make four massless MuJoCo sites at *measured* settled tibia contacts."""
    record=json.loads(saved_state.read_text())
    q=np.asarray(record['qpos'],float);qvel=np.asarray(record['qvel'],float)
    original=mj.MjModel.from_xml_path(str(model_xml))
    if q.shape!=(original.nq,) or qvel.shape!=(original.nv,):
        raise ValueError('Saved physical replay state incompatible with CAD30 model')
    d=mj.MjData(original);d.qpos[:]=q;d.qvel[:]=qvel;mj.mj_forward(original,d)
    geom2leg={int(original.geom(f'tibia{k}_collision_0').id):k for k in (*STANCE,*FRONT)}
    pts={k:[] for k in (*STANCE,*FRONT)};fz={k:0. for k in (*STANCE,*FRONT)}
    floor=int(original.geom('floor').id)
    for i in range(d.ncon):
        c=d.contact[i]
        gid=int(c.geom2) if int(c.geom1)==floor else int(c.geom1) if int(c.geom2)==floor else -1
        if gid not in geom2leg:continue
        k=geom2leg[gid]; w=np.zeros(6);mj.mj_contactForce(original,d,i,w)
        force=c.frame.reshape(3,3).T@w[:3]
        pts[k].append((np.asarray(c.pos).copy(),max(0.,float(force[2]))))
        fz[k]+=float(force[2])
    present={k for k in (*STANCE,*FRONT) if fz[k]>.08}
    if present!=set(STANCE):raise RuntimeError(f'Saved state does not have 4 loaded stance legs: {present}')
    spec=mj.MjSpec.from_file(str(model_xml))
    feet={}
    for leg in STANCE:
        body_id=int(original.body('tibia'+leg).id)
        p=np.average([x for x,w in pts[leg]],axis=0,
                     weights=[max(1e-8,w) for x,w in pts[leg]])
        rot=np.asarray(d.xmat[body_id]).reshape(3,3)
        p_local=rot.T@(p-np.asarray(d.xpos[body_id]))
        spec.body('tibia'+leg).add_site(name=PREFIX+leg,pos=p_local,
             type=mj.mjtGeom.mjGEOM_SPHERE,size=(.0005,),rgba=(0.,1.,0.,0.))
        feet[leg]=p.copy()
    model=spec.compile()
    assert (model.nq,model.nv,model.nu,model.nmesh)==(original.nq,original.nv,original.nu,original.nmesh)
    assert np.allclose(model.body_mass,original.body_mass) and np.allclose(model.body_inertia,original.body_inertia)
    jids=[int(model.body(part+leg).jntadr[0]) for leg in (*STANCE,*FRONT) for part in ('coxa','femur','tibia')]
    qa=np.array([int(model.jnt_qposadr[j]) for j in jids],int)
    dofs=np.array([int(model.jnt_dofadr[j]) for j in jids],int)
    if len(set(dofs))!=18:raise RuntimeError('Expected 18 independent leg joints')
    if np.max(np.abs(q[qa]))>max_joint_rad+1e-7:
        raise RuntimeError('Initial safe state exceeds requested research joint bound')
    for j in jids:
        model.jnt_range[j]=[-max_joint_rad,max_joint_rad]
    free=set(int(x) for x in dofs[:12])
    frozen=sorted(set(range(model.nv))-free)
    speed={str(model.joint(j).name):np.array([.02]) for j in jids[:12]}
    limits=[mink.ConfigurationLimit(model),mink.VelocityLimit(model,speed)]
    conf=mink.Configuration(model);conf.update(q)
    reconstructed={leg:conf.get_transform_frame_to_world(PREFIX+leg,'site').translation() for leg in STANCE}
    if max(np.linalg.norm(reconstructed[k]-feet[k]) for k in STANCE)>1e-8:
        raise RuntimeError('Contact sites do not reconstruct physical positions')
    return Context(model,q,qvel,feet,qa,dofs,limits,
                   [mink.DofFreezingTask(model,dof_indices=frozen)],record.get('source',''))


def com_xyz(model,data):
    return np.sum(data.xipos*model.body_mass[:,None],axis=0)/model.body_mass.sum()


def margin_xy(model,data,support):
    if len(support)<3:return float('-inf')
    pts=np.array([data.site_xpos[model.site(PREFIX+k).id,:2] for k in support])
    h=ConvexHull(pts).equations
    return float(np.min(-(h[:,:2]@com_xyz(model,data)[:2]+h[:,2])))


def wrench_feasibility(model,data,support,mu=1.1,min_fz=.2):
    """Nominal static force LP; equations as in support_wrench_qp.py.

    Unlike live physical contact helper, future positions have no measured
    Fz: put one *hypothetical* point force at each anchored stance site.
    Feasibility is only quasi-static. Friction 32-facet *inner* approximation.
    Optimization minimizes common actuator-limit utilization (not fixed 90%).
    """
    n=len(support);nv=model.nv
    pts=np.array([data.site_xpos[model.site(PREFIX+k).id].copy() for k in support])
    com=com_xyz(model,data)
    eq=np.zeros((6,3*n+1));eq[:3,:3*n]=np.tile(np.eye(3),(1,n))
    rhs=np.array([0.,0.,float(model.body_mass.sum())*-float(model.opt.gravity[2]),0.,0.,0.])
    temp=mj.MjData(model);temp.qpos[:]=data.qpos;temp.qvel[:]=0;temp.qacc[:]=0;mj.mj_forward(model,temp)
    h=np.zeros(nv);mj.mj_rne(model,temp,0,h)
    jids=model.actuator_trnid[:,0].astype(int);dofs=model.jnt_dofadr[jids].astype(int)
    h=h[dofs];G=np.zeros((model.nu,3*n))
    for i,leg in enumerate(support):
        x,y,z=pts[i]-com
        eq[3:,3*i:3*i+3]=[[0,-z,y],[z,0,-x],[-y,x,0]]
        jp=np.zeros((3,nv));jr=np.zeros((3,nv))
        mj.mj_jacSite(model,temp,jp,jr,model.site(PREFIX+leg).id)
        G[:,3*i:3*i+3]=jp[:,dofs].T
    aub=[];bub=[]
    for i in range(n):
        for theta in np.linspace(0.,2.*np.pi,32,endpoint=False):
            row=np.zeros(3*n+1)
            row[3*i:3*i+3]=[np.cos(theta),np.sin(theta),-mu*np.cos(np.pi/32)]
            aub.append(row);bub.append(0.)
    cap=np.asarray(model.actuator_ctrlrange[:,1])
    for sign in (-1.,1.):
        for i in range(model.nu):
            row=np.zeros(3*n+1);row[:3*n]=-sign*G[i,:];row[-1]=-cap[i]
            aub.append(row);bub.append(-sign*h[i])
    cost=np.zeros(3*n+1);cost[-1]=1.
    lp=linprog(cost,A_eq=eq,b_eq=rhs,A_ub=aub,b_ub=bub,
               bounds=[(None,None),(None,None),(min_fz,None)]*n+[(0,None)],method='highs')
    if not lp.success:
        return {'feasible':False,'reason':lp.message,'min_motor_fraction':None}
    forces=lp.x[:-1].reshape(n,3)
    return {'feasible':True,'min_motor_fraction':float(lp.x[-1]),
            'min_normal_force_N':float(forces[:,2].min()),
            'forces_z_N':{leg:float(f[2]) for leg,f in zip(support,forces)},
            'wrench_balance_residual':float(np.max(np.abs(eq@lp.x-rhs)))}


class StepPreflight:
    def __init__(self,ctx:Context):
        self.ctx=ctx;self.c=mink.Configuration(ctx.model);self.c.update(ctx.qpos)
        self.targets={leg:p.copy() for leg,p in ctx.foot_world.items()}
        self.posture=mink.PostureTask(ctx.model,cost=1e-5)
        self.posture.set_target_from_configuration(self.c)
        self.tasks={leg:mink.FrameTask(PREFIX+leg,'site',position_cost=10.,orientation_cost=0.) for leg in STANCE}
        self.rows=[]

    def checkpoint(self,label,support,base_delta=(0.,0.,0.)):
        m=self.ctx.model;v=np.zeros(m.nv);v[:3]=base_delta
        if np.any(v):
            new=self.c.q.copy();mj.mj_integratePos(m,new,v,1.);self.c.update(new)
        for leg,t in self.tasks.items():
            T=self.c.get_transform_frame_to_world(PREFIX+leg,'site').as_matrix()
            T[:3,3]=self.targets[leg];t.set_target(mink.SE3.from_matrix(T))
        err=float('inf')
        for it in range(100):
            v=mink.solve_ik(self.c,[*self.tasks.values(),self.posture],dt=1.,solver='daqp',damping=1e-8,
                            limits=self.ctx.limits,constraints=self.ctx.freeze)
            self.c.update(self.c.integrate(v,dt=1.))
            err=max(np.linalg.norm(self.c.get_transform_frame_to_world(PREFIX+leg,'site').translation()-self.targets[leg]) for leg in STANCE)
            if err<.0002:break
        data=mj.MjData(m);data.qpos[:]=self.c.q;mj.mj_forward(m,data)
        mar=margin_xy(m,data,support)
        w=wrench_feasibility(m,data,support)
        maxq=float(np.max(abs(self.c.q[self.ctx.joint_qpos_addrs])))
        row={'phase':label,'support':list(support),'max_contact_target_error_mm':float(err*1000),
             'max_abs_joint_rad':maxq,'margin_mm':float(mar*1000),
             'minimum_motor_fraction':w.get('min_motor_fraction'),
             'min_model_contact_Fz_N':w.get('min_normal_force_N'),
             'lp_success':w.get('feasible',False),'qp_iterations':it+1,'qpos':self.c.q.copy().tolist()}
        row['gate_passed']=bool(err<.0002 and maxq<=1.700001 and mar>=.005 and w.get('feasible') and w['min_motor_fraction']<=.9)
        self.rows.append(row)
        return row

    def attempt_rear_step(self,leg:str,body_xyz_m,foothold_forward_m=.020,swing_z_m=.010):
        if leg not in ('bl','br'):raise ValueError('Prototype handles one rear-foot candidate, not a complete gait')
        triple=tuple(x for x in STANCE if x!=leg)
        shift=np.asarray(body_xyz_m,float)
        for n in range(12):
            r=self.checkpoint(f'prebalance_{n+1}',STANCE,base_delta=shift/12)
            if not r['gate_passed']:return {'status':'BLOCKED_AT_NODE','blocking_node':r,'nodes':self.rows}
        y0=self.targets[leg][1];z0=self.targets[leg][2]
        for n in range(4):
            self.targets[leg][2]=z0+swing_z_m*(n+1)/4
            r=self.checkpoint(f'lift_{n+1}',triple)
            if not r['gate_passed']:return {'status':'BLOCKED_AT_NODE','blocking_node':r,'nodes':self.rows}
        for n in range(4):
            self.targets[leg][1]=y0+foothold_forward_m*(n+1)/4
            r=self.checkpoint(f'swing_{n+1}',triple)
            if not r['gate_passed']:return {'status':'BLOCKED_AT_NODE','blocking_node':r,'nodes':self.rows}
        for n in range(4):
            self.targets[leg][2]=z0+swing_z_m*(3-n)/4
            r=self.checkpoint(f'touchdown_{n+1}',STANCE if n==3 else triple)
            if not r['gate_passed']:return {'status':'BLOCKED_AT_NODE','blocking_node':r,'nodes':self.rows}
        return {'status':'ONE_STEP_STATIC_NODES_PASS','nodes':self.rows,
                'final_foot_world':{k:p.tolist() for k,p in self.targets.items()}}


def required_com_shift_for_middle_after_rear_steps(ctx, rear_forward_m=.020, margin_m=.005):
    """Projection is geometric only, ignores joint reach and force feasibility."""
    d=mj.MjData(ctx.model);d.qpos[:]=ctx.qpos;mj.mj_forward(ctx.model,d)
    orig=com_xyz(ctx.model,d)[:2]
    feet={k:v[:2].copy() for k,v in ctx.foot_world.items()}
    for k in ('bl','br'):feet[k][1]+=rear_forward_m
    out={}
    for missing in ('ml','mr'):
        tri=np.array([p for k,p in feet.items() if k!=missing]);h=ConvexHull(tri).equations
        current=float(np.min(-(h[:,:2]@orig+h[:,2]))*1000)
        cons={'type':'ineq','fun':lambda x:-(h[:,:2]@x+h[:,2]+margin_m),'jac':lambda x:-h[:,:2]}
        r=minimize(lambda x:.5*float(np.dot(x-orig,x-orig)),orig,jac=lambda x:x-orig,
                   method='SLSQP',constraints=cons,options={'ftol':1e-12,'maxiter':500})
        if np.max(h[:,:2]@r.x+h[:,2]+margin_m)>1e-6:raise RuntimeError('COM projection failed')
        out[missing]={'current_three_support_margin_mm':current,
                      'minimum_com_shift_for_5mm_margin_mm':float(np.linalg.norm(r.x-orig)*1000)}
    return out


def compact(report):
    return {'status':report['status'],
            'node_count':len(report['nodes']),
            'min_margin_mm':min(x['margin_mm'] for x in report['nodes']),
            'max_abs_joint_rad':max(x['max_abs_joint_rad'] for x in report['nodes']),
            'max_target_error_mm':max(x['max_contact_target_error_mm'] for x in report['nodes']),
            'max_motor_fraction':max(x['minimum_motor_fraction'] or float('inf') for x in report['nodes']),
            'blocked_node':report.get('blocking_node',{}).get('phase'),
            'autonomous_first_step_authorized':False,
            'reason':'No proven four-step horizon / terminal viability; kinematic/LP snapshots are not physical contacts.'}


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--model',type=Path,required=True);p.add_argument('--safe-state-json',type=Path,required=True)
    p.add_argument('--out',type=Path)
    args=p.parse_args();ctx=load_context(args.model,args.safe_state_json)
    cases={}
    for leg in ('bl','br'):
        result=StepPreflight(ctx).attempt_rear_step(leg,body_xyz_m=(0.,0.,.024))
        cases[leg]=compact(result)
    result={'schema':'mink_cad30_step_g2_local_diagnostic_v1','mujoco':mj.__version__,
            'mink':mink.__version__ if hasattr(mink,'__version__') else '1.4.0',
            'safe_snapshot':str(args.safe_state_json.name),'cases':cases,
            'after_two_hypothetical_rear_20mm':required_com_shift_for_middle_after_rear_steps(ctx),
            'four_step_cycle_found':False,'autonomous_first_step_authorized':False,
            'dynamics':'NOT CHECKED BY THIS SCRIPT',
            'warning':'Initial historical B trajectory exceeded provisional ±1.7 rad; no claim of strict 6-to-4 start.'}
    payload=json.dumps(result,ensure_ascii=False,indent=2)
    if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(payload+'\n')
    print(payload)

if __name__=='__main__':main()