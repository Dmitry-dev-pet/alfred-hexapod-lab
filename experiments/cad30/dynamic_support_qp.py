"""Experimental whole-body dynamic QP for *candidate* two-support contact mode.

Not a verified gait or full contact-implicit MPC. Contacts not in the schedule
are still detected by MuJoCo; positive toe lift must be measured, not assumed.
"""
from __future__ import annotations
import math
import numpy as np
import mujoco as mj
from qpsolvers import solve_qp
LEGS=('ml','mr','bl','br')

def dynamic_qp(m,d,site_ids,active=('ml','mr'),*,mu=1.1,min_fz=.2,
               swing_accel=.5, joint_accel_cap=35., angular_accel_cap=10.,
               linear_accel_cap=1.5, torque_util=.9, angular_target=None,
               prev_tau=None):
    active=tuple(active);ns=len(active);nv=m.nv;nu=m.nu
    if ns<2 or ns>4: raise ValueError('2..4 supports only')
    n=nv+nu+3*ns;si=nv;fi=nv+nu
    M=np.zeros((nv,nv));mj.mj_fullM(m,d,M)
    S=np.zeros((nv,nu));
    for i,j in enumerate(m.actuator_trnid[:,0].astype(int)): S[m.jnt_dofadr[j],i]=m.actuator_gear[i,0]
    caps=np.maximum(abs(m.actuator_ctrlrange[:,0]),abs(m.actuator_ctrlrange[:,1]))
    J={};Jdotv={}
    for leg in LEGS:
        jp=np.zeros((3,nv));jr=np.zeros((3,nv));mj.mj_jacSite(m,d,jp,jr,site_ids[leg]);J[leg]=jp
    # Exact contact bias needs point-attached Jacobian time derivative.
    eps=1e-5
    dnext=mj.MjData(m);dnext.qpos[:]=d.qpos;dnext.qvel[:]=d.qvel
    mj.mj_integratePos(m,dnext.qpos,d.qvel,eps);mj.mj_forward(m,dnext)
    for leg in LEGS:
        jp=np.zeros((3,nv));jr=np.zeros((3,nv));mj.mj_jacSite(m,dnext,jp,jr,site_ids[leg]);Jdotv[leg]=(jp-J[leg])@d.qvel/eps
    A=np.zeros((nv+3*ns,n));b=np.zeros(nv+3*ns)
    A[:nv,:nv]=M;A[:nv,si:fi]=-S
    b[:nv]=d.qfrc_passive-d.qfrc_bias
    for i,leg in enumerate(active):
        A[:nv,fi+i*3:fi+(i+1)*3]=-J[leg].T
        A[nv+i*3:nv+(i+1)*3,:nv]=J[leg]
        b[nv+i*3:nv+(i+1)*3]=-Jdotv[leg]
    G=[];h=[]
    for leg in LEGS:
        if leg in active: continue
        row=np.zeros(n);row[:nv]=-J[leg][2];G.append(row);h.append(float(Jdotv[leg][2]-swing_accel))
    for i in range(ns):
        for theta in np.linspace(0.,2*math.pi,16,endpoint=False):
            row=np.zeros(n);row[fi+3*i:fi+3*i+3]=[math.cos(theta),math.sin(theta),-mu*math.cos(math.pi/16)]
            G.append(row);h.append(0.)
    lb=np.empty(n);ub=np.empty(n)
    lb[:3]=-linear_accel_cap;ub[:3]=linear_accel_cap
    lb[3:6]=-angular_accel_cap;ub[3:6]=angular_accel_cap
    lb[6:nv]=-joint_accel_cap;ub[6:nv]=joint_accel_cap
    lb[si:fi]=-torque_util*caps;ub[si:fi]=torque_util*caps
    for i in range(ns):
        lb[fi+3*i:fi+3*i+3]=[-np.inf,-np.inf,min_fz]
        ub[fi+3*i:fi+3*i+3]=[np.inf,np.inf,24.]
    P=np.zeros(n);q=np.zeros(n)
    # Diagonal PSD objective: balance torques, accelerations, foot reaction.
    P[:3]=2.*(2.0/1.5**2)
    P[3:6]=2.*(3.0/(angular_accel_cap**2))
    P[6:nv]=2.*(.20/(joint_accel_cap**2))
    P[si:fi]=2.*(1.0/(caps**2))
    for i in range(ns):
        P[fi+3*i:fi+3*i+3]=[.025,.025,.003]
    if angular_target is not None:
        P[3:6]+=2.*(12./(angular_accel_cap**2))
        q[3:6]-=2.*(12./(angular_accel_cap**2))*np.asarray(angular_target)
    if prev_tau is not None:
        P[si:fi]+=2.*(.1/(caps**2));q[si:fi]-=2.*(.1/(caps**2))*np.asarray(prev_tau)
    x=solve_qp(np.diag(P),q,G=np.asarray(G),h=np.asarray(h),A=A,b=b,lb=lb,ub=ub,solver='daqp')
    if x is None:return None
    dd=x[:nv];tau=x[si:fi];forces=x[fi:].reshape(-1,3)
    return {'qdd':dd,'tau':tau,'forces':forces,
            'eq_res':float(np.max(abs(A@x-b))),'ineq_res':float(max(0.,np.max(np.asarray(G)@x-np.asarray(h)))),
            'max_motor_frac':float(np.max(abs(tau)/caps)),
            'swing_z_accel':{k:float(J[k][2]@dd+Jdotv[k][2]) for k in LEGS if k not in active}}
