"""Bounded inverse acceleration control using the current constrained model.

Only query MjData objects are evaluated; the live trajectory is never reset.
This is a model controller, not a voltage-to-Nm calibration of the real EN01.
"""
import math
import numpy as np
import mujoco


def inverse_segment(target, samples):
    """Interpolate only within sampled, bounded torque segments."""
    choices=list(samples)
    for (ta,aa),(tb,ab) in zip(samples,samples[1:]):
        if abs(ab-aa)>1e-9 and min(aa,ab)<=target<=max(aa,ab):
            t=ta+(target-aa)*(tb-ta)/(ab-aa)
            choices.append((t,target))
    return min(choices,key=lambda x:((x[1]-target)**2,abs(x[0])))[0]


class ComputedTorque:
    def __init__(self,m,torso,ha,hq,hv,wa,settling_time_s):
        self.m=m;self.q=mujoco.MjData(m);self.torso=torso
        self.ha=ha;self.hq=hq;self.hv=hv;self.wa=wa
        self.wn=4./settling_time_s;self.queries=0
        self.limit=min(float(m.actuator_ctrlrange[a,1]) for a in wa)
        if self.limit!=.03:raise ValueError('Frozen surrogate motor ceiling changed')

    def acceleration(self,d,tau,goals):
        q=self.q;m=self.m
        q.qpos[:]=d.qpos;q.qvel[:]=d.qvel;q.ctrl[:]=d.ctrl
        q.qacc_warmstart[:]=d.qacc_warmstart;q.time=d.time
        q.qfrc_applied[:]=0;q.xfrc_applied[:]=0
        for i in range(2):
            v=float(q.qvel[self.hv[i]])
            raw=3*(goals[i]-q.qpos[self.hq[i]])-.045*v
            lim=.14709975*(max(0.,1-abs(v)/11.6355) if raw*v>0 else 1)
            q.ctrl[self.ha[i]]=np.clip(raw,-lim,lim)
            q.ctrl[self.wa[i]]=tau
        mujoco.mj_forward(m,q);mujoco.mj_rnePostConstraint(m,q)
        acc=np.zeros(6)
        mujoco.mj_objectAcceleration(m,q,mujoco.mjtObj.mjOBJ_BODY,self.torso,acc,0)
        self.queries+=1
        return float(acc[1])

    def command(self,d,goals):
        R=d.xmat[self.torso].reshape(3,3)
        pitch=math.atan2(-R[2,0],R[2,2])
        omega=float(d.sensor('imu_gyro').data[1])
        target=-self.wn*self.wn*pitch-2*self.wn*omega
        samples=[(t,self.acceleration(d,t,goals)) for t in (-self.limit,0.,self.limit)]
        torque=float(inverse_segment(target,samples))
        predicted=self.acceleration(d,torque,goals)
        # A contact switch can invalidate local interpolation. Keep an evaluated
        # anchor if it matches the requested acceleration more closely.
        best=min(samples+[(torque,predicted)],key=lambda x:((x[1]-target)**2,abs(x[0])))
        torque,predicted=best
        return [torque,torque],{'requested_alpha_rad_s2':target,
            'predicted_alpha_rad_s2':predicted,'torque_nm':torque,
            'sampled_response':samples,'wn_rad_s':self.wn}
