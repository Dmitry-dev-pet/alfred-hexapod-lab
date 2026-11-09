"""Constrained-model wheel torque with COM trim and slow axle-position feedback."""
import math
import numpy as np
import mujoco
from .physics_torque import ComputedTorque, inverse_segment


class COMTorque(ComputedTorque):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.wheels=[mujoco.mj_name2id(self.m,mujoco.mjtObj.mjOBJ_BODY,s+'_wheel') for s in ('left','right')]
        self.reference_x=None

    def command(self,d,goals):
        R=d.xmat[self.torso].reshape(3,3)
        pitch=math.atan2(-R[2,0],R[2,2]);omega=float(d.sensor('imu_gyro').data[1])
        axle=np.mean(d.xpos[self.wheels],axis=0)
        lever=R.T@(d.subtree_com[self.torso]-axle)
        trim=-math.atan2(float(lever[0]),float(lever[2]))
        vel=np.zeros(6);velocities=[]
        for b in self.wheels:
            mujoco.mj_objectVelocity(self.m,d,mujoco.mjtObj.mjOBJ_BODY,b,vel,0)
            velocities.append(float(vel[3]))
        vx=float(np.mean(velocities));outer_accel=0.
        # Position capture is only an outer-loop reference, never a pose reset.
        if abs(pitch-trim)<math.radians(15):
            if self.reference_x is None:self.reference_x=float(axle[0])
            wxy=.7
            outer_accel=float(np.clip(-wxy*wxy*(axle[0]-self.reference_x)-2*wxy*vx,-1.5,1.5))
        target_pitch=trim+math.atan2(outer_accel,9.81)
        error=math.atan2(math.sin(pitch-target_pitch),math.cos(pitch-target_pitch))
        target=-self.wn*self.wn*error-2*self.wn*omega
        samples=[(t,self.acceleration(d,t,goals)) for t in (-self.limit,0.,self.limit)]
        torque=float(inverse_segment(target,samples));predicted=self.acceleration(d,torque,goals)
        torque,predicted=min(samples+[(torque,predicted)],key=lambda x:((x[1]-target)**2,abs(x[0])))
        return [torque,torque],{'requested_alpha_rad_s2':target,
            'predicted_alpha_rad_s2':predicted,'torque_nm':torque,'sampled_response':samples,
            'wn_rad_s':self.wn,'com_trim_rad':trim,'target_pitch_rad':target_pitch,
            'axle_speed_x_m_s':vx,'axle_reference_x_m':self.reference_x,
            'outer_requested_acceleration_m_s2':outer_accel}
