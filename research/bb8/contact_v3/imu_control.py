"""Discrete simulated IMU/encoder feedback, with no true pose/velocity input."""
import math
import numpy as np

class Controller:
    def __init__(self,zero=0.,kp=.5,kd=.05,kv=1.2,ki=.3,period=.01,enabled=True):
        self.zero=zero;self.kp=kp;self.kd=kd;self.kv=kv;self.ki=ki;self.period=period;self.enabled=enabled
        self.pitch=0.;self.initialized=False;self.integral=0.;self.speed=0.;self.previous_encoder=None
        self.yaw=0.;self.last={}

    def update(self,gyro,accel,encoder,target_speed=0.,target_yaw=0.):
        gy=np.asarray(gyro);ac=np.asarray(accel);encoder=np.asarray(encoder)
        norm=float(np.linalg.norm(ac));valid=.85*9.81<norm<1.15*9.81
        acc_pitch=math.atan2(-ac[0],ac[2])
        if not self.initialized and valid:self.pitch=acc_pitch;self.initialized=True
        self.pitch+=gy[1]*self.period;self.yaw+=gy[2]*self.period
        if valid:
            delta=math.atan2(math.sin(acc_pitch-self.pitch),math.cos(acc_pitch-self.pitch))
            self.pitch+=(1-math.exp(-self.period/.35))*delta
        rate=0. if self.previous_encoder is None else float(np.mean(encoder-self.previous_encoder)/self.period)
        self.previous_encoder=encoder.copy()
        # Contact-geometry estimate of ground speed: shell radius * core rate
        # plus tire radius * relative wheel rate. Coefficient remains a hypothesis.
        speed_raw=.1*gy[1]+.0275*rate
        self.speed+=(1-math.exp(-self.period/.08))*(speed_raw-self.speed)
        error=target_speed-self.speed
        candidate=float(np.clip(self.integral+error*self.period,-.25,.25))
        reference=self.zero+float(np.clip(self.kv*error+self.ki*candidate,-.12,.12))
        common=self.kp*(self.pitch-reference)+self.kd*gy[1]
        differential=.025*(target_yaw-gy[2])
        request=np.array([common+differential,common-differential])
        if abs(common)<.1:self.integral=candidate
        if not self.initialized or not self.enabled:request[:]=0.
        self.last={'estimated_pitch_rad':self.pitch,'estimated_speed_m_s':self.speed,'reference_pitch_rad':reference,
                   'estimated_yaw_rad':self.yaw,'gyro_rad_s':gy.tolist(),'accel_m_s2':ac.tolist(),
                   'encoder_position_rad':encoder.tolist(),'target_speed_m_s':target_speed,'target_yaw_rad_s':target_yaw,
                   'request_Nm':request.tolist(),'initialized':self.initialized}
        return request
