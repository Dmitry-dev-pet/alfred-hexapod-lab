"""Gymnasium API for the CPU-only original Stanford Pupper v3 MuJoCo model.

Training protocol is an Alfred research baseline, NOT Stanford's native MJLab RL.
All 12 actions are servo position offsets and all root movement comes from mj_step.
"""
from __future__ import annotations

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from physics import PupperCPU,DT,DECIMATION
from reward import REWARD_PROFILES,calculate_reward

CMD_SET = np.asarray([
    [0,0,0], [.20,0,0],[-.15,0,0],
    [0,.15,0],[0,-.15,0],
    [0,0,.40],[0,0,-.40],
    [.15,.10,0],[.15,-.10,0],
    [.15,0,.25],[.15,0,-.25],
    [.12,.10,.20],[.12,-.10,-.20],
],dtype=np.float32)



class PupperVelocityGym(gym.Env):
    metadata={'render_modes':[]}

    def __init__(self, *, profile='tracking', max_seconds=10.0, command_interval_seconds=2.0,
                 delay_steps=4, seed=0):
        super().__init__()
        if profile not in REWARD_PROFILES:raise ValueError('Unknown reward profile')
        if not 0 < max_seconds <= 60:raise ValueError('Invalid horizon')
        self.sim=PupperCPU(delay_steps=delay_steps)
        self.weights=REWARD_PROFILES[profile].copy()
        self.profile=profile
        self.max_steps=int(round(max_seconds/(DT*DECIMATION)))
        self.command_steps=max(1,int(round(command_interval_seconds/(DT*DECIMATION))))
        self.action_space=spaces.Box(low=-1,high=1,shape=(12,),dtype=np.float32)
        self.observation_space=spaces.Box(low=-10,high=10,shape=(960,),dtype=np.float32)
        self.np_random=np.random.default_rng(seed)
        self.steps=0
        self.cmd=np.zeros(3,dtype=np.float32)

    def _sample_command(self):
        cmd=CMD_SET[int(self.np_random.integers(len(CMD_SET)))].copy()
        self.cmd=cmd
        self.sim.set_command(cmd)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.steps=0
        self.sim.reset()
        self._sample_command()
        return self.sim.observation(), {'command_vx_vy_wz':self.cmd.tolist(), 'reward_profile':self.profile}

    def step(self,action):
        _,m=self.sim.step(action)
        self.steps+=1
        cmd=self.cmd
        reward,terms,lin_error,yaw_error=calculate_reward(m,cmd,self.profile)
        terminated=bool(m['fell'])
        truncated=bool(self.steps>=self.max_steps)
        info={
            'cmd':cmd.tolist(),'measured_vel_xy':m['vel_xy'].tolist(),
            'measured_yaw_rate':m['yaw_rate'],'tracking_error_xy':lin_error,
            'tracking_error_yaw':yaw_error,'fallen':terminated,
            'body_height_m':m['height_m'],'loaded_feet':m['feet_loaded'],
            'tilt_deg':math.degrees(m['tilt_rad']),
            'reward_terms':{name:round(float(value),7) for name,value in terms.items()},
        }
        if (not terminated and not truncated and self.steps%self.command_steps==0):
            self._sample_command()
        return self.sim.observation(),reward,terminated,truncated,info

    def close(self):
        self.sim=None
