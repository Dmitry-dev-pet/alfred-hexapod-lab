"""Research PPO reward terms; nonzero weights, finite and explicitly auditable.

These are our exploratory weights, NOT Stanford's zero-weight course defaults.
"""
from __future__ import annotations
import math
import numpy as np

REWARD_PROFILES = {
    'tracking': {
        'linear_xy':3.0,'yaw':1.5,'upright':1.0,'height':.20,
        'action_rate':-.05,'torque':-.005,'vertical':-.05,'fall':-10.0,
    },
    'stability': {
        'linear_xy':3.0,'yaw':1.5,'upright':2.0,'height':.5,
        'action_rate':-.10,'torque':-.025,'vertical':-.10,'fall':-15.0,
    },
}


def calculate_reward(m,command,profile):
    if profile not in REWARD_PROFILES:raise ValueError('Unknown reward profile')
    cmd=np.asarray(command,dtype=float)
    lin_error=float(np.linalg.norm(np.asarray(m['vel_xy'])-cmd[:2]))
    yaw_error=float(abs(m['yaw_rate']-cmd[2]))
    terms={
        'linear_xy': math.exp(-lin_error**2/(.25**2)),
        'yaw': math.exp(-yaw_error**2/(.4**2)),
        'upright': math.exp(-(m['tilt_rad']/.35)**2),
        'height': math.exp(-((m['height_m']-.155)/.05)**2),
        'action_rate':m['delta_action_l2'],
        'torque':m['torque_norm'],
        'vertical':(m['vertical_rate']/.50)**2,
        'fall':float(m['fell']),
    }
    if not all(math.isfinite(float(v)) for v in terms.values()):
        raise FloatingPointError('non-finite reward terms')
    reward=float(sum(REWARD_PROFILES[profile][k]*terms[k] for k in terms))
    return reward,terms,lin_error,yaw_error
