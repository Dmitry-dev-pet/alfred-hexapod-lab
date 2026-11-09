"""Declared gentle motion primitives; wheel commands do not prescribe body poses."""
from __future__ import annotations
import math

CASES = {
    'stand-low': {'q': .1, 'v': 0., 'w': 0.},
    'stand-mid': {'q': .3, 'v': 0., 'w': 0.},
    'drive-forward': {'q': .1, 'v': .03, 'w': 0.},
    'drive-backward': {'q': .1, 'v': -.03, 'w': 0.},
    'turn-left-low': {'q': .1, 'v': 0., 'w': .4},
    'turn-right-low': {'q': .1, 'v': 0., 'w': -.4},
    'turn-left-mid': {'q': .3, 'v': 0., 'w': .4},
    'turn-left-high': {'q': .5, 'v': 0., 'w': .4},
    'squat-cycle': {'q': .3, 'v': 0., 'w': 0., 'squat': True},
    'squat-turn-left': {'q': .3, 'v': 0., 'w': .4, 'squat': True},
    'squat-turn-right': {'q': .3, 'v': 0., 'w': -.4, 'squat': True},
    'drive-turn-left': {'q': .1, 'v': .03, 'w': .4},
}


def smooth(u):
    u = max(0., min(1., u))
    return .5-.5*math.cos(math.pi*u)


def command(case, t):
    c = CASES[case]
    envelope = smooth((t-1.5)/.5)*smooth((4.5-t)/.5)
    q = c['q']
    if c.get('squat'):
        q -= (c['q']-.05)*smooth((t-1.5)/1.)*smooth((4.5-t)/1.)
    return {'v_m_s': c['v']*envelope, 'yaw_rad_s': c['w']*envelope, 'hip_q_rad': q}


def wheel_mix(common, yaw, ceiling=.03):
    """+yaw increases the right wheel, turning the +X-facing robot toward +Y.

    Reserve the available differential headroom after balance torque allocation.
    """
    common = max(-ceiling, min(ceiling, common))
    room = max(0., ceiling-abs(common))
    yaw = max(-room, min(room, yaw))
    return common-yaw, common+yaw
