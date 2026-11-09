"""Source firmware commands, not an unmeasured CAD encoder-zero calibration.

Pinned Navbot-EN01 wl_pro_robot.ino c5c69e515d7d2a22edc3779667fc3edfe4045ae9.
README says encoder 2048 is the mechanical fully-squatted pose. The matching
absolute CAD joint angle remains unknown and must be provided explicitly.
"""
import math

COUNTS_PER_TURN=4096
ENCODER_ZERO=2048


def normal_positions(height,roll_correction=0.):
    if not all(math.isfinite(x) for x in (height,roll_correction)):
        raise ValueError('Nonfinite command')
    # C++ s16 assignment truncates each position before applying clamps.
    a=int(2048+8.4*(height-32)-roll_correction)
    b=int(2048-8.4*(height-32)-roll_correction)
    return max(2110,min(2510,a)),max(1586,min(1986,b))


def jump_positions(phase):
    if phase not in ('initial','return'):raise ValueError('Unknown jump phase')
    h=80 if phase=='initial' else 40
    return int(2048+12+8.4*(h-32)),int(2048-12-8.4*(h-32))


def logical_angle_since_squat(position,servo_id):
    if servo_id not in (1,2) or not math.isfinite(position):raise ValueError('Invalid encoder')
    return (position-2048)*(1 if servo_id==1 else -1)*math.tau/COUNTS_PER_TURN


def cad_angle(position,servo_id,*,squat_cad_angle_rad=None):
    if squat_cad_angle_rad is None or not math.isfinite(squat_cad_angle_rad):
        raise ValueError('Mechanical squat angle in CAD is not calibrated')
    return squat_cad_angle_rad+logical_angle_since_squat(position,servo_id)
