"""Predeclared behavior checks use actual axle motion and body yaw."""
import math
from .basic_commands import CASES


def classify(case, measured, acceptance):
    spec = CASES[case]
    checks = {
        'balance': measured['max_abs_tilt_deg'] <= acceptance['max_abs_pitch_or_roll_deg'] and measured['nonwheel_ground_steps'] == 0,
        'stop': measured['final_stop_speed_m_s'] <= acceptance['max_final_stop_speed_m_s'] and measured['final_stop_yaw_rate_rad_s'] <= acceptance['max_final_stop_yaw_rate_rad_s'],
        'slip': measured['active_slip_supported_s'] >= 2.5 and measured['active_contact_slip_rms_m_s'] <= acceptance['max_active_contact_slip_rms_m_s'],
        'hip_range': measured['hip_min_rad'] >= 0 and measured['hip_max_rad'] <= .65,
        'two_wheel_support': measured['two_wheel_support_fraction'] >= .95,
    }
    if spec['v']:
        checks['drive_direction_and_distance'] = math.copysign(1, spec['v'])*measured['signed_axle_travel_m'] >= acceptance['min_signed_travel_m']
    if spec['w']:
        checks['turn_direction_and_angle'] = math.copysign(1, spec['w'])*measured['yaw_change_deg'] >= acceptance['min_signed_turn_deg']
        checks['heading_settle'] = abs(measured['final_yaw_error_deg']) <= acceptance['max_yaw_error_at_end_deg']
        if not spec['v']:
            checks['turn_in_place'] = measured['axle_net_displacement_m'] <= acceptance['max_turn_axle_net_displacement_m']
    if spec.get('squat'):
        checks['squat_height_change'] = measured['squat_height_change_m'] >= acceptance['min_squat_height_change_m']
        checks['height_recovered'] = abs(measured['final_height_error_m']) <= .004
    return {'checks': checks, 'basic_pass': all(checks.values())}
