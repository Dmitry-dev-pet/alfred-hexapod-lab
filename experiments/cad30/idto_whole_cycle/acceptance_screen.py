"""Necessary-condition gates for IDTO CAD30 math solutions.

Do not interpret a passing sampled screen as proof of dynamics or a physical
periodic gait; all candidates require hard force contacts, real 6->4 prep, and
an independent whole-chain MuJoCo forward rollout.
"""
from __future__ import annotations
from math import isfinite

LEGS=("ml","mr","bl","br")

def screen(audit:dict)->dict:
    errors=[]
    def req(cond:bool,code:str):
        if not cond:errors.append(code)
    for key in ["root_world_y_translation_mm","terminal_leg_joint_closure_max_rad",
                "terminal_velocity_closure_max_rad_per_s","max_abs_leg_joint_rad",
                "max_abs_leg_tau_Nm","max_abs_head_tau_Nm"]:
        if not isinstance(audit.get(key),(float,int)) or not isfinite(audit[key]):
            return {'status':'INVALID','reasons':['NONFINITE_OR_MISSING_'+key]}
    req(audit['root_world_y_translation_mm']>=20.0-1e-8,'LESS_THAN_20MM_ROOT_ADVANCE')
    req(audit['terminal_leg_joint_closure_max_rad']<=.03,'JOINTS_NOT_PERIODIC')
    req(audit['terminal_velocity_closure_max_rad_per_s']<=.10,'VELOCITIES_NOT_PERIODIC')
    req(audit['max_abs_leg_joint_rad']<=1.7,'JOINT_MODEL_CAP')
    req(audit['max_abs_leg_tau_Nm']<=.7,'LEG_TORQUE_MODEL_CAP')
    req(audit['max_abs_head_tau_Nm']<=.2,'HEAD_TORQUE_MODEL_CAP')
    swings=audit.get('swing_kinematic_screen',{})
    for leg in LEGS:
        s=swings.get(leg,{})
        req(s.get('passes_clearance_2mm') is True,'NO_SAMPLED_AIRBORNE_'+leg.upper())
        req(s.get('all_other_working_feet_within_2mm_on_sampled_swing_nodes') is True,
            'NON_SWING_CONTACT_GEOMETRY_'+leg.upper())
    toes=audit.get('toe_landmark_world_y_delta_mm',{})
    for leg in LEGS:
        req(isinstance(toes.get(leg),(float,int)) and abs(toes[leg]-audit['root_world_y_translation_mm'])<=5,
            'FOOT_NOT_PERIODIC_'+leg.upper())
    # A preflight screen cannot provide actual contact Fz or dynamic feasibility.
    return {'status':'REJECT' if errors else 'SAMPLED_SCREEN_ONLY_NOT_GAIT',
            'reasons':errors,'physical_gait_proven':False,
            'unactuated_force_residual_not_hard_certified':True,
            'six_to_four_preparation_physically_verified':False}
