"""Strict global *screen* for candidate periodic CAD30 gaits.

This is a necessary-condition checker, NOT a trajectory optimiser, contact
simulator, or proof of a physical gait. Input values must be independently
verified against actual MuJoCo source traces before any physics claim.
"""
from __future__ import annotations

from collections import Counter
import math

WALK = ("ml", "mr", "bl", "br")
MIN_DELTA = .020
MIN_NET_AFTER_THREE = .020


def vec(data, n):
    out = tuple(float(x) for x in data)
    if len(out) != n or not all(math.isfinite(x) for x in out):
        raise ValueError(f"expected finite vector length {n}")
    return out


def diff(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def rotation_distance(qa, qb):
    a, b = vec(qa, 4), vec(qb, 4)
    if abs(sum(x*x for x in a)-1) > .002 or abs(sum(x*x for x in b)-1) > .002:
        raise ValueError("non-unit body quaternion")
    return 2*math.acos(min(1., abs(sum(x*y for x, y in zip(a, b)))))


def screen(candidate):
    """Return REJECT or GLOBAL_SCREEN_PASS_NOT_PHYSICAL_PROOF."""
    errors = []
    def require(predicate, code):
        if not predicate:
            errors.append(code)

    if candidate.get("schema") != "cad30-periodic-screen-v1":
        return {"status": "REJECT", "reasons": ["BAD_SCHEMA"]}

    try:
        a, b = candidate["start"], candidate["end"]
        prep = candidate["preparation"]
        root_a, root_b = vec(a["root_xyz_m"], 3), vec(b["root_xyz_m"], 3)
        com_a, com_b = vec(a["com_xyz_m"], 3), vec(b["com_xyz_m"], 3)
        q_a, q_b = vec(a["leg_q_rad"], 18), vec(b["leg_q_rad"], 18)
        v_a, v_b = vec(a["leg_qd_rad_s"], 18), vec(b["leg_qd_rad_s"], 18)
        vlin_a, vlin_b = vec(a["body_lin_velocity_world_m_s"], 3), vec(b["body_lin_velocity_world_m_s"], 3)
        vang_a, vang_b = vec(a["body_ang_velocity_world_rad_s"], 3), vec(b["body_ang_velocity_world_rad_s"], 3)
        feet_a = {k: vec(a["toe_world_xyz_m"][k], 3) for k in WALK}
        feet_b = {k: vec(b["toe_world_xyz_m"][k], 3) for k in WALK}
        original_six_y = float(prep["original_six_com_y_m"])
        prep_end_y = float(prep["cycle_start_com_y_m"])
        limits = candidate["bounds"]
        raw_tau = float(limits["max_raw_tau_Nm"])
        angle_max = float(limits["max_abs_leg_q_rad"])
        margin = float(limits["min_sampled_support_margin_mm"])
        if not all(math.isfinite(x) for x in (original_six_y, prep_end_y, raw_tau, angle_max, margin)):
            raise ValueError("nonfinite limits")
        r_err = rotation_distance(a["body_quat_wxyz"], b["body_quat_wxyz"])
    except (KeyError, ValueError, TypeError, OverflowError) as exc:
        return {"status": "REJECT", "reasons": [f"MALFORMED:{exc}"]}

    dy = com_b[1]-com_a[1]
    three_period_net = prep_end_y + 3*dy - original_six_y
    require(abs(prep_end_y-com_a[1]) <= 1e-6, "PREPARATION_BOUNDARY_MISMATCH")
    require(candidate.get("swing_sequence") is not None
            and len(candidate["swing_sequence"]) == 4
            and Counter(candidate["swing_sequence"]) == Counter(WALK),
            "NOT_FOUR_DISTINCT_SWINGS")
    require(dy >= MIN_DELTA-1e-9, "CYCLE_FORWARD_LESS_20MM")
    require(three_period_net >= MIN_NET_AFTER_THREE-1e-9, "NET_INCLUDING_PREP_LESS_20MM")
    require(abs((root_b[1]-root_a[1])-dy) <= .005, "ROOT_COM_TRANSLATION_DISAGREE")
    require(abs(root_b[0]-root_a[0]) <= .005, "ROOT_X_DRIFT")
    require(abs(root_b[2]-root_a[2]) <= .005, "ROOT_Z_NOT_PERIODIC")
    require(r_err <= .035, "ORIENTATION_NOT_PERIODIC")
    require(diff(q_a,q_b) <= .03, "JOINTS_NOT_PERIODIC")
    require(diff(v_a,v_b) <= .10, "JOINT_SPEED_NOT_PERIODIC")
    require(diff(vlin_a,vlin_b) <= .03, "ROOT_LIN_SPEED_NOT_PERIODIC")
    require(diff(vang_a,vang_b) <= .10, "ROOT_ANG_SPEED_NOT_PERIODIC")
    require(set(a.get("loaded_legs",[])) == set(WALK)
            and set(b.get("loaded_legs",[])) == set(WALK), "NOT_FOUR_SUPPORTS_AT_PERIOD_BOUNDARIES")
    require(a.get("front_loaded") is False and b.get("front_loaded") is False,
            "FRONT_LEGS_NOT_LIFTED")
    for leg in WALK:
        step = tuple(y-x for x,y in zip(feet_a[leg],feet_b[leg]))
        require(abs(step[0]) <= .005 and abs(step[1]-dy) <= .005 and abs(step[2]) <= .005,
                f"FOOT_{leg.upper()}_NOT_TRANSLATED")
    require(raw_tau <= .7+1e-9, "TORQUE_RESEARCH_LIMIT")
    require(angle_max <= 1.7+1e-9, "JOINT_RESEARCH_LIMIT")
    require(margin >= 5, "SAMPLED_MARGIN_BELOW_5MM")

    return {
        "status": "REJECT" if errors else "GLOBAL_SCREEN_PASS_NOT_PHYSICAL_PROOF",
        "reasons": errors,
        "period_forward_mm": round(1000*dy,3),
        "preparation_mm": round(1000*(prep_end_y-original_six_y),3),
        "projected_net_after_three_periods_mm": round(1000*three_period_net,3),
        "warning": "Summary-only necessary checks; no optimisation, continuous dynamics, or source-trace validation."
    }
