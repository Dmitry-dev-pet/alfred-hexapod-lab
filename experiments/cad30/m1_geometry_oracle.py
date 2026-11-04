"""M1/G1 geometry-only lookahead for CAD30 Freenove FNK0052 pose B.

Toe/hip samples come from a *physical MuJoCo replay*, recorded at t=9.3 s,
but this oracle only tests geometry. Not IK, MPC, force balance or a gait.
No original Freenove STL meshes are included.
"""
from __future__ import annotations

import itertools
import json

import numpy as np
from scipy.optimize import linprog, minimize
from scipy.spatial import ConvexHull

LEGS = ("ml", "mr", "bl", "br")
COM = np.array([0.0001142, -0.0255225])
TOES = {
    "ml": (-0.23897, 0.00109, 0.00030),
    "mr": (0.23897, 0.00109, 0.00030),
    "bl": (-0.14367, -0.19826, 0.0),
    "br": (0.14367, -0.19825, 0.0),
}
HIPS = {
    "ml": (-0.085, -0.03240, 0.13164),
    "mr": (0.085, -0.03239, 0.13163),
    "bl": (-0.055, -0.10831, 0.13528),
    "br": (0.055, -0.10831, 0.13527),
}
TOES = {k: np.asarray(v) for k, v in TOES.items()}
HIPS = {k: np.asarray(v) for k, v in HIPS.items()}
REACH_M = .233  # 33+90+110 mm, optimistic upper bound, NOT a valid IK check


def hull_edges(points):
    """Unit outward normals (a,b,c) with a*x+b*y+c <= 0 inside."""
    return ConvexHull(np.asarray(points)).equations


def support_margin(points, center):
    h = hull_edges(points)
    return float(np.min(-(h[:, :2] @ center + h[:, 2])))


def nearest_com_for_lift(lift, margin=.005):
    """Project COM into remaining three-foot triangle; ignores reach/torque."""
    h = hull_edges([TOES[k][:2] for k in LEGS if k != lift]).copy()
    h[:, 2] += margin
    cons = [{"type": "ineq", "fun": lambda x: -(h[:, :2] @ x + h[:, 2]),
             "jac": lambda x: -h[:, :2]}]
    result = minimize(lambda x: .5*float(np.sum((x-COM)**2)),
                      COM, jac=lambda x: x-COM, method="SLSQP",
                      constraints=cons, options={"ftol":1e-14, "maxiter":500})
    if np.max(h[:, :2]@result.x+h[:, 2]) > 1e-7:
        raise RuntimeError(f"Projection failed: {result.message}")
    return float(np.linalg.norm(result.x-COM)), result.x


def outer_reach_halfspaces(footholds, sides=96):
    """Relax each foot's reach disk with circumscribed polygon halfspaces.

    Assuming a frozen torso orientation/height, a translated hip-to-toe
    distance <=233 mm is a *necessary*, not sufficient, reach condition.
    Outer tangent polygons contain the true disks: their LP *infeasibility*
    is conclusive only under this restricted kinematic model.
    """
    for leg in LEGS:
        relative = np.asarray(footholds[leg]) - HIPS[leg]
        radius = np.sqrt(max(0., REACH_M**2 - relative[2]**2))
        center = COM+relative[:2]
        for theta in np.linspace(0, 2*np.pi, sides, endpoint=False):
            normal = np.array([np.cos(theta), np.sin(theta)])
            yield float(normal[0]), float(normal[1]), float(normal@center + radius)


def best_margin_with_fixed_foot_reach(lift):
    """LP upper bound: maximum support margin with all four feet reachable."""
    a, b = [], []
    for nx, ny, offset in hull_edges([TOES[k][:2] for k in LEGS if k != lift]):
        a.append([nx, ny, 1.]); b.append(-offset)
    for nx, ny, rhs in outer_reach_halfspaces(TOES):
        a.append([nx, ny, 0.]); b.append(rhs)
    solution = linprog([0, 0, -1], A_ub=a, b_ub=b,
                       bounds=[(None,None)]*3, method="highs")
    if not solution.success:
        raise RuntimeError(f"Margin LP failed: {solution.message}")
    return float(solution.x[2])


def straight_stride_horizon_feasible(order, stride=.020, margin=.005, progress=.020):
    """Necessary-condition LP for a four-step sequence of straight +Y landings.

    Four grounded toes must be reachable just BEFORE lift and just AFTER
    touchdown. During swing COM must remain in the other three-foot triangle
    with positive margin. End-of-cycle COM advancement >= progress.
    No actual servo IK, attitude changes, friction, torque or time evolution.
    """
    num_vars = 8  # four COM (x,y) pairs
    a, b = [], []
    feet = {k: v.copy() for k,v in TOES.items()}
    for phase, lifted in enumerate(order):
        for nx, ny, offset in hull_edges([feet[k][:2] for k in LEGS if k != lifted]):
            row = np.zeros(num_vars); row[2*phase:2*phase+2] = [nx,ny]
            a.append(row); b.append(-offset-margin)
        after = {k: v.copy() for k,v in feet.items()}
        after[lifted][1] += stride
        for stance in (feet, after):
            for nx, ny, rhs in outer_reach_halfspaces(stance):
                row = np.zeros(num_vars); row[2*phase:2*phase+2] = [nx,ny]
                a.append(row); b.append(rhs)
        feet = after
    row = np.zeros(num_vars); row[-1] = -1
    a.append(row); b.append(-(COM[1]+progress))
    result = linprog(np.zeros(num_vars), A_ub=a, b_ub=b,
                     bounds=[(None,None)]*num_vars, method="highs")
    return bool(result.success)


def summary():
    first = {}
    for leg in LEGS:
        remaining = [TOES[k][:2] for k in LEGS if k != leg]
        delta, target = nearest_com_for_lift(leg)
        first[leg] = {
            "initial_remaining_triangle_margin_mm": round(support_margin(remaining, COM)*1000, 3),
            "minimum_com_shift_for_5mm_margin_mm": round(delta*1000, 3),
            "target_com_xy_mm": np.round(target*1000,3).tolist(),
            "upper_bound_max_margin_with_4_reachable_feet_mm":
                round(best_margin_with_fixed_foot_reach(leg)*1000, 3),
        }
    return {
        "study": "m1_geometry_pose_B_necessary_conditions",
        "pose_B_com_xy_m": COM.tolist(),
        "four_toe_initial_margin_mm": round(support_margin([TOES[k][:2] for k in LEGS], COM)*1000,3),
        "first_lift":first,
        "four_step_orders":24,
        "valid_orders_20mm_straight_stride":sum(straight_stride_horizon_feasible(seq,.020)
            for seq in itertools.permutations(LEGS)),
        "valid_orders_40mm_straight_stride":sum(straight_stride_horizon_feasible(seq,.040)
            for seq in itertools.permutations(LEGS)),
        "assumptions":"COM forward +20mm, support margin +5mm; frozen torso attitude/height; hip-foot reach optimistic <=233mm; straight +Y foot placements; toe markers rather than force-weighted contact points.",
        "provenance":"Pose B t=9.3s: Drive Hexapod_Middle_Leg_Step_Probe_2026-11-04_no_CAD.zip viewer frame #92, COM from claw_tuck_compare semi_tucked.after_tuck (PR #25).",
        "warning":"Geometric negative result is specific to this restricted planning model; no proof about physical robot, arbitrary foot placement or MuJoCo."
    }


if __name__ == "__main__":
    print(json.dumps(summary(), ensure_ascii=False, indent=2))
