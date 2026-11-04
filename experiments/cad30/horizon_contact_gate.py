"""Four-step candidate *screen*, not a predictive trajectory optimizer.

This purely geometric feasibility gate checks proposed future liftoff COM
positions and touchdown coordinates; it cannot certify a real robot gait.
The actual MPC must generate such candidates and validate their transitions
using full-body IK, contact forces, torques and forward dynamics.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Mapping, Sequence
import numpy as np
from scipy.spatial import ConvexHull

LEGS = ("ml", "mr", "bl", "br")


@dataclass(frozen=True)
class ContactPhase:
    lift_leg: str
    com_during_swing_xy_m: tuple[float, float]
    touchdown_xy_m: tuple[float, float]
    com_after_touchdown_xy_m: tuple[float, float]


@dataclass(frozen=True)
class GeometricHorizonCheck:
    admissible_necessary_geometry: bool
    reason: str
    minimum_swing_margin_m: float
    final_com_progress_m: float
    terminal_immediate_liftoff_options: tuple[str, ...]


def _margin(pts, com):
    pts=np.asarray(pts,float)
    if pts.shape[0]<3 or np.linalg.matrix_rank(pts-pts[0])<2:
        return float('-inf')
    eq=ConvexHull(pts).equations
    return float(np.min(-(eq[:,:2]@com+eq[:,2])))


def screen_four_step_horizon(
    initial_footholds_xy_m: Mapping[str, Sequence[float]],
    initial_com_xy_m: Sequence[float],
    phases: Sequence[ContactPhase],
    *, margin_m: float=.005,
    max_toe_move_m: float=.050,
    minimum_forward_progress_m: float=.020,
) -> GeometricHorizonCheck:
    """Test four-step geometry and a strong terminal no-correction viability proxy.

    For each swing: the COM is checked against 3 fixed grounded toe positions.
    For each landing: the COM is checked against 4 grounded toe positions.
    Swing-foot clearance, continuous COM interpolation, joint reach, wrench
    feasibility and collisions are NOT checked. A pass is ONLY a necessary
    geometric snapshot condition.
    """
    if margin_m<=0 or max_toe_move_m<=0 or minimum_forward_progress_m<0:
        raise ValueError('Invalid thresholds')
    if set(initial_footholds_xy_m) != set(LEGS):
        raise ValueError('Need all four locomotion-foot xy positions')
    if len(phases)!=4 or set(p.lift_leg for p in phases)!=set(LEGS):
        raise ValueError('One liftoff/touchdown for each of four legs is required')
    feet={k:np.asarray(v,dtype=float).copy() for k,v in initial_footholds_xy_m.items()}
    initial=np.asarray(initial_com_xy_m,dtype=float)
    if initial.shape!=(2,) or any(p.shape!=(2,) for p in feet.values()):
        raise ValueError('Expected 2D positions')
    arrays=[initial,*feet.values()]
    for p in phases:
        arrays += [np.asarray(p.com_during_swing_xy_m,float),np.asarray(p.touchdown_xy_m,float),np.asarray(p.com_after_touchdown_xy_m,float)]
    if any(a.shape!=(2,) or not np.all(np.isfinite(a)) for a in arrays):
        raise ValueError('Nonfinite or non-2D phase data')
    def fail(reason, smallest=float('nan'), progress=0.):
        return GeometricHorizonCheck(False,reason,float(smallest),float(progress),())
    if _margin(list(feet.values()),initial)<margin_m-1e-9:
        return fail('Initial 4-foot COM outside safe polygon')
    smallest=float('inf')
    final_com=initial.copy()
    for k,p in enumerate(phases):
        before=np.asarray(p.com_during_swing_xy_m,float)
        contact_triangle=[xy for name,xy in feet.items() if name != p.lift_leg]
        swing_margin=_margin(contact_triangle,before)
        smallest=min(smallest,swing_margin)
        if swing_margin<margin_m-1e-9:
            return fail(f'phase {k+1}: 3-support margin insufficient',smallest)
        landing=np.asarray(p.touchdown_xy_m,float)
        if np.linalg.norm(landing-feet[p.lift_leg])>max_toe_move_m+1e-9:
            return fail(f'phase {k+1}: toe landing displacement too large',smallest)
        feet[p.lift_leg]=landing
        final_com=np.asarray(p.com_after_touchdown_xy_m,float)
        if _margin(list(feet.values()),final_com)<margin_m-1e-9:
            return fail(f'phase {k+1}: 4-support landing margin insufficient',smallest)
    progress=float(final_com[1]-initial[1])
    if progress<minimum_forward_progress_m-1e-9:
        return fail('final COM forward progress below requested threshold',smallest,progress)
    # Intentionally conservative terminal viability proxy: a fresh 3-contact
    # liftoff can start at the *final* COM without further body compensation.
    options=tuple(k for k in LEGS if _margin([v for leg,v in feet.items() if leg!=k],final_com)>=margin_m-1e-9)
    if not options:
        return fail('no immediate next liftoff from final COM',smallest,progress)
    return GeometricHorizonCheck(True,'snapshot geometry passed; IK/forces/dynamics remain unchecked',smallest,progress,options)


def select_candidate(initial_footholds_xy_m,initial_com_xy_m,candidates,**constraints):
    """Pick among externally generated schedules; never fabricate a footpath.

    Returns (index, check), or (None, None) if every candidate is rejected.
    Best = greatest minimum swing margin, then strongest forward progress.
    A viable necessary-condition candidate is NOT approval to drive motors.
    """
    valid=[]
    for i,candidate in enumerate(candidates):
        check=screen_four_step_horizon(initial_footholds_xy_m,initial_com_xy_m,candidate,**constraints)
        if check.admissible_necessary_geometry:
            valid.append((i,check))
    return max(valid,key=lambda r:(r[1].minimum_swing_margin_m,r[1].final_com_progress_m),default=(None,None))
