"""Kinematic WBC micro-correction for grounded feet (research prototype).

Solves (linearized about a *measured* robot state):
    J_base * delta_base + J_joints * delta_q = 0
while honoring conservative per-cycle joint step and joint position bounds.

The independent Jacobians must be supplied by the physics adapter (MuJoCo
mj_jac + actual contact positions); they are not fabricated here.

This is *not* a dynamics/force WBC and does not command hardware. A kinematically
approved result still requires full IK, contact wrench, torque and physical
rollout validation. Original CAD files are not included.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear


@dataclass(frozen=True)
class KinematicCorrection:
    approved_kinematic: bool
    reason: str
    joint_delta_rad: tuple[float, ...]
    max_contact_linear_residual_m: float
    requested_body_translation_m: float
    requested_body_rotation_rad: float


def micro_correct_stance(
    *,
    joint_positions_rad: np.ndarray,
    lower_limits_rad: np.ndarray,
    upper_limits_rad: np.ndarray,
    stance_jacobian_base: np.ndarray,
    stance_jacobian_joints: np.ndarray,
    desired_base_delta: np.ndarray,
    desired_joint_delta: np.ndarray | None = None,
    max_joint_delta_rad: float = 0.02,
    max_base_translation_m: float = 0.002,
    max_base_rotation_rad: float = 0.005,
    max_contact_linear_residual_m: float = 2e-5,
    joint_reference_weight: float = 1e-4,
) -> KinematicCorrection:
    """Compute one bounded local correction for all actuated leg joints.

    Inputs:
      desired_base_delta: 6-vector [dx,dy,dz, dRx,dRy,dRz] in the SAME
          world coordinates as the supplied translational Jacobians, where
          rotation uses the small-angle world-axis convention.
      Jacobians are shape (3*stance_count, 6) and (3*stance_count, n_joints).
          Rows are xyz foot movements at the actual fixed world contacts.
      desired_joint_delta: optional swing-leg posture increment; the stance
          constraints get priority. None is equivalent to zero increment.

    Method: constrained weighted linear least squares. The joint reference is
    soft, never more important than keeping stance contact Jacobian residual
    below the configured tolerance. No numerical integration or state update.

    Returns zero movement on veto. 'approved_kinematic' is only a first gate:
    force/torque/IK/collision/stability are *not checked* by this routine.
    """
    q = np.asarray(joint_positions_rad, dtype=float)
    lo = np.asarray(lower_limits_rad, dtype=float)
    hi = np.asarray(upper_limits_rad, dtype=float)
    Jb = np.asarray(stance_jacobian_base, dtype=float)
    Jq = np.asarray(stance_jacobian_joints, dtype=float)
    target = np.asarray(desired_base_delta, dtype=float)
    if q.ndim != 1 or len(q) == 0:
        raise ValueError("Expected a nonempty 1D joint vector")
    n = len(q)
    if lo.shape != (n,) or hi.shape != (n,):
        raise ValueError("Joint limit shape mismatch")
    if target.shape != (6,):
        raise ValueError("Base delta must contain 6 numbers")
    if Jb.ndim != 2 or Jb.shape[1] != 6 or Jb.shape[0] < 9 or Jb.shape[0] % 3:
        raise ValueError("Need at least 3 stance feet, with xyz Jacobian rows")
    if Jq.shape != (Jb.shape[0], n):
        raise ValueError("Joint Jacobian shape mismatch")
    reference = np.zeros(n) if desired_joint_delta is None else np.asarray(desired_joint_delta, dtype=float)
    if reference.shape != (n,):
        raise ValueError("Joint reference shape mismatch")
    if not all(np.all(np.isfinite(x)) for x in (q,lo,hi,Jb,Jq,target,reference)):
        raise ValueError("Non-finite kinematic input")
    if np.any(lo >= hi) or max_joint_delta_rad <= 0 or max_contact_linear_residual_m <= 0 or joint_reference_weight <= 0:
        raise ValueError("Nonpositive limit/tolerance or reversed joint bounds")
    translation = float(np.linalg.norm(target[:3]))
    rotation = float(np.linalg.norm(target[3:]))

    def veto(reason: str, residual: float = 0.) -> KinematicCorrection:
        return KinematicCorrection(False, reason, tuple([0.]*n), float(residual), translation, rotation)

    if translation > max_base_translation_m + 1e-12 or rotation > max_base_rotation_rad + 1e-12:
        return veto("requested torso correction exceeds small-displacement envelope")
    if np.any(q < lo - 1e-10) or np.any(q > hi + 1e-10):
        return veto("joint position already outside model limits")
    low = np.maximum(lo - q, -max_joint_delta_rad)
    high = np.minimum(hi - q, max_joint_delta_rad)
    # lsq_linear demands strictly ordered bounds. On a joint hard limit we
    # could eliminate that variable, but conservatively veto for now.
    if np.any(low >= high):
        return veto("joint at a hard limit (fixed-variable reduction not implemented)")

    # Tiny reference weight retains unconstrained swing-leg motions while
    # prioritizing stationary stance feet.
    A = np.vstack((Jq, joint_reference_weight*np.eye(n)))
    b = np.r_[-Jb@target, joint_reference_weight*reference]
    result = lsq_linear(A, b, bounds=(low,high), tol=1e-11, max_iter=300,
                        lsq_solver="exact")
    if not result.success:
        return veto("bounded linearized IK solver failed")
    delta = np.asarray(result.x)
    foot_error = np.asarray(Jb@target + Jq@delta).reshape(-1,3)
    residual = float(np.max(np.linalg.norm(foot_error, axis=1)))
    if residual > max_contact_linear_residual_m:
        return veto("stance foot drift exceeds linearized tolerance", residual)
    return KinematicCorrection(True, "linearized stance-foot constraint satisfied; dynamic safety not evaluated",
                               tuple(float(x) for x in delta), residual, translation, rotation)
