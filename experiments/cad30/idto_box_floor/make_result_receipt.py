"""Parse a real local IDTO smoke log into an auditable non-gait receipt."""
from __future__ import annotations
import hashlib
import json
import re
from pathlib import Path


def extract(stdout: str, exit_code: int) -> dict:
    rows = re.findall(r"^\|\s*(\d+)\s*\|\s*([0-9.eE+-]+)\s*\|", stdout, re.M)
    if exit_code != 0 or not rows or "Max actuated torque" not in stdout:
        return {"status": "SOLVER_NOT_COMPLETED", "exit_code": exit_code, "iter_rows": len(rows)}
    def metric(label: str) -> float:
        m = re.search(rf"^{re.escape(label)}\s*:\s*([0-9.eE+-]+)", stdout, re.M)
        if not m:
            raise ValueError(f"missing {label}")
        return float(m.group(1))
    def vector(label: str) -> list[float]:
        m = re.search(rf"^{re.escape(label)}\s*:\s*\[(.*?)\]", stdout, re.M)
        if not m:
            raise ValueError(f"missing {label}")
        return [float(x) for x in m.group(1).split(',')]
    q = vector("q[T]")
    goal = vector("q_nom[T]")
    if len(q)!=27 or len(goal)!=27:
        raise ValueError("CAD30 expected 27 generalized position coordinates")
    y0=-0.03648025664
    return {
        "status": "IDTO_SINGLE_TRAJECTORY_SOLVE_EXIT_0_NOT_PERIODIC_GAIT",
        "exit_code": exit_code,
        "iteration_rows": len(rows),
        "iteration0_cost": float(rows[0][1]),
        "final_iter_cost": float(rows[-1][1]),
        "solver_reported_max_iterations": 'Maximum iterations reached' in stdout,
        "max_optimized_actuated_generalized_torque_Nm": metric('Max actuated torque'),
        "max_optimized_unactuated_generalized_residual_mixed_units": metric('Max unactuated torque'),
        "saved_initial_base_world_y_m": y0,
        "terminal_base_world_y_m": q[5],
        "requested_terminal_base_world_y_m": goal[5],
        "planned_terminal_base_y_displacement_mm": (q[5]-y0)*1000,
        "requested_terminal_base_y_displacement_mm": (goal[5]-y0)*1000,
        "full_periodic_cycle": False,
        "independent_mujoco_replay": False,
        "measured_per_leg_contact_fz": False,
        "three_support_gate_checked": False,
        "hardware_calibrated": False,
    }


def main():
    p=Path(__file__).parent
    s=(p/'results/idto_60_iter_stdout.log').read_text()
    code=int((p/'results/idto_exit_code.txt').read_text())
    r=extract(s,code)
    r.update({
        "schema": "cad30-drake-idto-finite-floor-box-smoke-v1",
        "date_utc": "2026-11-05",
        "idto_commit": "de0629c7811aa9b330e56c4385629005b09495f0",
        "drake_version": "1.30.0",
        "mujoco_reference_version": "3.14.0",
        "drake_model": "Original private CAD30 collision OBJ meshes from verified STL topology, finite box floor top z=0",
        "drake_mujoco_g0_parity": "MATCH_KINEMATICS_AND_INERTIA_ONLY",
        "g0_drake_probe_equal_before_after_only_floor": True,
        "contact_model_equivalence_to_mujoco": "NOT_VERIFIED",
        "original_mujoco_model_unchanged": True,
        "source_only_public_git": True,
        "caveats": [
            "The solver reached the configured 60-iteration cap; this does not certify convergence or feasibility.",
            "A single half-second soft-cost trajectory is not four-legged gait, let alone repeated periodic walking.",
            "The base displacement is relative to the saved six-contact prebalanced checkpoint, not the unshifted original robot start.",
            "Actual per-tibia Fz, 3-contact support, slip, and joint/torque hard limits were not verified.",
            "Finite box is a Drake-only ground approximation; original MuJoCo uses an infinite plane.",
            "Drake imported actuator caps / joint armature differently; optimized torque is not hardware-certification.",
        ]
    })
    gp=json.loads((p/'results/geometry_receipt.json').read_text())
    r['scene_sha256']=gp['drake_scene_sha256']
    r['source_scene_sha256']=gp['original_scene_sha256']
    raw=p/'results/idto_60_iter_stdout.log'
    r['solver_stdout_sha256']=hashlib.sha256(raw.read_bytes()).hexdigest()
    r['solver_stderr_sha256']=hashlib.sha256((p/'results/idto_60_iter_stderr.log').read_bytes()).hexdigest()
    r['drake_probe_sha256']=hashlib.sha256((p/'results/box_drake_probe.txt').read_bytes()).hexdigest()
    (p/'results/IDTO_BOX_FLOOR_COMPACT_20261105.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:r[k] for k in ["status","exit_code","iteration_rows","final_iter_cost","planned_terminal_base_y_displacement_mm"]},indent=2))

if __name__=='__main__':
    main()