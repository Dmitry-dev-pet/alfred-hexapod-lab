"""Independent geometry/goal audit of all saved step-goal trials."""
from pathlib import Path
import argparse
import contextlib
import copy
import hashlib
import io
import json
import math
import numpy as np
import mujoco as mj
import tune_recovery as sim
from verify_source import verify

HERE = Path(__file__).resolve().parent


def check_trace(trace, entry, model, data):
    h, edge = trace["terrain"]["height_m"], trace["terrain"]["edge_x_m"]
    joints = model.actuator_trnid[:, 0]
    feet = {s: mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, s + "_foot_collision") for s in ("left", "right")}
    step = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "step_top")
    floor = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor")
    visible = [g for g in range(model.ngeom) if model.geom_group[g] == 2]
    assert len(visible) == 70 and model.nq == 35 and model.nu == 14
    actual_goal, final_quiet = [], []
    max_geom_error = max_surface_overlap = 0.
    margin = entry["params"].get("motor_target_margin_rad", .035)
    for f in trace["frames"]:
        assert np.isfinite(f["qpos"]).all() and np.isfinite(f["qvel"]).all()
        data.qpos[:] = f["qpos"]
        data.qvel[:] = f["qvel"]
        mj.mj_forward(model, data)
        full_sole = True
        for side, gid in feet.items():
            mesh = model.geom_dataid[gid]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            pts = model.mesh_vert[start:start + count].astype(float) @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]
            lower, upper = pts.min(axis=0), pts.max(axis=0)
            max_geom_error = max(max_geom_error, float(np.max(np.abs(lower - f["foot_bounds"][side]["min"]))),
                                 float(np.max(np.abs(upper - f["foot_bounds"][side]["max"]))))
            full_sole &= bool(lower[0] > edge + .001 and upper[0] < edge + 1.199
                              and lower[1] > -.599 and upper[1] < .599)
        top_loads = {s: 0. for s in feet}
        floor_loads = {s: 0. for s in feet}
        for contact in f["contacts"]:
            pair = contact["geoms"]
            if abs(contact["normal"][2]) > .85:
                for side, gid in feet.items():
                    if gid in pair and (floor in pair or step >= 0 and step in pair):
                        floor_loads[side] += max(0., contact["force_n"])
                    if gid in pair and step >= 0 and step in pair and contact["pos"][2] >= h - .001:
                        top_loads[side] += max(0., contact["force_n"])
        complete = bool(h and full_sole and all(v > .05 for v in top_loads.values()))
        assert complete == f["complete_top_support"]
        actual_goal.append(complete)
        z = float(data.qpos[2])
        tilt = math.degrees(math.acos(np.clip(data.xmat[1].reshape(3, 3)[2, 2], -1, 1)))
        stable = (tilt < 20 and .09 < z < .18 + h and np.linalg.norm(data.qvel[:3]) < .06
                  and np.linalg.norm(data.qvel[3:6]) < .7 and all(v > .05 for v in floor_loads.values()))
        final_quiet.append(stable)
        target = np.array(f["motor_target_rad"])
        assert np.all(target >= model.jnt_range[joints, 0] + margin - 1e-10)
        assert np.all(target <= model.jnt_range[joints, 1] - margin + 1e-10)
        if step >= 0:
            for gid in visible:
                distance = float(mj.mj_geomDistance(model, data, gid, step, .002, None))
                max_surface_overlap = max(max_surface_overlap, -distance)
    standing = all(final_quiet[-50:])
    reached = bool(h and standing and all(actual_goal[-50:]))
    assert standing == entry["standing_final_1s"]
    assert reached == entry["step_goal_reached"]
    assert max_geom_error < 2e-7
    # Surface overlap is measured independently; per-substep quality is retained.
    quality = (not entry["warnings"] and not entry["physics_reset_detected"]
               and entry["max_environment_penetration_m"] < .001
               and entry["max_self_contact_penetration_m"] < .001
               and entry["max_motor_limit_overshoot_rad"] < math.radians(1))
    assert quality == entry["quality_gate"]
    assert entry["status"] == ("PASS" if quality and (reached if h else standing) else "FAIL")
    return {"trace_file": entry["trace_file"], "independent_goal": reached,
            "quality_gate": quality, "foot_bounds_max_error_m": max_geom_error,
            "all_visible_mesh_max_step_overlap_m": max_surface_overlap}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", type=Path, required=True)
    a = ap.parse_args()
    source = verify()
    sim.initialize(a.workspace)
    summary = json.loads((HERE / "traces/summary.json").read_text())
    audits = []
    corruption_checks = []
    for entry in summary["episodes"]:
        path = HERE / "traces" / entry["trace_file"]
        assert sim.sha(path) == entry["trace_sha256"]
        trace = json.loads(path.read_text())
        assert trace["summary"] == {k: v for k, v in entry.items() if k not in ("trace_file", "trace_sha256")}
        sim.E.EDGE = entry["params"]["edge_x_m"]
        with contextlib.redirect_stdout(io.StringIO()):
            m, d, *_ = sim.BUILD(sim.IP, entry["params"]["height_m"])
        audit = check_trace(trace, entry, m, d)
        audits.append(audit)
        print(json.dumps(audit), flush=True)
        if entry['trace_file'] == summary['selected']['trace_file']:
            for kind in ('sole_geometry', 'illegal_target'):
                tampered = copy.deepcopy(trace)
                if kind == 'sole_geometry':
                    tampered['frames'][0]['foot_bounds']['left']['min'][0] += .02
                else:
                    tampered['frames'][0]['motor_target_rad'][0] += 10.
                try:
                    check_trace(tampered, entry, m, d)
                except AssertionError:
                    corruption_checks.append(kind)
                else:
                    raise AssertionError('Tampered trace was accepted: ' + kind)
    result = {"schema": "microduck-step-goal-independent-audit/v1", "source": source,
              "audits": audits, "byte_repeat": summary["repeat"],
              "accepted_step_trials": sum(e["step_goal_reached"] and e["quality_gate"] for e in summary["episodes"]),
              "goal_without_quality_trials": sum(e["step_goal_reached"] and not e["quality_gate"] for e in summary["episodes"]),
              "corruption_rejected": corruption_checks,
              "policy_weights_changed": False, "hardware_tested": False}
    (HERE / "verification.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
