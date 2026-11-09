#!/usr/bin/env python3
"""Frozen-policy roll/get-up handoff study; motor targets only, no root actuation."""
from __future__ import annotations
import argparse,contextlib,hashlib,io,json,math,sys,time
from pathlib import Path
import mujoco as mj
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'step_goal_v1'))
import tune_recovery as baseline
from tune_recovery import sha,sole_inside,quiet
DT,CDT=.0005,.02

def initialize(workspace):
    gate=baseline.initialize(workspace)
    global E,BUILD,IP,ROLL
    E,BUILD,IP,ROLL=baseline.E,baseline.BUILD,baseline.IP,baseline.ROLL
    return gate

def run(params):
    height, edge = params["height_m"], params["edge_x_m"]
    E.EDGE = edge
    with contextlib.redirect_stdout(io.StringIO()):
        m, d, ctrl, _ = BUILD(IP, height)
        mj.mj_resetDataKeyframe(m, d, mj.mj_name2id(m, mj.mjtObj.mjOBJ_KEY, "STAND"))
        joints = m.actuator_trnid[:, 0]
        sq = m.jnt_qposadr[joints]
        # Initial perturbations only; never applied after the first mj_step.
        seed = params.get("seed", 0)
        if seed:
            rng = np.random.default_rng(seed)
            d.qpos[sq] += rng.uniform(-.005, .005, len(sq))
        ctrl.reset(d.qpos)
        ctrl.last_ts = float(d.time)
        mj.mj_forward(m, d)
        p = IP.PolicyInference(m, d, walking_onnx_path=str(E.POLICY),
                               roulade_onnx_path=str(ROLL),
                               roulade_duration=params["roll_duration_s"],
                               bam_ctrl=ctrl, use_projected_gravity=True, new_cmd_obs=True)
        E.bench.install_encoder_view(p, ctrl, m, d)
        bq, _ = E.bench.backlash_indices(m)
        ctrl.q_target[:] = d.qpos[sq] + d.qpos[bq]
        p.set_vel_cmd(0, 0, 0)
    assert not p.use_delay and m.opt.timestep == DT
    margin = params.get("motor_target_margin_rad", .035)
    lo, hi = m.jnt_range[joints, 0] + margin, m.jnt_range[joints, 1] - margin
    guard = params.get("head_guard_margin_rad", margin)
    for i,j in enumerate(joints):
        if mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, int(j)) == "head_pitch":
            lo[i], hi[i] = m.jnt_range[j,0]+guard, m.jnt_range[j,1]-guard
    floor = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, "floor")
    step = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, "step_top")
    feet = {s: mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, s + "_foot_collision") for s in ("left", "right")}
    root = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "trunk_base")
    if root < 0:
        root = 1
    initial = {"qpos": d.qpos.tolist(), "qvel": d.qvel.tolist()}
    trigger = handback = first_top = None
    handoff_reason = handoff_state = None
    blend_s = params.get("blend_s", 0.)
    kind = params.get("handoff_kind", "timer")
    max_limit_event = None
    phase_limit_peaks = {}
    max_target_delta = 0.
    previous_target = None
    trigger_state = None
    frames, approach_velocity = [], []
    max_env = max_body_step = max_self = max_torque = max_limit = peak_force = 0.
    resets = False
    angle = frontier = 0.
    command = 0.
    goal_seen = False
    phase = "settle"
    # Every episode receives >= 4 s recovery, even with late triggers.
    end = 9.
    k = 0
    began = time.monotonic()
    while k * CDT < end - 1e-9:
        t = round(k * CDT, 8)
        mj.mj_forward(m, d)
        bounds = {s: E.foot_bounds(m, d, g) for s, g in feet.items()}
        gap = edge - max(b["max"][0] for b in bounds.values())
        if t >= 1 and phase == "settle":
            phase, command = "approach", params["approach_command_mps"]
        if phase == "approach":
            approach_velocity.append(float(d.qvel[0]))
            if gap <= params["trigger_gap_m"] or params.get("flat_control", False):
                trigger, phase, command = t, "roll", 0.
                trigger_state = {"qpos": d.qpos.tolist(), "qvel": d.qvel.tolist(), "toe_gap_m": gap}
                with contextlib.redirect_stdout(io.StringIO()):
                    p.trigger_behavior("roulade")
                end = max(end, t + (params["roll_duration_s"] if kind=="timer" else params.get("timeout_s",3.)) + blend_s + 4.)
            elif t >= 6.:
                phase, command = "no_trigger", 0.
        with contextlib.redirect_stdout(io.StringIO()):
            if kind == 'timer':
                old_behavior = p.behavior_mode
                p.update_behavior(CDT)
                if old_behavior and not p.behavior_mode:
                    handback, phase, handoff_reason = t, 'recovery', 'original_timer'
            elif trigger is not None and handback is None:
                age=t-trigger
                eligible=age>=params.get('minimum_roll_s',.12)
                by_angle=kind=='angle' and math.degrees(angle)>=params['angle_deg']
                by_contact=kind=='one_foot_top' and frames and any(v>.05 for v in frames[-1]['step_loads_n'].values())
                timeout=age>=params.get('timeout_s',3.)
                if timeout or eligible and (by_angle or by_contact):
                    p._end_behavior()
                    handback,phase=t,'recovery'
                    handoff_reason='timeout' if timeout else kind
            if handback is not None and handoff_state is None:
                handoff_state={'t':t,'age_s':t-trigger,'forward_rotation_deg':math.degrees(angle),
                               'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),
                               'foot_bounds':bounds,'previous_step_loads_n':frames[-1]['step_loads_n'] if frames else {}}
            if phase in ['recovery','blend']:
                command=params['recovery_command_mps'] if t < handback+2 and not goal_seen else 0.
            p.set_vel_cmd(command, 0, 0)
        blend_weight=0. if handback is None else 1.
        if handback is not None and blend_s>0 and t<handback+blend_s-1e-9:
            phase='blend'
            obs=p.get_observations().reshape(1,-1)
            rs=p.behavior_sessions['roulade'];ss=p.walking_session
            ar=rs.run([rs.get_outputs()[0].name],{rs.get_inputs()[0].name:obs})[0].squeeze(0).astype(np.float32)
            ast=ss.run([ss.get_outputs()[0].name],{ss.get_inputs()[0].name:obs})[0].squeeze(0).astype(np.float32)
            u=float(np.clip((t-handback+CDT)/blend_s,0,1));blend_weight=u*u*(3-2*u)
            action=(1-blend_weight)*ar+blend_weight*ast
        else:
            if handback is not None:phase='recovery'
            action=p.infer()
        target = np.clip(p.default_pose + action * p.action_scale, lo, hi)
        if previous_target is not None:max_target_delta=max(max_target_delta,float(abs(target-previous_target).max()))
        previous_target=target.copy()
        ctrl.q_target[:] = target
        p.last_action = ((target - p.default_pose) / p.action_scale).astype(np.float32)
        for _ in range(round(CDT / DT)):
            before = float(d.time)
            ctrl.update()
            mj.mj_step(m, d)
            resets |= abs(float(d.time) - before - DT) > 1e-8
            if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
                raise FloatingPointError("Nonfinite physics")
            max_torque = max(max_torque, float(np.abs(d.actuator_force).max()))
            errors=np.maximum(m.jnt_range[joints,0]-d.qpos[sq],d.qpos[sq]-m.jnt_range[joints,1])
            excursion=max(0.,float(errors.max()))
            phase_limit_peaks[phase]=max(phase_limit_peaks.get(phase,0.),excursion)
            if excursion>max_limit:
                j=int(errors.argmax());max_limit=excursion
                max_limit_event={'t_physics':float(d.time),'age_s':float(d.time)-trigger if trigger is not None else None,
                                 'phase':phase,'joint':mj.mj_id2name(m,mj.mjtObj.mjOBJ_JOINT,int(joints[j])),
                                 'overshoot_rad':excursion}
            if trigger is not None:
                vel = np.empty(6)
                mj.mj_objectVelocity(m, d, mj.mjtObj.mjOBJ_XBODY, root, vel, 1)
                angle += float(vel[1]) * DT
                frontier = max(frontier, angle)
            for ct in d.contact:
                pair = (int(ct.geom1), int(ct.geom2))
                env = floor in pair or step >= 0 and step in pair
                if env:
                    max_env = max(max_env, -float(ct.dist))
                    if step >= 0 and step in pair:
                        other = pair[1] if pair[0] == step else pair[0]
                        if m.body_weldid[m.geom_bodyid[other]] == root:
                            max_body_step = max(max_body_step, -float(ct.dist))
                else:
                    max_self = max(max_self, -float(ct.dist))
        mj.mj_forward(m, d)
        loads, top = {s: 0. for s in feet}, {s: 0. for s in feet}
        contacts = []
        for i, ct in enumerate(d.contact):
            pair = (int(ct.geom1), int(ct.geom2))
            wrench = np.zeros(6)
            mj.mj_contactForce(m, d, i, wrench)
            contacts.append({"geoms": list(pair), "force_n": float(wrench[0]),
                             "distance_m": float(ct.dist), "pos": ct.pos.tolist(),
                             "normal": ct.frame[:3].tolist()})
            if floor in pair or step >= 0 and step in pair:
                peak_force = max(peak_force, float(np.linalg.norm(wrench[:3])))
            if abs(ct.frame[2]) > .85:
                for s, g in feet.items():
                    if g in pair and (floor in pair or step >= 0 and step in pair):
                        loads[s] += max(0., float(wrench[0]))
                    if g in pair and step >= 0 and step in pair and ct.pos[2] >= height - .001:
                        top[s] += max(0., float(wrench[0]))
        bounds = {s: E.foot_bounds(m, d, g) for s, g in feet.items()}
        complete_top = bool(height and sole_inside(bounds, edge) and all(v > .05 for v in top.values()))
        goal_seen |= complete_top
        if complete_top and first_top is None:
            first_top = float(d.time)
        frames.append({"t": round(float(d.time), 6), "phase": phase,
                       **E.bench.sample(m, d, 0, 0), "command_mps": command,
                       "motor_target_rad": target.tolist(), "foot_bounds": bounds,
                       "foot_loads_n": loads, "step_loads_n": top, "contacts": contacts,
                       "complete_top_support": complete_top,
                       "forward_rotation_deg": math.degrees(angle), "blend_weight":blend_weight})
        k += 1
    last = frames[-50:]
    standing = all(quiet(f, height) for f in last)
    full_top = bool(height and standing and all(f["complete_top_support"] for f in last))
    warnings = {str(i): int(w.number) for i, w in enumerate(d.warning) if w.number}
    quality = not resets and not warnings and max_env < .001 and max_self < .001 and max_limit < math.radians(1)
    # Diagnostic ranking never changes the binary goal or quality thresholds.
    longest = current = 0
    for f in frames:
        current = current + 1 if f["complete_top_support"] else 0
        longest = max(longest, current)
    summary = {"params": params, "trigger_s": trigger, "actual_trigger": trigger_state,
               "handback_s": handback, "handoff_reason":handoff_reason, "handoff_state":handoff_state,
               "max_limit_event":max_limit_event, "phase_limit_peaks_rad":phase_limit_peaks,
               "max_motor_target_delta_rad":max_target_delta, "physical_steps":k*round(CDT/DT),
               "standing_final_1s": standing,
               "step_goal_reached": full_top, "quality_gate": quality,
               "status": "PASS" if quality and (full_top if height else standing) else "FAIL",
               "max_forward_rotation_deg": math.degrees(frontier),
               "final_forward_rotation_deg": math.degrees(angle),
               "first_complete_top_support_s": first_top,
               "longest_complete_top_support_s": longest * CDT,
               "max_trunk_x_m": max(f["qpos"][0] for f in frames),
               "final_position_m": frames[-1]["qpos"][:3],
               "max_environment_penetration_m": max_env, "max_body_step_penetration_m": max_body_step,
               "max_self_contact_penetration_m": max_self,
               "max_motor_limit_overshoot_rad": max(0., max_limit),
               "peak_torque_nm": max_torque, "peak_sampled_environment_force_n": peak_force,
               "max_actual_approach_vx_mps": max(approach_velocity, default=0.),
               "warnings": warnings, "physics_reset_detected": resets,
               "duration_s": frames[-1]["t"], "frames": len(frames)}
    return {"schema": "microduck-state-handoff/v1", "physics_dt_s": DT, "policy_dt_s": CDT,
            "stand_policy_sha256": sha(E.POLICY), "roll_policy_sha256": sha(ROLL),
            "terrain": {"height_m": height, "edge_x_m": edge, "half_size_xy_m": [.6, .6],
                        "center_xy_m": [edge + .6, 0]},
            "initial_state": initial, "summary": summary, "frames": frames}, time.monotonic() - began


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--workspace',type=Path,required=True)
    ap.add_argument('--cases',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    with contextlib.redirect_stdout(io.StringIO()):gate=initialize(a.workspace.resolve())
    entries=[]
    for params in json.loads(a.cases.read_text()):
        trace,elapsed=run(params);name=params['name']+'.json';p=a.out/name
        p.write_text(json.dumps(trace,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n')
        entry={**trace['summary'],'trace_file':name,'trace_sha256':sha(p)};entries.append(entry)
        print(json.dumps({'name':params['name'],'goal':entry['step_goal_reached'],'quality':entry['quality_gate'],
                          'status':entry['status'],'angle':round(entry['max_forward_rotation_deg'],1),
                          'overshoot_deg':round(math.degrees(entry['max_motor_limit_overshoot_rad']),3),
                          'handoff':entry['handback_s'],'reason':entry['handoff_reason'],'seconds':round(elapsed,2)}),flush=True)
    (a.out/'summary.json').write_text(json.dumps({'schema':'microduck-state-handoff-summary/v1','source_gate':gate,
             'code_sha256':sha(__file__),'episodes':entries},indent=2)+'\n')

if __name__=='__main__':main()
