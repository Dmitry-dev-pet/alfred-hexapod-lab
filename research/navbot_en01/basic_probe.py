"""Wheel-driven basic motions on the frozen v5 model; no body-pose prescription."""
from __future__ import annotations
import argparse, hashlib, json, math
from pathlib import Path
from .basic_commands import CASES, command, wheel_mix
from .basic_metrics import classify
from .linkage_model import load_evidence, set_configuration

HERE = Path(__file__).resolve().parent
PROTOCOL = HERE/'basic_protocol.json'


def run(out, *, case, model_path):
    import mujoco
    import numpy as np
    protocol = json.loads(PROTOCOL.read_text())
    if case not in CASES or mujoco.__version__ != '3.14.0':
        raise ValueError('Undeclared case or physics version')
    xml = Path(model_path).read_text()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    dt = float(m.opt.timestep)
    if dt != protocol['timestep_s']:
        raise ValueError('Wrong frozen timestep')
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    aid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
    bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    sid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n)
    torso, floor = bid('torso'), gid('floor')
    sides = ('left', 'right')
    wheel_bodies = [bid(s+'_wheel') for s in sides]
    wheel_geoms = [gid(s+'_wheel_geom') for s in sides]
    bodies = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(1, m.nbody)]
    joints = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j): int(m.jnt_qposadr[j]) for j in range(m.njnt)}
    set_configuration(m, d, load_evidence()['sides']['cad_z_positive']['points_xy_mm'], CASES[case]['q'])
    d.qpos[2] += .0185-min(float(d.xpos[b, 2]) for b in wheel_bodies)
    mujoco.mj_forward(m, d)
    period = round(protocol['control_period_s_ASSUMED']/dt)
    steps = round(protocol['duration_s']/dt)
    frames, force = [], np.zeros(6)
    jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    wheel_tau, goals = [0., 0.], [CASES[case]['q']]*2
    xref = yawref = last_speed = yaw = last_yaw = yawrate = 0.
    latched = False
    peak_hip = peak_wheel = max_pitch = max_roll = max_x = max_closure = max_floor_pen = max_self_pen = 0.
    saturated_hip = saturated_wheel = nonwheel_steps = both_support_steps = disabled_steps = cuts = 0
    hip_min, hip_max = [float('inf')]*2, [-float('inf')]*2
    slip_integral = slip_time = max_slip = signed_travel = 0.
    peak_wheel_loads = [0., 0.]
    stop_speeds, stop_yaw_rates = [], []
    start_axle = start_yaw = start_height = None
    min_height = float('inf')
    previous_axle = np.mean(d.xpos[wheel_bodies, :2], axis=0)
    cmd = command(case, 0.)
    for step in range(steps):
        t = float(d.time)
        if step % period == 0:
            cmd = command(case, t)
            xref += cmd['v_m_s']*period*dt
            yawref += cmd['yaw_rad_s']*period*dt
            R = d.xmat[torso].reshape(3, 3)
            pitch = math.atan2(-R[2, 0], R[2, 2])
            roll = math.atan2(R[2, 1], R[2, 2])
            gyro = float(d.sensor('imu_gyro').data[1])
            rate = sum(float(d.qvel[m.jnt_dofadr[jid('wheel_'+s)]]) for s in sides)/2
            odom = .0185*sum(float(d.qpos[joints['wheel_'+s]]) for s in sides)/2
            disable = abs(2*rate-last_speed) > 10 or abs(2*rate) > 50
            last_speed = 2*rate
            if disable:
                xref = odom
            common = .15*pitch+.05*gyro+(0. if disable else .008*(rate-cmd['v_m_s']/.0185)+.3*(odom-xref))
            differential = .004*(yawref-yaw)+.001*(cmd['yaw_rad_s']-yawrate)
            latched |= max(abs(pitch), abs(roll)) > math.radians(35)
            wheel_tau = [0., 0.] if latched else list(wheel_mix(common, differential))
            goals = [cmd['hip_q_rad']]*2
            disabled_steps += int(disable)*period
            cuts += int(latched)*period
        for i, side in enumerate(sides):
            j = jid('hip_'+side)
            q, v = float(d.qpos[joints['hip_'+side]]), float(d.qvel[m.jnt_dofadr[j]])
            raw = 3*(goals[i]-q)-.045*v
            limit = .14709975*(max(0, 1-abs(v)/11.6355) if raw*v > 0 else 1)
            d.ctrl[aid('a_hip_'+side)] = np.clip(raw, -limit, limit)
            d.ctrl[aid('a_wheel_'+side)] = wheel_tau[i]
            saturated_hip += int(abs(raw) > limit+1e-12)
            saturated_wheel += int(abs(wheel_tau[i]) >= .03-1e-10)
        peak_hip = max(peak_hip, *(abs(float(d.ctrl[aid('a_hip_'+s)])) for s in sides))
        peak_wheel = max(peak_wheel, *map(abs, wheel_tau))
        d.xfrc_applied[:] = 0
        mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
            raise RuntimeError('Nonfinite state')
        R = d.xmat[torso].reshape(3, 3)
        pitch, roll = math.atan2(-R[2, 0], R[2, 2]), math.atan2(R[2, 1], R[2, 2])
        angle = math.atan2(R[1, 0], R[0, 0])
        yaw += math.atan2(math.sin(angle-last_yaw), math.cos(angle-last_yaw))
        last_yaw = angle
        yawrate = float((R@d.sensor('imu_gyro').data)[2])
        axle = np.mean(d.xpos[wheel_bodies, :2], axis=0)
        delta = axle-previous_axle
        axle_speed = float(np.linalg.norm(delta)/dt)
        previous_axle = axle.copy()
        if start_axle is None and d.time >= 1.5:
            start_axle, start_yaw, start_height = axle.copy(), yaw, float(d.xpos[torso, 2])
        active = 1.5 <= d.time <= 4.5
        if active:
            signed_travel += float(delta@np.array([math.cos(yaw), math.sin(yaw)]))
            min_height = min(min_height, float(d.xpos[torso, 2]))
        if d.time >= 5.5:
            stop_speeds.append(axle_speed)
            stop_yaw_rates.append(abs(yawrate))
        max_pitch, max_roll = max(max_pitch, abs(pitch)), max(max_roll, abs(roll))
        max_x = max(max_x, abs(float(d.qpos[0])))
        closure = max(float(np.linalg.norm(d.site_xpos[sid(s+'_closure_calf')]-d.site_xpos[sid(s+'_closure_rod')])) for s in sides)
        max_closure = max(max_closure, closure)
        for i, side in enumerate(sides):
            hip_min[i] = min(hip_min[i], float(d.qpos[joints['hip_'+side]]))
            hip_max[i] = max(hip_max[i], float(d.qpos[joints['hip_'+side]]))
        names, touching, loads = [], set(), [0., 0.]
        normal = slip_sq_force = slip_force = 0.
        nonwheel = False
        for ci in range(d.ncon):
            c = d.contact[ci]
            if floor not in (int(c.geom1), int(c.geom2)):
                max_self_pen = max(max_self_pen, max(0., -float(c.dist)))
                continue
            other = int(c.geom2) if int(c.geom1) == floor else int(c.geom1)
            mujoco.mj_contactForce(m, d, ci, force)
            fn = max(0., float(force[0]))
            normal += fn
            max_floor_pen = max(max_floor_pen, max(0., -float(c.dist)))
            names.append(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, other))
            if fn > .05:
                touching.add(other)
                nonwheel |= other not in wheel_geoms
            if other in wheel_geoms:
                loads[wheel_geoms.index(other)] += fn
                if fn > .05:
                    mujoco.mj_jac(m, d, jacp, jacr, c.pos, int(m.geom_bodyid[other]))
                    velocity = jacp@d.qvel
                    n = c.frame[:3]
                    tangent = velocity-n*np.dot(velocity, n)
                    speed_sq = float(tangent@tangent)
                    slip_sq_force += fn*speed_sq
                    slip_force += fn
                    max_slip = max(max_slip, math.sqrt(speed_sq))
        for i in range(2):peak_wheel_loads[i] = max(peak_wheel_loads[i], loads[i])
        slip = math.sqrt(slip_sq_force/slip_force) if slip_force else 0.
        if active and slip_force:
            slip_integral += slip*slip*dt
            slip_time += dt
        nonwheel_steps += int(nonwheel)
        both_support_steps += int(all(x in touching for x in wheel_geoms))
        if step % 80 == 0:
            frames.append({'t':float(d.time), 'qpos':d.qpos.tolist(), 'qvel':d.qvel.tolist(), 'ctrl':d.ctrl.tolist(),
                'body_poses':[[*d.xpos[bid(n)].tolist(), *d.xquat[bid(n)].tolist()] for n in bodies],
                'ground_contact':bool(touching), 'nonwheel_ground_contact':nonwheel, 'contact_geoms':names,
                'ground_normal_force_n':normal, 'pitch_rad':pitch, 'roll_rad':roll, 'closure_error_m':closure,
                'command':dict(cmd), 'yaw_rad':yaw, 'yaw_reference_rad':yawref, 'yaw_rate_rad_s':yawrate,
                'axle_xy_m':axle.tolist(), 'axle_speed_m_s':axle_speed, 'wheel_normal_force_n':loads,
                'contact_slip_rms_m_s':slip, 'hip_targets_rad':goals[:]})
    metrics = {'max_abs_tilt_deg':math.degrees(max(max_pitch, max_roll)), 'nonwheel_ground_steps':nonwheel_steps,
        'final_stop_speed_m_s':float(np.mean(stop_speeds)), 'final_stop_yaw_rate_rad_s':float(np.mean(stop_yaw_rates)),
        'active_contact_slip_rms_m_s':math.sqrt(slip_integral/slip_time) if slip_time else 0.,
        'active_slip_supported_s':slip_time, 'peak_contact_slip_m_s':max_slip, 'peak_wheel_normal_force_n':peak_wheel_loads, 'hip_min_rad':min(hip_min), 'hip_max_rad':max(hip_max),
        'two_wheel_support_fraction':both_support_steps/steps, 'signed_axle_travel_m':signed_travel,
        'axle_net_displacement_m':float(np.linalg.norm(axle-start_axle)), 'yaw_change_deg':math.degrees(yaw-start_yaw),
        'final_yaw_error_deg':math.degrees(yawref-yaw), 'squat_height_change_m':start_height-min_height,
        'final_height_error_m':float(d.xpos[torso, 2])-start_height}
    assessment = classify(case, metrics, protocol['acceptance'])
    summary = {'schema':'en01-fourbar-diagnostic/v1', 'experiment':'basic_motion_v6', 'case':case, 'mode':'basic',
        'controller_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'command_source_sha256':hashlib.sha256((HERE/'basic_commands.py').read_bytes()).hexdigest(),
        'protocol_sha256':hashlib.sha256(PROTOCOL.read_bytes()).hexdigest(),
        'mujoco_version':mujoco.__version__, 'duration_s':6, 'timestep_s':dt, 'physics_steps':steps, 'frame_count':len(frames),
        'model_sha256':hashlib.sha256(xml.encode()).hexdigest(), 'geometry_source_blob':load_evidence()['source']['git_blob_sha1'],
        'model_profile':'cad_contact_v5', 'total_mass_kg':float(sum(m.body_mass)),
        'hip_continuous_ceiling_nm':.14709975, 'wheel_torque_ceiling_nm':.03, 'self_collision':'CAD_CONVEX_OUTER_ENVELOPES',
        'max_abs_pitch_deg':math.degrees(max_pitch), 'max_abs_roll_deg':math.degrees(max_roll), 'max_abs_x_m':max_x,
        'max_closure_error_m':max_closure, 'max_floor_penetration_m':max_floor_pen, 'max_self_proxy_penetration_m':max_self_pen,
        'hip_min_all_steps_rad':hip_min, 'hip_max_all_steps_rad':hip_max, 'peak_hip_torque_nm':peak_hip, 'peak_wheel_torque_nm':peak_wheel,
        'hip_saturated_fraction':saturated_hip/(2*steps), 'wheel_saturated_fraction':saturated_wheel/(2*steps),
        'disabled_distance_steps':disabled_steps, 'safety_wheel_cut_steps':cuts, 'upright_6s':metrics['max_abs_tilt_deg']<20 and nonwheel_steps==0,
        'metrics':metrics, **assessment, 'status':'UNCALIBRATED_BASIC_CONTROL_ADAPTATION',
        'hardware_motion_claim_allowed':False}
    payload = json.dumps({'summary':summary, 'body_names':bodies, 'joint_qpos_addresses':joints, 'frames':frames}, allow_nan=False, separators=(',',':'))+'\n'
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists() and out.read_text() != payload:
        raise ValueError('Immutable trace conflict')
    out.write_text(payload)
    return {**summary, 'trace_sha256':hashlib.sha256(payload.encode()).hexdigest()}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case', choices=CASES, required=True)
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(run(a.out, case=a.case, model_path=a.model), indent=2))
