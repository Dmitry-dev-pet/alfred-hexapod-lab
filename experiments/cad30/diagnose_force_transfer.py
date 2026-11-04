"""Probe load-transfer actuation on the *existing* FNK0052 CAD30 MuJoCo model.

All trials are cloned from exactly the same six-legged stable stand.  A small
inverse-kinematic downward foot target is applied with the same assumed PD
servo model and 0.7 Nm torque limit.  This is an actuator/plant sensitivity
experiment: no claim of locomotion improvement, physical calibration, or
faithful insect neural control.

Requires original CAD30 files restored privately alongside the hexlab package.
No third-party CAD meshes or physics model binaries are distributed in repo.
"""
from __future__ import annotations
import argparse
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = next((p for p in Path(__file__).resolve().parents if (p/'hexlab').is_dir()), None)
if ROOT is None:
    raise RuntimeError('Place script under existing CAD30 directory with hexlab/')
sys.path.insert(0, str(ROOT))
from hexlab.stand_v04 import static_pose, hold_under_gravity
from hexlab.stand_geometry_v04 import load_legs, tip_world
from hexlab.step_one_leg_v04 import solve_waypoint, floor_contacts

CODES = ('fl', 'ml', 'bl', 'br', 'mr', 'fr')
CASES = {'no_press': [], 'press_fl': ['fl'], 'press_ml': ['ml'],
         'press_fr': ['fr'], 'press_neighbors_of_fl': ['ml', 'fr'],
         'feedback_neighbors_of_fl': ['ml', 'fr']}


def loads(model, data):
    floor = int(model.geom('floor').id)
    tibias = {int(model.geom(f'tibia{c}_collision_0').id): c for c in CODES}
    out = {c: 0.0 for c in CODES}
    force = np.zeros(6)
    for i in range(data.ncon):
        contact = data.contact[i]
        a, b = int(contact.geom1), int(contact.geom2)
        other = b if a == floor else (a if b == floor else -1)
        if other not in tibias:
            continue
        mujoco.mj_contactForce(model, data, i, force)
        out[tibias[other]] += max(0.0, float(force[0]))
    return out


def run(press_depth_m=0.004, duration_s=1.5, friction=1.1):
    model = mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'))
    model.geom_friction[model.geom('floor').id, 0] = friction
    for c in CODES:
        model.geom_friction[model.geom(f'tibia{c}_collision_0').id, 0] = friction
    data = mujoco.MjData(model)
    legs = load_legs(model, data, mujoco)
    static_pose(model, data, mujoco, legs, .145, .15)
    fixed_stand_targets = data.qpos.copy()
    init = hold_under_gravity(model, data, mujoco, fixed_stand_targets, 1.)
    if not init['stable_by_proxy_thresholds'] or init['last_0p4s_min_contact_legs'] < 6:
        raise RuntimeError(f'Cannot initialize six-leg stand: {init}')
    stand_qpos = data.qpos.copy()
    stand_qvel = data.qvel.copy()
    stand_time = data.time
    base_id = int(model.body('base_link_2').id)
    stand_local_up = data.xmat[base_id].reshape(3,3).T @ np.array([0.,0.,1.])
    reference = mujoco.MjData(model)
    foot_delta = {}
    ik_err = {}
    for leg in legs:
        reference.qpos[:] = fixed_stand_targets
        reference.qvel[:] = 0.
        mujoco.mj_forward(model, reference)
        name = leg['suffix']
        q = reference.qpos[leg['qadrs']].copy()
        target = tip_world(model, reference, leg) + np.array([0., 0., -press_depth_m])
        fitted, err = solve_waypoint(model, reference, mujoco, leg, target, q)
        foot_delta[name] = fitted-q
        ik_err[name] = round(float(err)*1000, 3)
        if err > .001:
            raise RuntimeError(f'Cannot implement press {name}: IK error {err}')
    motor_jids = model.actuator_trnid[:, 0].astype(int)
    qa = np.array([model.jnt_qposadr[j] for j in motor_jids],dtype=int)
    da = np.array([model.jnt_dofadr[j] for j in motor_jids],dtype=int)
    motor_index = {int(j):i for i,j in enumerate(motor_jids)}
    motor_by_leg = {leg['suffix']:np.array([motor_index[j] for j in leg['jids']]) for leg in legs}
    qref = fixed_stand_targets[qa].copy()
    bounds = model.actuator_ctrlrange.copy()
    dt = float(model.opt.timestep)
    base = int(model.body('base_link_2').id)
    results = []
    for name, chosen in CASES.items():
        state = mujoco.MjData(model)
        state.qpos[:] = stand_qpos
        state.qvel[:] = stand_qvel
        state.time = stand_time
        mujoco.mj_forward(model, state)
        steps = round(duration_s/dt)
        sample_every = max(1,round(.02/dt))
        trace = []
        max_tau=0
        sat=0
        feedback_press_m=0.
        filtered_fl=loads(model,state)['fl']
        ema=1. - math.exp(-dt/.05)
        for step in range(steps):
            t=step*dt
            alpha=float(np.clip((t-.2)/.2,0.,1.))
            targets=qref.copy()
            if name=='feedback_neighbors_of_fl':
                # Only target leg's own force transmitted locally to its two
                # neighbor controllers.  No global force sum, COM, or gait clock.
                observed=loads(model,state)['fl']
                filtered_fl += ema*(observed-filtered_fl)
                desired_target_N=1.50
                if t > .2:
                    # Integral feedback and 20mm/s rate limit; 3mm maximum.
                    shift=float(np.clip(3.0*(filtered_fl-desired_target_N)*dt,
                                        -.02*dt, .02*dt))
                    feedback_press_m=float(np.clip(feedback_press_m+shift,0.,min(press_depth_m,.003)))
                scale=feedback_press_m/press_depth_m
            else:
                scale=alpha
            for leg in chosen:
                targets[motor_by_leg[leg]] += scale * foot_delta[leg]
            raw_tau=7.*(targets-state.qpos[qa])-.32*state.qvel[da]
            sat+=int(np.any((raw_tau<bounds[:,0]) | (raw_tau>bounds[:,1])))
            tau=np.clip(raw_tau,bounds[:,0],bounds[:,1]);state.ctrl[:]=tau
            max_tau=max(max_tau, float(np.max(np.abs(tau))))
            mujoco.mj_step(model,state)
            if step % sample_every == 0 or step==steps-1:
                f=loads(model,state)
                pos=state.xpos[base]
                rot=state.xmat[base].reshape(3,3)
                # Compare body up to initial standing orientation (CAD frame can be rotated).
                tilt=math.degrees(math.acos(float(np.clip((rot @ stand_local_up)[2], -1, 1))))
                trace.append({'t_s':round(t+dt,3),
                              'force_N':{c:round(f[c],4) for c in CODES},
                              'root_height_m':round(float(pos[2]),6),
                              'tilt_abs_deg':round(tilt,4),
                              'contacts':sorted(floor_contacts(model,state,mujoco)),
                              'max_tau_nm':round(float(np.max(np.abs(tau))),4),
                              'commanded_preload_mm':round((feedback_press_m if name=='feedback_neighbors_of_fl' else alpha*press_depth_m if chosen else 0.)*1000,4)})
        tail=[x for x in trace if x['t_s']>=duration_s-.4]
        avg={c:round(float(np.mean([x['force_N'][c] for x in tail])),4) for c in CODES}
        results.append({'name':name,'press_legs':chosen,'tail_mean_force_by_leg_N':avg,
                        'tail_mean_total_normal_N':round(sum(avg.values()),4),
                        'tail_mean_tilt_deg':round(float(np.mean([x['tilt_abs_deg'] for x in tail])),4),
                        'tail_mean_root_height_m':round(float(np.mean([x['root_height_m'] for x in tail])),6),
                        'min_contacts':min(len(x['contacts']) for x in trace),
                        'tail_mean_preload_mm':round(float(np.mean([x['commanded_preload_mm'] for x in tail])),4),
                        'max_tau_nm':round(max_tau,5),
                        'motor_saturation_fraction':round(sat/steps,6),
                        'trace_50hz':trace})
    baseline=results[0]['tail_mean_force_by_leg_N']
    for r in results:
        r['difference_from_no_press_N']={c:round(r['tail_mean_force_by_leg_N'][c]-baseline[c],4)
                                          for c in CODES}
    return {'schema':'fnk0052_contact_load_step_response/1',
            'mujoco':mujoco.__version__, 'robot_mass_kg':round(float(model.body_mass.sum()),6),
            'friction_mu_assumed':friction,'commanded_press_depth_mm':press_depth_m*1000,
            'duration_s':duration_s,'identical_stabilized_initial_state':True,
            'IK_position_error_mm':ik_err,'all_modes_same_baseline_PD':True,
            'torque_bound_Nm_assumed':.7,'initial_stand':{'min_tail_contacts':init['last_0p4s_min_contact_legs']},
            'cases':results,
            'limits':['This is steady standing, not gait or recovery from damage.',
                      'Servo torque-speed/ground stiffness and foot geometry assumed.',
                      'PD position bias changes force passively and does not track a target force.',
                      'Bodies dynamically free; no welded feet or forced torso motion.']}


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--out',default='results/diagnose_force_transfer_cad30.json')
    p.add_argument('--depth-mm',type=float,default=4.0)
    a=p.parse_args()
    if not (0.1 <= a.depth_mm <= 4.):
        p.error('--depth-mm must be 0.1..4')
    r=run(press_depth_m=a.depth_mm/1000)
    dest=Path(a.out)
    dest.parent.mkdir(exist_ok=True,parents=True)
    dest.write_text(json.dumps(r,indent=2,ensure_ascii=False)+'\n')
    for x in r['cases']:
        print(x['name'], 'FL',x['tail_mean_force_by_leg_N']['fl'],'sum',x['tail_mean_total_normal_N'],
              'tilt',x['tail_mean_tilt_deg'],'sat',x['motor_saturation_fraction'])