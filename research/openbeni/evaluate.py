"""CPU evaluation of the pinned OpenBeni browser controller, without training.

Observation, motor law, command delay and model preparation follow upstream
4545b86a131d05519050df11a424368ef9f06b4f, web/game/physics.mjs and
web/scripts/export-assets.py. Native MuJoCo, flat plane, no reset after release.
Upstream code/assets: MIT, Copyright (c) 2026 Open Beni.
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import math
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import onnxruntime as ort

COMMIT = "4545b86a131d05519050df11a424368ef9f06b4f"
ACTION = ("L1", "R1", "L2", "R2", "LW", "RW", "head_yaw_joint")
OBS = ("L1", "R1", "head_yaw_joint", "L2", "R2", "LW", "RW")
DEFAULT = dict(L1=.3, R1=.3, L2=.5, R2=.5, LW=0., RW=0., head_yaw_joint=0., head_roll_joint=0.)


def digest(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def prepare(upstream, destination, friction=1.):
    """Use the upstream exporter, then add only a declared flat test surface."""
    head = subprocess.check_output(["git", "-C", str(upstream), "rev-parse", "HEAD"], text=True).strip()
    if head != COMMIT:
        raise ValueError(f"Expected upstream {COMMIT}, got {head}")
    subprocess.run(["python", str(upstream / "web/scripts/export-assets.py")], check=True, capture_output=True)
    generated = upstream / "web/.generated/robot/beni.xml"
    tree = ET.parse(generated)
    root = tree.getroot()
    root.find("compiler").set("meshdir", str((upstream / "assets/beni_base/meshes").resolve()))
    root.find("compiler").set("texturedir", str((upstream / "assets/beni_base/textures").resolve()))
    world = root.find("worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 .1", friction=f"{friction} .005 .0001", rgba=".75 .78 .8 1")
    ET.SubElement(world, "light", pos="1 -1 2", dir="-.3 .3 -1", diffuse=".8 .8 .8")
    ET.SubElement(root, "visual").append(ET.Element("global", offwidth="1280", offheight="720"))
    ET.indent(root)
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="unicode")
    config = json.loads((upstream / "web/game/robot-config.json").read_text())
    return mujoco.MjModel.from_xml_path(str(destination)), config


def torque(name, position, velocity, action, config):
    c = config["motors"][name]
    scale = 20 if name in ("LW", "RW") else .5 if name == "head_yaw_joint" else .25
    damping = config["kneeDampingScale"] if name in ("L2", "R2") else 1.
    raw = c["stiffness"] * (DEFAULT[name] + scale * action - position)
    raw += c["damping"] * damping * ((scale * action if name in ("LW", "RW") else 0) - velocity)
    motor = np.clip(raw, -c["effort_limit"], c["effort_limit"])
    angle = position - 1.
    spring = c.get("spring_scale", 0.) * (.22 * math.sin(angle) + .15 * math.sin(2 * angle))
    limit = c.get("simulation_effort_limit", c["effort_limit"])
    return float(np.clip(motor + spring, -limit, limit))


class Controller:
    def __init__(self, model, config, policy):
        self.model, self.config = model, config
        settings = ort.SessionOptions()
        settings.intra_op_num_threads = settings.inter_op_num_threads = 1
        self.policy = ort.InferenceSession(str(policy), settings, providers=["CPUExecutionProvider"])
        inputs = self.policy.get_inputs()
        if [(x.name, x.shape) for x in inputs] != [("obs", [1, 40]), ("h_in", [1, 1, 512])]:
            raise ValueError("This adapter requires the pinned 40-input recurrent policy")
        self.joints = {}
        for name in DEFAULT:
            j = model.joint(name).id
            self.joints[name] = (model.jnt_qposadr[j], model.jnt_dofadr[j], model.actuator(name).id)
        self.reset()

    def reset(self):
        self.hidden = np.zeros((1, 1, 512), np.float32)
        self.last = np.zeros(7, np.float32)
        self.delay = collections.deque([np.zeros(7, np.float32) for _ in range(self.config["commandDelaySteps"])])

    def observation(self, data):
        out = np.zeros(40, np.float32)
        out[:3] = data.sensor("imu_gyro").data
        w, x, y, z = data.sensor("imu_quat").data
        out[3:6] = [2*(w*y-x*z), -2*(y*z+w*x), -(1-2*(x*x+y*y))]
        for i, name in enumerate(OBS):
            q, v, _ = self.joints[name]
            if i < 5:
                out[9+i] = data.qpos[q] - DEFAULT[name]
            out[14+i] = data.qvel[v]
        out[21:28] = self.last
        out[28:31] = [0., 0., .11]
        out[31:33] = [0., 1.]
        out[33:36] = data.sensor("head_imu_gyro").data
        # Zero velocity/yaw commands, no jump request throughout this battery.
        return out

    def predict(self, observation):
        action, self.hidden = self.policy.run(None, {"obs": observation[None], "h_in": self.hidden})
        action = action[0]
        if not np.isfinite(action).all() or max(abs(action)) > 1000:
            raise ValueError("Invalid policy output")
        return action

    def apply(self, data, action):
        self.delay.append(action.copy())
        delayed = self.delay.popleft()
        data.qfrc_applied[:] = 0
        for i, name in enumerate(ACTION):
            q, v, a = self.joints[name]
            data.ctrl[a] = torque(name, data.qpos[q], data.qvel[v], delayed[i], self.config)
        q, v, a = self.joints["head_roll_joint"]
        data.ctrl[a] = torque("head_roll_joint", data.qpos[q], data.qvel[v], 0, self.config)


def measurement(model, data):
    wheels = np.zeros(2)
    body = 0.
    penetration = 0.
    force = np.zeros(6)
    wheel_ids = [model.geom(n + "_collision").id for n in ("LW", "RW")]
    for i, contact in enumerate(data.contact):
        penetration = max(penetration, -float(contact.dist))
        a, b = contact.geom1, contact.geom2
        robot = b if model.geom_bodyid[a] == 0 else a if model.geom_bodyid[b] == 0 else -1
        if robot < 0:
            continue
        mujoco.mj_contactForce(model, data, i, force)
        if np.linalg.norm(force[:3]) <= .1:
            continue
        if robot in wheel_ids:
            wheels[wheel_ids.index(robot)] += abs((contact.frame.reshape(3, 3).T @ force[:3])[2])
        else:
            body += float(np.linalg.norm(force[:3]))
    q = data.qpos[3:7]
    tilt = math.acos(float(np.clip(1-2*(q[1]**2+q[2]**2), -1, 1)))
    speed = float(np.linalg.norm(data.qvel[:2]))
    angular = float(np.linalg.norm(data.qvel[3:6]))
    pose_error = max(abs(data.qpos[model.jnt_qposadr[model.joint(n).id]]-v)
                     for n, v in dict(L1=.2848744094, R1=.289914161, L2=.4957503676, R2=.5020063519).items())
    ready = (tilt < math.radians(20) and abs(data.qpos[2]-.1101198718) <= .025
             and pose_error <= .3 and min(wheels) > .1
             and sum(wheels) >= .7*sum(model.body_mass)*9.81 and body == 0 and angular < 2)
    quiet = ready and speed < .1 and angular < 1.
    return dict(tilt_deg=math.degrees(tilt), height_m=float(data.qpos[2]), speed_m_s=speed,
                angular_rad_s=angular, wheel_load_N=wheels.tolist(), body_load_N=body,
                penetration_m=penetration, ready=bool(ready), quiet=bool(quiet))


def lowest_collision(model, data):
    lowest = math.inf
    for i in range(model.ngeom):
        if model.geom_bodyid[i] == 0 or not (model.geom_contype[i] or model.geom_conaffinity[i]):
            continue
        zrow = data.geom_xmat[i].reshape(3, 3)[2]
        size = model.geom_size[i]
        kind = model.geom_type[i]
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            extent = np.abs(zrow) @ size
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            extent = size[0]*math.sqrt(max(0., 1-zrow[2]**2)) + size[1]*abs(zrow[2])
        else:
            raise ValueError(f"Unimplemented collision support shape {kind}")
        lowest = min(lowest, data.geom_xpos[i, 2]-extent)
    return lowest


def initial_data(model, case):
    data = mujoco.MjData(model)
    for name, value in DEFAULT.items():
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = value
    axis, angle = case["axis"], math.radians(case["degrees"])
    data.qpos[3:7] = [math.cos(angle/2), *[math.sin(angle/2)*(i == axis) for i in range(3)]]
    data.qpos[2] = 0
    mujoco.mj_forward(model, data)
    data.qpos[2] += .0002-lowest_collision(model, data)
    mujoco.mj_forward(model, data)
    return data


def analyse(rows, active_at):
    active = [r for r in rows if r["t"] >= active_at-1e-8]
    run = 0
    first = None
    for r in active:
        run = run+1 if r["ready"] else 0
        if run >= 100 and first is None:
            first = round(r["t"]-active_at-.99, 4)
    final = [r for r in rows if r["t"] >= rows[-1]["t"]-1.99-1e-8]
    return dict(recovered_1s=first is not None, recovery_onset_s=first,
                final_2s_ready=all(r["ready"] for r in final),
                final_2s_quiet=all(r["quiet"] for r in final),
                final=active[-1], peak_penetration_m=max(r["penetration_m"] for r in rows),
                max_tilt_deg=max(r["tilt_deg"] for r in active))


def run_case(model, controller, case, output, active=True, seconds=10.):
    data = initial_data(model, case)
    controller.reset()
    active_at = 0. if case["id"] == "upright" else .5
    rows, samples, observations, actions = [], [], [], []
    steps = round((active_at+seconds)/.01)
    peak_torque = np.zeros(model.nu)
    peak_penetration = 0.
    peak_floor_penetration = 0.
    peak_self_penetration = 0.
    peak_pair = None
    for step in range(steps+1):
        mujoco.mj_forward(model, data)
        stats = measurement(model, data)
        stats["t"] = round(float(data.time), 8)
        rows.append(stats)
        samples.append(dict(t=stats["t"], qpos=data.qpos.tolist(), qvel=data.qvel.tolist(), **{k:v for k,v in stats.items() if k != "t"}))
        if step == steps:
            break
        obs = controller.observation(data)
        enabled = active and data.time >= active_at-1e-8
        action = controller.predict(obs) if enabled else np.zeros(7, np.float32)
        observations.append(obs)
        actions.append(action)
        for _ in range(controller.config["decimation"]):
            if enabled:
                controller.apply(data, action)
            else:
                data.ctrl[:] = 0
            peak_torque = np.maximum(peak_torque, abs(data.ctrl))
            mujoco.mj_step(model, data)
            for c in data.contact:
                depth = -float(c.dist)
                if depth > peak_penetration:
                    peak_penetration = depth
                    peak_pair = dict(t=float(data.time), geoms=[model.geom(int(g)).name for g in (c.geom1, c.geom2)],
                                     bodies=[model.body(int(model.geom_bodyid[g])).name for g in (c.geom1, c.geom2)])
                if 0 in model.geom_bodyid[[c.geom1, c.geom2]]:
                    peak_floor_penetration = max(peak_floor_penetration, depth)
                else:
                    peak_self_penetration = max(peak_self_penetration, depth)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise ValueError("Nonfinite physical state")
        if enabled:
            controller.last = action.copy()
    name = case["id"] + ("" if active else "-idle")
    result = analyse(rows, active_at)
    result.update(case=name, policy_enabled=active, active_at_s=active_at,
                  physics_steps=steps*5, samples=len(samples),
                  peak_torque_Nm=peak_torque.tolist(),
                  peak_substep_penetration_m=peak_penetration,
                  peak_floor_penetration_m=peak_floor_penetration,
                  peak_self_penetration_m=peak_self_penetration,
                  peak_contact_pair=peak_pair,
                  warnings={mujoco.mjtWarning(i).name:int(w.number) for i,w in enumerate(data.warning) if w.number})
    payload = dict(schema="openbeni-measured-trace/v1", case=case, result=result,
                   source_commit=COMMIT, controller="pinned upstream browser law on native CPU MuJoCo",
                   samples=samples)
    path = output / (name+".json")
    path.write_text(json.dumps(payload, separators=(",", ":"), allow_nan=False))
    np.savez_compressed(output/(name+"-policy.npz"), observation=np.array(observations), action=np.array(actions))
    result["trace_sha256"] = digest(path)
    print(json.dumps({k:result[k] for k in ("case", "recovered_1s", "recovery_onset_s", "final_2s_ready", "final_2s_quiet", "peak_substep_penetration_m", "warnings")}), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    parser.add_argument("--cases", nargs="*")
    parser.add_argument("--seconds", type=float, default=10.)
    parser.add_argument("--idle-controls", action="store_true")
    parser.add_argument("--contact-timeconstant", type=float)
    args = parser.parse_args()
    args.upstream = args.upstream.resolve()
    args.out.mkdir(parents=True, exist_ok=True)
    model, config = prepare(args.upstream, args.out/"scene.xml")
    if args.contact_timeconstant is not None:
        if args.contact_timeconstant < 2*model.opt.timestep:
            raise ValueError("Contact time constant must be at least 2 physics steps")
        model.geom_solref[:, 0] = args.contact_timeconstant
        mujoco.mj_saveLastXML(str(args.out/"scene.xml"), model)
    policy = args.upstream/"assets/models/policy.onnx"
    controller = Controller(model, config, policy)
    cases = [dict(id=n, axis=a, degrees=d) for n,a,d in
             [("upright",1,0), ("front",1,90), ("back",1,-90),
              ("left",0,90), ("right",0,-90), ("upside-down",1,180)]]
    if args.cases:
        cases = [c for c in cases if c["id"] in args.cases]
    results = [run_case(model, controller, c, args.out, seconds=args.seconds) for c in cases]
    if args.idle_controls:
        results.extend(run_case(model, controller, c, args.out, active=False, seconds=args.seconds) for c in cases if c["id"] != "upright")
    summary = dict(upstream_commit=COMMIT, policy_sha256=digest(policy), engine=mujoco.__version__,
                   onnxruntime=ort.__version__, model_mass_kg=float(sum(model.body_mass)),
                   model_nq=model.nq, model_nv=model.nv, motor_config=config,
                   protocol=dict(duration_s=args.seconds, passive_settle_s=.5, measurement_hz=100,
                                 physics_hz=500, recovery_hold_s=1, final_hold_s=2,
                                 quiet_speed_limit_m_s=.1, quiet_angular_limit_rad_s=1.,
                                 terrain="infinite flat plane, friction 1.0",
                                 contact_timeconstant_override=args.contact_timeconstant),
                   results=results)
    (args.out/"summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
