"""Generate an approximate 18-DOF Freenove FNK0052 MJCF robot model.

Known: link lengths from Freenove's public Python code (33/90/110 mm).
Assumed: hip mounting coordinates, mass/inertia, contact material, servo torque,
angle limits, torso dimensions. These MUST be calibrated before hardware use.
No Freenove source code is copied.
"""
from __future__ import annotations

import math
from pathlib import Path
from xml.etree import ElementTree as ET

LINKS = (0.033, 0.090, 0.110)  # m: coxa, femur, tibia
# Each tuple: name, hip attachment (body frame, m), radial direction in degrees.
LEGS = (
    ("front_left",  (0.094, +0.071, 0.0), +42),
    ("middle_left", (0.000, +0.087, 0.0), +90),
    ("rear_left",   (-0.094, +0.071, 0.0), +138),
    ("front_right", (0.094, -0.071, 0.0), -42),
    ("middle_right",(0.000, -0.087, 0.0), -90),
    ("rear_right",  (-0.094,-0.071, 0.0), -138),
)
SERVO_PEAK_NM = 1.079  # illustrative: 11 kgf cm at ~6V, stall, not continuous!
SERVO_MODEL_LIMIT_NM = 0.70  # assumed derating, NOT verified safe continuous torque
TORSO_MASS_KG = 0.52
SEGMENT_MASSES_KG = (0.045, 0.065, 0.034)  # each leg, approximate
FOOT_MASS_KG = 0.003


def vec(vals: tuple[float, ...] | list[float]) -> str:
    return " ".join(f"{x:.7g}" for x in vals)


def create_model() -> ET.Element:
    root = ET.Element("mujoco", {"model": "freenove_fnk0052_approx"})
    ET.SubElement(root, "compiler", {"angle": "radian", "autolimits":"true", "inertiafromgeom":"true"})
    ET.SubElement(root, "option", {"timestep":"0.002", "gravity":"0 0 -9.81", "integrator":"implicitfast"})
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", {"damping":"0.035", "armature":"0.003"})
    ET.SubElement(default, "geom", {"condim":"4", "friction":"1.3 0.005 0.0001", "solref":"0.007 1", "solimp":"0.9 0.95 0.001"})
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", {"offwidth":"960", "offheight":"720"})
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", {"name":"sun", "pos":"-0.3 0 2", "dir":"0 0 -1"})
    ET.SubElement(world, "geom", {"name":"floor", "type":"plane", "size":"2 2 0.05", "rgba":"0.82 0.82 0.80 1"})
    torso = ET.SubElement(world, "body", {"name":"torso", "pos":"0 0 0.165"})
    ET.SubElement(torso, "freejoint", {"name":"root"})
    ET.SubElement(torso, "geom", {"name":"torso_geom", "type":"box", "size":"0.125 0.078 0.025", "mass":f"{TORSO_MASS_KG}", "rgba":"0.12 0.18 0.21 1"})
    ET.SubElement(torso, "site", {"name":"com_marker", "type":"sphere", "size":"0.007", "rgba":"1 0.2 0.1 1"})
    ET.SubElement(torso, "camera", {"name":"tracking", "pos":"-0.7 -0.6 0.45", "xyaxes":"0.62 -0.79 0 0.36 0.28 0.89", "mode":"trackcom"})
    actuators = ET.SubElement(root, "actuator")
    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "framequat", {"name":"torso_orientation", "objtype":"body", "objname":"torso"})
    ET.SubElement(sensor, "framepos", {"name":"torso_position", "objtype":"body", "objname":"torso"})
    key_angles = []
    joint_specs = (('yaw', '-0.65 0.65'),('hip','-0.35 1.25'),('knee','0.0 2.0'))
    for name, mount, heading in LEGS:
        yaw = math.radians(heading)
        co = ET.SubElement(torso, "body", {"name":f"{name}_coxa", "pos":vec(mount), "quat":vec((math.cos(yaw/2),0,0,math.sin(yaw/2)))})
        ET.SubElement(co, "joint", {"name":f"{name}_yaw", "type":"hinge", "axis":"0 0 1", "range":joint_specs[0][1]})
        ET.SubElement(co, "geom", {"type":"capsule", "fromto":f"0 0 0 {LINKS[0]} 0 0", "size":"0.014", "mass":f"{SEGMENT_MASSES_KG[0]}", "rgba":"0.15 0.43 0.62 1"})
        fem = ET.SubElement(co, "body", {"name":f"{name}_femur", "pos":f"{LINKS[0]} 0 0"})
        ET.SubElement(fem, "joint", {"name":f"{name}_hip", "type":"hinge", "axis":"0 1 0", "range":joint_specs[1][1]})
        ET.SubElement(fem, "geom", {"type":"capsule", "fromto":f"0 0 0 {LINKS[1]} 0 0", "size":"0.013", "mass":f"{SEGMENT_MASSES_KG[1]}", "rgba":"0.24 0.52 0.68 1"})
        tib = ET.SubElement(fem, "body", {"name":f"{name}_tibia", "pos":f"{LINKS[1]} 0 0"})
        ET.SubElement(tib, "joint", {"name":f"{name}_knee", "type":"hinge", "axis":"0 1 0", "range":joint_specs[2][1]})
        ET.SubElement(tib, "geom", {"type":"capsule", "fromto":f"0 0 0 {LINKS[2]} 0 0", "size":"0.010", "mass":f"{SEGMENT_MASSES_KG[2]}", "rgba":"0.33 0.63 0.72 1"})
        ET.SubElement(tib, "geom", {"name":f"{name}_foot_geom", "type":"sphere", "pos":f"{LINKS[2]} 0 0", "size":"0.013", "mass":f"{FOOT_MASS_KG}", "rgba":"0.90 0.54 0.19 1"})
        ET.SubElement(tib, "site", {"name":f"{name}_foot_site", "type":"sphere", "pos":f"{LINKS[2]} 0 0", "size":"0.014", "rgba":"1 0 0 0.3"})
        ET.SubElement(sensor,"touch", {"name":f"{name}_contact", "site":f"{name}_foot_site"})
        for kind, lohi in joint_specs:
            ET.SubElement(actuators, "position", {"name":f"{name}_{kind}_motor", "joint":f"{name}_{kind}", "kp":"8", "ctrlrange":lohi, "forcerange":f"{-SERVO_MODEL_LIMIT_NM} {SERVO_MODEL_LIMIT_NM}", "forcelimited":"true"})
        key_angles += [0.0, 0.55, 0.70]
    keys = ET.SubElement(root, "keyframe")
    ET.SubElement(keys, "key", {"name":"stand", "qpos":vec([0,0,0.165,1,0,0,0]+key_angles),"ctrl":vec(key_angles)})
    return root


def write_model(path: Path) -> None:
    tree = ET.ElementTree(create_model())
    ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="unicode", xml_declaration=True)


if __name__ == "__main__":
    write_model(Path("models/freenove_fnk0052_approx.xml"))
    print("Generated models/freenove_fnk0052_approx.xml")