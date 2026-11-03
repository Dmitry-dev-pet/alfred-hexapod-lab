"""Approximate 18-DOF Freenove FNK0052 robot, geometry aligned to OEM IK.

Verified against Freenove Code/Server/control.py (master, checked 2026-11-03):
  coordinate_to_angle(..., l1=33, l2=90, l3=110),
  transform_coordinates() rotations [54,0,-54,-126,180,126] degrees,
  local radial mount offsets [94,85,94,94,85,94] millimetres,
  transformed local z coordinate = input z -14 mm,
  calculate_posture_balance() body_structure:
       [[55,76],[85,0],[55,-76],[-55,-76],[-85,0],[-55,76]] mm.
These numbers describe effective kinematic axes, not the actual part outlines.

Assumed: masses, servo torque-speed/position regulator parameters, limits,
contact materials, geometry of the body shells, height to floor and joint radii.
The link orientation corresponds to OEM neutral legs at local [140,0,-39] mm.
Do not transfer servo limits to hardware without measurements/calibration.
No Freenove source code is copied.
"""
from __future__ import annotations

import math
from pathlib import Path
from xml.etree import ElementTree as ET

LINKS = (0.033, 0.090, 0.110)  # m: coxa, femur, tibia
# OEM ordering: 1..6 around the robot, from +X,+Y toward -X,+Y.
# Name, effective hip axis in body frame (m), heading in degrees.
LEGS = (
    ("leg1", (0.055, +0.076, -0.014), +54),
    ("leg2", (0.085,  0.000, -0.014),   0),
    ("leg3", (0.055, -0.076, -0.014), -54),
    ("leg4", (-0.055,-0.076, -0.014),-126),
    ("leg5", (-0.085, 0.000, -0.014), 180),
    ("leg6", (-0.055,+0.076, -0.014), 126),
)
OEM_BODY_STRUCTURE_MM = ((55, 76), (85, 0), (55, -76), (-55, -76), (-85, 0), (-55, 76))
OEM_FOOT_XY_MM = ((137.1, 189.4), (225, 0), (137.1, -189.4),
                  (-137.1, -189.4), (-225, 0), (-137.1, 189.4))
OEM_NEUTRAL_LOCAL_X_MM = 140.0
OEM_NEUTRAL_LOCAL_Z_MM = -39.0  # OEM body_points z=-25 and transform z-14
# Geometry-only IK, not an experimentally verified servo calibration.
def neutral_leg_angles() -> tuple[float, float, float]:
    l1, l2, l3 = LINKS
    target_x = OEM_NEUTRAL_LOCAL_X_MM / 1000 - l1
    target_z = -OEM_NEUTRAL_LOCAL_Z_MM / 1000
    cosine = (target_x**2 + target_z**2 - l2**2 - l3**2) / (2*l2*l3)
    assert -1 <= cosine <= 1
    knee = math.acos(cosine)
    hip = math.atan2(target_z, target_x) - math.atan2(l3*math.sin(knee), l2 + l3*math.cos(knee))
    return (0.0, hip, knee)

# Torso reference frame origin is 14 mm above hip axes, with the standing
# origin placed to put the spherical toe surface on the floor.
FOOT_RADIUS_M = 0.013
NEUTRAL_ROOT_HEIGHT_M = FOOT_RADIUS_M - (LEGS[0][1][2] + OEM_NEUTRAL_LOCAL_Z_MM / 1000)
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
    torso = ET.SubElement(world, "body", {"name":"torso", "pos":f"0 0 {NEUTRAL_ROOT_HEIGHT_M:.7g}"})
    ET.SubElement(torso, "freejoint", {"name":"root"})
    ET.SubElement(torso, "geom", {"name":"torso_geom", "type":"box", "size":"0.105 0.068 0.018", "mass":f"{TORSO_MASS_KG}", "rgba":"0.12 0.18 0.21 1"})
    ET.SubElement(torso, "site", {"name":"com_marker", "type":"sphere", "size":"0.007", "rgba":"1 0.2 0.1 1"})
    ET.SubElement(torso, "camera", {"name":"tracking", "pos":"-0.7 -0.6 0.45", "xyaxes":"0.62 -0.79 0 0.36 0.28 0.89", "mode":"trackcom"})
    actuators = ET.SubElement(root, "actuator")
    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "framequat", {"name":"torso_orientation", "objtype":"body", "objname":"torso"})
    ET.SubElement(sensor, "framepos", {"name":"torso_position", "objtype":"body", "objname":"torso"})
    key_angles = []
    joint_specs = (('yaw', '-0.65 0.65'),('hip','-1.05 1.50'),('knee','0.0 2.25'))  # assumed safe limits
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
        ET.SubElement(tib, "geom", {"name":f"{name}_foot_geom", "type":"sphere", "pos":f"{LINKS[2]} 0 0", "size":f"{FOOT_RADIUS_M:.7g}", "mass":f"{FOOT_MASS_KG}", "rgba":"0.90 0.54 0.19 1"})
        ET.SubElement(tib, "site", {"name":f"{name}_foot_site", "type":"sphere", "pos":f"{LINKS[2]} 0 0", "size":"0.014", "rgba":"1 0 0 0.3"})
        ET.SubElement(sensor,"touch", {"name":f"{name}_contact", "site":f"{name}_foot_site"})
        for kind, lohi in joint_specs:
            ET.SubElement(actuators, "position", {"name":f"{name}_{kind}_motor", "joint":f"{name}_{kind}", "kp":"8", "ctrlrange":lohi, "forcerange":f"{-SERVO_MODEL_LIMIT_NM} {SERVO_MODEL_LIMIT_NM}", "forcelimited":"true"})
        key_angles += list(neutral_leg_angles())
    keys = ET.SubElement(root, "keyframe")
    ET.SubElement(keys, "key", {"name":"stand", "qpos":vec([0,0,NEUTRAL_ROOT_HEIGHT_M,1,0,0,0]+key_angles),"ctrl":vec(key_angles)})
    return root


def write_model(path: Path) -> None:
    tree = ET.ElementTree(create_model())
    ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="unicode", xml_declaration=True)


if __name__ == "__main__":
    write_model(Path("models/freenove_fnk0052_approx.xml"))
    print("Generated models/freenove_fnk0052_approx.xml")