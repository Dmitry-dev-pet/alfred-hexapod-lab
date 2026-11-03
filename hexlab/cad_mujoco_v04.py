"""Build a CAD-derived, mass-scenario MuJoCo FNK0052 model, without vendoring Freenove CAD.

The community MJCF/meshes are fetched from exact GitHub commits and Git-blob
integrity checked. License: original Freenove meshes/STEP are NOT MIT. Attribution
and restrictions: upstream LICENSE-3RD-PARTY.md.

We preserve all source visual meshes, retain *only* the six tibia contact meshes,
and replace the expensive rest-of-robot mesh collisions with a chassis box.
Correction masses and mounted electronics are explicitly represented. This is
an *estimated* physical model, not a validated digital twin.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
import urllib.error
import urllib.request
from pathlib import Path
from xml.etree import ElementTree as ET

from hexlab.community_mass_v04 import REF_PATH, SCENARIOS_PATH, load, validate, totals

ROOT = Path(__file__).resolve().parents[1]
LOCK = ROOT / "data" / "fnk0052_cad_assets_lock.json"
DEFAULT_CACHE = ROOT / ".cache" / "fnk0052_cad"
SCENARIOS = ("lower", "nominal", "upper")
SIZE_LIMIT = 12 * 1024 * 1024


def git_blob_sha(payload: bytes) -> str:
    """Git hashes each blob using the header 'blob <len>\\0'."""
    header = b"blob " + str(len(payload)).encode("ascii") + bytes((0,))
    return hashlib.sha1(header + payload).hexdigest()


def pinned_records(lock: dict) -> list[tuple[str, dict]]:
    records = list(lock["mjcf"].items())
    records.extend((f"meshes/{x['name']}", x) for x in lock["meshes"])
    if len(lock["meshes"]) != 32 or len(records) != 34:
        raise ValueError("Unexpected source asset set")
    for filename, spec in records:
        path = Path(filename)
        if path.is_absolute() or ".." in path.parts or len(path.parts) != 2:
            raise ValueError(f"Unsafe source path {filename!r}")
        if spec["size_bytes"] <= 0 or spec["size_bytes"] > SIZE_LIMIT:
            raise ValueError(f"Unexpected source asset size: {filename}")
    return records


def fetch_pinned(lock: dict, cache: Path, *, offline: bool = False) -> Path:
    """Materialize permitted external assets into ignored, checksum-checked cache."""
    cache = Path(cache)
    source = cache / "source"
    for filename, spec in pinned_records(lock):
        target = source / filename
        if target.exists():
            old = target.read_bytes()
            if len(old) == spec["size_bytes"] and git_blob_sha(old) == spec["sha_git_blob"]:
                continue
            if offline:
                raise ValueError(f"Cached asset checksum mismatch: {target}")
        if offline:
            raise FileNotFoundError(f"Missing verified offline CAD asset {target}")
        url = ("https://raw.githubusercontent.com/" + lock["upstream"] + "/"
               + lock["pr_commit"] + "/" + lock["directory"] + "/" + filename)
        req = urllib.request.Request(url, headers={"User-Agent": "alfred-hexapod-lab-cad-rebuild/0.4"})
        last_error = None
        for attempt in range(3):
            try:
                with urllib.request.urlopen(req, timeout=40) as resp:
                    payload = resp.read(SIZE_LIMIT + 1)
                if len(payload) != spec["size_bytes"] or git_blob_sha(payload) != spec["sha_git_blob"]:
                    raise ValueError(f"Pinned content integrity mismatch: {filename}")
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(payload)
                last_error = None
                break
            except (urllib.error.URLError, TimeoutError) as exc:
                last_error = exc
                if attempt < 2:
                    time.sleep(attempt + 1)
        if last_error is not None:
            raise RuntimeError(f"Cannot fetch pinned Freenove CAD {filename}") from last_error
    return source


def _f(v: float) -> str:
    return f"{v:.12g}"


def _box_inertia(mass: float, dimensions_m: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = dimensions_m
    return (mass * (y * y + z * z) / 12,
            mass * (x * x + z * z) / 12,
            mass * (x * x + y * y) / 12)


def _cylinder_inertia(mass: float, radius: float, half_length: float) -> tuple[float, float, float]:
    length = 2 * half_length
    transverse = mass * (3 * radius * radius + length * length) / 12
    return (transverse, transverse, 0.5 * mass * radius * radius)


def _add_welded_mass(
    parent: ET.Element, name: str, mass: float, pos: tuple[float, float, float],
    dimensions: tuple[float, float, float], *,
    shape: str = "box", rgba: str = "0.25 0.65 0.25 0.75", visible: bool = True
) -> None:
    if mass <= 0:
        return
    child = ET.SubElement(parent, "body", {"name": name, "pos": " ".join(map(_f, pos))})
    if shape == "box":
        inertia = _box_inertia(mass, dimensions)
        sizes = " ".join(_f(x / 2) for x in dimensions)
    elif shape == "cylinder":
        radius, half_len, _unused = dimensions
        inertia = _cylinder_inertia(mass, radius, half_len)
        sizes = f"{_f(radius)} {_f(half_len)}"
    else:
        raise ValueError("Unsupported accessory shape")
    ET.SubElement(child, "inertial", {
        "pos": "0 0 0", "mass": _f(mass),
        "diaginertia": " ".join(_f(v) for v in inertia),
    })
    if visible:
        ET.SubElement(child, "geom", {
            "type": shape, "size": sizes, "mass": "0",
            "rgba": rgba, "contype": "0", "conaffinity": "0", "group": "2"
        })


def _scale_inertial(body: ET.Element, target_mass: float) -> None:
    inertial = body.find("inertial")
    if inertial is None:
        raise ValueError(f"Missing CAD inertia in {body.get('name')}")
    original = float(inertial.get("mass", "nan"))
    if not (original > 0 and target_mass > 0):
        raise ValueError("Invalid CAD or scenario mass")
    ratio = target_mass / original
    inertial.set("mass", _f(target_mass))
    for key in ("diaginertia", "fullinertia"):
        if key in inertial.attrib:
            inertial.set(key, " ".join(_f(float(x) * ratio) for x in inertial.get(key).split()))


def _src_visual_servo_positions(body: ET.Element) -> list[tuple[float, float, float]]:
    """Visual housing centers are *approximate* mass-correction sites."""
    markers = []
    for geom in body.findall("geom"):
        if geom.get("class") == "visual" and geom.get("mesh") == "bigservo1.stl":
            markers.append(tuple(float(x) for x in geom.get("pos", "0 0 0").split()))
    return markers


def patch_source_mjcf(base: ET.Element, scenario: dict, case: str) -> dict:
    """Apply mass scenario, keep CAD visuals, use limited colliders and torque."""
    if case not in SCENARIOS:
        raise ValueError(f"Unknown mass case {case}")
    rows = {r["id"]: r for r in scenario["calculations"]["rows"]}
    mass = lambda n: rows[n][case + "_kg"]
    bodies = {body.get("name"): body for body in base.findall(".//worldbody//body")}
    if len([k for k in bodies if k.startswith("coxa")]) != 6:
        raise ValueError("CAD robot must have six coxa bodies")
    if len([k for k in bodies if k.startswith("tibia")]) != 6:
        raise ValueError("CAD robot must have six tibia bodies")
    if len([k for k in bodies if k.startswith("femur")]) != 6:
        raise ValueError("CAD robot must have six femur bodies")

    # Original model has inertial masses for CAD bodies; these include servo
    # housing geometry already. NEVER add full servo masses on top.
    _scale_inertial(bodies["base_link_2"], mass("cad_static_chassis_and_holders"))
    _scale_inertial(bodies["neck"],
                    float(bodies["neck"].find("inertial").get("mass"))
                    * mass("head_pan_tilt_assembly") / .058499)
    _scale_inertial(bodies["face"],
                    float(bodies["face"].find("inertial").get("mass"))
                    * mass("head_pan_tilt_assembly") / .058499)
    correction_count = 0
    for name, body in list(bodies.items()):
        if name.startswith("femur"):
            _scale_inertial(body, mass("femur_group_per_leg"))
        if name.startswith("coxa") or name.startswith("tibia"):
            target = mass("coxa_group_per_leg" if name.startswith("coxa") else "tibia_group_per_leg")
            original = float(body.find("inertial").get("mass"))
            missing = target - original
            if missing < -1e-7:
                raise ValueError(f"Cannot subtract servo mass in scenario {case}: {name}")
            placements = _src_visual_servo_positions(body)
            expected = 2 if name.startswith("coxa") else 1
            if len(placements) != expected:
                raise ValueError(f"Expected {expected} servo housing center(s) on {name}, found {len(placements)}")
            for index, p in enumerate(placements):
                # Same local mechanical link: additional mass represents CAD
                # servo-density error; do not create a new actuator or joint.
                _add_welded_mass(body, f"servo_delta_{name}_{index}",
                                 missing / expected, p, (.040, .020, .040), visible=False)
                correction_count += 1

    # 4 removable cells, locations close to published CAD holder pair. Axes,
    # distances and exact mass still need to be measured from an assembled kit.
    torso = bodies["base_link_2"]
    for x in (-.0215, .0215):
        for z in (-.027, .027):
            _add_welded_mass(
                torso, "battery_%s_%s" % ("left" if x < 0 else "right",
                                          "rear" if z < 0 else "front"),
                mass("18650_high_current_cells"), (x, -.035, z),
                (.009, .0325, 0.0), shape="cylinder", rgba="0.22 0.35 0.67 0.75"
            )
    _add_welded_mass(torso, "raspberry_pi4", mass("raspberry_pi_4_board"),
                     (0, .025, .008), (.085, .004, .056), rgba="0.10 0.38 0.19 0.80")

    misc = mass("not_in_CAD_control_electronics_wires_screws")
    _add_welded_mass(torso, "control_shield_estimated", misc * .4,
                     (0, .037, .008), (.080, .004, .055), rgba="0.14 0.45 0.22 .80")
    _add_welded_mass(torso, "cable_harness_estimated", misc * .35,
                     (0, -.018, .0), (.095, .025, .070), visible=False)
    _add_welded_mass(torso, "fasteners_estimated", misc * .25,
                     (0, -.030, .0), (.130, .012, .085), visible=False)

    # Collision: visual CAD geometry remains untouched. MuJoCo treats the six
    # tibia meshes as convex contact shapes. Remove hundreds of heavy colliders
    # elsewhere; use one torso collision box instead.
    contact_meshes = []
    removed_colliders = 0
    visuals = 0
    for body in base.findall(".//worldbody//body"):
        for geom in list(body.findall("geom")):
            if geom.get("class") == "visual":
                visuals += 1
            elif geom.get("class") == "collision":
                if body.get("name", "").startswith("tibia") and geom.get("mesh") == "tibia.stl":
                    geom.set("friction", "1.1 0.005 0.0001")
                    contact_meshes.append(body.get("name"))
                else:
                    body.remove(geom)
                    removed_colliders += 1
    if len(contact_meshes) != 6 or visuals < 100:
        raise ValueError(f"Unexpected source geometries: tibia={contact_meshes}, visuals={visuals}")
    ET.SubElement(torso, "geom", {
        "name": "torso_simple_collision", "type": "box", "pos": "0 -0.032 0",
        "size": "0.095 0.017 0.065", "contype": "1", "conaffinity": "1",
        "condim": "3", "friction": "1.1 0.005 0.0001",
        "rgba": "0.6 0.25 0.25 0.0", "group": "3", "mass": "0",
    })

    # Source PR uses 10 N*m motor control limits: illustrative and far too
    # optimistic for the target kit. 0.7 N*m and 0.2 N*m are *assumed* caps;
    # this is NOT a fitted torque-speed model or safe continuous rating.
    default_motors = base.find("./default/default[@class='robot']/default[@class='motor']")
    if default_motors is None:
        raise ValueError("Missing upstream motor defaults")
    joint = default_motors.find("joint")
    motor = default_motors.find("motor")
    joint.set("damping", "0.035")
    joint.set("frictionloss", "0.015")
    joint.set("armature", "0.002")
    motor.set("ctrlrange", "-0.7 0.7")
    motor.set("forcerange", "-0.7 0.7")
    motor.set("ctrllimited", "true")
    motor.set("forcelimited", "true")
    actuator = base.find("actuator")
    if actuator is None or len(actuator.findall("motor")) != 20:
        raise ValueError("Expected 20 CAD motors")
    for entry in actuator.findall("motor"):
        if entry.get("joint") in {"motor0", "motor1"}:
            entry.set("ctrlrange", "-0.2 0.2")
            entry.set("forcerange", "-0.2 0.2")

    option = base.find("option")
    if option is None:
        option = ET.SubElement(base, "option")
    option.set("timestep", "0.002")
    option.set("gravity", "0 0 -9.81")
    option.set("integrator", "implicitfast")
    return {"scenario": case, "visual_mesh_instances": visuals,
            "tibia_contact_meshes": len(contact_meshes),
            "removed_heavy_colliders": removed_colliders,
            "estimated_servo_mass_sites": correction_count,
            "new_accessory_bodies": 4 + 1 + 3,
            "estimated_assembly_mass_kg": totals(scenario)[case]}


def build(
    case: str = "nominal",
    *,
    cache: Path = DEFAULT_CACHE,
    offline: bool = False,
) -> tuple[Path, dict]:
    ref, scenario, lock = load(REF_PATH), load(SCENARIOS_PATH), load(LOCK)
    validate(ref, scenario)
    source_dir = fetch_pinned(lock, cache, offline=offline)
    tree = ET.parse(source_dir / "mjcf/base.xml")
    report = patch_source_mjcf(tree.getroot(), scenario, case)
    # Materialize variants alongside externally cached source meshes so
    # original relative asset paths ../meshes/<file>.stl remain valid.
    target_model = source_dir / "mjcf" / f"base_{case}.xml"
    tree.write(target_model, encoding="unicode", xml_declaration=True)
    scene = ET.parse(source_dir / "mjcf/scene.xml")
    includes = scene.getroot().findall("include")
    if len(includes) != 1 or includes[0].get("file") != "base.xml":
        raise ValueError("Unexpected upstream MJCF scene layout")
    includes[0].set("file", target_model.name)
    target_scene = source_dir / "mjcf" / f"scene_{case}.xml"
    scene.write(target_scene, encoding="unicode", xml_declaration=True)
    report.update({
        "model_file": str(target_scene),
        "mjcf_source_sha": lock["mjcf"]["mjcf/base.xml"]["sha_git_blob"],
        "upstream_commit": lock["pr_commit"],
        "verified_asset_count": len(pinned_records(lock)),
        "is_physical_measurement": False,
        "collision_note": "Visual CAD meshes; 6 tibia hull collisions + approximate torso box",
    })
    return target_scene, report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=SCENARIOS, default="nominal")
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--load-mujoco", action="store_true",
                        help="Compile model and run a short smoke simulation; requires mujoco and numpy")
    parser.add_argument("--output-receipt", type=Path)
    args = parser.parse_args()
    scene, report = build(args.scenario, cache=args.cache, offline=args.offline)
    if args.load_mujoco:
        import mujoco
        import numpy as np
        model = mujoco.MjModel.from_xml_path(str(scene))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        loaded_mass = float(sum(model.body_mass))
        expected = report["estimated_assembly_mass_kg"]
        if not math.isclose(loaded_mass, expected, rel_tol=1e-5, abs_tol=1e-5):
            raise AssertionError(f"MuJoCo mass {loaded_mass} does not match source estimate {expected}")
        initial_body_z = float(data.xpos[model.body("base_link_2").id][2])
        for i in range(50):  # 0.1 simulated seconds: compile/contact smoke test only
            mujoco.mj_step(model, data)
            if not np.all(np.isfinite(data.qpos)):
                raise AssertionError(f"Non-finite joint state at step {i}")
        report.update({"mujoco_compiled": True, "mujoco_version": mujoco.__version__,
                       "loaded_mass_kg": loaded_mass, "nbody": model.nbody,
                       "njnt": model.njnt, "ngeom": model.ngeom,
                       "nactuator": model.nu, "initial_torso_z_m": initial_body_z,
                       "end_torso_z_m": float(data.xpos[model.body("base_link_2").id][2]),
                       "smoke_steps": 50})
    if args.output_receipt:
        args.output_receipt.parent.mkdir(parents=True, exist_ok=True)
        args.output_receipt.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
