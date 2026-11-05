"""Generate a private IDTO-only CAD30 contact proxy from the owner's CAD.

Only six distal tibia collisions remain (4 mm spheres); all other body
geometries are removed. This is an *approximation*, not contact parity.

Original masses, joints and body inertias remain unchanged. Do not publish
source MJCF, mesh data, or the generated private model.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

LEG_CODES = ("ml", "mr", "bl", "br", "fl", "fr")


def rewrite_collisions(xml_path: Path, tips_local: dict[str, list[float]], radius_m: float = 0.004) -> dict:
    """Rewrite only a private model copy. tips_local are in tibia body frames."""
    if not 0.001 <= radius_m <= 0.015:
        raise ValueError("Contact proxy radius must be in [1, 15] mm")
    if set(tips_local) != set(LEG_CODES):
        raise ValueError("Must provide six grounded-leg tip positions")
    tree = ET.parse(xml_path)
    replaced = []
    removed = 0
    for body in tree.getroot().iter("body"):
        body_name = body.get("name", "")
        for geom in list(body.findall("geom")):
            name = geom.get("name", "")
            suffix = body_name.removeprefix("tibia")
            if suffix in LEG_CODES and name == f"tibia{suffix}_collision_0":
                geom.set("type", "sphere")
                geom.set("size", f"{radius_m:.9f}")
                geom.set("pos", " ".join(f"{x:.12g}" for x in tips_local[suffix]))
                geom.attrib.pop("mesh", None)
                geom.attrib.pop("quat", None)
                geom.set("mass", "0")
                replaced.append(suffix)
            else:
                # These CAD visual and ancillary geometries are not permitted
                # in IDTO's 1.30 signed-distance query versus a Halfspace.
                # All original body inertial elements are kept intact.
                body.remove(geom)
                removed += 1
    if sorted(replaced) != sorted(LEG_CODES):
        raise RuntimeError(f"Expected exactly six tibia collisions, got {replaced}")
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)
    return {"radius_m": radius_m, "tibia_contact_spheres": sorted(replaced),
            "removed_visual_and_auxiliary_geometries": removed,
            "private_proxy_xml_sha256": hashlib.sha256(xml_path.read_bytes()).hexdigest()}


def build_proxy(original_mjcf: Path, already_converted_drake_dir: Path, output_dir: Path,
                radius_m: float = 0.004) -> dict:
    if output_dir.resolve() == already_converted_drake_dir.resolve():
        raise ValueError("Output must be a new private path, not the original CAD")
    import mujoco
    from hexlab.stand_geometry_v04 import load_legs
    m = mujoco.MjModel.from_xml_path(str(original_mjcf))
    d = mujoco.MjData(m)
    tips = {leg["suffix"]: leg["tip_local"].tolist()
            for leg in load_legs(m, d, mujoco)}
    shutil.copytree(already_converted_drake_dir, output_dir, dirs_exist_ok=True)
    result = rewrite_collisions(output_dir / "mjcf" / "base_local30_nominal.xml", tips, radius_m)
    result["source_mjcf_sha256"] = hashlib.sha256(original_mjcf.read_bytes()).hexdigest()
    result["method"] = "Farthest tibia CAD mesh vertex from knee, in local body frame"
    result["contact_model_parity"] = "NOT_VERIFIED; simplified sphere-plane only"
    return result


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--original-mjcf", type=Path, required=True)
    ap.add_argument("--drake-converted-dir", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--radius-mm", type=float, default=4)
    args = ap.parse_args()
    print(json.dumps(build_proxy(args.original_mjcf, args.drake_converted_dir,
                                 args.output_dir, args.radius_mm / 1000), indent=2))


if __name__ == "__main__":
    main()