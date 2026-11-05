"""Create an isolated Drake-only finite-box ground from an owner-supplied CAD30 MJCF.

Drake 1.30 IDTO's signed distance query cannot handle the CAD30 Convex/Box
versus Halfspace pair from a MuJoCo plane floor. An axis-aligned box keeps the
top face at the same height, avoiding edits to the six CAD tibia collision meshes.

The original MuJoCo model is never written. The copied mesh files remain private.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def build_drake_ground_model(
    source_root: Path,
    output_root: Path,
    scene: str = "scene_local30_nominal.xml",
    ground_half_extent_xy_m: float = 2.0,
    ground_half_thickness_m: float = 0.05,
) -> dict:
    """Copy a *previously STL->OBJ-converted* private model and change only the floor.

    Returns a report with hashes. Never overwrites an existing destination.
    This transformation is intentionally restricted to an axis-aligned floor.
    """
    source_root, output_root = source_root.resolve(), output_root.resolve()
    if source_root == output_root or source_root in output_root.parents:
        raise ValueError("Output must be outside the source model directory")
    if output_root.exists():
        raise FileExistsError(f"Refusing to overwrite existing model: {output_root}")
    source_scene = source_root / "mjcf" / scene
    if not source_scene.is_file():
        raise FileNotFoundError(source_scene)
    if ground_half_extent_xy_m <= 0 or ground_half_thickness_m <= 0:
        raise ValueError("Box dimensions must be strictly positive")

    # Validate first, so bad input does not create a partial output tree.
    tree = ET.parse(source_scene)
    floors = tree.findall("./worldbody/geom[@name='floor']")
    if len(floors) != 1 or floors[0].get("type") != "plane":
        raise ValueError("Exactly one worldbody plane named 'floor' is required")
    floor = floors[0]
    if any(k in floor.attrib for k in ("quat", "euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("Oriented or tilted planes are not supported")
    old_pos = [float(v) for v in floor.get("pos", "0 0 0").split()]
    if len(old_pos) != 3:
        raise ValueError("Invalid floor position")

    shutil.copytree(source_root, output_root)
    output_scene = output_root / "mjcf" / scene
    copied = ET.parse(output_scene)
    out_floor = copied.find("./worldbody/geom[@name='floor']")
    assert out_floor is not None and out_floor.get("type") == "plane"
    out_floor.set("type", "box")
    out_floor.set("size", f"{ground_half_extent_xy_m:.12g} {ground_half_extent_xy_m:.12g} {ground_half_thickness_m:.12g}")
    out_floor.set("pos", f"{old_pos[0]:.12g} {old_pos[1]:.12g} {old_pos[2]-ground_half_thickness_m:.12g}")
    # Keep original friction, contact flags, and material if supplied. Only
    # the source plane's finite extent and its underside are approximated.
    copied.write(output_scene, encoding="utf-8", xml_declaration=True)

    # Prove that no other model inputs were modified in the copy.
    changed = []
    for f in sorted(source_root.rglob("*")):
        if f.is_file():
            rel = f.relative_to(source_root)
            counterpart = output_root / rel
            if not counterpart.is_file():
                raise RuntimeError(f"Missing copied file {rel}")
            if sha256(f) != sha256(counterpart):
                changed.append(rel.as_posix())
    if changed != [f"mjcf/{scene}"]:
        raise RuntimeError(f"Unexpected changes: {changed}")

    report = {
        "schema": "cad30-drake-only-box-floor-v1",
        "original_scene_sha256": sha256(source_scene),
        "drake_scene_sha256": sha256(output_scene),
        "changed_model_files": changed,
        "floor_original_z_m": old_pos[2],
        "floor_box_top_z_m": old_pos[2],
        "floor_box_half_extents_m": [ground_half_extent_xy_m, ground_half_extent_xy_m, ground_half_thickness_m],
        "caveats": [
            "This is a Drake-only contact geometry substitute, not a physical equivalence proof.",
            "No original CAD STL or converted private OBJ should be committed to a public repository.",
            "Drake model imports motor limits and rotor armature differently from MuJoCo 3.14.",
        ],
    }
    return report


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source-root", type=Path, required=True, help="private converted CAD30 root containing mjcf/ and meshes/")
    ap.add_argument("--output-root", type=Path, required=True, help="new private Drake-only model copy")
    ap.add_argument("--scene", default="scene_local30_nominal.xml")
    ap.add_argument("--ground-half-xy-m", type=float, default=2.0)
    ap.add_argument("--ground-half-thickness-m", type=float, default=0.05)
    ap.add_argument("--receipt", type=Path, help="optional local receipt path")
    args = ap.parse_args()
    report = build_drake_ground_model(
        args.source_root, args.output_root, args.scene,
        args.ground_half_xy_m, args.ground_half_thickness_m,
    )
    value = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.receipt:
        args.receipt.parent.mkdir(parents=True, exist_ok=True)
        args.receipt.write_text(value)
    print(value, end="")


if __name__ == "__main__":
    main()
