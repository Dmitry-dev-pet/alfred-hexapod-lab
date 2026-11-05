"""Convert *private local* CAD STL files to local OBJ for Drake 1.30 MJCF parsing.

Neither the CAD files nor the converted OBJ meshes may be committed to GitHub.
This script preserves mesh scale/coordinates; it only rewrites copied MJCF files.
"""
from pathlib import Path
import argparse
import shutil
import trimesh


def convert(source: Path, destination: Path) -> dict:
    meshes = source / "meshes"
    mjcf = source / "mjcf"
    if not meshes.is_dir() or not mjcf.is_dir():
        raise ValueError("Expected source/{meshes,mjcf}/ directories")
    (destination / "meshes").mkdir(parents=True, exist_ok=True)
    (destination / "mjcf").mkdir(parents=True, exist_ok=True)
    count = 0
    for source_stl in sorted(meshes.glob("*.stl")):
        mesh = trimesh.load_mesh(source_stl, force="mesh", process=False)
        if isinstance(mesh, trimesh.Scene):
            mesh = mesh.dump(concatenate=True)
        (destination / "meshes" / (source_stl.stem + ".obj")).write_text(
            trimesh.exchange.obj.export_obj(mesh, include_normals=False,
                                            include_color=False, include_texture=False),
            encoding="utf-8")
        count += 1
    for source_xml in sorted(mjcf.glob("*.xml")):
        text = source_xml.read_text(encoding="utf-8")
        if ".stl" not in text:
            shutil.copy2(source_xml, destination / "mjcf" / source_xml.name)
        else:
            (destination / "mjcf" / source_xml.name).write_text(
                text.replace(".stl", ".obj"), encoding="utf-8")
    return {"converted_private_mesh_count": count, "new_xml_count": len(list((destination / 'mjcf').glob('*.xml')))}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", type=Path, required=True, help="original private source/ directory")
    ap.add_argument("--destination", type=Path, required=True, help="private scratch directory")
    args = ap.parse_args()
    print(convert(args.source, args.destination))
