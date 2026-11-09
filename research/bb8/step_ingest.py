#!/usr/bin/env python3
"""Extract *static* placed solids from a verified STEP assembly.

The expected workflow is SLDASM+SLDPRT -> STEP in a CAD converter -> this
script. Only mesh geometry, position and scale can be recovered here; STEP
cannot recover motor joints, joint limits, material or actuator performance.
Output defaults outside the repository; original third-party CAD is not vendored.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re

from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer

UNITS_TO_METERS = {"mm": 0.001, "cm": 0.01, "m": 1.0, "inch": 0.0254}
HEX40 = re.compile(r"[0-9a-f]{40}\Z")


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode("ascii") + b"\0" + data).hexdigest()


def load_solids(step_path: Path):
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(step_path))
    if status != IFSelect_RetDone:
        raise ValueError(f"STEP parser cannot read source: {step_path}")
    if reader.TransferRoots() < 1:
        raise ValueError("STEP has no transferred roots")
    root = reader.OneShape()
    it = TopExp_Explorer(root, TopAbs_SOLID)
    solids = []
    while it.More():
        solids.append(it.Current())
        it.Next()
    if not solids:
        raise ValueError("STEP contains no solid parts")
    return solids


def bounds_native(solid):
    box = Bnd_Box()
    BRepBndLib.Add_s(solid, box)
    coord = box.Get()
    if len(coord) != 6 or any(not math.isfinite(x) for x in coord):
        raise ValueError("Invalid solid bounds")
    mi, ma = coord[:3], coord[3:]
    if any(a <= b for a, b in zip(ma, mi)):
        raise ValueError("Zero-volume/invalid shape")
    return [float(x) for x in mi], [float(x) for x in ma]


def convert(step_path: Path, output_dir: Path, units: str, *, expected_step_sha256=None,
            linear_deflection: float = 0.5, expected_sldasm_sha1=None,
            source_sldasm_path: Path | None=None) -> dict:
    if units not in UNITS_TO_METERS:
        raise ValueError("Specify CAD length units explicitly: mm/cm/m/inch")
    if linear_deflection <= 0 or not math.isfinite(linear_deflection):
        raise ValueError("Invalid meshing deflection")
    if expected_sldasm_sha1:
        if not HEX40.fullmatch(expected_sldasm_sha1) or not source_sldasm_path:
            raise ValueError("Invalid/missing source SLDASM verification")
        source_bytes=source_sldasm_path.read_bytes()
        if git_blob_sha(source_bytes) != expected_sldasm_sha1:
            raise ValueError("Upstream SLDASM hash mismatch")
    payload = step_path.read_bytes()
    sha = hashlib.sha256(payload).hexdigest()
    if expected_step_sha256 and expected_step_sha256 != sha:
        raise ValueError("Converted STEP SHA-256 mismatch")
    solids = load_solids(step_path)
    result = {
        "schema": "alfred-bb8-step-static-solids/v1",
        "status": "STATIC_GEOMETRY_ONLY_NOT_PHYSICS",
        "source_step_sha256": sha,
        "source_step_name": step_path.name,
        "source_sldasm_git_sha1": expected_sldasm_sha1,
        "cad_units_explicit": units,
        "to_meters": UNITS_TO_METERS[units],
        "joint_axes_verified": False,
        "parts": [],
        "limitations": [
            "OpenCascade STEP solid traversal preserves already-applied CAD placements, not mates/joints",
            "Do not infer rotor axes, joint limits, mass or inertias from tessellated meshes",
            "No permission to rehost third-party CAD implied by public repository access",
            "Meshes are research exports only; not assembled MuJoCo physics",
        ],
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    writer = StlAPI_Writer()
    for idx, solid in enumerate(solids, start=1):
        mi, ma = bounds_native(solid)
        BRepMesh_IncrementalMesh(solid, linear_deflection, False, 0.4, True)
        filename = f"part-{idx:03d}.stl"
        target = output_dir / filename
        if not writer.Write(solid, str(target)):
            raise ValueError("STL conversion failed: " + str(target))
        chunk = target.read_bytes()
        if len(chunk) < 84:
            raise ValueError("Generated STL is empty: " + filename)
        result["parts"].append({
            "id": f"solid-{idx:03d}", "file": filename,
            "stl_sha256": hashlib.sha256(chunk).hexdigest(),
            "stl_size_bytes": len(chunk),
            "aabb_min_m": [round(x * UNITS_TO_METERS[units], 8) for x in mi],
            "aabb_max_m": [round(x * UNITS_TO_METERS[units], 8) for x in ma],
            "aabb_extents_m": [round((b-a) * UNITS_TO_METERS[units], 8) for a,b in zip(mi,ma)],
            "applied_placement": "STEP solid world coordinates (baked)",
        })
    result["solid_count"] = len(result["parts"])
    manifest = output_dir / "static_solids.json"
    manifest.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return result


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--step", type=Path, required=True, help="Private local STEP from native SolidWorks assembly converter")
    ap.add_argument("--out", type=Path, default=Path.home() / ".cache" / "alfred-bb8-cad" / "converted-step")
    ap.add_argument("--units", choices=list(UNITS_TO_METERS), required=True)
    ap.add_argument("--expected-step-sha256")
    ap.add_argument("--source-sldasm", type=Path)
    ap.add_argument("--expected-sldasm-git-sha1")
    ap.add_argument("--deflection", type=float, default=0.5)
    args = ap.parse_args()
    r = convert(args.step, args.out, args.units, expected_step_sha256=args.expected_step_sha256,
                expected_sldasm_sha1=args.expected_sldasm_git_sha1, source_sldasm_path=args.source_sldasm,
                linear_deflection=args.deflection)
    print(json.dumps({"sha256":r["source_step_sha256"], "solid_count":r["solid_count"],
                      "units":r["cad_units_explicit"], "output_dir":str(args.out),
                      "status":r["status"]}, indent=2))


if __name__ == "__main__":
    main()
