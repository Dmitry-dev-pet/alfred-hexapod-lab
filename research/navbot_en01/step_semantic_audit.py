#!/usr/bin/env python3
"""Evidence-only STEP entity audit for the pinned NavBot EN01 CAD.

This does NOT derive joint transforms, part masses, articulation, or MuJoCo MJCF.
No third-party CAD bytes are redistributed. Inputs must be local original files.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re

UPSTREAM_COMMIT = "3139f2fb32b38d10611feb905e16f8aa012f2f2e"
UPSTREAM_BLOB_SHA1 = "9156683a0bb694c84dc541a3e26b5f6b08db1a41"
STEP_PATH = "Hardware/RebotModel/OriginalRobotModel.stp"
MAX_BYTES = 90_000_000
ENTITY = re.compile(r"#(\d+)=([\s\S]*?);", re.ASCII)
REFERENCE = re.compile(r"#(\d+)")


def _refs(value: str | None) -> list[int]:
    return [int(n) for n in REFERENCE.findall(value or "")]


def _name(value: str) -> str:
    # ISO 10303-21 escaped UTF-16 sequences.
    def replace(m: re.Match) -> str:
        h = m.group(1)
        if len(h) % 4:
            return m.group(0)
        return bytes.fromhex(h).decode("utf-16-be", errors="replace")
    return re.sub(r"\\X2\\([0-9a-fA-F]+)\\X0\\", replace, value)


def _tuple3(entities: dict[int, str], entity_id: int, keyword: str) -> list[float] | None:
    val = entities.get(entity_id, "")
    match = re.match(rf"{keyword}\('.*?',\(([^()]+)\)\)$", val)
    if not match:
        return None
    parts = match.group(1).split(",")
    if len(parts) != 3:
        return None
    xyz = [float(x) for x in parts]
    return xyz if all(math.isfinite(x) for x in xyz) else None


def analyze_bytes(raw: bytes, expected_blob: str | None = UPSTREAM_BLOB_SHA1) -> dict:
    if not 100 < len(raw) < MAX_BYTES:
        raise ValueError("STEP input size outside allowed bounds")
    blob = hashlib.sha1(b"blob " + str(len(raw)).encode("ascii") + b"\0" + raw).hexdigest()
    if expected_blob and blob != expected_blob.lower():
        raise ValueError("STEP upstream Git blob SHA1 mismatch")
    source = raw.decode("latin1")
    entities = {int(m.group(1)): " ".join(m.group(2).split()) for m in ENTITY.finditer(source)}
    if not entities:
        raise ValueError("No STEP entities found")
    product, formation, definition, pdshape, sdr, srr, brep, solid, shell, face = ({} for _ in range(10))
    cylinder = {}
    for key, value in entities.items():
        m = re.match(r"PRODUCT\('([^']*)'", value)
        if m:
            product[key] = _name(m.group(1))
        elif value.startswith("PRODUCT_DEFINITION_FORMATION("):
            ids = _refs(value)
            if ids:
                formation[key] = ids[-1]
        elif value.startswith("PRODUCT_DEFINITION("):
            ids = _refs(value)
            if len(ids) >= 2:
                definition[key] = ids[-2]
        elif value.startswith("PRODUCT_DEFINITION_SHAPE("):
            ids = _refs(value)
            if ids:
                pdshape[key] = ids[-1]
        elif value.startswith("SHAPE_DEFINITION_REPRESENTATION("):
            ids = _refs(value)
            if len(ids) == 2:
                sdr[ids[-1]] = ids[0]
        elif value.startswith("SHAPE_REPRESENTATION_RELATIONSHIP("):
            ids = _refs(value)
            if len(ids) == 2:
                srr[ids[-1]] = ids[0]
        elif value.startswith("ADVANCED_BREP_SHAPE_REPRESENTATION("):
            ids = _refs(value)
            if ids:
                brep[key] = ids[0]
        elif value.startswith("MANIFOLD_SOLID_BREP("):
            ids = _refs(value)
            if ids:
                solid[key] = ids[-1]
        elif value.startswith("CLOSED_SHELL("):
            shell[key] = _refs(value)
        elif value.startswith("ADVANCED_FACE("):
            ids = _refs(value)
            if ids:
                face[key] = ids[-1]
        elif value.startswith("CYLINDRICAL_SURFACE("):
            m = re.match(r"CYLINDRICAL_SURFACE\('.*?',#(\d+),([+\-\d.Ee]+)\)$", value)
            if m:
                cylinder[key] = (int(m.group(1)), float(m.group(2)))

    parts = []
    for brep_id, solid_id in sorted(brep.items()):
        shape_repr = srr.get(brep_id)
        shape_def = sdr.get(shape_repr)
        product_def = pdshape.get(shape_def)
        form_id = definition.get(product_def)
        product_id = formation.get(form_id)
        name = product.get(product_id)
        face_ids = shell.get(solid.get(solid_id), [])
        cylinders = []
        for face_id in face_ids:
            sf = cylinder.get(face.get(face_id))
            if not sf:
                continue
            axis_id, radius = sf
            a = _refs(entities.get(axis_id, ""))
            xyz = _tuple3(entities, a[0], "CARTESIAN_POINT") if len(a) >= 2 else None
            direction = _tuple3(entities, a[1], "DIRECTION") if len(a) >= 2 else None
            cylinders.append({"radius_mm": radius, "axis_point_mm": xyz, "axis_direction": direction})
        parts.append({"part_name": name, "product_id": product_id, "brep_id": brep_id,
                      "solid_id": solid_id, "face_count": len(face_ids),
                      "cylinder_count": len(cylinders), "cylinders": cylinders})

    tire_parts = [p for p in parts if p["part_name"] == "2208电机轮胎"]
    tire_axes = []
    for p in tire_parts:
        for c in p["cylinders"]:
            if c["radius_mm"] >= 15:
                tire_axes.append(c)
    return {
        "schema": "navbot-en01-semantic-step-v1",
        "input": {"upstream_commit": UPSTREAM_COMMIT, "upstream_path": STEP_PATH,
                  "git_blob_sha1": blob, "sha256": hashlib.sha256(raw).hexdigest(),
                  "file_bytes": len(raw)},
        "units": {"length_mm_declared": "SI_UNIT(.MILLI.,.METRE.)" in source},
        "counts": {"entities": len(entities), "products": len(product),
                   "solids": len(solid), "cylindrical_surfaces": len(cylinder),
                   "brep_representations": len(brep), "linked_brep_parts": sum(p["part_name"] is not None for p in parts)},
        "named_products": sorted(set(product.values())),
        "wheel_tire": {"name": "2208电机轮胎", "candidate_cylinders": tire_axes,
                       "model_v0_radius_mm": 24.0,
                       "status": "candidate outer radius; not independent joint/assembly verification"},
        "selected_brep_parts": [
            {k: v for k, v in p.items() if k != "cylinders"}
            for p in parts if p["part_name"] and
            any(n in p["part_name"] for n in ("2208", "大腿", "小腿", "OUTRUNNER", "BASE"))
        ],
        "limitations": [
            "Raw cylindrical-surface radii are local part geometry, not validated final articulated assembly poses.",
            "STEP product labels do not prove the as-sold motor SKU, torque curve, voltage, dimensions or weight.",
            "No physical MuJoCo execution, COM/inertia identification or joint coordinate transform is inferred.",
            "Original CAD is not redistributed and may lack a project-wide reuse license.",
        ],
    }


def analyze(path: Path, expected_blob: str | None = UPSTREAM_BLOB_SHA1) -> dict:
    if not path.is_file() or path.is_symlink():
        raise ValueError("Original STEP must be a regular local file")
    return analyze_bytes(path.read_bytes(), expected_blob)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=Path, required=True)
    parser.add_argument("--expected-git-blob-sha1", default=UPSTREAM_BLOB_SHA1)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.step, args.expected_git_blob_sha1)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf8")
    print(json.dumps({"input": result["input"], "counts": result["counts"],
                      "wheel_tire": result["wheel_tire"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
