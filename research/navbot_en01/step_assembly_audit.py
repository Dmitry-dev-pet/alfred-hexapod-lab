#!/usr/bin/env python3
"""Fail-closed, read-only STEP assembly placement audit for pinned EN01.

Reads STEP entity references and placement frames, not joint constraints. This
cannot infer physical hinge axes, wheel axle, mass, inertia or a CAD-accurate MJCF.
Original third-party CAD stays outside GitHub and is never redistributed.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import re

SOURCE_COMMIT = "3139f2fb32b38d10611feb905e16f8aa012f2f2e"
SOURCE_BLOB = "9156683a0bb694c84dc541a3e26b5f6b08db1a41"
SOURCE_PATH = "Hardware/RebotModel/OriginalRobotModel.stp"
ENTITY = re.compile(r"#(\d+)\s*=\s*([\s\S]*?);", re.ASCII)
REFERENCE = re.compile(r"#(\d+)")
TRANSFORM = re.compile(r"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\(#(\d+)\)")
MAX_BYTES = 90_000_000


def refs(s: str) -> list[int]:
    return [int(x) for x in REFERENCE.findall(s)]


def name(s: str) -> str:
    return re.sub(r"\\X2\\([0-9a-fA-F]+)\\X0\\",
                  lambda m: bytes.fromhex(m.group(1)).decode("utf-16-be"), s)


def vector(entities: dict[int, str], id_: int, kind: str) -> list[float]:
    s = entities.get(id_, "")
    match = re.fullmatch(rf"{kind}\('(?:[^']*)',\(([^()]*)\)\)", s)
    if not match:
        raise ValueError(f"#{id_} is not a complete {kind}")
    parts = match.group(1).split(",")
    if len(parts) != 3:
        raise ValueError(f"#{id_} is not a 3-vector")
    v = [float(x) for x in parts]
    if not all(math.isfinite(x) for x in v):
        raise ValueError(f"#{id_} has nonfinite coordinates")
    return v


def frame(entities: dict[int, str], id_: int) -> dict:
    s = entities.get(id_, "")
    if not s.startswith("AXIS2_PLACEMENT_3D(") or len(refs(s)) != 3:
        raise ValueError(f"#{id_} lacks explicit STEP placement axes")
    point, z, x = refs(s)
    v = {"origin_mm": vector(entities, point, "CARTESIAN_POINT"),
         "z_direction": vector(entities, z, "DIRECTION"),
         "x_direction": vector(entities, x, "DIRECTION")}
    norm = lambda a: math.sqrt(sum(t*t for t in a))
    if abs(norm(v["x_direction"]) - 1) > 0.0001 or abs(norm(v["z_direction"]) - 1) > 0.0001:
        raise ValueError(f"#{id_} is not normalized")
    if abs(sum(a*b for a, b in zip(v["x_direction"], v["z_direction"]))) > 0.0001:
        raise ValueError(f"#{id_} has nonorthogonal axes")
    return v


def analyze_bytes(raw: bytes, expected_blob: str | None = SOURCE_BLOB) -> dict:
    if not 100 < len(raw) < MAX_BYTES:
        raise ValueError("STEP source size outside accepted bounds")
    blob = hashlib.sha1(b"blob " + str(len(raw)).encode() + b"\0" + raw).hexdigest()
    if expected_blob and blob != expected_blob.lower():
        raise ValueError("Upstream STEP blob SHA1 mismatch")
    entities = {int(m[1]): " ".join(m[2].split()) for m in ENTITY.finditer(raw.decode("latin1"))}
    if len(entities) < 10:
        raise ValueError("No STEP entity graph")
    shape_for_usage: dict[int, int] = {}
    rel_for_shape: dict[int, int] = {}
    for id_, s in entities.items():
        if s.startswith("PRODUCT_DEFINITION_SHAPE("):
            rr = refs(s)
            if rr:
                shape_for_usage[rr[-1]] = id_
        elif s.startswith("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION("):
            rr = refs(s)
            if len(rr) != 2 or rr[1] in rel_for_shape:
                raise ValueError("Duplicate/malformed context-dependent shape relationship")
            rel_for_shape[rr[1]] = rr[0]

    placements = []
    for id_, s in sorted(entities.items()):
        if not s.startswith("NEXT_ASSEMBLY_USAGE_OCCURRENCE("):
            continue
        match = re.match(r"NEXT_ASSEMBLY_USAGE_OCCURRENCE\('([^']*)'", s)
        rr = refs(s)
        if match is None or len(rr) != 2:
            raise ValueError(f"Malformed assembly usage #{id_}")
        shape_id = shape_for_usage.get(id_)
        relation = entities.get(rel_for_shape.get(shape_id, -1), "")
        transform = TRANSFORM.search(relation)
        if transform is None:
            raise ValueError(f"No placement relation for usage #{id_}")
        item = entities.get(int(transform.group(1)), "")
        pair = refs(item)
        if not item.startswith("ITEM_DEFINED_TRANSFORMATION(") or len(pair) != 2:
            raise ValueError(f"No complete item-defined transform for #{id_}")
        placements.append({"usage_id": id_, "instance": name(match.group(1)),
                           "parent_definition": rr[0], "child_definition": rr[1],
                           "frame_from": frame(entities, pair[0]),
                           "frame_to": frame(entities, pair[1])})
    if expected_blob and len(placements) != 76:
        raise ValueError("Unexpected original EN01 assembly-usage count")
    if not placements:
        raise ValueError("No traceable placements")
    pairs = {}
    for key, labels in {
        "wheel_motor_assembly": ("QUANUM GIMBAL MOTOR 2208 ASM:1", "QUANUM GIMBAL MOTOR 2208 ASM:2"),
        "hip_servo_models": ("STS3032:1", "STS3032:2"),
        "wheel_hub_models": ("Hub:1", "Hub:2"),
    }.items():
        matching = [next((p for p in placements if p["instance"] == label), None) for label in labels]
        if any(p is None for p in matching):
            if expected_blob:
                raise ValueError(f"Missing source evidence for {key}")
            continue
        a, b = (p["frame_to"]["origin_mm"] for p in matching)
        delta = [b[i]-a[i] for i in range(3)]
        pairs[key] = {"instance_ids": [p["usage_id"] for p in matching],
                      "source_origins_mm": [a, b],
                      "offset_mm": delta,
                      "origin_distance_mm": math.sqrt(sum(d*d for d in delta)),
                      "classification": "STEP assembly placement only, NOT a measured joint or wheel axle"}
    return {
        "schema": "navbot-en01-step-assembly-audit-v1",
        "source": {"commit": SOURCE_COMMIT, "path": SOURCE_PATH, "blob_sha1": blob,
                   "sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw)},
        "counts": {"entities": len(entities), "assembly_placements": len(placements)},
        "pairs": pairs,
        "placements": placements,
        "status": "SOURCE-CAD-FRAMES-ONLY_NOT-KINEMATIC-OR-PHYSICS-VALIDATED",
        "limits": [
            "Frame coordinates are from the default CAD assembly pose, not verified hinge axes.",
            "The main servo part local origins may be far from their real output shafts.",
            "Do not infer mass, inertia, encoder zero, torque, or reliable wheel radius.",
            "This is not a segmented robot model. Full OpenCascade geometric checks remain necessary.",
        ],
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--step", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    if not args.step.is_file() or args.step.is_symlink():
        ap.error("Original STEP must be a regular local file")
    result = analyze_bytes(args.step.read_bytes())
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"source": result["source"], "counts": result["counts"],
                      "pairs": result["pairs"]}, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
