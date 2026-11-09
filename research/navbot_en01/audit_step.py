#!/usr/bin/env python3
"""Inspect *original* NavBot EN01 STEP without redistributing CAD or inventing joints.

Uses CadQuery/OpenCascade; produces reproducible per-solid volumes, boxes and
centres plus a SHA256 receipt. An assembly of unlabeled solids is NOT an MJCF.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path

UPSTREAM_COMMIT = "3139f2fb32b38d10611feb905e16f8aa012f2f2e"
MAX_SIZE = 90_000_000


def inspect_step(path: Path, expected_sha256: str | None = None) -> dict:
    if not path.is_file() or path.is_symlink():
        raise ValueError("Expected a regular local STEP file")
    if not 1_000 < path.stat().st_size < MAX_SIZE:
        raise ValueError("Unexpected STEP size")
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    if expected_sha256 and sha.lower() != expected_sha256.lower():
        raise ValueError("Original STEP SHA256 mismatch")
    try:
        import cadquery as cq
    except ImportError as exc:
        raise RuntimeError("Install CadQuery in the local CAD audit environment") from exc
    model = cq.importers.importStep(str(path))
    solids = list(model.solids().vals())
    if not solids:
        raise ValueError("No solid objects were imported")
    parts = []
    for index, s in enumerate(solids):
        bb = s.BoundingBox()
        c = s.Center()
        vals = [float(bb.xmin), float(bb.xmax), float(bb.ymin), float(bb.ymax),
                float(bb.zmin), float(bb.zmax), float(s.Volume()),
                float(c.x), float(c.y), float(c.z)]
        if not all(math.isfinite(v) for v in vals) or vals[6] <= 0:
            raise ValueError(f"Invalid imported solid geometry: {index}")
        parts.append({
            "index": index,
            "bbox_importer_units": [round(v, 7) for v in vals[:6]],
            "volume_importer_units_cubed": round(vals[6], 7),
            "center_importer_units": [round(v, 7) for v in vals[7:]],
        })
    bounds = [min(p["bbox_importer_units"][i] for p in parts)
              if i % 2 == 0 else max(p["bbox_importer_units"][i] for p in parts)
              for i in range(6)]
    return {
        "schema": "navbot-en01-step-audit-v1",
        "upstream_commit": UPSTREAM_COMMIT,
        "upstream_path": "Hardware/RebotModel/OriginalRobotModel.stp",
        "step_sha256": sha,
        "file_bytes": path.stat().st_size,
        "solid_count": len(parts),
        "overall_bbox_importer_units": bounds,
        "parts": parts,
        "verification_status": "geometric-audit-only_NOT-joint-axes-masses-or-dynamics",
        "limitations": [
            "STEP geometry may represent fixed poses and need segmentation into articulated parts.",
            "Importer units must be checked against the STEP units and physical dimensions.",
            "Volume is NOT mass; no density, joint axes, inertia, servo limits or wiring verified.",
            "CAD file stays outside the repository; do not redistribute without a licence.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=Path, required=True)
    parser.add_argument("--expected-sha256")
    parser.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args()
    receipt = inspect_step(args.step, args.expected_sha256)
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: receipt[k] for k in (
        "step_sha256", "file_bytes", "solid_count", "overall_bbox_importer_units", "verification_status")}, indent=2))


if __name__ == "__main__":
    main()
