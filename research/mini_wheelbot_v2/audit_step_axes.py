#!/usr/bin/env python3
"""Source-pinned STEP audit of the Mini Wheelbot's two orthogonal axle seats.

Reports *CAD part coordinates*. It does not assume which axle is the driving
wheel, or that either CAD part alone defines the assembly origin.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import urllib.request

SOURCE_COMMIT = "73815e49371c5fb5f49eec59d364e4d9d1b57b10"
URL = ("https://raw.githubusercontent.com/wheelbot/Mini-Wheelbot/"
       + SOURCE_COMMIT + "/hardware/mechanical/step/")
SOURCE = {
    "frame.stp": (346868, "6a8371c66ebde62d2d95aba8ce37269230b6dba8"),
    "reactionwheel.stp": (19270, "873db173b745cdc4e7698f67c97e0b2981008208"),
    "lid.stp": (820237, "a95f656342186e31b2cabb731f57e59d32e6338c"),
}
ENTITY = re.compile(r"#(\d+)\s*=\s*([A-Z0-9_]+)\((.*?)\);", re.S)
FLOAT = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[Ee][-+]?\d+)?")
REF = re.compile(r"#(\d+)")


def blob_sha(data):
    return hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()


def get_source(name, folder):
    expected_len, expected_blob = SOURCE[name]
    path = folder / name
    if path.exists():
        data = path.read_bytes()
    else:
        req = urllib.request.Request(URL + name, headers={
            "User-Agent": "Alfred-Wheelbot-STEP-Audit/1"})
        with urllib.request.urlopen(req, timeout=45) as result:
            data = result.read(expected_len + 1)
    if len(data) != expected_len or blob_sha(data) != expected_blob:
        raise RuntimeError("Original STEP integrity failure: " + name)
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    return data


def parse_cylinders(text):
    entities = {int(i): (kind, value) for i, kind, value in ENTITY.findall(text)}
    def refs(entity):
        return [int(i) for i in REF.findall(entity)]
    def point(i):
        kind, value = entities[i]
        if kind not in ("CARTESIAN_POINT", "DIRECTION"):
            raise ValueError("Unexpected STEP entity " + kind)
        values = [float(v) for v in FLOAT.findall(value.split("(", 1)[-1])]
        if len(values) != 3:
            raise ValueError("Unexpected STEP vector")
        return tuple(round(v, 6) for v in values)
    result = []
    for key, (kind, value) in entities.items():
        if kind != "CYLINDRICAL_SURFACE":
            continue
        placement = refs(value)[0]
        position, orientation, *_ = refs(entities[placement][1])
        radius = float(value.rsplit(",", 1)[1])
        result.append({
            "radius_mm": round(radius, 5),
            "axis_point_mm": point(position),
            "axis_direction": point(orientation)
        })
    return result


def axial_seats(cylinders):
    # Cylindrical mating seats of the two orthogonal motors,
    # not an automated claim of wheel identity / drive-axis assignment.
    return [c for c in cylinders if abs(c["radius_mm"] - 15) < .001]


def audit(folder):
    out = {"source_commit": SOURCE_COMMIT, "classification": "ORIGINAL_STEP_AXES_ONLY",
           "files": {}, "motor_seats": []}
    for filename in SOURCE:
        data = get_source(filename, folder)
        cylinders = parse_cylinders(data.decode("latin1"))
        out["files"][filename] = {
            "sha256": hashlib.sha256(data).hexdigest(),
            "git_blob_sha1": SOURCE[filename][1], "bytes": len(data),
            "cylinder_count": len(cylinders),
        }
        if filename == "frame.stp":
            out["motor_seats"] = axial_seats(cylinders)
        if filename == "reactionwheel.stp":
            out["reactionwheel_cylinder_radii_mm"] = sorted(
                {c["radius_mm"] for c in cylinders})
            out["reactionwheel_axes"] = [
                c for c in cylinders if c["radius_mm"] >= 30]
    seats = out["motor_seats"]
    if len(seats) != 2:
        raise RuntimeError("Expected exactly two orthogonal 15-mm motor seats")
    a, b = seats
    for p in (a, b):
        if p["axis_direction"] not in ((1., 0., 0.), (-1., 0., 0.),
                                       (0., 0., 1.), (0., 0., -1.)):
            raise RuntimeError("Unexpected motor seat orientation")
    if sorted(round(c["axis_point_mm"][1], 3) for c in seats) != [-32.5, 32.5]:
        raise RuntimeError("Expected source motor center levels ±32.5mm")
    out["axle_center_y_separation_mm"] = 65.0
    out["interpretation"] = (
        "Frame has mutually perpendicular motor seats at CAD Y=±32.5 mm. "
        "Wheel identity, tire outer diameter, center of mass and assembly "
        "transform to MuJoCo require separate checks."
    )
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache", type=Path, default=Path.home()/"mini-wheelbot-step")
    options = parser.parse_args()
    result = audit(options.cache)
    print(json.dumps(result, indent=2))
    (options.cache/"step-axis-audit.json").write_text(
        json.dumps(result, indent=2)+"\n")


if __name__ == "__main__":
    main()
