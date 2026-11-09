"""Source-bound planar EN01 four-bar geometry; not calibrated hardware dynamics."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

PINNED_BLOB = "9156683a0bb694c84dc541a3e26b5f6b08db1a41"
PINNED_BYTES = 12358579
UPSTREAM = "3139f2fb32b38d10611feb905e16f8aa012f2f2e"


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def sub(a, b):
    return [x-y for x, y in zip(a, b)]


def rotate(v, angle):
    c, s = math.cos(angle), math.sin(angle)
    return [c*v[0]-s*v[1], s*v[0]+c*v[1]]


def cross(a, b):
    return a[0]*b[1]-a[1]*b[0]


def angle(v):
    return math.atan2(v[1], v[0])


def wrap(v):
    return math.atan2(math.sin(v), math.cos(v))


def solve(points, delta):
    """Rotate input AB in CAD XY, close circles B/BC and D/DC, preserve branch.

    All input coordinates in mm; positive delta is counterclockwise in CAD XY.
    A MuJoCo +Y hinge in mapped (CAD X, -CAD Z, CAD Y) has delta=-q.
    The real servo encoder zero and direction are deliberately not asserted.
    """
    if not math.isfinite(delta):
        raise ValueError("Nonfinite input angle")
    A, B0, C0, D, W0 = (points[k] for k in "ABCDW")
    if any(len(v) != 2 or not all(math.isfinite(x) for x in v)
           for v in (A, B0, C0, D, W0)):
        raise ValueError("Invalid source points")
    B = add(A, rotate(sub(B0, A), delta))
    r, R = math.dist(B0, C0), math.dist(D, C0)
    dv = sub(D, B)
    d = math.hypot(*dv)
    if d < 1e-9 or d >= r+R-1e-8 or d <= abs(r-R)+1e-8:
        raise ValueError("Unreachable or singular four-bar configuration")
    x = (r*r-R*R+d*d)/(2*d)
    h = math.sqrt(max(0, r*r-x*x))
    e = [v/d for v in dv]
    branch = cross(sub(D, B0), sub(C0, B0))
    if abs(branch) < 1e-8:
        raise ValueError("Singular source assembly branch")
    sign = math.copysign(1, branch)
    C = add(add(B, [x*v for v in e]), [-sign*h*e[1], sign*h*e[0]])
    calf_delta = wrap(angle(sub(C, B))-angle(sub(C0, B0)))
    rod_delta = wrap(angle(sub(C, D))-angle(sub(C0, D)))
    W = add(B, rotate(sub(W0, B0), calf_delta))
    residual = max(abs(math.dist(A, B)-math.dist(A, B0)),
                   abs(math.dist(B, C)-r), abs(math.dist(D, C)-R),
                   abs(math.dist(B, W)-math.dist(B0, W0)))
    return {"A": A, "B": B, "C": C, "D": D, "W": W,
            "input_delta_rad": delta, "calf_delta_rad": calf_delta,
            "rod_delta_rad": rod_delta, "closure_residual_mm": residual}


def audit(step: Path):
    """Extract actual cylindrical axes, corroborated by mating parts on both sides."""
    raw = step.read_bytes()
    blob = hashlib.sha1(b"blob "+str(len(raw)).encode()+b"\0"+raw).hexdigest()
    if len(raw) != PINNED_BYTES or blob != PINNED_BLOB:
        raise ValueError("Pinned original STEP mismatch")
    from .cad_parts import read_parts
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    parts = read_parts(step)
    if len(parts) != 73:
        raise ValueError("Unexpected assembly occurrence count")
    rows = {}
    for p in parts:
        cylinders = []
        for fi, face in enumerate(p["shape"].Faces()):
            s = BRepAdaptor_Surface(face.wrapped)
            if s.GetType() != GeomAbs_Cylinder:
                continue
            c = s.Cylinder()
            v, a = c.Location(), c.Axis().Direction()
            if abs(a.Z()) > 1-1e-8:
                cylinders.append({"face_index": fi, "radius_mm": c.Radius(),
                                  "xy_mm": [v.X(), v.Y()], "axis": [a.X(), a.Y(), a.Z()]})
        rows[p["id"]] = {"name": p["name"], "reference_name": p["reference_name"],
                         "bounds_mm": p["bounds"], "cylinders": cylinders}

    def bore(pid, ref, radius, near=None):
        row = rows[pid]
        if row["reference_name"] != ref:
            raise ValueError("Part identity mismatch")
        candidates = [c for c in row["cylinders"] if abs(c["radius_mm"]-radius) < 1e-7]
        centers = []
        for c in candidates:
            if near is not None and math.dist(c["xy_mm"], near) > 1e-5:
                continue
            if not any(math.dist(c["xy_mm"], x) < 1e-6 for x in centers):
                centers.append(c["xy_mm"])
        if len(centers) != 1:
            raise ValueError(f"Ambiguous/missing bore in {pid}")
        return centers[0], {"part_id": pid, "name": row["name"],
                            "reference_name": ref, "radius_mm": radius,
                            "xy_mm": centers[0],
                            "face_indices": [c["face_index"] for c in candidates
                                             if math.dist(c["xy_mm"], centers[0]) < 1e-6]}

    sides = {}
    specs = [("cad_z_positive", [22, 31, 37, 36, 65, 24, 62, 43, 28],
              ["舵盘", "bearing4-7-2.5", "轴承3-6-2", "轴承3-6-2", "2208电机轮胎", "大腿", "小腿_2208电机版本", "侧板_MIR", "小腿副拉杆"]),
             ("cad_z_negative", [23, 30, 39, 38, 64, 26, 63, 44, 29],
              ["舵盘_MIR1", "bearing4-7-2.5", "轴承3-6-2", "轴承3-6-2", "2208电机轮胎", "大腿_MIR1", "小腿_2208电机版本_MIR", "侧板", "小腿副拉杆"])]
    for side, ids, refs in specs:
        points, evidence = {}, []
        for key, idx, r in zip("ABCDW", range(5), [2.2, 2, 1.5, 1.5, 18.5]):
            pt, e = bore(ids[idx], refs[idx], r)
            points[key] = pt
            evidence.append(e)
        for idx, r, key in [(5, 1.2, "A"), (5, 3.505, "B"),
                            (6, 2, "B"), (6, 1.505, "C"),
                            (6, 19.35, "W"), (7, .95, "D"),
                            (8, 2.96, "C"), (8, 2.96, "D")]:
            _, e = bore(ids[idx], refs[idx], r, points[key])
            evidence.append(e)
        w = rows[ids[4]]["bounds_mm"]
        sides[side] = {"points_xy_mm": points, "axis_evidence": evidence,
                       "tire_midplane_cad_z_mm": (w[2]+w[5])/2,
                       "tire_width_mm": w[5]-w[2],
                       "lengths_mm": {a+b: math.dist(points[a], points[b])
                                      for a, b in [("A","B"),("B","C"),("D","C"),("A","D"),("B","W")]}}
    error = max(math.dist(sides["cad_z_positive"]["points_xy_mm"][k],
                          sides["cad_z_negative"]["points_xy_mm"][k]) for k in "ABCDW")
    if error > 1e-5:
        raise ValueError("Mirrored source axes disagree")
    return {"schema": "en01-fourbar-source-evidence/v1", "source": {
        "repo": "fuwei007/Navbot-EN01", "commit": UPSTREAM,
        "path": "Hardware/RebotModel/OriginalRobotModel.stp",
        "git_blob_sha1": blob, "sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw)},
        "part_count": len(parts), "sides": sides, "mirror_xy_error_mm": error,
        "status": "SOURCE_GEOMETRY_ONLY_SERVO_ZERO_AND_DYNAMICS_UNVALIDATED",
        "omissions": ["torsion spring stiffness/preload", "servo encoder zero and direction",
                      "material densities and actual mass/COM/inertia", "wheel motor electrical parameters",
                      "effective loaded tire rolling radius", "link collision and assembly interference"]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--step", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    result = audit(a.step)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({"part_count": result["part_count"],
                      "lengths_mm": result["sides"]["cad_z_positive"]["lengths_mm"],
                      "mirror_error_mm": result["mirror_xy_error_mm"]}, indent=2))


if __name__ == "__main__":
    main()
