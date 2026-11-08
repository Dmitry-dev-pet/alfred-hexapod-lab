#!/usr/bin/env python3
"""Acquire and audit *original* Mini Wheelbot CAD without committing upstream binaries.

Strict provenance: every download comes from a fixed upstream commit and must
match its Git blob SHA-1 and byte size. This is a source/geometry gate only:
it does NOT perform MuJoCo physics, claim stable self-righting or publish replay.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import tempfile
import urllib.request

UPSTREAM = "wheelbot/Mini-Wheelbot"
COMMIT = "73815e49371c5fb5f49eec59d364e4d9d1b57b10"
RAW = f"https://raw.githubusercontent.com/{UPSTREAM}/{COMMIT}/"
ASSETS = {
    "hardware/mechanical/meshes/mini-wheelbot.stl":
        (14879784, "06c9ae3d5480a5d88257c544006097433fbe2c35"),
    "hardware/mechanical/meshes/body.stl":
        (12754984, "07fd4ee34fae40d137703db689ea6456b71527b0"),
    "hardware/mechanical/meshes/actuator_rotor.stl":
        (1035784, "c85170451d005a12e82d955a2883aaaf54e6b71f"),
    "simulation/python/examples/mini_wheelbot_sysid_parameters.json":
        (317, "ff2516b3963764a2738b9f108a27dbd019c5a293"),
}
PARAM_NAMES = (
    "m_WR", "m_B", "I_Wxz_Ryz", "I_Wy_Rx", "I_Bx", "I_By", "I_Bz",
    "r_W", "l_WB", "fric_magn", "fric_slope",
)


def git_blob_sha1(payload: bytes) -> str:
    header = f"blob {len(payload)}\0".encode("ascii")
    return hashlib.sha1(header + payload).hexdigest()


def verify_asset(path: str, payload: bytes) -> dict:
    if path not in ASSETS:
        raise ValueError("Unapproved upstream file: " + path)
    length, git_sha = ASSETS[path]
    if len(payload) != length or git_blob_sha1(payload) != git_sha:
        raise ValueError("Pinned upstream source mismatch: " + path)
    return {
        "path": path, "bytes": length, "git_blob_sha1": git_sha,
        "sha256": hashlib.sha256(payload).hexdigest(),
        "upstream_commit": COMMIT, "url": RAW + path,
    }


def stl_bounds(data: bytes) -> dict:
    """Bounds in raw STL units; do not silently assume mm or meters."""
    if len(data) < 84:
        raise ValueError("STL too small")
    count = struct.unpack_from("<I", data, 80)[0]
    if count < 1 or len(data) != 84 + 50 * count:
        raise ValueError("Expected an exact binary STL triangle stream")
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for (offset,) in ((84 + 50 * i,) for i in range(count)):
        xyz = struct.unpack_from("<9f", data, offset + 12)
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError("STL contains non-finite vertices")
        for j in range(3):
            lo[j] = min(lo[j], xyz[j], xyz[j + 3], xyz[j + 6])
            hi[j] = max(hi[j], xyz[j], xyz[j + 3], xyz[j + 6])
    spans = [hi[j] - lo[j] for j in range(3)]
    if not any(v > 0 for v in spans):
        raise ValueError("Degenerate STL")
    return {
        "triangles": count, "min_raw": lo, "max_raw": hi, "extent_raw": spans,
        "units": "not established (inspect CAD/STEP before scale)",
    }


def audit_parameters(data: bytes) -> dict:
    params = json.loads(data)
    if set(params) != set(PARAM_NAMES):
        raise ValueError("Unexpected parameter schema")
    if not all(isinstance(params[k], (int, float)) and
               math.isfinite(params[k]) for k in PARAM_NAMES):
        raise ValueError("Non-finite original model parameter")
    if any(params[k] <= 0 for k in (
            "m_WR", "m_B", "I_Wxz_Ryz", "I_Wy_Rx", "I_Bx",
            "I_By", "I_Bz", "r_W", "l_WB")):
        raise ValueError("Non-positive mass/inertia/geometry")
    params["computed_total_mass_kg"] = params["m_B"] + 2 * params["m_WR"]
    return params


def download(path: str, expected_bytes: int) -> bytes:
    req = urllib.request.Request(
        RAW + path, headers={"User-Agent": "Alfred-Hexapod-CAD-Audit/1"})
    with urllib.request.urlopen(req, timeout=45) as response:
        data = response.read(expected_bytes + 1)
        if len(data) > expected_bytes:
            raise ValueError("Oversized source: " + path)
        return data


def acquire(cache: Path) -> dict:
    cache.mkdir(parents=True, exist_ok=True)
    reports = []
    raw_params = None
    for path, (size, _) in ASSETS.items():
        target = cache / path
        payload = target.read_bytes() if target.is_file() else download(path, size)
        item = verify_asset(path, payload)
        if target.suffix == ".stl":
            item["mesh"] = stl_bounds(payload)
        else:
            raw_params = audit_parameters(payload)
        if not target.is_file():
            target.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as f:
                temp = Path(f.name)
                try:
                    f.write(payload)
                except BaseException:
                    temp.unlink(missing_ok=True)
                    raise
            temp.replace(target)
        reports.append(item)
    output = {
        "status": "SOURCE_AUDIT_ONLY",
        "upstream": UPSTREAM,
        "upstream_commit": COMMIT,
        "mesh_visual_available": True,
        "physical_contacts_verified": False,
        "self_righting_verified": False,
        "params": raw_params,
        "source_files": reports,
        "notes": [
            "CAD position/orientation must be aligned to the official dynamics frames",
            "Use identified inertia/mass separately from STL volume",
            "Motor torque-speed envelope and ground contact need independent calibration",
            "Do not reuse the v1 self-righting trajectory for v2 physics"
        ],
    }
    (cache / "source-audit.json").write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path,
                        default=Path.home() / "mini-wheelbot-original-cad")
    args = parser.parse_args()
    output = acquire(args.cache.expanduser())
    print("SOURCE_AUDIT_ONLY: original CAD/parameters verified")
    print("Commit:", output["upstream_commit"])
    print("Total mass, kg:", output["params"]["computed_total_mass_kg"])
    for item in output["source_files"]:
        print(item["path"], item["sha256"][:12],
              item.get("mesh", {}).get("extent_raw", ""))
    print("Report:", args.cache.expanduser() / "source-audit.json")
    print("PHYSICS NOT YET VERIFIED — do not register as a self-righting replay")


if __name__ == "__main__":
    main()
