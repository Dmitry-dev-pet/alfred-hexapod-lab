#!/usr/bin/env python3
"""Lock and inspect BB-8 upstream CAD without vendoring unlicensed assets.

This is *not* a MuJoCo model or a physical simulation. Inspection only.
Usage: python research/bb8/cad_audit.py inventory|fetch|inspect|verify
"""
from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
from urllib.parse import quote
from urllib.request import urlopen

SCHEMA = "alfred-bb8-cad-sources/v1"
ROOT = Path(__file__).resolve().parents[2]
LOCK = ROOT / "research/bb8/sources.lock.json"
DEFAULT_CACHE = Path.home() / ".cache" / "alfred-bb8-cad"
HEX40 = re.compile(r"[0-9a-f]{40}\Z")
REPO = re.compile(r"[a-zA-Z0-9_.-]+/[a-zA-Z0-9_.-]+\Z")
MAX_SINGLE_FILE = 35_000_000


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode("ascii") + b"\0" + data).hexdigest()


def validate_manifest(manifest: dict) -> None:
    if not isinstance(manifest, dict) or manifest.get("schema") != SCHEMA:
        raise ValueError("Unexpected source-lock schema")
    if not isinstance(manifest.get("sources"), list) or not manifest["sources"]:
        raise ValueError("Missing sources")
    seen = set()
    for source in manifest["sources"]:
        repo, revision = source.get("repo", ""), source.get("revision", "")
        if not REPO.fullmatch(repo) or not HEX40.fullmatch(revision):
            raise ValueError("Invalid repo or unpinned revision")
        if not isinstance(source.get("license_status"), str):
            raise ValueError("Missing license status")
        for entry in source.get("files", []):
            path = entry.get("path", "")
            if (not path or path.startswith("/") or "\\" in path or ":" in path
                    or any(x in (".", "..") for x in path.split("/"))
                    or not HEX40.fullmatch(entry.get("git_blob_sha", ""))
                    or type(entry.get("size")) is not int
                    or not 0 < entry["size"] <= MAX_SINGLE_FILE):
                raise ValueError("Invalid upstream file entry: " + repr(path))
            if (repo, path) in seen:
                raise ValueError("Repeated source path")
            seen.add((repo, path))
        if not source.get("files"):
            raise ValueError("Empty source file group")


def entries(manifest: dict, repo_filter: str = "*", path_filter: str = "*"):
    for source in manifest["sources"]:
        if not fnmatch.fnmatchcase(source["repo"], repo_filter):
            continue
        for file in source["files"]:
            if fnmatch.fnmatchcase(file["path"], path_filter):
                yield source, file


def local_path(cache: Path, source: dict, entry: dict) -> Path:
    return cache / source["repo"] / source["revision"] / entry["path"]


def verify_payload(blob: bytes, entry: dict) -> None:
    if len(blob) != entry["size"]:
        raise ValueError("Unexpected byte size: " + entry["path"])
    if git_blob_sha(blob) != entry["git_blob_sha"]:
        raise ValueError("Git blob SHA mismatch: " + entry["path"])


def download(source: dict, entry: dict) -> bytes:
    # Pin immutable commit SHA; never use a floating branch.
    url = "https://raw.githubusercontent.com/" + source["repo"] + "/" + source["revision"] + "/" + quote(entry["path"], safe="/")
    with urlopen(url, timeout=40) as response:
        if response.geturl().split("/", 3)[2].lower() != "raw.githubusercontent.com":
            raise ValueError("Unexpected remote download host")
        payload = response.read(MAX_SINGLE_FILE + 1)
    verify_payload(payload, entry)
    return payload


def stl_bounds(blob: bytes) -> dict:
    points = []
    if len(blob) >= 84:
        triangles = struct.unpack_from("<I", blob, 80)[0]
        if triangles > 0 and len(blob) == 84 + 50 * triangles:
            for i in range(triangles):
                # 12B normal followed by 3 x 12B vertices.
                coords = struct.unpack_from("<9f", blob, 84 + 50 * i + 12)
                points.extend(zip(coords[0::3], coords[1::3], coords[2::3]))
    if not points:
        try:
            content = blob.decode("ascii")
        except UnicodeDecodeError as exc:
            raise ValueError("Unknown STL encoding") from exc
        for line in content.splitlines():
            words = line.strip().split()
            if len(words) == 4 and words[0].lower() == "vertex":
                try:
                    points.append(tuple(float(s) for s in words[1:]))
                except ValueError as exc:
                    raise ValueError("Malformed ASCII STL vertex") from exc
    if not points or any(not math.isfinite(x) for v in points for x in v):
        raise ValueError("STL has no finite vertices")
    bounds = [[min(v[i] for v in points), max(v[i] for v in points)] for i in range(3)]
    return {"bounds_native_units_xyz": bounds,
            "dimensions_native_units_xyz": [round(b[1] - b[0], 6) for b in bounds],
            "vertices_seen": len(points), "units_proven": False}


def execute(manifest: dict, cache: Path, action: str, repo_filter="*", path_filter="*") -> dict:
    validate_manifest(manifest)
    result = {"action": action, "cache": str(cache), "files": [], "not_cached": 0}
    for source, entry in entries(manifest, repo_filter, path_filter):
        item = {"repo": source["repo"], "path": entry["path"], "size": entry["size"]}
        filename = local_path(cache, source, entry)
        if action == "inventory":
            item["license_status"] = source["license_status"]
        elif action == "fetch":
            if filename.is_file():
                verify_payload(filename.read_bytes(), entry)
                item["status"] = "already_verified"
            else:
                data = download(source, entry)
                filename.parent.mkdir(parents=True, exist_ok=True)
                tmp = filename.with_suffix(filename.suffix + ".partial")
                tmp.write_bytes(data)
                os.replace(tmp, filename)
                item["status"] = "downloaded_and_verified"
        else:
            if not filename.is_file():
                result["not_cached"] += 1
                item["status"] = "not_cached"
            else:
                data = filename.read_bytes()
                verify_payload(data, entry)
                item["status"] = "verified"
                if action == "inspect" and filename.suffix.lower() == ".stl":
                    item["geometry"] = stl_bounds(data)
        result["files"].append(item)
    result["file_count"] = len(result["files"])
    return result


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("action", choices=("inventory", "fetch", "verify", "inspect"))
    ap.add_argument("--manifest", type=Path, default=LOCK)
    ap.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    ap.add_argument("--repo", default="*", help="Filter e.g. HWeigo/BB8_Robot")
    ap.add_argument("--path", default="*", help="Glob: *.STL, Model/*.SLDASM etc")
    args = ap.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    print(json.dumps(execute(manifest, args.cache, args.action, args.repo, args.path),
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
