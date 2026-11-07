#!/usr/bin/env python3
"""Fetch exactly pinned MIT-licensed Stanford Pupper v3 STL meshes.

For the original static viewer, the 13 external meshes are expected in
research/stanford_pupper/upstream/assets/ next to pupper_v3.xml.
A model's Git SHA-1 (including Git blob header) and byte length are checked.
Does not depend on MuJoCo, train a policy, or generate any replacement geometry.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import urllib.parse
import urllib.request

HERE = Path(__file__).resolve().parent
MANIFEST = HERE / "upstream" / "STL_SOURCE_MANIFEST.json"
DEST = HERE / "upstream" / "assets"


def git_blob_sha(raw: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(raw)).encode() + b"\0" + raw).hexdigest()


def download(url: str, limit: int) -> bytes:
    req = urllib.request.Request(
        url, headers={"User-Agent": "Alfred-Pupper-MIT-Asset-Mirror/1"})
    with urllib.request.urlopen(req, timeout=35) as response:
        if response.status != 200:
            raise RuntimeError(f"Unexpected upstream status for {url}")
        result = response.read(limit + 1)
    if len(result) > limit:
        raise ValueError(f"Upstream file exceeds expected size {url}")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-only", action="store_true",
                        help="Offline: check already mirrored files without downloading")
    args = parser.parse_args()
    m = json.loads(MANIFEST.read_text(encoding="utf8"))
    assert m["schema"] == "stanford-pupper-v3-original-stl/v1"
    assert m["source_repo"] == "G-Levine/pupper_v3_description"
    assert m["source_commit"] == "1a3c8590189d3a406219d155f4177afc491a7d77"
    assert m["source_path"] == "description/meshes/stl"
    assert m["source_license"] == "MIT" and len(m["files"]) == 13
    assert sum(x["size_bytes"] for x in m["files"]) == 7457492
    local_license = HERE / "upstream" / "LICENSE_STL_MIT"
    assert git_blob_sha(local_license.read_bytes()) == m["source_license_git_blob"]

    DEST.mkdir(parents=True, exist_ok=True)
    cache: dict[str, bytes] = {}
    saved = []
    base = "https://raw.githubusercontent.com/" + m["source_repo"] + "/" + m["source_commit"] + "/" + m["source_path"] + "/"
    for item in m["files"]:
        name, sha, n = item["name"], item["git_blob_sha"], item["size_bytes"]
        assert name.endswith(".stl") and Path(name).name == name
        assert len(sha) == 40 and all(c in "0123456789abcdef" for c in sha)
        p = DEST / name
        if p.is_file() and p.stat().st_size == n:
            payload = p.read_bytes()
            if git_blob_sha(payload) == sha:
                cache[sha] = payload
                saved.append({"name": name, "sha256": hashlib.sha256(payload).hexdigest(), "size": n})
                continue
        if args.check_only:
            raise RuntimeError(f"Missing or wrong original STL: {name}")
        if sha not in cache:
            cache[sha] = download(base + urllib.parse.quote(name), n)
        payload = cache[sha]
        if len(payload) != n or git_blob_sha(payload) != sha:
            raise ValueError(f"Original STL mismatch {name}; stopping without installing")
        with tempfile.NamedTemporaryFile(dir=DEST, prefix=".pupper-", delete=False) as tmp:
            tmp.write(payload)
            tmp_name = tmp.name
        os.replace(tmp_name, p)
        saved.append({"name": name, "sha256": hashlib.sha256(payload).hexdigest(), "size": n})

    report = {
        "schema": "stanford-pupper-v3-original-stl-verified/v1",
        "original_source": m["source_repo"],
        "source_commit": m["source_commit"],
        "license": "MIT",
        "count": len(saved),
        "unique_git_blobs": len(set(x["git_blob_sha"] for x in m["files"])),
        "bytes": sum(x["size"] for x in saved),
        "files": saved,
    }
    out = HERE / "upstream" / "STL_VERIFIED_SHA256.json"
    out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf8")
    print("STANFORD_PUPPER_STL_VERIFIED", report["count"], report["bytes"],
          "unique_blobs", report["unique_git_blobs"])


if __name__ == "__main__":
    main()
