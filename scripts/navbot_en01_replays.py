#!/usr/bin/env python3
"""One command: EN01 recorded CPU MuJoCo -> existing Android 3D replay, NO MP4.

Optional --archive restores the four exact Nov 8 source traces by *content hash*
from the owner's evidence ZIP, without trusting ZIP entry names or extracting
any unrelated file. Existing differing trace files are never overwritten.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from tempfile import NamedTemporaryFile
from zipfile import ZipFile

from replay_pipeline import ROOT, load_spec, run

MODES = ("stand", "crouch", "jump", "jump-fast")
EXPECTED_SHA256 = {
    "stand": "a5d0cad75d4c4cf1e78c245ebfeda4dc23e9431addcec0741ecf8a8196cdf273",
    "crouch": "ac2f053b44353f0f15066fa9908f54afb6a5f3e73f432c1f550cfd82b5ab43cc",
    "jump": "608cfa5778bf96d4578b36ec65408e64e54dd07d72366ff5faf8b058137174d3",
    "jump-fast": "98920e20e4506a2f644288cc4c4083c7b264d38207700bf42fca2d571220b62c",
}
MAX_MEMBER_BYTES = 8_000_000
MAX_ARCHIVE_ITEMS = 500


def restore_verified_archive(archive: Path, root: Path = ROOT,
                             expected: dict[str, str] = EXPECTED_SHA256) -> list[dict]:
    """Restore only the four verified raw JSON byte streams. Never unzip paths."""
    if not archive.is_file():
        raise ValueError(f"Archive not found: {archive}")
    by_sha = {sha: mode for mode, sha in expected.items()}
    if len(by_sha) != len(expected):
        raise ValueError("Duplicate expected digests")
    matched: dict[str, bytes] = {}
    with ZipFile(archive) as zip_file:
        entries = zip_file.infolist()
        if len(entries) > MAX_ARCHIVE_ITEMS:
            raise ValueError("Archive has too many entries")
        for member in entries:
            if member.is_dir() or not member.filename.lower().endswith(".json"):
                continue
            # Never extract ZIP paths. Bound size BEFORE decompressing.
            if member.file_size < 1 or member.file_size > MAX_MEMBER_BYTES:
                raise ValueError("Archive has an oversized/empty JSON member")
            raw = zip_file.read(member)
            if len(raw) != member.file_size:
                raise ValueError("Incomplete ZIP member")
            mode = by_sha.get(hashlib.sha256(raw).hexdigest())
            if mode is None:
                continue  # Manifests and unrelated JSON cannot become a source trace.
            if mode in matched:
                raise ValueError(f"Duplicate verified trace: {mode}")
            parsed = json.loads(raw)
            if not isinstance(parsed, dict) or parsed.get("mode") != mode.replace("-", "_"):
                raise ValueError(f"Invalid mode in verified trace: {mode}")
            matched[mode] = raw
    missing = set(expected) - set(matched)
    if missing:
        raise ValueError("Archive lacks verified EN01 traces: " + ", ".join(sorted(missing)))

    target_dir = root / "results" / "navbot_en01"
    destinations = {mode: target_dir / (mode.replace("-", "_") + "_verified.json")
                    for mode in expected}
    for mode, target in destinations.items():
        if target.exists() and target.read_bytes() != matched[mode]:
            raise ValueError(f"Existing trace differs, refusing overwrite: {target}")
    target_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for mode in MODES:
        target = destinations[mode]
        if not target.exists():
            with NamedTemporaryFile(mode="wb", dir=target_dir,
                                    prefix=".en01-trace-", delete=False) as tmp:
                tmp.write(matched[mode])
                temp = Path(tmp.name)
            try:
                os.replace(temp, target)
            finally:
                temp.unlink(missing_ok=True)
        results.append({"mode": mode, "path": str(target.relative_to(root)),
                        "sha256": expected[mode]})
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-execution", action="store_true",
                        help="Use raw physical traces already present under results/navbot_en01")
    parser.add_argument("--archive", type=Path,
                        help="Read EXACT four verified Nov 8 raw traces from ZIP; no physics rerun")
    args = parser.parse_args()
    restored = restore_verified_archive(args.archive) if args.archive else []
    receipts = []
    for mode in MODES:
        job = load_spec(ROOT / "experiments/replay-pipeline" /
                        ("navbot-en01-" + mode + "-20261108.json"))
        receipts.append(run(ROOT, job,
                            skip_execution=bool(args.archive) or args.skip_execution,
                            do_publish=False))
    print(json.dumps({"created": len(receipts), "published": False, "mp4_created": False,
                      "source": "verified archived MuJoCo" if args.archive else
                                "existing traces" if args.skip_execution else "new CPU mj_step",
                      "restored": restored,
                      "entries": [{"id": r["id"], "added": r["added"], "revision": r["revision"],
                                   "trace_sha256": r["trace_sha256"]} for r in receipts]},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
