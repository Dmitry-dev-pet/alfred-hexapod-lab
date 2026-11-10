#!/usr/bin/env python3
"""Safely refresh UI and local server code in an installed offline Hexapod.

Does not touch cached CAD meshes, Three.js, recorded physics data, or any other
Termux app. Refuses to run while port 8808 is in use. Keeps timestamped backup.
"""
import argparse
import hashlib
import json
import os
import shutil
import socket
import tempfile
from datetime import datetime, timezone
from pathlib import Path

UI_FILES = (
    "hexapod/index.html",
    "hexapod/app.js",
    "hexapod/claws/index.html",
    "hexapod/claws/app.js",
    "hexapod/six-to-four/index.html",
    "hexapod/six-to-four/app.js",
    "hexapod/player-shell.js",
    "hexapod/player-shell.css",
)
OLD_JS = "https://cdn.jsdelivr.net/npm/three@0.169.0/build/three.module.js"
OLD_ADDONS = "https://cdn.jsdelivr.net/npm/three@0.169.0/examples/jsm/"
NEW_JS = "/vendor/three/build/three.module.js"
NEW_ADDONS = "/vendor/three/examples/jsm/"


def upgrade(source: Path, dest: Path):
    source = source.resolve()
    dest = dest.expanduser().resolve()
    if source == dest or dest.is_relative_to(source):
        raise ValueError("Installed directory must be separate from source checkout")
    if not (dest / "OFFLINE_MANIFEST.json").is_file():
        raise ValueError("Existing offline installation not found")
    if not (dest / "vendor/three/build/three.module.js").is_file():
        raise ValueError("Local Three.js cache missing")
    manifest = json.loads((dest / "OFFLINE_MANIFEST.json").read_text())
    for path, digest in manifest.get("files_sha256", {}).items():
        if path.startswith("assets/stl/"):
            asset = dest / path
            if not asset.is_file() or hashlib.sha256(asset.read_bytes()).hexdigest() != digest:
                raise ValueError("CAD cache checksum mismatch: " + asset.name)
    with socket.socket() as sock:
        sock.settimeout(.75)
        if sock.connect_ex(("127.0.0.1", 8808)) == 0:
            raise ValueError("Stop the running viewer server before upgrading")
    files = list(UI_FILES) + ["local_viewer_server.py"]
    if not (dest / "hexapod/library.json").exists():
        files.append("hexapod/library.json")
    backup = dest / ("backup-ui-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    if backup.exists():
        raise ValueError("Backup path already exists; retry later")
    stage = Path(tempfile.mkdtemp(prefix=".hexapod-ui-new-", dir=dest))
    try:
        for rel in files:
            original = (source / "tools/local_viewer_server.py") if rel == "local_viewer_server.py" else (source / "docs" / rel)
            if not original.is_file():
                raise ValueError("Source missing: " + rel)
            payload = original.read_bytes()
            if rel.endswith(".html"):
                if payload.count(OLD_JS.encode()) != 1 or payload.count(OLD_ADDONS.encode()) != 1:
                    raise ValueError("Unexpected Three.js importmap: " + rel)
                payload = payload.replace(OLD_JS.encode(), NEW_JS.encode()).replace(OLD_ADDONS.encode(), NEW_ADDONS.encode())
            target = stage / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(payload)
        backup.mkdir()
        for rel in files:
            installed = dest / rel
            if installed.exists():
                saved = backup / rel
                saved.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(installed, saved)
        changed = []
        try:
            for rel in files:
                installed = dest / rel
                installed.parent.mkdir(parents=True, exist_ok=True)
                os.replace(stage / rel, installed)
                changed.append(rel)
        except Exception:
            for rel in reversed(changed):
                saved = backup / rel
                installed = dest / rel
                if saved.is_file():
                    shutil.copy2(saved, installed)
                else:
                    installed.unlink(missing_ok=True)
            raise
        return {"status": "UI_UPDATED", "changed": files, "backup": str(backup),
                "start": 'python "' + str(dest / "local_viewer_server.py") + '" start'}
    finally:
        shutil.rmtree(stage, ignore_errors=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path("."))
    parser.add_argument("--dest", type=Path, default=Path.home() / "hexapod-viewer-local")
    options = parser.parse_args()
    try:
        print(json.dumps(upgrade(options.source, options.dest), ensure_ascii=False, indent=2))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise SystemExit("BLOCKED: " + str(exc))
