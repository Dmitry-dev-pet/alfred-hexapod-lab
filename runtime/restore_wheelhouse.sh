#!/usr/bin/env bash
# After copying wheelhouse archive from Google Drive into a clean ChatGPT execution session:
set -euo pipefail
archive="${1:-dist/mujoco-3.14.0-py313-linux-x64-offline.tar.gz}"
cd "$(dirname "$0")/.."
mkdir -p runtime
# Never unpack untrusted archives; verify SHA-256 against pinned manifest first.
if [ -f "${archive}.sha256" ]; then sha256sum -c "${archive}.sha256"; fi
tar -xzf "$archive" -C runtime
python scripts/make_wheelhouse_manifest.py --check runtime/wheelhouse runtime/wheelhouse-manifest.json
python -m pip install --no-index --find-links runtime/wheelhouse -r runtime/requirements.txt
python -m hexlab.mujoco_experiment --mode stand --seconds 2