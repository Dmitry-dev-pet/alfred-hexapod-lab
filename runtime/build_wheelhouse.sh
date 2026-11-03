#!/usr/bin/env bash
# Run on NETWORKED Linux x86_64 with CPython 3.13. Will not run in this offline ChatGPT sandbox.
set -euo pipefail
cd "$(dirname "$0")/.."
python3.13 -m pip download --only-binary=:all: -r runtime/requirements.txt -d runtime/wheelhouse
python3 scripts/make_wheelhouse_manifest.py runtime/wheelhouse runtime/wheelhouse-manifest.json
mkdir -p dist
# Reproducible contents; archive timestamp is not normalized.
tar -czf dist/mujoco-3.14.0-py313-linux-x64-offline.tar.gz -C runtime wheelhouse wheelhouse-manifest.json requirements.txt
sha256sum dist/mujoco-3.14.0-py313-linux-x64-offline.tar.gz > dist/mujoco-3.14.0-py313-linux-x64-offline.tar.gz.sha256