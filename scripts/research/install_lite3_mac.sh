#!/usr/bin/env bash
# Mac M4 one-command local install and 60-run S2 CPU benchmark; no Actions physics.
set -Eeuo pipefail
if [[ "$(uname -s)" != "Darwin" || "$(uname -m)" != "arm64" ]]; then
  echo 'Requires a native Apple Silicon Mac (no Rosetta)' >&2
  exit 2
fi
PYTHON="$(command -v python3.13 || true)"
if [[ -z "$PYTHON" ]] && command -v brew >/dev/null 2>&1; then
  PREFIX="$(brew --prefix python@3.13 2>/dev/null || true)"
  if [[ -n "$PREFIX" && -x "$PREFIX/bin/python3.13" ]]; then
    PYTHON="$PREFIX/bin/python3.13"
  else
    echo 'Installing Python 3.13 using existing Homebrew (no sudo)...'
    brew install python@3.13
    PREFIX="$(brew --prefix python@3.13)"
    PYTHON="$PREFIX/bin/python3.13"
  fi
fi
if [[ -z "$PYTHON" || ! -x "$PYTHON" ]]; then
  echo 'Python 3.13 unavailable; install python@3.13 or put python3.13 on PATH.' >&2
  exit 2
fi
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
exec "$PYTHON" "$ROOT/scripts/research/install_lite3_mac.py" "$@"
