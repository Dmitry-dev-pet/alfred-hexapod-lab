#!/bin/bash
# Existing Mac mini M4: no new runner, no GitHub Actions, no real hardware.
set -euo pipefail
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PYTHON="${PUPPER_PYTHON:-python3.13}"
VENV="${PUPPER_VENV:-$HOME/alfred-robotics/.venvs/pupper-sb3-py313}"
if [[ "$(uname -s)" != "Darwin" || "$(uname -m)" != "arm64" ]]; then
  echo 'PUPPER_ABORT: this launcher is for macOS Apple Silicon, not for Linux/Intel' >&2
  exit 2
fi
if [[ ! -x "$VENV/bin/python" ]]; then
  command -v "$PYTHON" >/dev/null || { echo "PUPPER_ABORT: Python 3.13 unavailable: $PYTHON" >&2; exit 2; }
  "$PYTHON" -m venv "$VENV"
  "$VENV/bin/python" -m pip install --disable-pip-version-check -r requirements-mac-py313.txt
fi
"$VENV/bin/python" - <<'PY'
import platform,mujoco,numpy,torch,gymnasium,stable_baselines3
assert platform.machine() == 'arm64'
assert mujoco.__version__=='3.14.0'
assert numpy.__version__=='2.3.5'
assert torch.__version__.startswith('2.10.0')
assert gymnasium.__version__=='1.2.2'
assert stable_baselines3.__version__=='2.9.2'
print('PUPPER_MAC_ENV',platform.machine(),platform.platform())
print('DEPENDENCIES',mujoco.__version__,numpy.__version__,torch.__version__,gymnasium.__version__,stable_baselines3.__version__)
print('MPS_AVAILABLE',torch.backends.mps.is_available())
PY
"$VENV/bin/python" -m unittest discover -s . -p 'test_*.py' -v
OUT="${PUPPER_RUN_OUTPUT:-$HOME/alfred-robotics/pupper-training/smoke-cpu-20261107}"
DEVICE="${PUPPER_DEVICE:-cpu}"
"$VENV/bin/python" train_sb3.py --mode smoke --device "$DEVICE" --num-envs 4 --steps 4096 --out "$OUT"
"$VENV/bin/python" evaluate_sb3.py --checkpoint "$OUT/policy_final.zip" --out "$OUT/evaluation.json"
printf 'PUPPER_MAC_SMOKE_COMPLETE: %s\n' "$OUT"
