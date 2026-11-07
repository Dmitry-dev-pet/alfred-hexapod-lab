#!/usr/bin/env bash
# Browser-free Colab GPU availability check; NO training or vendor assets.
# A one-time authorized Google OAuth/ADC connection may be required beforehand.
set -Eeuo pipefail
umask 077

if ! command -v colab >/dev/null 2>&1; then
  echo "ERROR: colab CLI unavailable. Install official google-colab-cli first." >&2
  exit 127
fi

gpu="${PUPPER_COLAB_GPU:-T4}"
case "$gpu" in
  T4|L4|A100|G4) ;;
  *) echo "ERROR: unsupported PUPPER_COLAB_GPU='$gpu'" >&2; exit 2 ;;
esac

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
probe_file="$project_root/research/stanford_pupper/colab/gpu_probe.py"
test -f "$probe_file" || { echo "ERROR: missing $probe_file" >&2; exit 2; }

run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$"
session="pupper-gpu-$run_id"
results_base="${PUPPER_COLAB_RESULTS_ROOT:-$HOME/alfred-robotics/pupper-colab-gpu}"
results="$results_base/$run_id"
if [[ -e "$results" ]]; then
  echo "ERROR: result directory already exists; refusing overwrite: $results" >&2
  exit 2
fi
mkdir -p "$results"
chmod 700 "$results"
attempted_allocation=0

cleanup() {
  local result=$?
  trap - EXIT
  if [[ "$attempted_allocation" == "1" ]]; then
    if colab stop -s "$session" >"$results/stop.log" 2>&1; then
      echo "COLAB_STOP_EXIT=0" | tee "$results/stop_status.txt"
    else
      echo "COLAB_STOP_EXIT=NONZERO; CHECK FOR UNRELEASED VM" | tee "$results/stop_status.txt" >&2
      if [[ $result -eq 0 ]]; then result=10; fi
    fi
    # Confirm backend state separately; allow propagation delay.
    colab sessions >"$results/sessions_after_stop.log" 2>&1 || true
  fi
  echo "Private diagnostic directory: $results" >&2
  exit "$result"
}
trap cleanup EXIT

echo "Session: $session (GPU request: $gpu); no training will occur."
colab version >"$results/colab_version.log" 2>&1
echo "Requesting GPU runtime; allocation consumes Colab compute units."
attempted_allocation=1
colab new -s "$session" --gpu "$gpu" >"$results/allocate.log" 2>&1
colab status -s "$session" >"$results/status.log" 2>&1 || true
colab exec -s "$session" -f "$probe_file" >"$results/gpu_probe.log" 2>&1

if ! grep -Fq "PUPPER_GPU_PROBE_PASS" "$results/gpu_probe.log"; then
  echo "G0=FAIL: requested GPU not physically available to PyTorch CUDA." >&2
  exit 3
fi

echo "G0=GPU_PRESENT; see stop_status.txt + sessions_after_stop.log for cleanup proof."
echo "Do NOT start PPO until the release and receipt are verified."
