#!/usr/bin/env bash
# Reproducible CPU-only simulation of XGO-Rider2. No real robot, flashing or Actions.
set -Eeuo pipefail
UPSTREAM_URL="https://github.com/LuwuDynamics/xgorider2_rl.git"
UPSTREAM_SHA="f534986353ce3fe451caf0da72ba06b980a59114"
MODE=baseline
if [[ "$#" -gt 0 ]]; then MODE="$1"; fi

usage() {
  cat <<'USAGE'
XGO-Rider2 Mac M4 simulation (no robot).
Usage: bash xgorider2_mac.sh [baseline|train-smoke|view|all|dry-run]
  baseline    Verify exported policy vs native C++ + simulate stand, pushes, drive.
  train-smoke Baseline plus independent 32,768-step PPO training.
  view        Baseline then interactive MuJoCo viewer with random pushes.
  all         Baseline, PPO smoke training and interactive viewer.
  dry-run     Print the plan; no network or installation.
Set RIDER2_WORKDIR to override $HOME/alfred-robotics/xgorider2.
USAGE
}
case "$MODE" in
  --help|-h) usage; exit 0 ;;
  baseline|train-smoke|view|all|dry-run) ;;
  *) usage >&2; exit 2 ;;
esac
if [[ "$MODE" == "dry-run" ]]; then
  echo "NO EXECUTION: $UPSTREAM_URL @ $UPSTREAM_SHA"
  echo "baseline = native-C++ verification + 4 seeds each: stand, stand_push, drive"
  echo "Optional PPO training and GUI. No robot firmware or CAD30 edits."
  exit 0
fi
for tool in git clang++ uname; do
  command -v "$tool" >/dev/null 2>&1 || { echo "Missing: $tool" >&2; exit 2; }
done
if command -v python3.11 >/dev/null 2>&1; then
  PYTHON="$(command -v python3.11)"
elif command -v python3 >/dev/null 2>&1 && [[ "$(python3 -c 'import sys; print(str(sys.version_info.major) + "." + str(sys.version_info.minor))')" == "3.11" ]]; then
  PYTHON="$(command -v python3)"
else
  echo "Python 3.11 is required. On macOS: brew install python@3.11" >&2
  exit 2
fi
WORKDIR="$(printenv RIDER2_WORKDIR || true)"
if [[ -z "$WORKDIR" ]]; then WORKDIR="$HOME/alfred-robotics/xgorider2"; fi
CHECKOUT="$WORKDIR/upstream-f534986"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)-$$"
RUN_DIR="$WORKDIR/runs/$STAMP"
mkdir -p "$RUN_DIR"
on_exit() {
  rc=$?
  echo "exit_code=$rc" > "$RUN_DIR/status.txt"
  echo "XGO-Rider2 exit=$rc; experiment receipts: $RUN_DIR"
}
trap on_exit EXIT

{
  echo "upstream=$UPSTREAM_URL"
  echo "upstream_sha=$UPSTREAM_SHA"
  echo "mode=$MODE"
  echo "started_utc=$(date -u +%FT%TZ)"
  echo "host=$(uname -s)-$(uname -m)"
  echo "python=$("$PYTHON" --version 2>&1)"
  echo "clang=$(clang++ --version | head -n 1)"
} > "$RUN_DIR/provenance.txt"

if [[ ! -e "$CHECKOUT" ]]; then
  echo "Cloning official upstream to $CHECKOUT"
  git clone --depth 1 "$UPSTREAM_URL" "$CHECKOUT"
  if [[ "$(git -C "$CHECKOUT" rev-parse HEAD)" != "$UPSTREAM_SHA" ]]; then
    git -C "$CHECKOUT" fetch --depth 1 origin "$UPSTREAM_SHA"
    git -C "$CHECKOUT" checkout --detach "$UPSTREAM_SHA"
  fi
else
  [[ -d "$CHECKOUT/.git" ]] || { echo "Refusing non-Git directory: $CHECKOUT" >&2; exit 2; }
  [[ "$(git -C "$CHECKOUT" remote get-url origin)" == "$UPSTREAM_URL" ]] || { echo "Unexpected origin" >&2; exit 2; }
fi
ACTUAL_SHA="$(git -C "$CHECKOUT" rev-parse HEAD)"
[[ "$ACTUAL_SHA" == "$UPSTREAM_SHA" ]] || { echo "Pinned SHA mismatch: $ACTUAL_SHA" >&2; exit 2; }
echo "verified_upstream_sha=$ACTUAL_SHA" >> "$RUN_DIR/provenance.txt"

cd "$CHECKOUT/training"
if [[ ! -x .venv/bin/python ]]; then "$PYTHON" -m venv .venv; fi
echo "Installing pinned Python dependencies into isolated .venv"
.venv/bin/python -m pip install -r requirements.txt 2>&1 | tee "$RUN_DIR/dependencies.log"
.venv/bin/python -m pip freeze > "$RUN_DIR/pip-freeze.txt"

# Verifier writes its own JSON. Work on a COPY, not on the supplied weights.
cp -R artifacts/balance-v6 "$RUN_DIR/reference-policy"
echo "Verifying copied policy against native C++ controller and physics"
.venv/bin/python verify_export2.py "$RUN_DIR/reference-policy" 2>&1 | tee "$RUN_DIR/verify.log"

echo "Evaluating 4 fixed seeds in stand/stand_push/drive"
sh run.sh evaluate --scenarios stand stand_push drive --seeds 4 --seed-start 51000 --seconds 15 --workers 2 --output "$RUN_DIR/baseline-evaluate.json" 2>&1 | tee "$RUN_DIR/evaluate.log"

if [[ "$MODE" == "train-smoke" || "$MODE" == "all" ]]; then
  echo "PPO smoke test: 32,768 steps; not the 4M-step full training"
  .venv/bin/python train2.py --output "$RUN_DIR/train-smoke" --steps 32768 --warmup-steps 4096 --envs 2 --stand-fraction 0.5 --n-steps 512 --batch-size 1024 --eval-every 16384 --eval-seeds 1 --eval-workers 2 --corner-fraction 0.5 --seed 7171 2>&1 | tee "$RUN_DIR/train-smoke.log"
fi
if [[ "$MODE" == "view" || "$MODE" == "all" ]]; then
  echo "Opening MuJoCo GUI with pushes; close it to finish."
  sh run.sh view --pushes
fi
echo "Simulation baseline completed: $RUN_DIR"
echo "This does not establish adaptive walking on 4 or 6 legs."
