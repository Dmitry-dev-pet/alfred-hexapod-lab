#!/usr/bin/env bash
# Reproduce upstream TRI IDTO Mini Cheetah, pinned to a verified commit.
# Does NOT claim that an exit code of 0 proves a successful walking gait.
set -Eeuo pipefail

MODE=preflight
if [ "$#" -gt 0 ]; then MODE="$1"; fi
UPSTREAM=https://github.com/ToyotaResearchInstitute/idto.git
IDTO_SHA=de0629c7811aa9b330e56c4385629005b09495f0
PLATFORM=linux/amd64
WORK_DIR="$HOME/.cache/alfred-hexapod-idto"
if [ -n "${IDTO_LAB_WORKDIR:-}" ]; then WORK_DIR="$IDTO_LAB_WORKDIR"; fi
SRC="$WORK_DIR/idto"
OUT="$WORK_DIR/runs"
IMAGE=alfred-idto:de0629c7811a

die() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }
need() { command -v "$1" >/dev/null 2>&1 || die "Missing required program: $1"; }
need_docker() { need docker; docker info >/dev/null 2>&1 || die "Docker daemon unavailable"; }

preflight() {
  printf 'IDTO reference / mode: preflight\n'
  printf 'Host: '; uname -sm
  printf 'Pinned commit: %s\n' "$IDTO_SHA"
  printf 'Target platform: %s\n' "$PLATFORM"
  need git
  need_docker
  if [ "$(uname -m)" != "x86_64" ]; then
    printf 'WARNING: non-x86_64 host: amd64 emulation is unverified and may be slow.\n' >&2
  fi
  printf 'Docker daemon available; disk space:\n'
  df -h "$HOME" || true
  printf 'Preflight complete. This did not build or run IDTO.\n'
}

prepare() {
  need git
  mkdir -p "$WORK_DIR" "$OUT"
  if [ ! -d "$SRC/.git" ]; then
    [ ! -e "$SRC" ] || die "Path exists but is not a git repo: $SRC"
    git clone "$UPSTREAM" "$SRC"
  fi
  [ -z "$(git -C "$SRC" status --porcelain)" ] || die "Upstream worktree has edits; refusing to overwrite"
  if ! git -C "$SRC" cat-file -e "$IDTO_SHA^{commit}" 2>/dev/null; then
    git -C "$SRC" fetch origin "$IDTO_SHA"
  fi
  git -C "$SRC" checkout --detach "$IDTO_SHA"
  [ "$(git -C "$SRC" rev-parse HEAD)" = "$IDTO_SHA" ] || die "Upstream SHA mismatch"
  printf 'Prepared pinned upstream: %s\n' "$SRC"
}

build() {
  need_docker
  prepare
  printf 'Building official upstream Dockerfile (may require considerable disk/memory)...\n'
  if docker build --platform "$PLATFORM" -t "$IMAGE" "$SRC" > "$WORK_DIR/build.log" 2>&1; then
    printf 'Build succeeded. Log: %s\n' "$WORK_DIR/build.log"
  else
    printf 'Build failed. Log: %s\n' "$WORK_DIR/build.log" >&2
    tail -n 30 "$WORK_DIR/build.log" >&2 || true
    exit 1
  fi
}

run() {
  need_docker
  [ -d "$SRC/.git" ] || die "First run prepare and build"
  [ "$(git -C "$SRC" rev-parse HEAD)" = "$IDTO_SHA" ] || die "Pinned source revision not checked out"
  [ -z "$(git -C "$SRC" status --porcelain)" ] || die "Upstream worktree differs from baseline"
  docker image inspect "$IMAGE" >/dev/null 2>&1 || die "Docker image missing; run build"
  RUN_DIR="$OUT/$(date -u +%Y%m%dT%H%M%SZ)"
  mkdir -p "$RUN_DIR"
  printf '%s\n' "$IDTO_SHA" > "$RUN_DIR/upstream_commit.txt"
  printf '%s\n' "$PLATFORM" > "$RUN_DIR/docker_platform.txt"
  uname -a > "$RUN_DIR/host_uname.txt"
  docker version > "$RUN_DIR/docker_version.txt"
  docker image inspect "$IMAGE" --format '{{.Id}}' > "$RUN_DIR/docker_image_id.txt"
  if command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$SRC/examples/mini_cheetah/mini_cheetah.yaml" > "$RUN_DIR/upstream_yaml_sha256.txt"
  else
    need sha256sum
    sha256sum "$SRC/examples/mini_cheetah/mini_cheetah.yaml" > "$RUN_DIR/upstream_yaml_sha256.txt"
  fi
  printf 'not_run\n' > "$RUN_DIR/status.txt"
  printf 'Running original Mini Cheetah example. Meshcat: http://localhost:7000 (while active)\n'
  printf 'Receipts: %s\n' "$RUN_DIR"
  if docker run --rm --platform "$PLATFORM" -p 127.0.0.1:7000:7000 \
      --mount "type=bind,src=$RUN_DIR,dst=/artifacts" \
      "$IMAGE" bash -ic 'set -o pipefail; cd /home/idto; ./build/examples/mini_cheetah/mini_cheetah 2>&1 | tee /artifacts/mini_cheetah.log'; then
    printf 'exit_0_unverified\n' > "$RUN_DIR/status.txt"
    printf 'Exited 0. Do NOT call the physics reproduced before reviewing logged/recorded trajectories.\n'
  else
    printf 'run_failed\n' > "$RUN_DIR/status.txt"
    printf 'Simulation process failed; preserve logs at %s\n' "$RUN_DIR" >&2
    exit 1
  fi
}

case "$MODE" in
  preflight) preflight ;;
  prepare) prepare ;;
  build) build ;;
  run) run ;;
  *) die "Usage: bash experiments/idto/run_reference.sh {preflight|prepare|build|run}" ;;
esac
