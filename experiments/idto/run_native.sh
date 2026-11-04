#!/usr/bin/env bash
# Native (no Docker) build/run of upstream TRI IDTO Mini Cheetah.
# Only the unmodified upstream is used. An exit code 0 is NOT a verified gait.
set -Eeuo pipefail

MODE="${1:-preflight}"
UPSTREAM="https://github.com/ToyotaResearchInstitute/idto.git"
IDTO_SHA="de0629c7811aa9b330e56c4385629005b09495f0"
WORK_DIR="${IDTO_LAB_WORKDIR:-$HOME/.cache/alfred-hexapod-idto}"
SRC="$WORK_DIR/idto"
BUILD="$SRC/build"
OUT="$WORK_DIR/runs"
DRAKE_ROOT="${DRAKE_ROOT:-$HOME/drake}"

die() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }
need() { command -v "$1" >/dev/null 2>&1 || die "Missing: $1"; }
check_source() {
  [ -d "$SRC/.git" ] || die "Run prepare first"
  [ "$(git -C "$SRC" rev-parse HEAD)" = "$IDTO_SHA" ] || die "Incorrect upstream commit"
  [ -z "$(git -C "$SRC" status --porcelain)" ] || die "Upstream source has local changes"
}
check_build_prereqs() {
  need cmake
  need git
  need c++
  [ -d "$DRAKE_ROOT" ] || die "Drake not found at $DRAKE_ROOT. Install native Drake (recommended upstream version 1.30.0) and set DRAKE_ROOT."
  printf 'Drake directory: %s\n' "$DRAKE_ROOT"
  if ! find "$DRAKE_ROOT" -maxdepth 6 \( -name 'drake-config.cmake' -o -name 'drakeConfig.cmake' \) -print -quit | grep -q .; then
    die "CMake Drake package not found beneath DRAKE_ROOT=$DRAKE_ROOT"
  fi
}
preflight() {
  need git
  need cmake
  need c++
  printf 'Native IDTO preflight. Host: '; uname -sm
  printf 'Pinned SHA: %s\n' "$IDTO_SHA"
  printf 'CMake: '; cmake --version | head -1
  printf 'Python: '; python3 --version 2>&1 || true
  printf 'Drake: %s\n' "$DRAKE_ROOT"
  check_build_prereqs
  printf 'Preflight completed. No simulation executed.\n'
}
prepare() {
  need git
  mkdir -p "$WORK_DIR" "$OUT"
  if [ ! -d "$SRC/.git" ]; then
    [ ! -e "$SRC" ] || die "Source path exists without .git: $SRC"
    git clone "$UPSTREAM" "$SRC"
  fi
  [ -z "$(git -C "$SRC" status --porcelain)" ] || die "Refusing to modify dirty upstream source tree"
  if ! git -C "$SRC" cat-file -e "$IDTO_SHA^{commit}" 2>/dev/null; then
    git -C "$SRC" fetch origin "$IDTO_SHA"
  fi
  git -C "$SRC" checkout --detach "$IDTO_SHA"
  check_source
  printf 'Pinned source ready at %s\n' "$SRC"
}
configure() {
  check_build_prereqs
  check_source
  cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$DRAKE_ROOT${CMAKE_PREFIX_PATH:+;$CMAKE_PREFIX_PATH}"
}
build() {
  configure
  cmake --build "$BUILD" --target mini_cheetah --parallel "${IDTO_BUILD_JOBS:-4}"
  printf 'Native executable: %s\n' "$BUILD/examples/mini_cheetah/mini_cheetah"
}
run() {
  need git
  check_source
  EXE="$BUILD/examples/mini_cheetah/mini_cheetah"
  [ -x "$EXE" ] || die "Executable absent. Run configure and build."
  RUN_DIR="$OUT/native-$(date -u +%Y%m%dT%H%M%SZ)"
  mkdir -p "$RUN_DIR"
  printf '%s\n' "$IDTO_SHA" > "$RUN_DIR/upstream_commit.txt"
  uname -a > "$RUN_DIR/host_uname.txt"
  cmake --version > "$RUN_DIR/cmake_version.txt"
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$SRC/examples/mini_cheetah/mini_cheetah.yaml" > "$RUN_DIR/upstream_yaml_sha256.txt"
  else
    need shasum
    shasum -a 256 "$SRC/examples/mini_cheetah/mini_cheetah.yaml" > "$RUN_DIR/upstream_yaml_sha256.txt"
  fi
  printf 'not_run\n' > "$RUN_DIR/status.txt"
  printf 'Native IDTO running. Receipts: %s\n' "$RUN_DIR"
  if (cd "$SRC" && "$EXE" 2>&1 | tee "$RUN_DIR/mini_cheetah.log"); then
    printf 'exit_0_unverified\n' > "$RUN_DIR/status.txt"
    printf 'Exited 0; trajectory, contacts and achieved distance require separate verification.\n'
  else
    printf 'run_failed\n' > "$RUN_DIR/status.txt"
    printf 'Native run failed; logs: %s\n' "$RUN_DIR" >&2
    exit 1
  fi
}
case "$MODE" in
  preflight) preflight ;;
  prepare) prepare ;;
  configure) configure ;;
  build) build ;;
  run) run ;;
  *) die "Usage: bash experiments/idto/run_native.sh {preflight|prepare|configure|build|run}" ;;
esac
