#!/usr/bin/env python3
"""One-command local Mac M4 installer + 60-run Lite3 CPU benchmark.

No robot, GitHub Actions physics, or repository writes. Codex only checks out
the existing draft PR and invokes this installer. Private assets are verified
and kept in a separate, persistent owner workspace.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import venv
import zipfile

REPO = Path(__file__).resolve().parents[2]
BUNDLE_SHA256 = "8ff5d787fb27e36622d25012f8cecc071b9a21643fbdc8f5c590dafdad5e4a25"
SCENE_SHA256 = "f054f2ef66eb53af08120040e41e3aded4826a00f794669f556430086c0e52d7"
ONNX_SHA256 = "773a1bbd0fee45ec38604d979d5fd6fb339ae15d13bfb5d6e2056a171d2a3422"
UPSTREAM_SHA256 = "6cec11464c065e1f6eba9fdb921ceeffefe8b8cf08f99f1189e8661c88d2b84b"
RUN_ID = "42243825347"
ARTIFACT_NAME = "xgo-lite3-source-policy-42243825347"
ARTIFACT_REPO = "Dmytry-lab/runner-smoke"
CHATGPT_WALL_SECONDS = 30.961651129
SCENE = Path("upstream/src/assets/robots/lite3/xmls/scene_lite3.xml")
ONNX = Path("lite3-hf-policy/lite3_walk/policy.onnx")
ASSET_MEMBERS = (
    "lite3-hf-policy/lite3_walk/policy.onnx",
    "lite3-hf-policy/lite3_walk/model_1499.pt",
    "lite3-hf-policy/lite3_walk/params/agent.yaml",
    "lite3-hf-policy/lite3_walk/params/env.yaml",
    "lite3-upstream-model-task.tar.gz",
)


def sha256_path(path: Path) -> str:
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def valid_assets(root: Path) -> bool:
    return (
        (root / SCENE).is_file()
        and (root / ONNX).is_file()
        and len(list((root / "upstream/src/assets/robots/lite3/xmls/assets").glob("*.STL"))) == 17
        and sha256_path(root / SCENE) == SCENE_SHA256
        and sha256_path(root / ONNX) == ONNX_SHA256
    )


def checked_member_path(root: Path, member: str) -> Path:
    p = PurePosixPath(member)
    if p.is_absolute() or not p.parts or ".." in p.parts or "\\" in member:
        raise ValueError(f"Unsafe archive path: {member!r}")
    return root.joinpath(*p.parts)


def bundle_source(bundle: Path):
    """Read a pinned owner ZIP or an already extracted Actions artifact directory."""
    if bundle.is_file():
        if sha256_path(bundle) != BUNDLE_SHA256:
            raise ValueError(f"Owner bundle SHA256 does not match: {bundle}")
        z = zipfile.ZipFile(bundle)
        return z, z.read
    if bundle.is_dir():
        return None, lambda name: checked_member_path(bundle, name).read_bytes()
    raise FileNotFoundError(f"Cannot find owner bundle: {bundle}")


def install_assets(bundle: Path, target: Path) -> None:
    if valid_assets(target):
        print(f"ASSETS_ALREADY_VERIFIED {target}", flush=True)
        return
    if target.exists():
        raise RuntimeError(f"Refusing to overwrite existing invalid assets: {target}")
    z, read = bundle_source(bundle)
    try:
        receipt = json.loads(read("RECEIPT.json"))
        expected = receipt.get("files", {})
        for name in ASSET_MEMBERS:
            data = read(name)
            item = expected.get(name, {})
            if hashlib.sha256(data).hexdigest() != item.get("sha256") or len(data) != item.get("size"):
                raise ValueError(f"Asset manifest mismatch: {name}")
            if name.endswith(".tar.gz") and hashlib.sha256(data).hexdigest() != UPSTREAM_SHA256:
                raise ValueError("Wrong pinned upstream archive")
        target.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="lite3-assets-", dir=target.parent) as td:
            staged = Path(td) / "original_assets"
            staged.mkdir()
            for name in ASSET_MEMBERS:
                if name.endswith(".tar.gz"):
                    continue
                dest = checked_member_path(staged, name)
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(read(name))
            raw_tar = read("lite3-upstream-model-task.tar.gz")
            with tarfile.open(fileobj=io.BytesIO(raw_tar), mode="r:gz") as tf:
                for item in tf:
                    if item.isdir():
                        continue
                    if not item.isfile() or item.name.startswith("/") or ".." in PurePosixPath(item.name).parts:
                        raise ValueError(f"Unexpected tar member: {item.name!r}")
                    if any(part.startswith("._") for part in PurePosixPath(item.name).parts):
                        continue  # macOS AppleDouble metadata, not original robot geometry
                    if not (item.name.startswith("src/assets/robots/lite3/")
                            or item.name.startswith("src/tasks/velocity/config/lite3_walk/")):
                        raise ValueError(f"Unexpected upstream path: {item.name!r}")
                    dest = checked_member_path(staged / "upstream", item.name)
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    with tf.extractfile(item) as src, dest.open("wb") as out:
                        shutil.copyfileobj(src, out)
            if not valid_assets(staged):
                raise RuntimeError("Unpacked source scene/policy failed SHA256 verification")
            staged.rename(target)
    finally:
        if z is not None:
            z.close()
    print(f"ASSETS_INSTALLED {target}", flush=True)


def choose_bundle(requested: Path | None, workdir: Path, offline: bool) -> Path:
    if requested is not None:
        return requested.expanduser().resolve()
    env = os.environ.get("LITE3_BUNDLE_ZIP")
    if env:
        return Path(env).expanduser().resolve()
    for candidate in (
        workdir / "XGO-Lite3-original-model-and-PPO-policy-2026-11-06.zip",
        Path.home() / "Downloads/XGO-Lite3-original-model-and-PPO-policy-2026-11-06.zip",
        workdir / "downloads/lite3-m4-artifact",
    ):
        if candidate.is_file() or (candidate.is_dir() and (candidate / "RECEIPT.json").is_file()):
            return candidate
    if offline or shutil.which("gh") is None:
        raise RuntimeError(
            "Private assets unavailable. Set LITE3_BUNDLE_ZIP to the owner Drive ZIP. "
            "No credentials are stored in this PR."
        )
    download_dir = workdir / "downloads/lite3-m4-artifact"
    download_dir.mkdir(parents=True, exist_ok=True)
    print("Retrieving existing private artifact (data only; no physics Actions)...", flush=True)
    subprocess.run([
        "gh", "run", "download", RUN_ID, "--repo", ARTIFACT_REPO,
        "--name", ARTIFACT_NAME, "--dir", str(download_dir)
    ], check=True)
    return download_dir


def require_mac() -> None:
    if platform.system() != "Darwin" or platform.machine() != "arm64":
        raise RuntimeError("Benchmark installer requires native Apple Silicon macOS")
    if sys.version_info[:2] != (3, 13):
        raise RuntimeError("Use Python 3.13 for the comparable pinned environment")


def ensure_environment(workdir: Path) -> Path:
    requirements = REPO / "runtime/lite3-mac-requirements.txt"
    if not requirements.is_file():
        raise FileNotFoundError(f"Missing pinned requirements: {requirements}")
    stamp = hashlib.sha256(requirements.read_bytes()).hexdigest()
    venv_dir = workdir / "venv-py313"
    python = venv_dir / "bin/python"
    if not python.is_file():
        print(f"Creating persistent, isolated environment: {venv_dir}", flush=True)
        venv.EnvBuilder(with_pip=True).create(venv_dir)
    marker = venv_dir / ".lite3-requirements-sha256"
    if not marker.is_file() or marker.read_text().strip() != stamp:
        subprocess.run([
            str(python), "-m", "pip", "install", "--disable-pip-version-check",
            "--only-binary=:all:", "-r", str(requirements)
        ], check=True)
        marker.write_text(stamp + "\n")
    subprocess.run([str(python), "-m", "pip", "check"], check=True)
    smoke = (
        "import mujoco,onnxruntime,numpy,torch,scipy; "
        "assert mujoco.__version__=='3.14.0'; "
        "assert onnxruntime.__version__=='1.23.2'; "
        "assert numpy.__version__=='2.3.5'; "
        "assert scipy.__version__=='1.17.0'; "
        "assert torch.__version__.split('+')[0]=='2.10.0'; "
        "assert 'CPUExecutionProvider' in onnxruntime.get_available_providers(); "
        "print('RUNTIME_OK',mujoco.__version__,onnxruntime.__version__)"
    )
    subprocess.run([str(python), "-c", smoke], check=True)
    return python


def record_host(workdir: Path, python: Path) -> None:
    def command(*args: str) -> str:
        p = subprocess.run(args, capture_output=True, text=True)
        return p.stdout.strip() if p.returncode == 0 else "UNAVAILABLE"
    facts = {
        "host": {
            "macos": command("sw_vers", "-productVersion"),
            "cpu_brand": command("sysctl", "-n", "machdep.cpu.brand_string"),
            "physical_cpu": command("sysctl", "-n", "hw.physicalcpu"),
            "logical_cpu": command("sysctl", "-n", "hw.logicalcpu"),
            "memory_bytes": command("sysctl", "-n", "hw.memsize"),
            "machine": platform.machine(),
        },
        "python": str(python),
        "git_head": command("git", "-C", str(REPO), "rev-parse", "HEAD"),
        "pip_freeze": command(str(python), "-m", "pip", "freeze").splitlines(),
    }
    results = workdir / "results"
    results.mkdir(exist_ok=True, parents=True)
    (results / "mac_environment.json").write_text(json.dumps(facts, indent=2) + "\n")


def run_benchmark(workdir: Path, python: Path) -> None:
    results = workdir / "results"
    results.mkdir(exist_ok=True, parents=True)
    output = results / "mac_m4_s2_perf.json"
    log = results / "mac_m4_s2_perf.log"
    env = os.environ.copy()
    env.update({
        "LITE3_RESEARCH_ROOT": str(workdir),
        "OPENBLAS_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "VECLIB_MAXIMUM_THREADS": "1",
    })
    cmd = [
        str(python), str(REPO / "research/xgo_lite3/lite3_perf_benchmark.py"),
        "--seeds", "10", "--output", str(output),
    ]
    print(f"Starting 60 fixed S2 episodes on CPU; logs at {log}", flush=True)
    start = time.perf_counter()
    with log.open("w") as stream:
        p = subprocess.run(
            cmd, cwd=REPO / "research/xgo_lite3", env=env,
            stdout=stream, stderr=subprocess.STDOUT
        )
    if p.returncode:
        raise RuntimeError(f"Benchmark failed (exit {p.returncode}); see {log}")
    data = json.loads(output.read_text())
    aggregate = data["aggregate"]
    rows = data["runs"]
    if len(rows) != 60 or aggregate["physics_steps"] != 142500:
        raise RuntimeError("Invalid run count / physics-step count; not a benchmark success")
    baseline = next(
        r for r in rows
        if r["variation"] == "original_baseline" and r["seed"] == 20261106
    )
    comparison = {
        "status": "PASS_COUNTS; CHECK_CROSS_PLATFORM_NUMERICS",
        "mac_wall_seconds": aggregate["wall_seconds"],
        "chatgpt_wall_seconds_reference": CHATGPT_WALL_SECONDS,
        "mac_speedup_vs_chatgpt": CHATGPT_WALL_SECONDS / aggregate["wall_seconds"],
        "mac_physics_steps_per_sec": aggregate["physics_steps_per_wall_second"],
        "baseline_saturation_mac": baseline["torque_saturation_ticks"],
        "baseline_saturation_chatgpt_reference": 659,
        "numeric_difference_expected_across_platforms": True,
        "elapsed_including_subprocess_overhead": time.perf_counter() - start,
        "benchmark_json": str(output),
        "benchmark_log": str(log),
    }
    (results / "comparison_receipt.json").write_text(json.dumps(comparison, indent=2) + "\n")
    print("BENCHMARK_DONE " + json.dumps(comparison, sort_keys=True), flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Install pinned Lite3 S2 on Apple M4; Codex only executes the installer"
    )
    ap.add_argument("--bundle", type=Path, help="Owner Drive ZIP or extracted private GitHub artifact")
    ap.add_argument("--workdir", type=Path, default=Path.home() / "alfred-robotics/lite3-benchmark")
    ap.add_argument("--offline", action="store_true", help="Do not attempt private artifact download")
    ap.add_argument("--install-only", action="store_true", help="Install but skip 60-run benchmark")
    ap.add_argument("--assets-only", action="store_true", help="Check/extract assets only (portable validation)")
    args = ap.parse_args()
    if not args.assets_only:
        require_mac()
    workdir = args.workdir.expanduser().resolve()
    workdir.mkdir(parents=True, exist_ok=True)
    bundle = None if valid_assets(workdir / "original_assets") else choose_bundle(args.bundle, workdir, args.offline)
    if bundle is not None:
        install_assets(bundle, workdir / "original_assets")
    else:
        print("ASSETS_ALREADY_VERIFIED", flush=True)
    if args.assets_only:
        print("ASSETS_ONLY_PASS")
        return
    python = ensure_environment(workdir)
    record_host(workdir, python)
    env = os.environ.copy()
    env["LITE3_RESEARCH_ROOT"] = str(workdir)
    subprocess.run([
        str(python), "-c",
        "import lite3_onnx_joint_audit as a; a.get_model(); a.make_policy(); print('MODEL_AND_POLICY_OK')",
    ], cwd=REPO / "research/xgo_lite3", env=env, check=True)
    if args.install_only:
        print("INSTALL_ONLY_PASS", flush=True)
    else:
        run_benchmark(workdir, python)


if __name__ == "__main__":
    main()
