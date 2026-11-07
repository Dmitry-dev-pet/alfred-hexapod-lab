"""G1: bounded ORIGINAL Stanford Pupper MJLab/RSL-RL GPU training in Colab.

This script runs on a freshly allocated Google Colab T4 VM via Colab CLI.
It clones an immutable public Stanford source SHA, resolves its original
uv.lock with CUDA PyTorch (cu128 extra), performs exactly 2 native PPO
iterations × 24 steps × 16 parallel envs, saves independently verified RSL-RL
model checkpoints, and writes /content/pupper-g1-result.zip for download.

No W&B credentials, no external model weights, no real robot or browser.
Main research code is unchanged. A separate Mac workflow controls VM cleanup.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
import zipfile

SOURCE_SHA = "aa475e9a393a209b29fd5de7e0ee10f3de643230"
REPO_URL = "https://github.com/cs123-stanford/pupper-mjlab.git"
TASK = "Mjlab-VelocityFS-Flat-Pupper-v3"
ROOT = Path("/content/stanford-original-g1")
CHECKOUT = ROOT / "source"
RESULT = ROOT / "result"
BUNDLE = Path("/content/pupper-g1-result.zip")
NUM_ENVS = 16
ITERATIONS = 2
STEPS_PER_ENV = 24
SEED = 20261107
REWARDS = {
    "track_linear_velocity": 3.0,
    "track_yaw_velocity": 1.5,
    "upright": 1.0,
    "base_height": 0.20,
    "termination": -10.0,
    "action_rate_l2": -0.05,
}
TRAIN_SCRIPT = r'''
import hashlib
import json
import os
from pathlib import Path
import time

import torch
import warp
import mujoco
import mujoco_warp
import mjlab.tasks
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
from mjlab.scripts.train import TrainConfig, launch_training

TASK = "Mjlab-VelocityFS-Flat-Pupper-v3"
OUTPUT = Path("/content/stanford-original-g1/result")
OUTPUT.mkdir(parents=True, exist_ok=True)
assert torch.cuda.is_available(), "Actual CUDA is unavailable: stop; no CPU fallback"
assert torch.cuda.device_count() > 0, "No visible GPU"
device = torch.cuda.get_device_name(0)
assert os.getenv("CUDA_VISIBLE_DEVICES") == "0", "Requires exactly one pinned GPU"

# Actual GPU tensor computation; metadata alone is insufficient.
assert (torch.tensor([3], device="cuda:0") * 7).item() == 21
env_cfg = load_env_cfg(TASK)
agent_cfg = load_rl_cfg(TASK)
env_cfg.scene.num_envs = 16
weights = {
    "track_linear_velocity": 3.0,
    "track_yaw_velocity": 1.5,
    "upright": 1.0,
    "base_height": 0.20,
    "termination": -10.0,
    "action_rate_l2": -0.05,
}
for term, weight in weights.items():
    assert term in env_cfg.rewards, f"Missing upstream reward term: {term}"
    assert env_cfg.rewards[term].weight == 0, f"Upstream baseline changed: {term}"
    env_cfg.rewards[term].weight = weight
agent_cfg.seed = 20261107
agent_cfg.logger = "tensorboard"
agent_cfg.upload_model = False
agent_cfg.max_iterations = 2
agent_cfg.num_steps_per_env = 24
agent_cfg.save_interval = 1
agent_cfg.experiment_name = "pupper_original_g1_colab_t4"
agent_cfg.run_name = "2iterations_16env"
protocol = {
    "schema": "stanford-original-mjlab-g1-cuda-v1",
    "upstream_commit": "aa475e9a393a209b29fd5de7e0ee10f3de643230",
    "task": TASK,
    "engine": "ORIGINAL_STANFORD_MJLAB_RSL_RL_NO_SB3",
    "gpu_name": device,
    "torch_version": str(torch.__version__),
    "torch_cuda_build": str(torch.version.cuda),
    "warp_version": str(warp.__version__),
    "mujoco_version": str(mujoco.__version__),
    "num_envs": 16,
    "iterations": 2,
    "num_steps_per_env": 24,
    "total_env_steps": 768,
    "seed": 20261107,
    "reward_weights": weights,
    "wandb": "disabled",
}
(OUTPUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
print("STANFORD_ORIGINAL_MJLAB_CUDA_TRAINING_START", flush=True)
t0 = time.monotonic()
launch_training(
    task_id=TASK,
    args=TrainConfig(env=env_cfg, agent=agent_cfg, gpu_ids=[0],
                     log_root=str(OUTPUT / "logs")),
)
elapsed = time.monotonic() - t0
print("STANFORD_ORIGINAL_MJLAB_CUDA_TRAINING_DONE elapsed_s", round(elapsed, 3), flush=True)

models = sorted((OUTPUT / "logs").rglob("model_*.pt"))
assert len(models) >= 2, f"Expected ≥2 native RSL-RL checkpoints; got {len(models)}"
checks = []
for model in models:
    data = model.read_bytes()
    assert len(data) > 100_000
    checks.append({
        "path": str(model.relative_to(OUTPUT)),
        "size_bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
    })
receipt = {
    "status": "NATIVE_STANFORD_RSL_RL_CUDA_CHECKPOINTS_VERIFIED",
    "training_wall_seconds": elapsed,
    "checkpoints": checks,
    "protocol": protocol,
}
(OUTPUT / "verified_receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
print("G1_NATIVE_CUDA_CHECKPOINTS_VERIFIED", len(checks), flush=True)
'''


def run_logged(args: list[str], label: str, timeout_s: int, cwd: Path | None = None) -> None:
    print("G1_STAGE", label, flush=True)
    log = RESULT / (label + ".log")
    with log.open("w", encoding="utf-8") as fh:
        proc = subprocess.run(
            args, cwd=str(cwd) if cwd else None,
            stdout=fh, stderr=subprocess.STDOUT,
            timeout=timeout_s, check=False,
            env=os.environ | {"UV_NO_PROGRESS": "1", "WANDB_MODE": "disabled"},
        )
    if proc.returncode:
        raise RuntimeError(f"{label}_EXIT_{proc.returncode}")


def main() -> int:
    if ROOT.exists() or BUNDLE.exists():
        raise RuntimeError("Refusing to overwrite an existing G1 path")
    RESULT.mkdir(parents=True, exist_ok=False)
    t0 = time.monotonic()
    status: dict[str, object] = {
        "schema": "pupper-g1-preflight-result-v1",
        "status": "IN_PROGRESS",
        "stage": "initial",
        "source_sha": SOURCE_SHA,
        "num_envs": NUM_ENVS,
        "iterations": ITERATIONS,
        "steps_per_env": STEPS_PER_ENV,
        "gpu_only": True,
        "os": platform.system(),
        "python_version": sys.version.split()[0],
    }
    exit_code = 1
    try:
        import torch

        status["torch_preexisting"] = str(torch.__version__)
        status["cuda_preexisting"] = str(torch.version.cuda)
        status["gpu_name"] = torch.cuda.get_device_name(0) if torch.cuda.is_available() else ""
        status["stage"] = "cuda_preflight"
        if not torch.cuda.is_available():
            raise RuntimeError("Colab Python has no CUDA; refusing CPU fallback")
        result = (torch.tensor([1],device="cuda:0")+torch.tensor([2],device="cuda:0")).item()
        if result != 3:
            raise RuntimeError("CUDA arithmetic failed")

        status["stage"] = "pinned_source_checkout"
        CHECKOUT.mkdir()
        run_logged(["git", "init", "-q", str(CHECKOUT)], "git_init", 30)
        run_logged(["git", "remote", "add", "origin", REPO_URL], "git_remote", 30, CHECKOUT)
        run_logged(["git", "fetch", "--depth=1", "origin", SOURCE_SHA], "git_fetch", 120, CHECKOUT)
        run_logged(["git", "checkout", "-q", "--detach", "FETCH_HEAD"], "git_checkout", 30, CHECKOUT)
        head = subprocess.check_output(["git","rev-parse","HEAD"],cwd=CHECKOUT,text=True).strip()
        if head != SOURCE_SHA or not (CHECKOUT / "uv.lock").is_file():
            raise RuntimeError("Source SHA or original uv.lock mismatch")
        status["uv_lock_sha256"] = hashlib.sha256((CHECKOUT / "uv.lock").read_bytes()).hexdigest()

        status["stage"] = "uv_install"
        run_logged([sys.executable, "-m", "pip", "install", "--quiet", "uv==0.9.18"],"install_uv",180)
        uv = shutil.which("uv")
        if not uv:
            # Find the environment-specific Scripts/bin path.
            uv = str(Path(sys.executable).parent / "uv")
        if not Path(uv).is_file():
            raise RuntimeError("Pinned uv executable missing")
        status["uv_version"] = subprocess.check_output([uv,"--version"],text=True).strip()

        status["stage"] = "uv_lock_cuda_sync"
        # cu128 is the original upstream CUDA extra. Never replace with CPU Torch.
        run_logged(
            [uv, "sync", "--locked", "--no-dev", "--extra", "cu128",
             "--python", sys.executable, "--no-progress"],
            "uv_sync_original_lock_cuda", 720, CHECKOUT,
        )

        status["stage"] = "native_gpu_train"
        local_script = ROOT / "run_original_train.py"
        local_script.write_text(TRAIN_SCRIPT)
        env_python = CHECKOUT / ".venv" / "bin" / "python"
        if not env_python.exists():
            raise RuntimeError("Pinned lock did not create Python venv")
        # Pin CUDA visible as required by Stanford's original TrainConfig.
        os.environ["CUDA_VISIBLE_DEVICES"] = "0"
        run_logged([str(env_python), str(local_script)], "native_train", 1000, CHECKOUT)

        receipt = json.loads((RESULT / "verified_receipt.json").read_text())
        if receipt.get("status") != "NATIVE_STANFORD_RSL_RL_CUDA_CHECKPOINTS_VERIFIED":
            raise RuntimeError("Native training receipt failed acceptance")
        status["status"] = "G1_CUDA_NATIVE_CHECKPOINT_PASS"
        status["stage"] = "completed"
        status["checkpoint_count"] = len(receipt["checkpoints"])
        status["gpu_name"] = receipt["protocol"]["gpu_name"]
        exit_code = 0
    except Exception as exc:
        status["status"] = "G1_FAIL"
        status["error_type"] = type(exc).__name__
        # Avoid embedding secret-like environment data / private URLs.
        status["error_label"] = str(exc)[:160] if isinstance(exc, RuntimeError) else ""
        exit_code = 2
    finally:
        status["wall_seconds"] = time.monotonic() - t0
        (RESULT / "driver_status.json").write_text(json.dumps(status, indent=2) + "\n")
        # Bundle only exact expected report names, original RSLRL checkpoints
        # and sanitized structured receipt. No git checkout, lock cache, keys.
        with zipfile.ZipFile(BUNDLE, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(RESULT.rglob("*")):
                if not path.is_file():
                    continue
                if path.suffix == ".pt" or path.suffix in (".json", ".yaml", ".log"):
                    archive.write(path, str(path.relative_to(RESULT)))
        print("G1_RESULT_BUNDLE_READY", BUNDLE, "bytes", BUNDLE.stat().st_size, flush=True)
        print("G1_STAGE_RESULT", status["stage"], status["status"], flush=True)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
