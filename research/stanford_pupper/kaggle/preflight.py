#!/usr/bin/env python3
"""Bounded, no-training GPU preflight for the original Stanford Pupper v3 MJLab.

Runs inside a private Kaggle script kernel with Internet and GPU enabled.
No secrets, W&B, deployment, policy training or hardware control.
"""
import glob
import json
import os
import shutil
import subprocess
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

UPSTREAM = "https://github.com/cs123-stanford/pupper-mjlab.git"
UPSTREAM_SHA = "aa475e9a393a209b29fd5de7e0ee10f3de643230"
TASK = "Mjlab-VelocityFS-Flat-Pupper-v3"
OUTPUT = Path("/kaggle/working/pupper_gpu_preflight.json")
REPO = Path("/kaggle/working/pupper-mjlab")


def checked(args, timeout=180):
    """Run exact argv, no shell and no credential logging."""
    return subprocess.run(args, check=True, timeout=timeout, text=True)


def main():
    result = {
        "schema": "pupper-kaggle-gpu-preflight-v1",
        "upstream": UPSTREAM,
        "upstream_commit": UPSTREAM_SHA,
        "task": TASK,
        "num_envs": 16,
        "training": False,
        "time_utc": datetime.now(timezone.utc).isoformat(),
        "status": "started",
    }
    started = time.monotonic()
    try:
        # Kaggle's container may omit nvidia-smi even when CUDA exists.
        # Record both driver/device visibility and torch's actual CUDA probe.
        result["runtime"] = {
            "python": sys.version.split()[0],
            "nvidia_smi_path": shutil.which("nvidia-smi"),
            "device_nodes": sorted(glob.glob("/dev/nvidia*"))[:16],
            "nvidia_proc_present": os.path.exists("/proc/driver/nvidia/gpus"),
            "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES", "<unset>"),
        }
        smi = result["runtime"]["nvidia_smi_path"]
        if smi:
            smi_output = subprocess.check_output(
                [smi, "--query-gpu=name,memory.total,compute_cap",
                 "--format=csv,noheader"],
                text=True, timeout=30,
            )
            result["gpus"] = smi_output.strip().splitlines()
        else:
            result["gpus"] = []
            result["nvidia_smi_note"] = "Tool not installed; CUDA check is authoritative"

        try:
            import torch as torch_existing
        except ImportError as exc:
            result["torch_existing"] = {"import_error": type(exc).__name__}
            torch_existing = None
        if torch_existing is not None:
            result["torch_existing"] = {
                "version": torch_existing.__version__,
                "compiled_cuda": torch_existing.version.cuda,
                "cuda_available": bool(torch_existing.cuda.is_available()),
                "device_count": int(torch_existing.cuda.device_count()),
            }
            if not torch_existing.cuda.is_available():
                raise RuntimeError("CUDA unavailable in Kaggle kernel before dependency installation")
        elif not result["runtime"]["device_nodes"] and not result["runtime"]["nvidia_proc_present"]:
            raise RuntimeError("No NVIDIA device nodes; no CUDA GPU allocated to this Kaggle kernel")

        # Pinned upstream source: never execute a moving branch.
        checked(["git", "init", str(REPO)])
        checked(["git", "-C", str(REPO), "remote", "add", "origin", UPSTREAM])
        checked(["git", "-C", str(REPO), "fetch", "--depth", "1",
                 "origin", UPSTREAM_SHA], timeout=300)
        checked(["git", "-C", str(REPO), "checkout", "--detach", "FETCH_HEAD"])
        actual = subprocess.check_output(
            ["git", "-C", str(REPO), "rev-parse", "HEAD"], text=True
        ).strip()
        if actual != UPSTREAM_SHA:
            raise RuntimeError("Pinned upstream commit mismatch")

        # Same editable-install approach as the official CS123 Colab notebook.
        result["phase"] = "install"
        checked([sys.executable, "-m", "pip", "install", "-q",
                 "-e", str(REPO)], timeout=1200)
        sys.path.insert(0, str(REPO / "src"))

        import torch
        import warp as wp
        import mujoco
        import mjlab.tasks  # noqa: F401; registers Stanford tasks
        from mjlab.envs import ManagerBasedRlEnv
        from mjlab.tasks.registry import load_env_cfg

        result["versions"] = {
            "python": sys.version.split()[0],
            "torch": torch.__version__,
            "mujoco": mujoco.__version__,
            "warp": wp.__version__,
        }
        result["torch_cuda_available"] = bool(torch.cuda.is_available())
        if not torch.cuda.is_available():
            raise RuntimeError("Torch cannot use CUDA")

        result["phase"] = "mjlab_environment"
        env_cfg = load_env_cfg(TASK)
        env_cfg.scene.num_envs = 16
        env = ManagerBasedRlEnv(cfg=env_cfg, device="cuda:0")
        try:
            reset_value = env.reset()
            result["reset_return_type"] = type(reset_value).__name__
        finally:
            env.close()
        result["status"] = "pass"
        result["phase"] = "complete"
        print("PUPPER_KAGGLE_CUDA_PREFLIGHT_PASS", flush=True)
    except Exception as exc:
        result["status"] = "fail"
        result["error_type"] = type(exc).__name__
        result["error"] = str(exc)[:1000]
        result["traceback_tail"] = traceback.format_exc()[-3500:]
        print("PUPPER_KAGGLE_CUDA_PREFLIGHT_FAIL", flush=True)
        raise
    finally:
        result["wall_seconds"] = round(time.monotonic() - started, 3)
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
        print("Receipt:", OUTPUT, flush=True)


if __name__ == "__main__":
    main()
