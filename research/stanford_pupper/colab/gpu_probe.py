"""Minimal, non-secret factual NVIDIA GPU preflight for a fresh Colab VM.

Run this on the Colab runtime via colab exec. It neither trains a model nor
downloads repositories, and exits nonzero unless a working CUDA device exists.
"""

from __future__ import annotations

import datetime as dt
import json
import subprocess
import sys


def probe() -> int:
    receipt: dict[str, object] = {
        "timestamp_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "python_version": sys.version.split()[0],
        "scope": "GPU_ONLY_NO_TRAINING",
    }

    try:
        smi = subprocess.run(
            ["nvidia-smi", "-L"],
            capture_output=True,
            text=True,
            check=False,
            timeout=20,
        )
        receipt["nvidia_smi_exit_code"] = smi.returncode
        receipt["nvidia_smi_gpu_list"] = smi.stdout.strip()[:2000]
        nvidia_present = smi.returncode == 0 and "GPU " in smi.stdout
    except (OSError, subprocess.TimeoutExpired) as exc:
        receipt["nvidia_smi_error"] = type(exc).__name__
        nvidia_present = False

    try:
        import torch

        receipt["torch_version"] = str(torch.__version__)
        receipt["torch_cuda_build"] = str(torch.version.cuda)
        cuda_available = bool(torch.cuda.is_available())
        device_count = int(torch.cuda.device_count()) if cuda_available else 0
        receipt["torch_cuda_available"] = cuda_available
        receipt["torch_cuda_device_count"] = device_count
        if device_count:
            receipt["torch_cuda_gpu_names"] = [
                torch.cuda.get_device_name(i) for i in range(device_count)
            ]
            # Exercise the actual CUDA execution path, not only device metadata.
            a = torch.tensor([2.0], device="cuda:0")
            b = (a * 3.0).cpu().item()
            receipt["cuda_compute_result"] = float(b)
            cuda_compute_ok = b == 6.0
        else:
            cuda_compute_ok = False
    except Exception as exc:  # e.g. CPU-only torch, missing torch, CUDA init failure
        receipt["torch_probe_error_type"] = type(exc).__name__
        cuda_compute_ok = False

    passed = bool(nvidia_present and cuda_compute_ok)
    receipt["gpu_verified"] = passed
    print("PUPPER_GPU_PROBE_RECEIPT=" + json.dumps(receipt, ensure_ascii=True, sort_keys=True))
    print("PUPPER_GPU_PROBE_PASS" if passed else "PUPPER_GPU_PROBE_FAIL")
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(probe())
