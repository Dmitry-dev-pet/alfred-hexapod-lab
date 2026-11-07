#!/usr/bin/env python3
"""Kaggle GPU allocation differential: fresh Jupyter notebook vs script.

No Stanford installation or PPO training. A successful GPU allocation must
have actual /dev/nvidia* nodes and torch.cuda.is_available() == True.
Default is --dry-run. Submitting requires --submit and an account API token.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path


SLUG = "pupper-v3-gpu-notebook-diag"
TITLE = "Pupper v3 GPU Notebook Diag"
CELL = """import json, os, glob, shutil, sys, subprocess, time
from pathlib import Path
report = {
    'kind': 'kaggle-gpu-notebook-differential-v1',
    'python': sys.version.split()[0],
    'gpu_visible_devices': os.environ.get('CUDA_VISIBLE_DEVICES', '<unset>'),
    'nvidia_nodes': sorted(glob.glob('/dev/nvidia*'))[:16],
    'nvidia_proc': Path('/proc/driver/nvidia/gpus').exists(),
    'nvidia_smi_path': shutil.which('nvidia-smi'),
}
try:
    import torch
    report['torch'] = torch.__version__
    report['torch_compiled_cuda'] = torch.version.cuda
    report['torch_cuda_available'] = bool(torch.cuda.is_available())
    report['torch_device_count'] = int(torch.cuda.device_count())
    report['torch_devices'] = [torch.cuda.get_device_name(i) for i in range(torch.cuda.device_count())]
    if torch.cuda.is_available():
        x = torch.ones((1024, 1024), device='cuda:0')
        report['cuda_sum'] = float(x.sum().item())
        report['gpu_math_ok'] = (report['cuda_sum'] == 1024*1024)
    else:
        report['gpu_math_ok'] = False
except BaseException as exc:
    report['exception'] = '%s: %s' % (type(exc).__name__, str(exc)[:600])
    report['gpu_math_ok'] = False
p = Path('/kaggle/working/pupper_gpu_notebook_diag.json')
p.parent.mkdir(parents=True, exist_ok=True)
p.write_text(json.dumps(report, indent=2) + '\\n')
print('KAGGLE_GPU_NOTEBOOK_RECEIPT', json.dumps(report), flush=True)
"""


def metadata(username):
    if not re.fullmatch(r"[a-zA-Z0-9_-]+", username or ""):
        raise ValueError("Invalid Kaggle account handle")
    return {
        "id": f"{username}/{SLUG}",
        "title": TITLE,
        "code_file": "probe.ipynb",
        "language": "python",
        "kernel_type": "notebook",
        "is_private": True,
        "enable_gpu": True,
        "enable_tpu": False,
        "enable_internet": False,
        "machine_shape": "NvidiaTeslaT4",
        "docker_image_pinning_type": "latest",
        "dataset_sources": [],
        "kernel_sources": [],
        "competition_sources": [],
        "model_sources": [],
    }


def notebook():
    return {
        "cells": [{
            "cell_type": "code",
            "source": CELL.splitlines(keepends=True),
            "execution_count": None,
            "metadata": {},
            "outputs": [],
        }],
        "metadata": {
            "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
            "language_info": {"name": "python"},
            "accelerator": "GPU",
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--username", default=os.environ.get("KAGGLE_USERNAME"))
    p.add_argument("--submit", action="store_true", help="Launch one Kaggle notebook (uses GPU allocation if available)")
    args = p.parse_args()
    try:
        meta = metadata(args.username)
    except ValueError as exc:
        p.error(str(exc))
    if not args.submit:
        print(json.dumps(meta, indent=2))
        print("DRY_RUN: no Kaggle kernel submitted")
        return
    if not shutil.which("kaggle"):
        p.error("Official Kaggle CLI is required")
    with tempfile.TemporaryDirectory(prefix="kaggle-notebook-gpu-") as d:
        dest = Path(d)
        (dest / "kernel-metadata.json").write_text(json.dumps(meta, indent=2) + "\\n")
        (dest / "probe.ipynb").write_text(json.dumps(notebook(), indent=2) + "\\n")
        subprocess.run(
            ["kaggle", "kernels", "push", "--path", str(dest),
             "--accelerator", "NvidiaTeslaT4", "--timeout", "600"],
            check=True, timeout=180,
        )
    print("SUBMITTED", meta["id"])


if __name__ == "__main__":
    main()
