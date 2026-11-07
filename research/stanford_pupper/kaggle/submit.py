#!/usr/bin/env python3
"""Prepare (default) or explicitly submit private Stanford Pupper CUDA preflight.

Requires installed Kaggle CLI and account credentials only for --submit.
Never prints or copies Kaggle credentials into a kernel package.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
KERNEL_SLUG = "stanford-pupper-v3-mjlab-cuda-preflight"


def build_metadata(username):
    if not re.fullmatch(r"[A-Za-z0-9_-]+", username or ""):
        raise ValueError("KAGGLE_USERNAME must be a Kaggle account slug")
    return {
        "id": f"{username}/{KERNEL_SLUG}",
        "title": "Stanford Pupper v3 MJLab CUDA Preflight",
        "code_file": "preflight.py",
        "language": "python",
        "kernel_type": "script",
        "is_private": True,
        "enable_gpu": True,
        "enable_internet": True,
        "machine_shape": "NvidiaTeslaT4",
        "dataset_sources": [],
        "competition_sources": [],
        "kernel_sources": [],
        "model_sources": [],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--username", default=os.environ.get("KAGGLE_USERNAME"))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--submit", action="store_true",
                      help="Explicitly spend Kaggle free GPU quota")
    mode.add_argument("--dry-run", action="store_true",
                      help="Validate and print metadata without sending a job (default)")
    args = parser.parse_args()

    try:
        metadata = build_metadata(args.username)
    except ValueError as exc:
        parser.error(str(exc))

    if not args.submit:
        print(json.dumps(metadata, indent=2))
        print("DRY_RUN: no Kaggle API call, no GPU quota used")
        return

    if not shutil.which("kaggle"):
        parser.error("Install the official Kaggle CLI first: python -m pip install kaggle")

    # Never put secrets in code, metadata, temp files or GitHub.
    with tempfile.TemporaryDirectory(prefix="pupper-kaggle-") as tmpdir:
        staging = Path(tmpdir)
        shutil.copyfile(HERE / "preflight.py", staging / "preflight.py")
        (staging / "kernel-metadata.json").write_text(
            json.dumps(metadata, indent=2) + "\n"
        )
        subprocess.run(
            ["kaggle", "kernels", "push", "-p", str(staging),
             "--accelerator", "NvidiaTeslaT4", "--timeout", "2400"],
            check=True,
        )
    print("SUBMITTED:", metadata["id"])
    print("Check: kaggle kernels status " + metadata["id"])
    print("Logs: kaggle kernels logs " + metadata["id"])
    print("Output: kaggle kernels output " + metadata["id"])


if __name__ == "__main__":
    main()
