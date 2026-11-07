#!/usr/bin/env python3
"""One-shot Kaggle GPU image test using Kaggle's own published GPU image.

No training. Source code immutable in GitHub. Use --submit explicitly.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

from submit import build_metadata

HERE = Path(__file__).resolve().parent
KERNEL_SLUG = "stanford-pupper-v3-gpu-image-check"
OFFICIAL_KAGGLE_GPU_IMAGE = "gcr.io/kaggle-gpu-images/python"


def gpu_image_metadata(username):
    metadata = build_metadata(username)
    metadata.update({
        "id": f"{username}/{KERNEL_SLUG}",
        "title": "Stanford Pupper v3 GPU Image Check",
        "docker_image": OFFICIAL_KAGGLE_GPU_IMAGE,
    })
    assert metadata["is_private"] is True
    assert metadata["enable_gpu"] is True
    assert metadata["machine_shape"] == "NvidiaTeslaT4"
    return metadata


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--username", default=os.environ.get("KAGGLE_USERNAME"))
    parser.add_argument("--submit", action="store_true")
    args = parser.parse_args()
    if not args.username or not re.fullmatch(r"[A-Za-z0-9_-]+", args.username):
        parser.error("KAGGLE_USERNAME must be the user's Kaggle account handle")
    metadata = gpu_image_metadata(args.username)
    if not args.submit:
        print(json.dumps(metadata, indent=2))
        print("DRY_RUN: no Kaggle API call")
        return
    if not shutil.which("kaggle"):
        parser.error("Kaggle CLI must be installed")
    with tempfile.TemporaryDirectory(prefix="pupper-kaggle-gpu-") as temp:
        d = Path(temp)
        shutil.copyfile(HERE / "preflight.py", d / "preflight.py")
        (d / "kernel-metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        subprocess.run(
            ["kaggle", "kernels", "push", "-p", str(d),
             "--accelerator", "NvidiaTeslaT4", "--timeout", "2400"],
            check=True,
        )
    print("SUBMITTED", metadata["id"])


if __name__ == "__main__":
    main()
