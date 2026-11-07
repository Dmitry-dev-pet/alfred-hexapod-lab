# Kaggle GPU preflight — original Stanford Pupper v3 (7 Nov 2026)

**Status: PREPARED / NOT SUBMITTED / GPU NOT TESTED.** This is deliberately separate from the independent SB3 Mac M4 experiment in draft PR #76. Source branch is stacked on PR #76; do not merge into `main` without owner approval.

## What is tested

This bounded **preflight only** tests an actual NVIDIA GPU, installs the **original** [Stanford Pupper MJLab](https://github.com/cs123-stanford/pupper-mjlab) at immutable commit `aa475e9a393a209b29fd5de7e0ee10f3de643230`, imports the official CUDA/MuJoCo Warp stack and attempts to create and reset **16 parallel environments** of `Mjlab-VelocityFS-Flat-Pupper-v3`. No weights, PPO iterations, W&B login, Android player changes, robot control or billing. It may fail on T4 due to driver, dependency, VRAM or Kaggle restrictions; that is a real outcome to record, not grounds to claim GPU training happened.

Original notebook: [CS123_Pupper_mjlab.ipynb](https://github.com/cs123-stanford/pupper-mjlab/blob/aa475e9a393a209b29fd5de7e0ee10f3de643230/notebooks/CS123_Pupper_mjlab.ipynb). It uses an interactive W&B login and needs explicit nonzero reward weights before meaningful training. This adapter does not silently run those cells.

## No-browser API operation

Install the [official Kaggle CLI](https://github.com/Kaggle/kaggle-cli) on a trusted machine with Python 3.11+:

```bash
python -m pip install kaggle
export KAGGLE_USERNAME="YOUR_KAGGLE_ACCOUNT_HANDLE"
python research/stanford_pupper/kaggle/submit.py --dry-run
```

For authenticated noninteractive operation use a **Kaggle API token** as `KAGGLE_API_TOKEN`, or existing Kaggle OAuth credentials, or a legacy `~/.kaggle/kaggle.json` with mode 0600. Do not paste tokens into chats, GitHub issues, code, notebooks or Action logs. For our **private** runner repository `Dmytry-lab/runner-smoke`, store it as a repository or environment secret `KAGGLE_API_TOKEN`; keep the account handle as `KAGGLE_USERNAME` variable. We cannot infer those credentials from the existence of a Kaggle account.

When credentials are ready and a user authorizes using their free GPU quota:

```bash
python research/stanford_pupper/kaggle/submit.py --submit
kaggle kernels status "$KAGGLE_USERNAME/stanford-pupper-v3-mjlab-cuda-preflight"
kaggle kernels logs "$KAGGLE_USERNAME/stanford-pupper-v3-mjlab-cuda-preflight"
kaggle kernels output "$KAGGLE_USERNAME/stanford-pupper-v3-mjlab-cuda-preflight"
```

The submission requests a **private** Kaggle kernel, NVIDIA T4, Internet enabled to download the pinned upstream source/dependencies, and a Kaggle timeout of 2400 seconds (40 minutes). Actual execution is subject to GPU quota, account verification and Kaggle capacity. The script writes `/kaggle/working/pupper_gpu_preflight.json` as a measurement receipt even on most failures. Verify its `status` and source commit; do not label preflight as training.

## Verification and next gate

Local offline checks, no Kaggle credentials needed:

```bash
python -m unittest discover -s research/stanford_pupper/kaggle -p 'test_*.py' -v
python -m py_compile research/stanford_pupper/kaggle/preflight.py research/stanford_pupper/kaggle/submit.py
```

Only after the T4 preflight passes: design a separately reviewed native Stanford PPO training run with nonzero reward weights, a limited iteration budget, realistic metrics (velocity/yaw RMSE and actual displacement), per-run checkpoint preservation, and comparison against original pretrained Stanford policy and prior zero-action result. Do not run full 4096-env / 3000-iteration training unreviewed.
