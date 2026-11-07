# Kaggle T4 ×2 Pupper CUDA preflight — first measured results (2026-11-07)

## Verified source and authentication
- Kaggle owner: `dmitriiglukhov`; private kernel `dmitriiglukhov/stanford-pupper-v3-mjlab-cuda-preflight`.
- Authenticated noninteractive using `KAGGLE_API_TOKEN` stored in private `Dmytry-lab/runner-smoke` Actions secret (value not disclosed).
- Official Kaggle CLI 2.2.5, `enable_gpu=true`, accelerator `NvidiaTeslaT4`, Internet enabled, requested 16 environments. Stanford source commit pinned at `aa475e9a393a209b29fd5de7e0ee10f3de643230`.
- Executed CI: [run #42396460552](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42396460552), 2 offline Python tests PASS, script compiled, API authentication PASS, Kaggle accepted kernel version 1.
- [Run #42396609864](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42396609864) unintentionally re-submitted the same script as version 2 when a status workflow was added to the PR. Corrected submission workflow to PR `opened` events **only** at [runner-smoke PR #76](https://github.com/Dmytry-lab/runner-smoke/pull/76); do not represent the duplicate as a separate experiment.

## Actual Kaggle result
- Both first versions finished `KernelWorkerStatus.ERROR`, **before** clone, `pip install`, MuJoCo simulation or any PPO training.
- Kaggle Python runtime was CPython 3.13.
- First script raised `FileNotFoundError: [Errno 2] No such file or directory: 'nvidia-smi'` on the first GPU-probe line.
- Receipt `pupper_gpu_preflight.json`: `status=fail`, `error_type=FileNotFoundError`, `wall_seconds=0.033` before notebook cleanup.
- [Archived private exact JSON and logs](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42396609966/artifacts/13583831050); first JSON SHA-256 `b8f29b16c52da50aa41284a30814be779c5dfc9c2bc8d9dac296823b943cb00b`.
- **Interpretation:** `nvidia-smi` executable absence is NOT by itself a verified `torch.cuda.is_available()==False`. We cannot yet claim GPU allocation or failed MuJoCo Warp compatibility.

## Next bounded gate
- Updated `preflight.py`: record device nodes and `/proc/driver/nvidia`, use preinstalled PyTorch CUDA visibility as the authoritative check, treat missing `nvidia-smi` as nonfatal when CUDA is available; fail before large package installation if no GPU.
- Re-submit **one** new private version pinned to reviewed source commit, collect actual JSON and logs; do not run long training or change `main`.
