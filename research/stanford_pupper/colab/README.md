# Stanford Pupper / Colab CLI — GPU preflight (G0)

**Issue:** [#81](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/81).  
**State:** code prepared, **NOT EXECUTED on the owner's Colab account**; no GPU is confirmed and no PPO learning was performed by this PR.

This is a small, independent probe of **actual CUDA execution** on a freshly allocated Colab VM, via Google's [official Colab CLI](https://github.com/googlecolab/google-colab-cli). No browser notebook, no Stanford policy download, no action against hardware or the existing viewer, no changes to upstream source.

## Prerequisites

- Trusted Linux/macOS terminal, preferably owner's existing Mac M4. Do not create a new runner or elevate privileges.
- Official CLI installed in a local isolated tool environment, e.g. `uv tool install google-colab-cli`. Record the version (`colab version`).
- Valid Google OAuth/ADC credentials **for Colab CLI** under the actual executing account. The user's ChatGPT Google Drive connection does **not** automatically supply Colab CLI credentials. Follow the upstream CLI's documented authentication flow; a **one-time interactive Google sign-in/consent may be needed**, but training jobs do not require a browser. **Never copy tokens, cookie files, ADC JSON, session IDs, or other credentials into GitHub or chat.**

## Single-command G0 execution (after CLI auth)

From the repository root, on the research branch:

```bash
bash research/stanford_pupper/colab/run_gpu_preflight.sh
```

Optional: `PUPPER_COLAB_GPU=L4` (default `T4`); `PUPPER_COLAB_RESULTS_ROOT=/some/private/path` for private local receipts. The script creates a **new uniquely named** remote session, runs `gpu_probe.py`, and attempts `colab stop` in an EXIT trap even after failure. Every allocation consumes Colab compute units; the script does **not** perform training or package installs on Colab.

The Python probe checks both `nvidia-smi -L` and an **actual CUDA tensor multiplication through PyTorch**, not just accelerator metadata. All local diagnostic logs remain beneath `$HOME/alfred-robotics/pupper-colab-gpu/<run-id>/` by default, outside the git checkout with restrictive permissions.

## Acceptance — G0 only

Review **all** of the following before promoting G0 to PASS:

1. `gpu_probe.log` contains `PUPPER_GPU_PROBE_PASS` and a JSON `PUPPER_GPU_PROBE_RECEIPT` where `gpu_verified=true`, `torch_cuda_available=true`, device_count > 0, and `cuda_compute_result=6`.
2. `stop_status.txt` contains `COLAB_STOP_EXIT=0`, and `sessions_after_stop.log` no longer lists this run's unique session (allow for backend propagation).
3. `colab_version.log`, `allocate.log`, `status.log`, `gpu_probe.log` and `stop.log` exist, and sanitized GPU/driver/CLI version and timestamp can be recorded without any account credentials.
4. A returned failure / absent CUDA / missing authorization is `G0_FAIL`, **not** a reason for retries or claims of available GPU. Inspect quota and authentication; never proceed automatically to PPO. Clean up any residual Colab session manually if `stop` failed.

## Next stage (not implemented here)

After G0 PASS, prepare and review a separate pinned native `cs123-stanford/pupper-mjlab@aa475e9a393a209b29fd5de7e0ee10f3de643230` MJLab/RSL-RL PPO smoke. Preserve the explicit nonzero reward weights from [Issue #78](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/78), begin 4/16/64 env runs, checkpoint the trainer, compare against zero-action baseline and the separate pretrained policy. Do not reinterpret Mac CPU training as GPU training, or checkpoint generation as evidence of learned walking.

**Important:** The script has not been tested against the owner's Colab API credentials; it is a deliberately gated preflight, not a completed GPU result. Keep this work in Draft. No merge to main without explicit owner approval.
