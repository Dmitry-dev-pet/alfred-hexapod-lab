# Kaggle Pupper CUDA diagnostic v3: GPU requested, CPU actually provisioned (2026-11-07)

## Executed, verified

- Private Kaggle kernel `dmitriiglukhov/stanford-pupper-v3-mjlab-cuda-preflight`, version 3.
- Submission and collection: [private Actions run #42397148907](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42397148907), [exact receipt archive](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42397148907/artifacts/13583782345).
- Actual Kaggle statuses: QUEUED → RUNNING → ERROR. True JSON receipt returned `status=fail`, `error_type=RuntimeError`, `error=CUDA unavailable in Kaggle kernel before dependency installation`.
- Measured inside Kaggle: Python 3.13.18, PyTorch **2.11.0+cpu**, `torch.version.cuda = null`, `torch.cuda.is_available() = false`, CUDA device count **0**, no `/dev/nvidia*`, no `/proc/driver/nvidia/gpus`, no `nvidia-smi`.
- The preflight terminated in 4.276 s without cloning Stanford or installing MuJoCo Warp. **No RL training or GPU physics executed.**
- Kaggle CLI independently confirms correct token authentication, but this does not imply GPU allocation.

## What the Kaggle API reports

Read-only official CLI queries executed in [private Actions run #42397290663](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42397290663), [private exact artifact](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42397290663/artifacts/13583722777):

- `kaggle quota --format json`: GPU used **0.00h**, remaining **30.00h** of 30.00h, refresh `2026-11-14T00:00:00Z`; TPU used 0.00h, remaining 20.00h of 20.00h.
- `kaggle kernels pull -m` persisted `enable_gpu=true`, `enable_internet=true`, `machine_shape=NvidiaTeslaT4`.
- Crucially, persisted `docker_image` was from the **CPU image** `gcr.io/kaggle-images/python@sha256:0717e81990be591f8c5f98f033d29ab23e9ffde057e5b3a6bd4371f93969e962`. The official Kaggle GPU image uses `gcr.io/kaggle-gpu-images/python`, per [Kaggle/docker-python README](https://github.com/Kaggle/docker-python).
- **Interpretation:** the API accepted the T4 request and the user has quota, but a CPU-only runtime was provided. Root cause is not yet established (GPU scheduling vs image selection).

## Next controlled hypothesis

Run a **separate fresh private kernel** with `enable_gpu=true`, `machine_shape=NvidiaTeslaT4`, and an **explicit** official Kaggle GPU `docker_image=gcr.io/kaggle-gpu-images/python`; this avoids inheriting the original kernel's CPU image pin.

Only a new version actually reporting `torch.cuda.is_available()=true` and successful Pupper environment reset would count as a GPU PASS. Explicit image may be rejected by Kaggle; do not fall back silently, retry indefinitely, or start PPO training. No merge to `main`.
