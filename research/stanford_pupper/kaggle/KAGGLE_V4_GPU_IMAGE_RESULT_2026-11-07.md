# Kaggle official GPU-image fresh-kernel diagnostic — negative result (2026-11-07)

## What was actually executed

One private, explicitly GPU-pinned fresh Kaggle kernel:
- Kernel [dmitriiglukhov/stanford-pupper-v3-gpu-image-check](https://www.kaggle.com/code/dmitriiglukhov/stanford-pupper-v3-gpu-image-check), version 1, submission succeeded.
- Submitted and collected by [private GitHub Actions #42397605379](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42397605379), pinned project source `1049644c56c04b3bd8e2c314d34a3905fd3cf2ce`.
- Three offline validation tests PASS; official Kaggle CLI authenticated with private `KAGGLE_API_TOKEN`, never printed.
- Submitted as private T4×2 `NvidiaTeslaT4`, `enable_gpu=true`, and explicitly requested **official Kaggle GPU Docker image** `gcr.io/kaggle-gpu-images/python` in the metadata.
- Kaggle status from the API: QUEUED → RUNNING → **ERROR**.
- Exact [private receipt and logs artifact](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42397605379/artifacts/13584291987).

## Observations, not guesses

The actual Kaggle execution again returned:
- Python 3.13.18, `torch==2.11.0+cpu`, `torch.version.cuda=null`.
- `torch.cuda.is_available()=False`, CUDA device count **0**.
- No `/dev/nvidia*`, no `/proc/driver/nvidia/gpus`, no `nvidia-smi`.
- JSON receipt `status=fail`, `RuntimeError: CUDA unavailable in Kaggle kernel before dependency installation`.
- Crucially, `kaggle kernels pull -m` **persisted** `enable_gpu=true` and `machine_shape=NvidiaTeslaT4` but **returned a CPU image** `gcr.io/kaggle-images/python@sha256:0717e81990be591f8c5f98f033d29ab23e9ffde057e5b3a6bd4371f93969e962`. This overwrote/ignored our explicit GPU-image selection.

This demonstrates that **for this Kaggle account and CLI/API push path in this observed run, accepted GPU metadata did not lead to GPU allocation.** It does not prove all Kaggle accounts or sessions are affected, nor isolate whether a Kaggle backend policy, server bug, or account restriction is responsible.

## Quota and next decision

Separate official `kaggle quota --format json` showed GPU 0.00/30.00h used, remaining **30.00h**, refresh 2026-11-14; TPU 0.00/20.00h used, remaining 20.00h. Hence **quota exhaustion is not the cause**.

**Stop automatic Kaggle retries now**: all versions (v1–v3 original slug plus v1 fresh GPU-image slug) failed before any MuJoCo Warp, policy learning, or GPU simulation; no Pupper behavior is claimed. Retain private evidence, do not merge `main`, do not modify robot or Android viewer.

Potential routes: resolve Kaggle's backend/account GPU provisioning with provider, or run official Stanford Colab notebook manually on a confirmed GPU runtime; a verified self-hosted RTX 3090 runner is another option but not yet connected in this project. Avoid silently falling back to CPU or attributing success to an accepted API push.
