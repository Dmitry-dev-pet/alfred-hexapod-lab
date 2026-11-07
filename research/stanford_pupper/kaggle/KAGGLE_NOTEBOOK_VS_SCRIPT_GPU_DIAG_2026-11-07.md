# Kaggle GPU allocation: notebook vs script — differential experiment (2026-11-07)

## Exact one-shot execution

- Account handle: `dmitriiglukhov`. Authenticated using existing private Actions secret `KAGGLE_API_TOKEN`.
- Project source: immutable commit `c6982f5b5adf716f0e0a6d73b80209ec777348ef` on our research branch.
- [Private runner-smoke PR #80](https://github.com/Dmytry-lab/runner-smoke/pull/80), [workflow #42398505000](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42398505000), [private artifact #13583919334](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42398505000/artifacts/13583919334).
- A **fresh, private Jupyter notebook kernel**, not script, [`dmitriiglukhov/pupper-v3-gpu-notebook-diag`](https://www.kaggle.com/code/dmitriiglukhov/pupper-v3-gpu-notebook-diag), version 1; metadata `kernel_type=notebook`, `enable_gpu=true`, `machine_shape=NvidiaTeslaT4`, `docker_image_pinning_type=latest`, no `docker_image` field in upload and no copied Kaggle notebook image metadata.
- Explicit CLI command `kaggle kernels push --path <temporary-folder> --accelerator NvidiaTeslaT4 --timeout 600`. Offline validation 6/6 PASS. No internet, no Stanford dependencies, no training. The only cell queries CUDA and attempts a minimal GPU sum if available.
- API statuses: `QUEUED` -> `RUNNING` -> **`COMPLETE`**. Importantly, Kaggle kernel execution success **does not mean GPU was allocated**.

## Inside Kaggle (actual emitted machine receipt)

```json
{
  "python": "3.13.18",
  "gpu_visible_devices": "<unset>",
  "nvidia_nodes": [],
  "nvidia_proc": false,
  "nvidia_smi_path": null,
  "torch": "2.11.0+cpu",
  "torch_compiled_cuda": null,
  "torch_cuda_available": false,
  "torch_device_count": 0,
  "torch_devices": [],
  "gpu_math_ok": false
}
```

Server-persisted metadata remains `enable_gpu=true`, `machine_shape=NvidiaTeslaT4`, but `docker_image=gcr.io/kaggle-images/python@sha256:0717e81990be591f8c5f98f033d29ab23e9ffde057e5b3a6bd4371f93969e962` (CPU). This rules out only the **hypothesis that our original Python-script kernel type or a copied image pin caused the absence of CUDA**.

## Official CLI source findings

Official [Kaggle CLI `kernels_push` implementation](https://github.com/Kaggle/kaggle-cli/blob/main/src/kaggle/api/kaggle_api_extended.py) passes `--accelerator` directly as `ApiSaveKernelRequest.machine_shape`. It uses `enable_gpu` from metadata and an optional `docker_image`; this is **not** a separate GPU-allocation command. [Official CLI documentation](https://github.com/Kaggle/kaggle-cli/blob/main/docs/kernels.md) and [developer clarification #1196](https://github.com/Kaggle/kaggle-cli/issues/1196) both confirm `NvidiaTeslaT4` for GPU T4×2.

## Highest-priority unverified hypothesis

[Kaggle staff](https://www.kaggle.com/discussions/product-feedback/268333) explicitly says GPU use requires **phone verification**. Independent users report that API GPU requests may silently downgrade to CPU until verified. Our evidence has not established the owner's phone verification state. The public CLI does not provide a documented user phone-verification query or a way to complete that verification. Authentication via Google sign-in/API token and an untouched 30h GPU quota **do not** demonstrate GPU entitlement.

**Stop submitting repeated GPU jobs.** Next user gate: inspect Kaggle Settings → Phone Verification (no API credentials necessary) and confirm it is already verified; if not, perform account verification. Only after this gate should we perform one bounded recheck. If already verified, preserve evidence and escalate backend CPU-downgrade through Kaggle support. No Stanford RL training was executed on Kaggle, no checkpoint created, no merge to `main`.
