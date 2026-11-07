# Kaggle GPU request after verified phone — queued, not yet executed (2026-11-07)

The owner confirmed phone verification in the Kaggle account. We initiated exactly one fresh private Jupyter Notebook request with a different slug. No code or model from Stanford was installed and no PPO training was performed.

## Evidence

- Kaggle account: `dmitriiglukhov`.
- New private kernel: `dmitriiglukhov/pupper-v3-post-phone-gpu-probe`, version 1, `enable_gpu=true`, `machine_shape=NvidiaTeslaT4`, `kernel_type=notebook`, `docker_image_pinning_type=latest`, no hard-coded Docker image.
- Authenticating with the existing private Actions secret `KAGGLE_API_TOKEN` worked. Submitted via the official Kaggle CLI v2.2.5. Scripts and unit tests passed.
- Exact source commit: `9669a3cc2f34f39aa99f883d066f20863cf5a792`.
- Private [GitHub Actions run #42399071843](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42399071843), [draft PR #81](https://github.com/Dmytry-lab/runner-smoke/pull/81), [raw evidence artifact #13584797930](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42399071843/artifacts/13584797930).
- API returned **QUEUED** for all 36 checks from `18:00:37Z` through `18:09:42Z` (approximately 9 minutes). It did not return RUNNING, COMPLETE or ERROR during this observation window.
- `kaggle kernels pull -m` at end returned `enable_gpu=true`, `machine_shape=NvidiaTeslaT4`, **`docker_image=""`** (unassigned), not the earlier pinned CPU Docker image. This difference does not prove the GPU is or will be allocated.
- **No runtime CUDA JSON receipt exists** because the kernel has not started during the window. Do not claim CUDA, GPU physics, or trained policy.
- `kaggle quota --format json` remained **30.00 GPU hours remaining** (0.00h used; next refresh 14 Nov UTC). No GPU billing/usage confirmed.

## Interpretation and next steps

Compared with earlier immediate CPU-only executions, after phone verification the request now stays in the GPU queue with no CPU image selected. That is consistent with newly permitted GPU scheduling but is not proof of causality or completion.

Do **not** submit another GPU kernel while this one is queued. Read-only check of its existing status is the correct next action; if RUNNING or COMPLETE, inspect `pupper_gpu_notebook_diag.json` and confirm `torch.cuda.is_available()==true`, `gpu_math_ok==true`, and a real NVIDIA device. Only then run a bounded original Stanford MJLab preflight. Do not merge into main.
