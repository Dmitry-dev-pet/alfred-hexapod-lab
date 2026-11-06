# XGO-Lite3 S2 — ChatGPT CPU versus Mac M4 benchmark (6 November 2026)

**Purpose:** measure speed of the *same existing physical experiment* on the current ephemeral ChatGPT Linux CPU and the owner's Mac mini M4. No ML training, no GitHub Actions physics, no hardware commands, no change to `main`. This branch is stacked on draft PR #69.

## Strict reproducibility

- Entry point: `research/xgo_lite3/lite3_perf_benchmark.py`. It invokes **the existing unmodified** `lite3_s2_robustness.run_perturbed` from PR #69, not a synthetic engine-only benchmark.
- 6 variations × 10 deterministic seeds `20261106..20261115`, 9.5 simulated seconds/episode → 570 simulated seconds and 142,500 physical integration steps.
- Original verified 17-STL `scene_lite3.xml` SHA256 `f054f2ef66eb53af08120040e41e3aded4826a00f794669f556430086c0e52d7`.
- Original `policy.onnx` SHA256 `773a1bbd0fee45ec38604d979d5fd6fb339ae15d13bfb5d6e2056a171d2a3422`; inference at 50 Hz, MuJoCo at 250 Hz; no policy retraining.
- Original `lite3_onnx_joint_audit.py` and `lite3_s2_robustness.py` untouched. Same contact/force/Jacobian audit every physics step.
- MuJoCo **3.14.0**, ONNX Runtime **1.23.2** and CPU provider only; record Python/NumPy/PyTorch and OS versions.
- Run serially (one process), with `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1`.
- The benchmark creates the ONNX session and performs one warmup episode **outside** the timed 60-episode loop. Include model construction and all contact audit inside each timed episode.
- Reject mismatched source assets and dependency versions; do not substitute a different robot or policy.

## Measured ChatGPT result (actual execution)

- Host: ephemeral Linux x86_64, **4 vCPU cgroup quota, 4 GiB memory cgroup limit**.
- Python 3.13.5, MuJoCo 3.14.0, ONNX Runtime 1.23.2, NumPy 2.3.5, Torch 2.10.0 CPU, SciPy 1.17.0.
- Original private STL/ONNX assets restored from owner's private Drive archive and validated by SHA256. Offline MuJoCo wheelhouse restored from owner's private Drive archive with matching SHA256.
- **60/60 executed**, wall clock **30.961651129 s** (timed episodes only), mean simulated-time throughput **18.4098709× realtime**, **4602.4677 physics steps/sec**.
- Median timed episode **0.5110013 s**, p95 **0.5946625 s**, process peak RSS **945808 KiB (~924 MiB)**.
- Baseline seed 20261106 saturation **659/2375** physics steps, as in the original S2. Baseline mean contact-slip proxy (speed >0.05m/s) **12.8873%**, friction=0.20 **23.3495%**; agree with previous scientific S2 report.
- The ChatGPT run used a mirrored S2 implementation and the original unchanged baseline code. Numerical checkpoints above matched the pinned draft PR #69 S2 report. To remove any remaining ambiguity, the Mac executable calls the GitHub S2 module directly.
- Full per-episode JSON was produced as an attachment in the user conversation; no full traces or vendor binaries added to GitHub.

## Mac M4 status

**NOT EXECUTED. No Mac benchmark number is claimed.** Existing Mac runners successfully transported Lite3 source and binary assets previously, but did not run physical simulations.

### Mac execution protocol (local only)

Use existing Mac mini under an approved non-admin local user; no new runner. Obtain the owner-private model/ONNX ZIP from Drive (folder `Alfred-Hexapod-Lab`, name `XGO-Lite3-original-model-and-PPO-policy-2026-11-06.zip`) using an authenticated method. Never commit these binaries or credentials.

1. Use this benchmark branch from `Dmitry-dev-pet/alfred-hexapod-lab` and create a persistent isolated Python environment. Pin the same MuJoCo/ORT versions; verify Python wheel availability for native arm64. Record exact package versions. Do not overwrite other projects' environments.
2. Unpack owner ZIP into `$LITE3_RESEARCH_ROOT/original_assets/`. Unpack its nested `lite3-upstream-model-task.tar.gz` into `$LITE3_RESEARCH_ROOT/original_assets/upstream/`. Verify SHA256 as above; policy at `original_assets/lite3-hf-policy/lite3_walk/policy.onnx`, scene at `original_assets/upstream/src/assets/robots/lite3/xmls/scene_lite3.xml`.
3. Run the benchmark *without Actions* from the repository's `research/xgo_lite3` directory:

```bash
export LITE3_RESEARCH_ROOT="$HOME/alfred-robotics/lite3-benchmark"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
python lite3_perf_benchmark.py --seeds 10 \
  --output "$LITE3_RESEARCH_ROOT/results/mac_m4_s2_perf.json"
```

4. Confirm 60 rows and 142500 steps, original baseline `seed=20261106` saturation 659/2375, no error records. Preserve JSON, log, `sysctl -n hw.memsize` and `sysctl -n hw.ncpu`, macOS version, toolchain and hardware details.
5. Compute speed ratio `30.961651129 / mac_wall_seconds` only once Mac result exists. Report `speedup_mac_over_chatgpt` with the measured denominator and clarify it is **single-process CPU execution**, not PPO training. Compare any numerical divergences separately.
6. Do **not** run this physical benchmark via GitHub Actions; do not flash the robot; do not merge `main` without explicit owner approval.

## Interpretation

The existing ChatGPT environment already ran 570 simulated seconds in 31 seconds. A Mac will help with long-running persistence and possibly concurrent workloads even if a single episode is not faster; acceleration remains *unmeasured* until Mac runs the same protocol.
