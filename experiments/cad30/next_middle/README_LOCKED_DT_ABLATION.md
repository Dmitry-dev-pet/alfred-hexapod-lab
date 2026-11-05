# CAD30 — integration-step sensitivity with fixed 500 Hz servo commands (2026-11-05)

**Status: experiment runner and five pure tests are prepared; the new 2/1/0.5 ms locked-command physics matrix is NOT RUN.** This is stacked on [draft PR #49](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/49) and supports [Issue #46](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/46). No changes to the physical robot, no merging into main.

## Important distinction

PR #47, with position-only PD, showed 8 isolated 2ms ML support-force gaps during MR swing. [PR #48](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/48) audited the raw trace. PR #49 introduced causal Fz-based landing feedback; a full 6→ML→MR continuous MuJoCo run at 2ms eliminated the 8 gaps, and a local post-ML 1ms replay was also a strict PASS. **The old missing support is not proof of numerical contact chatter.**

This new ablation answers a narrower question: for the **same command stream sampled at 500Hz** (including the recorded effective target produced by the controller), how sensitive are the actual free-body contacts to integration steps of 2ms / 1ms / 0.5ms?

- For all 3 integration resolutions, compute Kp=7,Kd=0.32 only every 2ms using the actual current joint state, freeze the resulting PD torque across 1, 2, or 4 internal MuJoCo steps. All other numerical settings and recorded motor targets remain identical.
- At **every physical substep**, measure real contact reactions Fz via `mj_contactForce`, actual floor/tibia contact pairs, support triangle margin, stance loss count, max joint position/torque and terminal four-support hold fraction. Never smooth or average Fz before judging the strict support gate.
- The 2ms run must first reproduce the archived trace against qpos and per-leg Fz (runner's fidelity gate). If that baseline fails, do **not** interpret finer-dt differences as a validated dt study; troubleshoot the missing MuJoCo solver warmstart / initialization or exact model before proceeding.
- This **open-loop recorded-command replay is NOT the same as rerunning online Fz-feedback at faster sampling**. Its purpose is to isolate numerical integration while keeping the 500Hz motor target input fixed. The PR #49 live-controller 1ms sensitivity result remains a separate experiment.

## Inputs

All original Freenove STL/CAD binaries remain in the owner's **private Drive**:
- [CAD30 original local model archive](https://drive.google.com/file/d/1qN8KAgekhXm5RasMXSHNxHAsMxivpZ44/view?usp=drivesdk) (extract to `CAD30_ROOT`, containing `source/mjcf/scene_local30_nominal.xml`, `source/meshes/`, `hexlab/`).
- [Post-ML physical checkpoint](https://drive.google.com/file/d/1oM9ETjrnWYuJdOBNnUVMZO7KPwKjwzFb/view?usp=drivesdk), inner `source/results/after_ml_physical_checkpoint_20261105.json`, SHA256 `0e73f9b7aa699bc39da373ad59f5807d92a2863e8400da8b260f4553c9e6d05e`.
- [PR49 original baseline/feedback traces](https://drive.google.com/file/d/1LHKeKZ7fBffePbp6PCI0VsHsdz1CwRTp/view?usp=drivesdk), inner `measurements/minimal_baseline_continuous_MR_2ms.json.gz` (SHA256 `c38175c892bb0bac115565a3d423a1254f5ce3fc079c06626ad526dd0c8e53f5`) and `measurements/minimal_continuous_MR_2ms.json.gz` (SHA256 `c48e94e34f8d31e0444989bc2637021f4453bc462a7b32511c780de36343fce9`).

Original simulation: Python 3.13, `mujoco==3.14.0`, numpy, saved model/model parameters; execute with original private assets, **no Docker or GitHub Actions**.

## Commands

```bash
# Prepare with unzip, using names as extracted from the two private input ZIPs:
export CAD30_ROOT=/private/extracted/CAD30
export POST_ML=/private/inputs/source/results/after_ml_physical_checkpoint_20261105.json
export BASELINE=/private/inputs/measurements/minimal_baseline_continuous_MR_2ms.json.gz
export FEEDBACK=/private/inputs/measurements/minimal_continuous_MR_2ms.json.gz

python -m unittest discover -s experiments/cad30/next_middle -p 'test_locked_command_dt.py' -v

for mode in baseline feedback; do
  if [ "$mode" = baseline ]; then source_trace="$BASELINE"; else source_trace="$FEEDBACK"; fi
  for substeps in 1 2 4; do
    python experiments/cad30/next_middle/locked_command_dt.py \
      --cad-root "$CAD30_ROOT" --checkpoint "$POST_ML" \
      --trace "$source_trace" --substeps "$substeps" \
      --out "/tmp/cad30_${mode}_substeps_${substeps}.json"
  done
done
```

Substeps 1, 2, 4 correspond to 2ms, 1ms and 0.5ms physical integration. Before accepting any cross-dt comparison inspect the status of **both** 2ms substeps-1 reference runs, the largest qpos/Fz deviation vs the historical traces, and corresponding SHA256 inputs. If either baseline mismatches, the comparison is not validated.

Outputs: compact JSON and 2ms-aligned compressed diagnostic trace in `/tmp`; archive full raw traces and hashes in Drive only, put compact receipts / code / hashes in GitHub. Do not add third-party CAD binaries to repository.

## Existing evidence (not from this new ablation)

From PR #49 at μ=1.1: causal feedback MR at 2ms yielded 0 stance loss ticks, +29.839mm MR world toe advancement, minimum stance margin +12.815mm, strict PASS; its **separate** local 1ms feedback study reported 0 loss ticks and +28.947mm toe advancement. No 0.5ms live-feedback MuJoCo result is yet verified. Full 4-step locomotion and forward net travel from the six-support start remain **NOT ACHIEVED**.
