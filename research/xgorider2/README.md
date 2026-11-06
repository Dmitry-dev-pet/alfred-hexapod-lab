# XGO-Rider2 RL: reproducible simulation experiment (Issue #66)

**Status: PREPARED / NOT EXECUTED ON OWNER MAC.** This research-only draft does not change `main`, CAD30, Hexapod Lab viewer or robot firmware. There is no robot hardware involved.

## Why this experiment

Reproduce a *genuine closed-loop* wheel-balance controller on a physical MuJoCo model before buying hardware or pursuing quadruped training. The reference project is [LuwuDynamics/xgorider2_rl](https://github.com/LuwuDynamics/xgorider2_rl), pinned to **`f534986353ce3fe451caf0da72ba06b980a59114`** (upstream default branch HEAD verified on 2026-11-06).

The shipped 18→32→32→4 float32 policy has 1,796 parameters. It controls **wheel balance**, not learned leg placement: both leg-policy channels are held at zero in the release. It therefore **does not solve our hexapod's adaptive stepping**. The value is a well-documented path from simulated dynamics, native C++ control, system identification and PPO to embedded control.

## Platform and isolation

- Apple silicon macOS (Mac mini M4 intended); Python **3.11**, `git`, `clang++`, online installation of pinned Python packages. CPU only: no CUDA/RTX GPU needed for Rider2.
- All downloads, Python venv, temporary copies of pretrained model and receipts go under **`$HOME/alfred-robotics/xgorider2`**, or `RIDER2_WORKDIR` if set. No downloaded upstream code or large artifacts are committed to our repo.
- The launcher refuses an unexpected origin or upstream commit. Reruns use new timestamped `runs/` folders and do not reset existing checkout.
- No Docker, GitHub Actions physics, remote execution, device flashing, vendor GUI credentials, robot commands or merges.

## Run on the Mac

From the draft branch containing this file:

```bash
bash scripts/research/xgorider2_mac.sh dry-run
bash scripts/research/xgorider2_mac.sh baseline
```

The default `baseline` performs:

1. Clone the pinned reference source (if needed); install its pinned Python packages into isolated `.venv`.
2. Copy `training/artifacts/balance-v6/` to a new run folder and run `verify_export2.py` against this **copy** (the verifier writes `native_verification.json`).
3. Evaluate the supplied native-controller policy in MuJoCo for **stand, stand_push, drive**, four deterministic seeds (51000–51003) each, 15 simulated seconds per seed, two CPU workers. Produces `baseline-evaluate.json`.

Optional:

```bash
bash scripts/research/xgorider2_mac.sh view         # baseline, then interactive MuJoCo with pushes
bash scripts/research/xgorider2_mac.sh train-smoke  # baseline, then 32,768 PPO training steps in separate run
```

`all` also exists. Do not confuse the 32,768-step PPO smoke run with the full upstream 4,000,000-step training recipe. A smoke training pass is *not* automatically a better policy; it still needs exported weights and a controlled A/B.

## Evidence and reporting

Each run prints its `RUN_DIR` (by default `~/alfred-robotics/xgorider2/runs/<UTC>-<pid>/`):

- `provenance.txt`: pinned upstream SHA, OS, Python and compiler metadata
- `pip-freeze.txt`, `dependencies.log`: exact installed Python environment
- `reference-policy/native_verification.json`, `verify.log`: exported neural policy ↔ native C++ check
- `baseline-evaluate.json`, `evaluate.log`: all 12 seed-scenario episodes and aggregated survival, wheel-command high-frequency energy, pitch/velocity drift
- `train-smoke/` and `train-smoke.log` only if requested
- `status.txt`: `exit_code=0` only if all requested commands exited successfully

**No results have been claimed from this new Mac experiment.** Upstream reports its own evaluation (not our reproduction) and acknowledges limits outside the training distribution and no learned leg control. A physical robot's stability is not established from simulator PASS.

Share a short result report in [Issue #66](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/66); publish logs/checkpoints privately on Drive, and store only provenance, SHA-256 and compact metrics in GitHub. Do not upload original upstream CAD to the public repository: the model code/weights are Apache-2.0, while some visual meshes are CC BY-NC 4.0.

## Later: four-legged XGO, then our hexapod

A separate upstream [Luwu Dynamics luwu_mjlab](https://github.com/LuwuDynamics/luwu_mjlab) contains 12-DOF `Lite3-Walk-Flat` and `Mini2S-Walk-Flat` environments, XML robot models and training scripts. This is the actual *adaptive quadruped locomotion* experiment. The upstream documentation targets NVIDIA GPU under Ubuntu 22.04 / Python 3.11, using MuJoCo Warp; our RTX 3090 may be suitable, but this exact configuration remains untested. No model-checkpoint files were detected in the upstream repository tree. `--agent zero` only smoke-tests the environment; it is **not a pretrained walking policy**.

Once Rider2 native-controller behavior is independently measured, start a fresh draft for the quadruped; do not alter the ongoing CAD30 four-support studies until we have a reproducible baseline and valid adaptation hypothesis.
