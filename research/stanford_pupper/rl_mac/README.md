# Stanford Pupper v3 — Mac M4 / SB3 PPO training pilot (2026-11-07)

**Status: CODE_PREPARED / CPU_PHYSICS_TESTED / MAC_PPO_NOT_RUN.**
Separate draft research branch; **do not merge into `main`** before owner approval. No Apple runner, GitHub Actions physics, background service, real robot or automated robot deployment.

## Scientific goal

Train a *new policy* with Stable-Baselines3 PPO on the Apple M4 using an independently written Gymnasium adapter. Compare trained speed, yaw accuracy, zero-command drift and falls to the unchanged [Stanford pretrained reference](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/75) using the same physical model. **Do not represent this as native Stanford MJLab training.** Stanford original [pupper-mjlab](https://github.com/cs123-stanford/pupper-mjlab) runs GPU MuJoCo Warp; its official README says NVIDIA for training and macOS for evaluation.

## Model and interfaces

- Source MJCF pinned to Stanford `cs123-stanford/pupper-mjlab` commit `aa475e9a393a209b29fd5de7e0ee10f3de643230`, Git blob SHA-1 `7771e176c42b3aed1d48cc62434771b0fceb7c8c`. Required source file is already in this PR's ancestor: `research/stanford_pupper/upstream/pupper_v3.xml`.
- 3.17kg, 12 joints, 4 feet; original inertias and collision spheres; 13 render-only meshes removed. `mujoco==3.14.0`; free-floating body, 250Hz physical `mj_step`, 50Hz policy actions, 4-physical-step servo delay, gain `kp=5.5`, `kd=.25`, actuator torque limit ±3Nm. Original STL not needed for training.
- Observations: newest-first 20×48 floats = 960; angular velocity, projected gravity, 3-axis `vx/vy/wz`, target world up, joint positions relative to default, last action, **12 zero gait-reference values** (the VelocityFS task does not supply a gait reference). We clip observations to [-10,10]; this is a proposed adapter, not a tested native deploy equivalence.
- Actor: 960→256→128→128→128→12, ELU. A separate PPO value network trains as critic. SB3 checkpoint is **not** binary compatible with Stanford `default_policy.json`; never overwrite that file.
- Actions: 12 bounded [-1,+1], applied as 0.75rad offset from Stanford default joint pose, clipped to model joint limits. There is no scripted body/root movement.
- Training commands: finite set of neutral, forwards/backwards, lateral and yaw turns, as well as combined movements. Maximum |vx|=.30m/s, |vy|=.20m/s, |wz|=.60rad/s. Training horizon 10s; command changes every 2s. **No run/jump yet**.

## Rewards (experimental, nonzero)

`reward.py` contains auditable `tracking` and `stability` profiles. Tracking: velocity XY and yaw rate positive, uprightness and height positive; fall, action jerk, actuator effort and vertical motion negative. `stability` increases safety weights in a second, separately documented comparison. **These weights are Alfred's research proposal, not Stanford's course baseline**, whose task defaults are all zero.

Compare *tracking errors*, falls, standstill drift and scenario success; do not claim a learning improvement only because scalar reward rises.

## Mac M4: one command from a clean checkout

From repository root, using existing local Mac interactive executor (not a new runner):

```bash
# Check out the exact reviewed PR branch or commit first.
bash research/stanford_pupper/rl_mac/run_on_mac.sh
```

This script creates an **isolated persistent venv** at `$HOME/alfred-robotics/.venvs/pupper-sb3-py313` once, using Python 3.13 and pinned `torch==2.10.0`, `mujoco==3.14.0`, `numpy==2.3.5`, `gymnasium==1.2.2`, `stable-baselines3==2.9.2`. It prints Mac CPU/MPS capability; runs **4096 training environment timesteps over four independent Gym environments** on CPU (not 4096 per env), then loads the checkpoint on CPU and evaluates one full 20-second multi-axis route. Writes checkpoint+receipts under `$HOME/alfred-robotics/pupper-training/smoke-cpu-20261107/` **outside GitHub**. On rerun use a new `PUPPER_RUN_OUTPUT` directory (never clobber checkpoints). Does not transmit proprietary binaries or request secrets. The original MJCF is already part of the reviewed research branch.

For a later larger run (only after smoke has completed and been examined):

```bash
# Run directly with persistent venv — unique output folder and explicitly selected device.
$HOME/alfred-robotics/.venvs/pupper-sb3-py313/bin/python \
 research/stanford_pupper/rl_mac/train_sb3.py --mode train --device cpu \
 --num-envs 4 --steps 100000 --profile tracking \
 --out "$HOME/alfred-robotics/pupper-training/cpu-tracking-100k-01"
```

For an MPS training-throughput comparison, run the same seed+steps with `--device mps` **only if torch reports MPS available**. Separate benchmark directories; compare full wall time, realized updates/sec, learning curves, CPU/RAM, and fixed physical evaluation; don't confuse GPU training time with MuJoCo CPU physics time. SB3/PPO commonly runs MLP training faster on CPU; MPS improvement is a hypothesis, not a result.

## Tests and acceptance

Offline in ChatGPT CPU Linux x86_64 / Python 3.13.5, MuJoCo 3.14.0 independently restored from owner Drive wheelhouse:

- 5/5 true physical tests `test_physics.py` passed: pinned source model, free-body 5-second stance at z≈0.1565585m, 12 actuators, 4 contacts, deterministic action response, validation rejects NaNs/incorrect shapes.
- 6/6 reward-design tests `test_reward.py` passed; 2/2 evaluation-path tests `test_evaluation.py` passed; Python compile and shell syntax passed.
- 2 Gymnasium integration tests are defined but **not executed in current CPU container** because Gymnasium/SB3 wheels aren't available here. They **must** pass on Mac before PPO smoke.
- Placeholder zero-action policy evaluated on the 20s route: XY RMSE 0.13368m/s, yaw RMSE 0.16658rad/s, 0 fall-guard ticks, world X≈−7.5mm. This is **not an RL-trained result** and illustrates why fall count alone is insufficient.

On Mac, report actual macOS version, `uname -m` (require `arm64`), Python/tool versions, peak RSS, Mac M4 vs MPS availability, installed package hashes (`pip freeze` in a sanitized receipt), both integration tests, PPO smoke exit code, checkpoint SHA-256, training wall seconds, steps/sec, PPO reward/error stats, 20s independent evaluation output, and logs. **Mac execution status currently NOT RUN.**

## Known limits

- Single-episode flat-floor reward design, no domain randomization, no native MJLab exporter or sim2real calibration.
- Even a trained checkpoint with high reward may stand still, over-rotate or fall under route switching. Compare explicitly to original 39 Stanford-policy experiments; strict model parity with native C++/MJLab is not established.
- No automatic publication to Android player: only publish measured `mj_forward` replays after owner review of success/failure.
- Leave the GitHub project `main`, previously archived experiments and robot hardware untouched.
