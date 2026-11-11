# MicroDuck: independent self-righting benchmark (2026-11-09)

Status: **review_only**, research branch only. Do **not** merge/publish to the Android
viewer until a successful upstream-model run, source asset licence review, and
viewer geometry mapping are recorded. This is separate from EN01 PR #95 and
the replay pipeline PR #91.

## Fixed sources

- Upstream robot MJCF/inference: `pollen-robotics/microduck_rl` at
  `bae39f5900b318f7e87fc485c30a31213cfa14eb`.
- MJCF: `src/mjlab_microduck/robot/microduck/scene.xml`, which includes
  `robot_groundcontact.xml` (the model intended for floor recovery).
- Official ONNX: `pollen-robotics/microduck-policies`,
  `v8/velstand.onnx`, SHA256
  `2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492`.
- Official policy inference implementation: `scripts/infer_policy.py`,
  used directly to preserve 61 observations, 14 actions, zero commands,
  action history and 50 Hz cadence. **No policy is re-trained.**
- Comparison resource (not a measurement from our lab):
  `xinhuagu/my-microduck-journal/docs/benchmark.md` published a 256-env,
  3-seed Warp/mjlab battery against official **v7** and community policies;
  its v7 scores must not be presented as ours or as evidence for v8.

## Reproduce

The PR workflow runs automatically on GitHub-hosted Ubuntu (Python 3.12) and
uploads raw JSON as a **CI artifact**. It does not require a Mac, Android,
GPU, custom machine setup or producing MP4. The exact commands, wheel
versions, source SHA and policy SHA are in
`.github/workflows/microduck-standup.yml`.

For offline/local re-use after installing `numpy`, `mujoco==3.10.0`, and
`onnxruntime>=1.24.4`, use the two pinned upstream inputs:

```bash
python research/microduck/standup_cpu.py \
  --rl-root upstream_microduck_rl \
  --policy upstream_policies/velstand.onnx \
  --output results/microduck/standup-cpu.json
```

## Cases and gates

- `stand`: reference posture (control test).
- `sit`, `fold`: official scene keyframes.
- `roll180`, `pitch180`, `side90`: **synthetic** roll/pitch pose disturbances
  followed by a 0.6 s physical fall with joint pose held. They are not
  described as genuine historical falls. The names specify a root rotation,
  not the exact side of the head making contact.
- `push`: a horizontal root velocity perturbation of 0.7 m/s after 1 s,
  **not** a measured push force.

`PASS` requires the final continuous 1.0 s of the 6 s rollout to have trunk
height in [0.09, 0.19] m, tilt <= 20 degrees, trunk speed <= 0.25 m/s,
angular speed <= 1 rad/s and at least one foot-floor contact. The first such
one-second window is also logged. These are **provisional research metrics**,
not a manufacturer's acceptance specification. All failing cases and
numerical warnings are retained rather than filtered. All physics rollout
frames preserve 50 Hz `qpos`, `qvel` and `ctrl`.

## Limits and next gate

**Important: this CPU smoke uses the official position-actuator XML**,
rather than the BAM XL330 voltage/friction simulation used for RL training.
As a consequence, it tests a *real MuJoCo system with approximate motors*,
not verified sim-to-real fidelity. Upstream v8 `velstand` was trained in
Rough-Backlash; this reference scene is the non-backlash ground-contact
variant. Do not extrapolate PASS/FAIL to hardware.

Before interpreting success, the next benchmark must use upstream BAM M6
with the appropriate backlash/encoder observation path, controlled random
seeds, multiple independent fall orientations, force and impact logging, and
a comparison against the existing independent benchmark methodology. Do
not change EN01 geometry, collision meshes, or its acceptance criteria.

### Android viewer

A physical result is not itself a full 3D model. After validating licence and
the actual MicroDuck mesh/kinematic mapping, adapt the saved full `qpos`
trajectory to the existing `replay_pipeline.py` contract from draft PR #91;
keep `publish_policy: review_only`. No copying STL into a public artefact
before checking CC BY-NC-SA 4.0 attribution and non-commercial restrictions.
