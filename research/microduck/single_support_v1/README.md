# MicroDuck: single-foot support before an obstacle step

The official v8 gait achieves short single-foot support on flat ground in this
CPU model. In three matched initial-joint perturbations, consecutive 50 Hz
observations show **0.12 s on the left and 0.14 s on the right** with the other
entire sole at least 5 mm above the floor. These are observed intervals, not
the robot's maximum capability and not continuous-substep contact guarantees.

**The proposed slow weight-transfer controller is unsuccessful:** all ten
tested settings fall, with peak trunk tilt about 88–101 degrees. No setting
achieves a quiet 1 s single-foot hold. The conditional obstacle-placement test
was therefore **not attempted**. This diagnoses this controller attempt; it
does not demonstrate a mechanical impossibility or a defective physical robot.

## What was tested

| Test | Result |
| --- | --- |
| Stock v8 standing control, 7.2 s | Upright, peak tilt 1.06 degrees; quality gate passes |
| Stock v8 gait, 0.30 m/s command, three initial-joint perturbations | All remain upright; peak tilt 4.56–5.71 degrees; single-foot intervals above |
| Left/right transfer to 55% or 70% of the lateral distance to the sole centre, with/without gravity feedforward | 8 failed holds |
| Left/right bounded COM-feedback follow-up | 2 failed holds |
| Walking and failed-hold repeats | Both reproduce byte-for-byte |

The initial perturbations use seeds 20261109–20261111 and independent uniform
joint offsets of +/-0.005 rad, applied before physics begins. The gait's
commanded speed is not its measured travel speed.

## Fixed model and controller boundary

Original `microduck_rl` revision
`bae39f5900b318f7e87fc485c30a31213cfa14eb`, BAM revision
`62bd8ce12154340be97e06f7f41a0ca8f116d967`, original v8 velstand ONNX checksum,
all 70 original visual meshes, 0.73724318 kg model mass, unchanged inertias,
joint ranges and BAM XL330 M6 parameters. Uses the corrected environment
collision coverage already in `../step_goal_v1/source/collision_model.py`,
0.5 ms physics, 20 ms control, 7.4 V supply with the existing sag model, and
output-encoder backlash semantics. No added ankle joint or increased torque.
The model has one ankle rotation per leg, not independent pitch and roll
ankle joints; side-transfer constraints must respect that kinematics.

The two-foot and walking controls use the unchanged neural policy. The ten
hold attempts use a **new experimental task controller**, not a capability
claimed for the stock policy. It solves separate scratch-data IK for foot
positions, support-foot orientation and horizontal COM, interpolates motor
targets and optionally adds gravity feedforward or bounded lateral feedback.
Gravity compensation changes targets, not the motor model or torque bounds.
The head stays at its settled pose in the canonical hold series.

The planner has nonzero pose residuals (COM residuals about 0.9–2.7 mm and
remaining foot/orientation error). A solved kinematic objective is not a
verified support transition. The motor replay falls during transfer, before
the requested lift/hold; the result does not establish which alternative
feedback controller could succeed. Some trials later recover under v8; that
does not satisfy the earlier single-foot hold objective.

Physical root positions/velocities are initialized once. Thereafter only
motor targets change; root motion comes from `mj_step`. The planner writes
only its own separate `MjData`. Replay/verification also has separate data.

## Acceptance and verification

Single support requires the selected foot to carry >80% of model weight, the
other foot load <0.02 N, the whole other sole >=5 mm above the floor, and
non-foot environment load <0.02 N. Quiet hold additionally requires tilt
<20 degrees, base linear speed <0.06 m/s and angular speed <0.7 rad/s for
one consecutive second. These fixed thresholds are experiment criteria.

The quality gate separately checks no numerical warning/reset, environment
and represented self-contact penetration <1 mm and motor/output-joint range
overshoot <1 degree. Targets are clamped 0.07 rad inside original motor
ranges. A quality pass alone never means a successful hold.

`verify.py` independently rebuilds original-CAD sole bounds and COM from all
5,540 canonical saved poses, reconstructs loads from contact IDs/normals,
checks target limits and both-foot support intervals, and rejects fabricated
sole-clearance and foot-load fields. Two complete reruns are byte-identical.
There are 221,600 physical integration steps in the main 14 episodes, plus
30,800 in the two repeats. Source verification covers 259 upstream RL blobs,
112 BAM blobs, installed BAM bytes and seven pinned local helper blobs.

Support duration is measured at 50 Hz. Substep extrema cover penetration,
torque and joint range, not continuous single-foot clearance. Sparse original
self-collision coverage remains a limitation. No real robot was tested, no
hardware reliability estimate is made and no independent CI was run.

## Reproduction

Restore the previously verified input archive into an `inputs` directory:
[source archive](https://drive.google.com/file/d/1q_CmK4XggV4fv_21_Pa75-B9tKNp-que/view?usp=drivesdk),
SHA-256 `881977efcf04af626928c779be39f99c79e59091f2a19250ad58ad947cd16208`.
It contains `workspace/microduck-work`, full pinned RL/BAM source and ONNX
weights. Use Python 3.12, install this directory's requirements and install
the restored BAM source with `pip install --no-deps -e PATH_TO_BAM`.
From the repository root:

```bash
python research/microduck/single_support_v1/probe.py --workspace /absolute/inputs/workspace --out /absolute/results --cases research/microduck/single_support_v1/cases.json
python research/microduck/single_support_v1/verify.py --workspace /absolute/inputs/workspace --traces /absolute/results
```

`summary.json` and `verification.json` are the compact receipts. `replays/`
contains two compact recorded-state extracts with source hashes; they are
research data, not newly published player entries. Full contact/velocity/
target traces and the two repeat traces are in the result archive referenced
by `storage.json`. Existing Site UI, live catalog and main were not changed.
Research run: [Truthrail #332](https://github.com/Dmitry-dev-pet/truthrail/issues/332).
