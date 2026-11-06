# XGO-Lite3 — corrected original ONNX CPU locomotion evaluation

**Experiment date:** 2026-11-06. **Research status:** genuine pretrained ONNX policy drives the pinned original STL-backed XGO-Lite3 free body in MuJoCo on CPU; **not native MJLab** and **not validated hardware control**. Full evidence: `LITE3_ONNX_CORRECTED_AGGREGATE.json` plus raw `qpos/qvel/ctrl/foot-force` JSONs in the matching private Drive research ZIP.

## Source of truth and reproducibility

- Manufacturer: [`LuwuDynamics/luwu_mjlab`](https://github.com/LuwuDynamics/luwu_mjlab/tree/447945fd40fb5585271dce4ea8cc82567916032a) @ `447945fd40fb5585271dce4ea8cc82567916032a`, `lite3.xml` Git blob `9c9699c5fc3c5f4ed4c69ebf516c346ee6a57026`.
- Pretrained *community*-published model: [`robomotic/XGO`](https://huggingface.co/robomotic/XGO/tree/1646bc65ed37feff3698a43524a65cb3cd5b6981/lite3_walk), HF snapshot `1646bc65ed37feff3698a43524a65cb3cd5b6981`.
- `policy.onnx` SHA-256 `773a1bbd0fee45ec38604d979d5fd6fb339ae15d13bfb5d6e2056a171d2a3422` (777,245 bytes); `model_1499.pt` SHA-256 `cca3a020163309790f8dc8ecb7553ff0e98e75d02e78a1773ad3734ca535a67c` (4,819,061 bytes).
- Private original source+policy ZIP: [Drive](https://drive.google.com/file/d/1uzAHREDGxxp2ehs8jEeJp6PcujCFvueQ/view?usp=drivesdk), fetched by [Mac M4 workflow](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42243825347).
- `onnxruntime==1.23.2` binary and dependencies transferred via separate [Mac M4 workflow](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42250970814), PyPI SHA-256s checked. Physics `mujoco==3.14.0` from the user's pre-existing offline Drive bundle; no CUDA/GPU. Python 3.13.5, NumPy 2.3.5. `torch==2.10.0+cpu` used **only** to compare the published PyTorch checkpoint with the ONNX actor.
- Our executed, independent CPU code `lite3_onnx_joint_audit.py`; source unit tests `test_lite3_onnx_joint_audit.py` **8/8 PASS** on pinned binaries.

## Critical correction to the previous 8-second exploratory trial

Previous `CPU_DIAGNOSTIC_2026-11-06.md` applied raw neural actions in the **XML actuator order**. However, the **ONNX metadata** explicitly lists `joint_names` in **natural model joint order**. The pinned [`mjlab v1.2.0` `find_joints_by_actuator_names`](https://github.com/mujocolab/mjlab/blob/v1.2.0/src/mjlab/entity/entity.py) explicitly builds its action IDs in that natural joint order. **Every one of 14 positions differs** between the two index arrays; the prior +47.80 mm exploratory trial is therefore **invalid as an evaluation of the learned policy**, though the actual MuJoCo numerical trace remains a real outcome of the *miswired* test.

Correctly implemented action `j` goes to `ctrl[actuator_index_for(joint_name[j])]`, not `ctrl[j]`. Initial pose is taken from ONNX metadata; `action_scale=.25` rad. The MuJoCo model was restored from original STL files with the source task's **four-shank-only contact configuration**, timestep `dt=.004s`, `implicitfast` integrator, and a single `MjData` per episode with no root teleport. The policy receives 53 float32 observations (base angular velocity, gravity, desired twist, sin/cos gait phase, joint position/velocity relative to defaults, last action) and returns 14 actions every `dt*5=.02s` (50 Hz). No hardware or simulated 12/3 parallel training.

**Second discrepancy:** on 201 identical 53-dimensional test inputs, the published ONNX and our independently reconstructed neural network from `model_1499.pt` do **not** give numerically identical actions: max absolute difference **0.48884**, mean absolute difference **0.05030**. The PyTorch reconstruction independently agrees with a direct `torch.nn.Sequential` forward pass to ~`6e-6`; root cause of ONNX/PT artifact mismatch has **not** yet been established. The physical results below use the **actual ONNX file** without substituting checkpoint weights.

## A/B #1 — standard commands, 8 seconds, 10 matched seeds (20 real physics rollouts)

Perturbed initial roll/pitch each within ±1°, yaw ±2.29°. Each ONNX episode compared against a matched **zero-action / default-position hold** episode; zero does not mean zero torque. Commands: stand 0–2s, forward 0.2 m/s 2–4s, yaw +0.35 rad/s 4–6s, stop 6–8s.

| Observed | Pretrained ONNX policy | Zero-action baseline |
| --- | ---: | ---: |
| Forward segment local displacement | **401.3 mm** (goal 400 mm) | ~0 mm |
| Yaw segment rotation | **+47.1°** (goal +40.1°) | ~0° |
| Fall criterion triggered (tilt >60° or root z <30mm) | **0/10** | **0/10** |
| Highest tilt across all runs | **3.18°** | **1.12°** |
| Actuator-force saturation ticks per 2,000 | **326.1 average** (16.3%) | **0** |
| Stop-segment net translation | **2.6 mm mean** | ~0 mm |

World-root total 8s ΔX ~+384.9 mm for ONNX; note this is *not* the forward segment displacement, as yaw/standing segments move slightly too. Average forward segment range 398.7–404.7 mm world X (body-x mean ~401.3 mm).

## A/B #2 — mid-stride command switches, 9.5 seconds, 10 matched seeds (20 further real physics rollouts)

Commands change at `t=1.3, 3.1, 4.7, 6.3, 7.9 s`, deliberately **not aligned to the 0.4-second gait phase**. Values are local body velocity targets. World XY increments were converted to the body orientation at the start of each segment for comparison; yaw measured from simulated base quaternion.

| Segment | Command / nominal target | Mean measured with ONNX (10 seeds) |
| --- | --- | ---: |
| Forward, 1.8s | +0.25 m/s → **+450 mm** | **+439.1 mm** |
| Lateral, 1.6s | +0.13 m/s → **+208 mm** | **+211.5 mm** |
| Yaw, 1.6s | −0.50 rad/s → **−45.84°** | **−45.17°** |
| Reverse, 1.6s | −0.15 m/s → **−240 mm** | **−257.1 mm** |
| Stop, final 1.6s | 0 velocity → **0 mm** | **9.6 mm residual translation** |

- Same matched zero-action reference does not track any of the requested motions.
- No episode exceeded the simple fall criterion (0/10 per controller), peak ONNX tilt **3.89°** among these seeds.
- At least one simulated actuator reached its force cap in **656.2 of 2,375 ticks per ONNX episode on average** (~27.6%); 0 for zero actions. This is significant; do **not** extrapolate to hardware or report a torque-safe gait.
- Recorded actual MuJoCo `qpos/qvel`, joint controls, per-foot `mj_contactForce` normal reactions, orientation and sensor-derived raw observations. Some sampled forward-gait frames have **zero simultaneously loaded feet** (flight phase), so do **not** call this a strict quasi-static four-support gait.

## Scientific interpretation and remaining gaps

**Demonstrated:** A pretrained policy from public ONNX, running **entirely on CPU**, can close the loop with original Lite3 model and naturally named joint indices and follow forward/lateral/reverse/yaw commands even when changed inside an ongoing step. Matching zero-action controls show that the commanded motion is policy-driven, not pre-recorded or an imposed root velocity. Repeated 10-seed tests work under **very small initial angle perturbations**, not broad robustness.

**Not demonstrated:** native `luwu_mjlab/mjlab` identical simulation, ONNX checkpoint numerical equivalence, friction/mass robustness, calibrated real actuator torque/current, nonflat terrain, collision of body/manipulator, safe fall recovery or hardware actuation. The simulation's listed inertial masses sum to **0.345 kg** vs public product nominal **0.610 kg**; replacing model with measured hardware remains necessary. Body yaw control overshoot, stop drift and frequent actuator saturation remain open.

**Next experiments:** freeze this CPU baseline; reproduce with full native MJLab or equivalently validated physics parameters and observation time alignment; test μ=.2/.3/.6 across seeds and sustained long turns; quantify support contacts and foot slip at **every** physics step; inspect whether `policy.onnx` corresponds to another checkpoint than `model_1499.pt`; publish a genuine replay to existing tablet library only as a separately labeled **simulated ONNX** record with provenance and warnings. No merge or hardware movement without explicit owner approval.

## Reproduction in an isolated CPU environment (after assets are restored from Drive)

```bash
# Extract the original private model/policy ZIP into ./original_assets,
# then extract its nested lite3-upstream-model-task.tar.gz under original_assets/upstream.
export LITE3_RESEARCH_ROOT="$PWD"
python3 -m pip install mujoco==3.14.0 onnxruntime==1.23.2 numpy scipy torch
python3 -m unittest test_lite3_onnx_joint_audit.py -v
python3 lite3_onnx_joint_audit.py --seeds 10 --start-seed 20261106 --seconds 8 --protocol standard --output-dir ./onnx_corrected
python3 lite3_onnx_joint_audit.py --seeds 10 --start-seed 20261106 --seconds 9.5 --protocol midstride --output-dir ./midstride_corrected
python3 summarize_onnx_lite3.py --standard ./onnx_corrected --midstride ./midstride_corrected
```

No vendor STL or weights are committed into GitHub. The model, exact snapshot, full 40 raw physics traces, and execution code are stored in the private Drive archive(s) with SHA-256 receipts.
