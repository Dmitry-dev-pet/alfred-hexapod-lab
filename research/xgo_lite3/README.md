# XGO-Lite3 — S0 source/model audit (2026-11-06)

Research-only branch for [Issue #68](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/68). This is Luwu Dynamics XGO-Lite3, **not** Deep Robotics Lite3.

## Audited upstream and evidence status

| Artifact | Reference | Verification |
| --- | --- | --- |
| \`LuwuDynamics/luwu_mjlab\` | [commit 447945fd](https://github.com/LuwuDynamics/luwu_mjlab/tree/447945fd40fb5585271dce4ea8cc82567916032a) | GitHub source files fetched and inspected |
| Original training MJCF | \`src/assets/robots/lite3/xmls/lite3.xml\`, git blob \`9c9699c5fc3c5f4ed4c69ebf516c346ee6a57026\` | Parsed from original upstream UTF-8; 14 joints, 14 position actuators, 17 explicit inertial masses |
| Policy task | \`src/tasks/velocity/config/lite3_walk/{__init__,env_cfgs,rl_cfg}.py\` | Registered Lite3-Walk-Flat; timing and action parameters read from upstream |
| Manufacturer exported MJCF ZIP | v2026.09.17, MD5 \`8d244185d90c666cd039a46f7c91af4c\` | Official release listing verified; zip NOT downloaded |
| Pretrained model | [robomotic/XGO](https://huggingface.co/robomotic/XGO) \`lite3_walk/\` | Public model card verified; ONNX/PT binary NOT downloaded, exact revision/hash NOT independently checked |
| Native physical simulator / trained policy rollout | n/a | **NOT RUN**; neither MuJoCo nor inference engine is installed in this analysis container |

## Findings

- Raw train MJCF: exactly 12 leg joints + two arm joints \`mainarm_joint\` and \`forearm_joint\` = 14 position actuators. No claw actuator in the MJCF. Current product datasheet: 12+3=15 actuators (incl. gripper).
- Sum of all 17 explicitly declared MJCF masses = **0.345 kg**; nominal product weight **0.610 kg** (ratio **1.7681**). Some physical elements could be represented differently in the simulator. This gap cannot be resolved by a uniform rescale without weighing/calibration.
- MJCF \`position\` actuator default \`forcerange="-0.31 0.31"\`; manufacturer servo nominal torque **2.3 kg·cm**, approximately **0.2256 N·m**. Compare with great caution: neither continuous torque curve nor actual transmission, supply voltage, and saturation have been measured.
- Training \`cfg.sim.mujoco.timestep = 0.004\`, \`cfg.decimation = 5\`: policy step **20 ms, 50 Hz**; position action scale **0.25**, phase period **0.4s**.
- Command training ranges: forward vx **−0.2 to +0.4 m/s**, lateral vy **−0.15 to +0.15 m/s**, yaw **−0.8 to +0.8 rad/s**. Range limits describe sampled training commands, **not demonstrated tracked speeds**.
- \`LITE3_WALK_COLLISION\` disables all collision geometries except four shank-foot names. Self collisions/body-floor impacts are therefore not represented as full hardware contact behavior.
- GPU training stack pinned by upstream: Python 3.11, \`mjlab==1.2.0\`, \`mujoco==3.6.0\`, \`mujoco-warp==3.6.0\`, \`warp-lang==1.12.1\`. Official path documents Ubuntu with NVIDIA. Mac CPU may do read-only preflight or standalone ONNX inference **after** verifying policy input contract, but it is not a guaranteed replacement for native training.

## Reproducible read-only S0 audit (requires Python 3.10+ and Git)

Clone exact upstream source into a scratch directory, **do not commit manufacturer meshes to our repository**:

\`\`\`bash
git clone --filter=blob:none https://github.com/LuwuDynamics/luwu_mjlab.git /tmp/luwu-mjlab-lite3
git -C /tmp/luwu-mjlab-lite3 checkout 447945fd40fb5585271dce4ea8cc82567916032a
python scripts/research/xgo_lite3_audit.py \
  --xml /tmp/luwu-mjlab-lite3/src/assets/robots/lite3/xmls/lite3.xml \
  --env /tmp/luwu-mjlab-lite3/src/tasks/velocity/config/lite3_walk/env_cfgs.py \
  --strict-upstream-sha \
  --output /tmp/lite3_source_audit.json
python -m unittest discover -s tests -p 'test_xgo_lite3_audit.py' -v
\`\`\`

The audit runs no MuJoCo/actuation; expected S0 results: joint_count=14, actuator_count=14, leg_joint_count=12, model_mass_kg=0.345, nominal_policy_hz=50, source blob exact. Unit tests (4/4) passed in this ChatGPT container on the original auditor implementation and synthetic XML. **Executing against downloaded upstream files is pending**.

## S1 physical acceptance protocol (not yet executed)

1. Pin exact Hugging Face model revision, download \`lite3_walk/policy.onnx\`, \`model_1499.pt\`, and matching YAMLs; verify SHA256, ONNX actor input/output shape, action order, observation normalization, default pose, actuator clip, latency, and task version compatibility. Community policy is not necessarily vendor-approved.
2. Install native upstream pinned mjlab on a compatible machine; run the exact task with one environment and 10 fixed seeds, including policy, zero-action, and hold-pose baselines. Same model, seeds, commands for fair comparisons.
3. Episode scripts: stand, forward, lateral, yaw, mid-stride command change, stop; collect per-step world root/COM movement, per-foot real contact force, slips, pitch/roll, saturation and falls. No fabricated physics or browser-only animation.
4. Separate sensitivity to friction, mass/inertia mismatch, force/servo cap; any modified dynamics must be labeled as assumptions and cannot serve as evidence of hardware success.
5. **Only after real recorded frames** export an immutable XGO-Lite3 replay for existing Android Hexapod Lab viewer; no changes to original CAD30, no merges or hardware actuation without owner approval.

## S0 gates

| Gate | Status |
| --- | --- |
| Upstream source inspected | **PASS** |
| Read-only auditor tests | **4/4 PASS** |
| Full upstream asset hash/mesh load | NOT RUN |
| Model/hardware parity | **GAP** |
| ONNX inference | NOT RUN |
| Native physics rollout | NOT RUN |
| Real hardware | NOT RUN |

No third-party binaries, STL, model exports, or policy weights included in this branch. Upstream \`luwu_mjlab\` license not established by a top-level LICENSE in the inspected revision; the community HF policy card declares Apache-2.0. Source metadata only are committed here.
