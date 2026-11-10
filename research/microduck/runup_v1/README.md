# MicroDuck: forward roulade after a motor-driven approach

Bounded local CPU experiment requested by the owner on 2026-11-09. Use the existing player at `/hexapod/microduck-runup-roll-v1/`. No real hardware was tested, no policy training or external root impulses were used.

## Observations

Twelve episodes contain 7,600 samples at 50 Hz. Both flat controls complete a forward revolution and regain a stable stance. None of the ten 5 cm step episodes reaches a stable stance with both complete soles on the upper platform. Eight step episodes actually launch the roulade; the initial two short 4.5 s approach trials stop before their trigger and are retained as timeouts. These outcomes do not establish the robot's maximum climbable height.

The moving flat control and the 1 cm-gap step trial have exactly identical positions, velocities, motor controls and targets through the trigger at 4.60 s (230 recorded samples). The flat control then completes ~364 degrees; the step blocks the active roulade near 75–80 degrees. Later recovery can rotate the body further, so the index exposes both the maximum during the active roll and the total maximum. A 330-degree accumulated local forward rotation plus loaded head pivot is the roll classifier; it does not imply continuous ground contact. Launch conditions include 10/6/3/1 cm toe gaps and first loaded vertical-face contact. Extending the original 2 s roulade command to 3 and 4 s does not clear this step.

Commanded approach speed is 0.24 or 0.30 m/s; it must not be reported as achieved speed. Gait oscillation makes instantaneous trigger velocity misleading, so the player displays mean forward root displacement over the last approximately 0.25 s before the trigger. These approaches are slow (typically 0.05–0.08 m/s).

## Model and execution

Original model: `pollen-robotics/microduck_rl@bae39f5900b318f7e87fc485c30a31213cfa14eb`; BAM `62bd8ce12154340be97e06f7f41a0ca8f116d967`. Original mass 0.73724318 kg, 14 actuators, all 70 original visible mesh instances and 38 STL files. The contact-v2 environment-only correction covers all visible meshes with convex hulls, adding 60 massless environment geoms. Original masses, inertia, joints and actuators are retained. Convex hulls can fill cavities. Impact forces/contact parameters have not been calibrated against hardware.

Physics runs in MuJoCo 3.10.0 with a 0.5 ms step; original ONNX inference/BAM command update is every 20 ms. Motor targets are clamped to original joint limits with a 0.035 rad margin, and the policy's previous-action observation follows the clamped target. Encoder feedback includes motor and passive-backlash motion. Root orientation starts at yaw zero, settled STAND; only motors create the subsequent movement. Walking command is zeroed when upstream `trigger_behavior('roulade')` swaps the ONNX policy. The normal standing policy resumes at trigger + duration - 0.02 s, matching upstream update ordering.

- Standing ONNX SHA-256: `2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492`.
- Forward roulade ONNX SHA-256: `3d60da08fc13f29c1b57f41977aa898132c0d60042100149d8e775affcbca32b`, v8 commit `665ae4eb20bfe13bca1ef4ea6d4bb6b4a096e448`.
- Shared original geometry package SHA-256: `e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b`.

A step success requires the last second to have quiet upright motion, both feet loaded on top, and the complete two sole bounds beyond edge + 1 mm. Stable standing below the step is not a climb. All twelve quality gates pass, with no MuJoCo warning or physics reset. Torso/step overlap is zero throughout these trials; peak overlap among all step-contact parts is 0.991 mm. Sparse original self-contact penetration is below 0.358 mm.

## Verification and replay

`verification.json` records independent quaternion rotation checks (maximum sampled integration disagreement 0.364 degrees), independent success assessment, matched approach, and two byte-identical complete repeats. `playback-verification.json` checks 191,700 torso/step distances at 1/4, 1/2 and 3/4 of actual linear-position/slerp-quaternion playback intervals: no torso/step overlap. Source SHA/geometry order and physical invariants are checked by the exporter. Four Node tests decode all clips and test corrupt files, index mismatches, varying durations and offline reuse.

Binary tracks use `MDRUNP1\0`, a 16-byte header and little-endian float32 rows of stride 498: time, root xyz/wxyz, then 70 original geom xyz/wxyz poses. There are 500/700/800 frames per 10/14/16 s clip. The maximum float32 absolute conversion error is 4.58e-7. Actual MuJoCo original-STL renders show the matched comparison; browser WebGL rendering is separately subject to browser capability.

## Restore

The result ZIP preserves raw traces, policies, helper hierarchy, source manifest, shared geometry and exported tracks. Restore pinned RL/BAM directories from the full source archive on project Drive: file `1S7FgYnz_Y31OedO98oUUC_Gp1loifPqz`, SHA-256 `633f948b3340fb6c63087e84a25b33e6f69d6025ae63a4c5384f09befdb9a10e`; verify its manifest. Overlay the result ZIP in that workspace, create a Python 3.12 virtualenv and install `microduck-runup/requirements.txt`.

```
python microduck-runup/runup_cpu.py
python microduck-runup/verify_runup.py
python microduck-runup/export_runup.py
python microduck-runup/verify_playback.py
MUJOCO_GL=egl python microduck-runup/render_comparison.py
node --test microduck-publish/site/tests/test_microduck_runup_replays.mjs
```

`runup_cpu.py` reuses existing exact traces; remove the trace files first for a fresh full sweep. Verification always recomputes two controls. Preserve the archive's relative hierarchy: helpers locate the workspace through `microduck-work/source-manifest.json`. For the browser, the existing player supplies Three.js, model-package loading and mobile shell assets. Heavy artifacts stay on Drive and native Site; repository contains code and compact receipts only. Code/policies follow upstream Apache-2.0; original geometry is Pollen Robotics CC-BY-NC-SA-4.0, attribution retained.

## Episode results

| Episode | Trigger, s | Roll phase maximum, degrees | Final stable stance | Step goal |
| --- | ---: | ---: | --- | --- |
| runup-flat | 1.0 | 365.3 | True | False |
| runup-6 | 3.94 | 82.3 | True | False |
| runup-3 | 4.38 | 82.4 | False | False |
| runup-1 | timeout | 0.0 | True | False |
| runup-contact | timeout | 0.0 | True | False |
| runup-1-long | 4.6000000000000005 | 78.9 | False | False |
| runup-contact-long | 4.82 | 81.9 | True | False |
| runup-contact-fast | 3.88 | 81.4 | True | False |
| runup-10-early | 2.66 | 78.4 | True | False |
| runup-flat-moving | 4.6000000000000005 | 364.5 | True | False |
| runup-1-roll3 | 4.6000000000000005 | 78.9 | True | False |
| runup-1-roll4 | 4.6000000000000005 | 78.9 | True | False |
