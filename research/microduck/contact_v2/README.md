# MicroDuck body–step contact correction v2

The user reported torso/step interpenetration. The legacy 5 cm trial has original visible `left_shell` surface vertices **10.336 mm inside the step** and a convex-hull signed overlap up to **11.086 mm**. Both side shells were visual-only. The earlier 6.277 mm self-contact figure measured collisions between robot parts and did not quantify this bug. That distinction is now explicit, and the earlier 1 cm partial edge-stance claim is withdrawn.

## Correction

All 70 visible original-STL instances now have coincident environment collision geometry. Ten were already covered; sixty missing mesh instances are added as massless geoms on welded children of the corresponding moving link. The new collision bit is accepted by floor/step only: no new robot-to-robot collision pairs are enabled. This avoids spurious assembly self-collisions from internal parts. Original collision pairs remain unchanged.

The visible meshes are untouched. All original body masses, inertia tensors, inertia frames, joint ranges, mesh vertices, actuators and DOFs are preserved. Robot mass is 0.73724318 kg; its complete 34 x 34 generalized inertia matrix is exactly identical at the checked common state. Fifteen additional welded bodies have zero mass and no joints. Tests also verify that each new collision proxy follows its visible part in ten moving configurations.

Contact solving uses a 0.5 ms physics step, `solref=[0.002,1]`, `solimp=[0.99,0.999,0.0001,0.5,2]`; the v8 policy still runs every 20 ms. These settings reduce transient contact penetration rather than concealing surfaces in the viewer. Same 1.2 s authored backward-hip command, encoder correction, target margin, policy weights, supply voltage and BAM model. No root repositioning, external impulses or changed gravity.

## Corrected outcomes

Nine episodes replace the eight legacy wall clips on the existing `/hexapod/microduck-wall-v1/` route. They cover close placements at heights 1/2/3/5/10/25 cm, the former placements at 1/2 cm, and a flat-floor control. All are genuine newly computed trajectories.

The earlier 1 cm partial edge stance **does not recur**. No trial reaches the complete-sole upper-platform goal. The 1–5 cm close trials remain lying; 10/25 cm return to standing below. The flat backward-fall/get-up control still passes. These are fixed-command outcomes, not height limits or hardware reliability estimates.

In the formerly problematic 5 cm case, maximum torso-to-step contact penetration is **0.28469 mm over all 0.5 ms physics substeps**. Across all nine episodes that is also the maximum torso–step value. Across all robot parts, the maximum step contact penetration is 1.06776 mm during a transient. These are separate metrics. At saved 50 Hz frames, the largest torso convex overlap is only 0.01596 mm. All nine have finite trajectories with zero initial obstacle contact, physics reset or warning.

## Validation

- `audit_model.py`: all 70 visible parts have matching geometry, pose and moving attachment for environment contact; 60 additions; original physical parameters and mass matrix preserved; zero new self-collision pairs.
- `verify_contacts.py`: independent signed convex-geometry queries and actual surface-vertex checks against the step, reference-bug reproduction, legal target ranges, complete-sole goal checks and two byte-identical nominal repeats (5 cm close and flat).
- `verify_playback.py`: checks the actual exported position/quaternion interpolation at quarter, half and three-quarter points of every frame interval for every torso mesh in all eight wall recordings. See `playback-verification.json` for the measured maximum. Thus keyframe correctness is not used as a substitute for checking the player's interpolated geometry.
- `export_contact.py`: exports 4,500 frames x 70 parts from saved physics states. Original STL geometry package is retained; visible-part ordering is mapped to the new physics geom IDs. Per-part poses and unit quaternions are SHA-verified in the client. New binary magic/schema/cache namespace prevent mixing legacy recordings with corrected metadata.
- Five Node client tests cover all nine recordings, truncation/hash/header errors, unsafe paths, offline cache, required contact-model identity and withdrawal of the legacy edge-stance claim.
- `render_comparison.py`: actual MuJoCo/EGL renders of the original STLs at t=7.92 s, before and after, with a translucent step for inspection. Floor reflections are disabled only in this diagnostic rendering. `body-step-before-after.png` is a rendered simulation comparison, not a generated illustration. Cloud-browser WebGL remains unavailable; browser data/controls are checked separately.

## Limits and archived results

MuJoCo mesh collision shapes here are **convex hulls** of the original STL instances. They can fill real holes or recesses and therefore conservatively block motion that might physically fit. This is an environment-collision correction, not calibrated deformable contact or complete assembly self-collision validation. Impacts and peak forces are uncalibrated, and hardware/noise robustness is untested. Small numerical penetration remains; no claim of mathematically zero overlap is made.

Legacy step/roll/height/fall pages remain as explicitly marked archives for comparison. Their earlier step-passing results are not confirmed under this correction. Legacy wall source/data remain in https://drive.google.com/file/d/1-Ulb0XAyYANk5MVXZ3tIo5lXagBsvuwt/view (SHA256 `afd190249567546a39c84197936444715fa895947388ab5371fc271a5aa826af`). They are not the corrected current viewer defaults.

## Reproduction

Use Python 3.12 and the included pinned requirements. Restore the original source archive https://drive.google.com/file/d/1S7FgYnz_Y31OedO98oUUC_Gp1loifPqz/view (SHA256 `633f948b3340fb6c63087e84a25b33e6f69d6025ae63a4c5384f09befdb9a10e`), then overlay this archive's relative paths. Install the pinned BAM source and retain the restored workspace layout. Commands:

```
python microduck-contact/contact_cpu.py
python microduck-contact/audit_model.py
python microduck-contact/verify_contacts.py
python microduck-contact/export_contact.py
python microduck-contact/verify_playback.py
MUJOCO_GL=egl python microduck-contact/render_comparison.py
node --test microduck-publish/site/tests/test_microduck_wall_replays.mjs
```

Upstream RL `bae39f5900b318f7e87fc485c30a31213cfa14eb`; BAM `62bd8ce12154340be97e06f7f41a0ca8f116d967`; v8 stand policy SHA256 `2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492`. The source/installed-BAM blob gate runs before simulation. Geometry package SHA256 `e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b`, same 38 STLs and 70 visible instances as before. Geometry: Pollen Robotics, CC BY-NC-SA 4.0; original code/policies Apache-2.0. Retain upstream notices.
