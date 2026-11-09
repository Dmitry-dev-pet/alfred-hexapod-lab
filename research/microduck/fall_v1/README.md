# MicroDuck: commanded fall and existing get-up, 2026-11-09

A bounded CPU experiment: drive a legal joint pose for 1.2 seconds (0.25 s smooth ramp), then hand control to the pinned v8 `velstand.onnx`. No root-state edit after the initial STAND setup, external force, injected momentum, gravity change or new learned policy. This tests falling and recovery, not a complete 360-degree roll.

## Observed results

| Direction | Flat search | 10 mm step | 20 mm step |
| --- | --- | --- | --- |
| Backward | 2 / 6 directed fall + recovery | Recovered below platform | Recovered below platform |
| Left | 2 / 6 directed fall + recovery | Both complete soles above edge; stable last second | Recovered; upper-platform goal failed |
| Right | 2 / 6 directed fall + recovery | Recovered; upper-platform goal failed | Recovered; upper-platform goal failed |

The flat counts describe candidate commands, **not repeated-run reliability**. Four representative episodes repeated byte-for-byte: `flat-back-4`, `flat-left-4`, `flat-right-4`, `step10-left`. Noise, terrain variation and hardware robustness were not tested. Backward flat recovery also drifted sideways (~16 cm for candidate 4), so this is not precise displacement control. Left/right step outcomes are asymmetric despite mirrored commands and require further investigation, not a claim of a hardware asymmetry.

All six step trials use the first successful flat candidate in each direction, an 8 cm edge and the same fixed command timing. The platform always lies along +world X; the robot's **initial** yaw is pi (back), -pi/2 (left), +pi/2 (right). The terrain is thus behind or beside the robot in body coordinates. Simulation handles the entire movement after setup. Initial yaw is recorded and included in every quaternion track.

## Criteria and selection

The first >=65-degree fall must lie within a 45-degree horizontal cone around the requested body-relative direction, and a non-foot body collision with the environment must occur while tilt >45 degrees. Final recovery requires the last 50 frames (one second) all to have tilt <20 degrees, trunk height 0.09..0.18+step_height m, speed <0.06 m/s, angular speed <0.7 rad/s and both feet carrying >0.05 N on the environment. Upper-platform success additionally requires both entire sole meshes beyond x=0.081 m and both feet carrying >0.05 N on the top surface throughout that second. A diagonal forward fall is excluded even if recovery works.

Original joint ranges are retained. All 14 commanded motor targets, including recovery outputs, are clamped 2 degrees inside each rotor joint range. The ONNX last-action observation is updated to the actually sent target relative to its default pose during manual control and recovery. No action delay is enabled. Neural weights and observation encoder correction are unchanged.

## Model limits and audit

Original 38 STL files / 70 visible instances; collision processing uses 11 sparse original robot collision meshes, not all 70 rendered instances. The additional `power_support` mask is type 2; the other environment-contact meshes are type 1. Masks, masses, inertia, joint ranges and exclusions are unchanged. Initial STAND has no collision contacts or explicit contact exclusions. This does not establish that every visible plastic part is protected against self-intersection.

BAM XL330 M6, firmware Kp=200, 7.4 V with existing 0.1 V/Nm sag and 6 V floor; no firmware current limit, matching the inherited simulation. Firm-contact profile `solref=[0.005,1]`, 1 ms physics and 20 ms control. MuJoCo limits are soft: maximum rotor overshoot across all 24 trials was 1.139 degrees despite legal targets. Maximum sparse self-contact penetration was 0.893 mm; environment penetration 3.022 mm. Peak contact forces are uncalibrated. No physics resets, nonfinite states or MuJoCo warnings occurred. No real robot was tested.

## Source and restoration

- RL commit `bae39f5900b318f7e87fc485c30a31213cfa14eb`; BAM commit `62bd8ce12154340be97e06f7f41a0ca8f116d967`.
- v8 stand policy SHA256 `2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492`.
- Original geometry package SHA256 `e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b`.
- Heavy pinned source archive: https://drive.google.com/file/d/1S7FgYnz_Y31OedO98oUUC_Gp1loifPqz/view (SHA256 `633f948b3340fb6c63087e84a25b33e6f69d6025ae63a4c5384f09befdb9a10e`). Restore its pinned RL/BAM directories and source manifest beneath `microduck-work/`, then overlay this archive's relative paths. Install BAM from that pinned source in the supplied requirements environment.
- Run from the restored workspace root: `python microduck-fall/audit.py`, `python microduck-fall/fall_cpu.py`, `python microduck-fall/fall_cpu.py --steps`, `python microduck-fall/verify_falls.py`. Original source/BAM blob verification runs before the search.
- `export_falls.py` uses saved states without regenerating motion. It requires the shared original geometry package under `microduck-publish/site/dist/assets/robot-models/`.

Every raw trial is preserved, including commands in the opposite direction, insufficient tilts, and the one backward trial that did not recover. Interactive replay tracks store all 70 original-part poses at 50 Hz, one SHA-verified binary per selection. Numeric export error <1e-6. Browser tests cover every clip, invalid content, unsafe paths and offline cached replay. Cloud-browser WebGL is unavailable: 3D rendering and Android multitouch were not visually verified here.

Geometry: Pollen Robotics, CC BY-NC-SA 4.0. Original software/policies: Apache-2.0; see pinned source notices. This report concerns simulation only.
