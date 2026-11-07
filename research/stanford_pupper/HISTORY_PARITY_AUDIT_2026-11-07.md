# Stanford Pupper v3: source contract and recovered-Drive CPU physics (2026-11-07)

**Status:** `MUJOCO_RESTORED_FROM_DRIVE / BUFFER_CLEANUP_OUTPUT_EQUIVALENT / 5S_EXACT_REPLAY_PASS / EXTENDED_ROLLOUTS_COMPLETE / NATIVE_CPP_PARITY_OPEN`.

Based on research PR #73 @ `549b6ecaa1b6fed63a69b0cf42d8b1531dbc9e18`; this draft PR #74 is stacked over it. No changes to `main`, viewer or physical robot.

## Source-history review: correction of our initial diagnosis

The Stanford policy declares 20 newest-first frames x 48 dimensions. Within each frame:
- `last_action[24:36]`
- `gait_reference[36:48]`

The old Python controller rotated `history[0]` into `history[1]` **before** incorrectly writing `last_action` into `history[0,36:48]`. On the next control tick, `obs()` overwrote the *whole* `history[0]` before network inference. Therefore this wrong scratch-buffer write **did not contaminate the history consumed by the network**. It was dead code rather than a trajectory-changing observation defect. Our original claim that the 46.9 cm result was invalid because of this write was **incorrect**.

The new helper `history_contract.advance_history()` follows the native C++ sequence: rotate all 20 frames and write last action at offsets `[24:36]` of the next frame. The helper is clearer and independently tested against the C++ history rotation (960/960 equal) but has the same network inputs and physical result as the old code. Original [neural_controller.cpp](https://github.com/cs123-stanford/pupper_gait_deploy/blob/70df56dd47e98195b7d51fe7ff7194a2581b0e0d/src/neural_controller.cpp), [obs_spec.hpp](https://github.com/cs123-stanford/pupper_gait_deploy/blob/70df56dd47e98195b7d51fe7ff7194a2581b0e0d/src/obs_spec.hpp).

## Environment actually recovered through owner Google Drive

- Offline Linux x86-64 / Python 3.13 wheelhouse in [MuJoCo-3.14.0-cp313-linux-x64](https://drive.google.com/drive/folders/1n0qHoHFFivqQVGw-DazJqq_0nhHkk4VK), archive SHA-256 `6d14d0c17294bac5436e1167ca4a97c30061da8efaa0a2b904c171bcac4a7b13`. Checked external SHA-256 and all 10 internal wheel checksums, installed MuJoCo 3.14.0 offline in isolated local site packages. NumPy 2.3.5 retained from original Python runtime.
- Original pinned MJCF and pretrained JSON from two archives in [Alfred-Hexapod-Lab](https://drive.google.com/drive/folders/1QB_AOZuxteCA8FA0tAqyIwV2-ET75MQM). MJCF Git SHA-1 blob `7771e176c42b3aed1d48cc62434771b0fceb7c8c`, policy SHA-256 `ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712` both independently checked.
- Original MuJoCo CPU model: 12 position actuators, 3.17 kg, four feet, dt 0.004s, kp=5.5, kd=.25, |torque| ≤3Nm; 13 noncolliding visual mesh geoms dropped for physics only, original inertia/collision geometry retained.
- Offline reconstruction of the reviewed PR #73/74 CPU runner; the old-path 5s output matched the archived original JSON **byte-for-byte** with SHA-256 `662a6c60f4a283cdf2ce5d18bcbfd49553d12615e853d8e040a20eeba899f573`. The fixed-helper path produced the **same SHA-256**. This is strong independent reproduction of the original experiment, not merely a unit-test assertion.
- Original 5s measured movement: X +0.46891788m, final body height 0.15884779m, max torque 1.7871357496Nm, 0 fall-guard triggers, 0 torque saturation. Contact frame counts from 250 recorded 20ms frames (>0.2N per foot): 0-foot 3, 1-foot 0, 2-foot 99, 3-foot 25, 4-foot 123. This is not proof of strict four-support crawl or hardware robustness.

## Extended same-model exploratory runs (no training)

All use the same original policy and geometry, 1s warmup, 1s action fade-in, commanded velocity until last second, and fixed four-physics-step actuator delay. These are single-scenario deterministic physical integrations, not robustness statistics.

| case | duration | commanded vx | root net X | fall-guard ticks | max tilt | max torque | saturation fraction |
|---|---:|---:|---:|---:|---:|---:|---:|
| Stand | 10s | 0 | +0.00144474m | 0 | 0.8428° | 1.10445Nm | 0 |
| Reverse | 15s | -0.20m/s | -1.31365553m | 0 | 19.1457° | 3.0Nm | 0.0005333 |
| Forward long | 30s | +0.20m/s | +1.35130310m | 0 | 23.7168° | 3.0Nm | 0.002 |
| Forward fast | 15s | +0.40m/s | +1.92987487m | **319** | **177.6648°** | 3.0Nm | 0.02827 |

**Interpretation:** the fast command physically falls despite large net displacement. For the 30s forward run, only 169/1500 recorded frames have four feet loaded, 1035 have two loaded; dynamic trot is not quasi-static four-foot contact. No single-scene result establishes a safe command envelope.

## Validation gate, limitations, and next step

- The new `test_pretrained_cpu.py` now correctly expects the exact original 5s summary, frames and policy events, not artificial inequality.
- The history-buffer C++ unit test is **not** full native C++ neural-controller parity. Command clip, heading hold, IMU projections, observation clipping, policy cadence and RTNeural weight-layout parity remain to be tested against the native reference.
- Run matched native C++/mjlab A/B, then seeded perturbations (friction, actuator delay, mass, pitch/roll, speed sweep), and only then define a bounded speed envelope.
- No new training; no hardware tests; keep this a **Draft** pending owner approval. The newly measured exploratory traces are local research receipts, not automatically published to the viewer.
