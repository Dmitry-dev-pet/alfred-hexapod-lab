# Stanford Pupper v3: observation history parity audit (2026-11-07)

**Status:** `HISTORY_INDEX_BUG_CONFIRMED / HISTORY_ROTATION_PATCHED / CORRECTED_PHYSICS_NOT_RUN`.

This is a follow-up draft to [PR #73](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/73), based on its fixed commit `549b6ecaa1b6fed63a69b0cf42d8b1531dbc9e18`. It does **not** revise or re-label the original recorded physics and does **not** modify main, the Android viewer or real robot.

## Error isolated

The exact Stanford policy `obs_spec` is 20 newest-first frames, each 48 numbers:
- `base_ang_vel[0:3]`, `projected_gravity[3:6]`, `command[6:9]`, `desired_world_z[9:12]`
- `joint_pos_rel[12:24]`, `last_action[24:36]`, `gait_reference[36:48]`.

Our `infer_pupper.py` previously did this after inference:

```python
history[0,36:48]=last_action
history[1:]=history[:-1].copy()
```

This **replaced the gait reference of the newest frame with the previous action before moving that frame into history**. A 20-frame input still had the correct dimensions and deterministic output but did not obey the policy's trained observation contract. Thus the 5-second `+0.46891788 m` measured motion is **historically real for the buggy Python controller, not a validated Stanford policy walk**.

Original source: [Stanford C++ neural_controller.cpp @ 70df56d](https://github.com/cs123-stanford/pupper_gait_deploy/blob/70df56dd47e98195b7d51fe7ff7194a2581b0e0d/src/neural_controller.cpp), especially the gait reference before inference (lines ~982-1024), `std::rotate` after inference (lines ~1109-1112), and `last_action` written at the declared offset in the *next* frame (lines ~1127-1141). Original [obs_spec.hpp](https://github.com/cs123-stanford/pupper_gait_deploy/blob/70df56dd47e98195b7d51fe7ff7194a2581b0e0d/src/obs_spec.hpp).

## Patch

`history_contract.py` now rotates complete 20x48 frames in C++ order, then writes the faded action into the new newest frame `[24:36]`, **never** into `[36:48]`. The Python `obs()` populates that newest frame afresh on the next control tick, with the most recent action and proper current gait reference. `infer_pupper.py` calls `advance_history` after each policy inference.

`test_history_contract.py` checks all frame positions, reference preservation and invalid shape rejection. `test_history_cpp_parity.py` compiles a small, faithful C++14 `std::rotate` + last-action-index test; its 960 float results are compared elementwise to the NumPy implementation.

**Actually executed in the current isolated CPU environment:** 2/2 Python history tests PASS (0.006 s); standalone compiled C++14/NumPy 960/960 values identical. These are observation-buffer tests only. This environment lacks the `mujoco` package and cannot fetch wheels, so **no corrected physical simulation was executed here**. This does **not** demonstrate full native C++ inference parity.

## Re-evaluation gate

1. Preserve the original `trial_5s.json` (SHA-256 `662a6c60f4a283cdf2ce5d18bcbfd49553d12615e853d8e040a20eeba899f573`), `pupper-pretrained-5s-replay.json`, and `PRETRAINED_CPU_RESULT_2026-11-07.json` as *legacy buggy-controller evidence*. Do not overwrite or republish them as corrected.
2. On a CPU machine with pinned `mujoco==3.14.0` and `numpy==2.3.5`, run fast history tests, then the patched 5-second trajectory:
   ```bash
   python -m unittest discover -s research/stanford_pupper -p 'test_history*.py' -v
   python research/stanford_pupper/infer_pupper.py \
     --policy research/stanford_pupper/default_policy.json \
     --duration 5 --vx 0.2 --delay 4 \
     --output /tmp/pupper_history_corrected_5s.json
   python -m unittest discover -s research/stanford_pupper -p 'test_pretrained_cpu.py' -v
   ```
3. New immutable JSON receipt must include exact pinned source/blob and software versions, SHA-256 of the corrected trace, actual displacement and velocity per second, upright/tilt and contact histogram (0,1,2,3,4 feet loaded), saturation and stop drift. Use a **new filename and new viewer library entry only after** these gates. No manual root-position or root-velocity scripting.
4. Compare *same input observations* against Stanford deploy C++ outputs, and separately compare command-filter/heading-hold behavior, IMU frames, observation clipping, policy cadence and 16ms actuation delay. Only after those gates call it source-parity validated. Then try multiple commands/terrain perturbations and longer walking.
5. Keep Draft; no merge or robot actuation without owner approval. Do not create a new runner or use GitHub Actions for physics.

## Existing 5-second trace diagnostics (the superseded controller)

From 250 original 20ms frames: 3 frames with 0 loaded feet (initial settling), 99 with 2, 25 with 3 and 123 with 4, threshold >0.2 N; maximum tilt 8.9644°, final height 0.15884779 m; root X at t=1s -0.00682m, t=2s 0.06278m, t=3s 0.27191m, t=4s 0.45104m, t=5s 0.46892m. These numbers describe a reproducible **outdated** program and cannot establish robust walking.
