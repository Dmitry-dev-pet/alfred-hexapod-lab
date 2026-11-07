# Stanford Pupper v3 — full command physical evaluation

**Date:** 2026-11-07. **State:** independent CPU MuJoCo reproduction of the pinned pretrained Stanford deploy policy, **not** native C++/MJLab parity and **not** hardware qualification. No RL training was performed. Original research PRs: [#73](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/73), [#74](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/74).

## Exact environment and protocol

- Original Stanford v3 MJCF pinned Git blob SHA1 `7771e176c42b3aed1d48cc62434771b0fceb7c8c` and default trained-policy SHA256 `ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712` were restored in the current working container from the owner's [Drive project folder](https://drive.google.com/drive/folders/1QB_AOZuxteCA8FA0tAqyIwV2-ET75MQM).
- MuJoCo 3.14.0, Python 3.13.5, NumPy 2.2.4; isolated CPU environment reconstructed using the owner's [offline wheelhouse on Google Drive](https://drive.google.com/drive/folders/1n0qHoHFFivqQVGw-DazJqq_0nhHkk4VK). 250 Hz physical integrator, 50 Hz neural policy inference; model mass 3.17 kg, 12 servo actuators with Kp 5.5 / Kd 0.25, torque limited to ±3 Nm, four-physics-step delayed servo targets. No synthetic root movement or reset between commands.
- Commands are `vx, vy` (m/s), `wz` (rad/s), run button, rising-edge jump button, and desired roll/pitch pose in radians. Source-declared raw bounds: `vx ∈ [-1.5,1.5]`, `vy ∈ [-0.5,0.5]`, `wz ∈ [-2,2]`; game-mode walk cap on both linear components ±0.49, run cap ±1.5; heading hold when yaw command quiet. Body orientation read through actual MuJoCo state.
- Source-aligned implementations of reference-gait lookup, heading hold, jump/hop slot on 0.75s gait grid, command clipping, 20×48 history, fade-in and stop. These are **local reconstructions**. Stanford C++ neural network parity, observation saturation and full native MJLab sensor/actuator delays have **not** been independently verified.
- Each scenario starts from an identical neutral state on a flat floor. `NO_FALL_GUARD` means tilt never exceeded 45° and torso height never fell below 0.05m; it is **not** robust-locomotion validation. All 39 trajectories are independent, deterministic, unperturbed single-scenario runs, not 39 randomized robustness seeds.

## Overall outcome

- **39 scenarios**; **360 seconds** of simulated motion; **90,000 `mj_step`** physics ticks; **28 did not trigger the fall guard**, **11 did**. No numerical-integration aborts.
- **6/6 local control tests passed**, including a deterministic same-trajectory replay; separate 20s switched-joystick and 9s jump experiments were repeated, exact byte-for-byte JSON SHA-256 confirmed for each.
- Jump at rest: trigger t=3.0s, scheduled execution t=3.25s, maximum torso height **0.35248m**, final torso height **0.15919m**, no fall guard; 0 loaded feet for a total of ~0.46s among 20ms recording frames after the jump. This is a physically integrated single jump, not a promise of repeatable landing.
- Turning **overshoots** dramatically: +0.45 rad/s for 7s (command integral ~+180.5°) produced +390.39° actual yaw; -0.45 commanded gave -374.72°. One closed-loop path with quick command switches completed 20s without triggering the fall guard, but control fidelity is not yet acceptable.
- High-speed gallop/run `.50–.65 m/s` often falls. Early running hop (speed .51 or .55) activates before the eventual fall, but sustained running is not stable.

## Representative command results

| Scenario | Net world X (m) | Net world Y (m) | Unwrapped yaw (°) | Max tilt (°) | Fall-guard steps | Status |
|---|---:|---:|---:|---:|---:|---|
| `stand` | +0.001 | +0.001 | +0.2 | 0.8 | 0 | NO_FALL_GUARD |
| `forward` | +1.312 | +0.017 | -14.1 | 15.7 | 0 | NO_FALL_GUARD |
| `reverse` | -0.164 | +0.103 | -134.9 | 178.2 | 450 | FALL |
| `strafe_left` | +0.013 | +0.687 | -8.6 | 7.6 | 0 | NO_FALL_GUARD |
| `strafe_right` | +0.039 | -0.508 | +2.0 | 13.6 | 0 | NO_FALL_GUARD |
| `yaw_ccw` | +0.062 | +0.046 | +390.4 | 14.6 | 0 | NO_FALL_GUARD |
| `yaw_cw` | -0.147 | -0.168 | -374.7 | 33.7 | 0 | NO_FALL_GUARD |
| `diagonal` | +1.180 | +0.388 | -16.4 | 9.5 | 0 | NO_FALL_GUARD |
| `arc_left` | -0.073 | +0.446 | +213.9 | 27.5 | 0 | NO_FALL_GUARD |
| `arc_right` | -0.117 | +0.071 | -357.1 | 21.8 | 0 | NO_FALL_GUARD |
| `mixed_3axis` | -0.065 | -0.065 | +305.3 | 15.6 | 0 | NO_FALL_GUARD |
| `run_forward` | +1.168 | +0.007 | -162.3 | 179.2 | 1553 | FALL |
| `realtime_switch` | +1.062 | +0.950 | +105.1 | 27.1 | 0 | NO_FALL_GUARD |
| `rapid_joystick` | +0.188 | +0.046 | -15.4 | 17.5 | 0 | NO_FALL_GUARD |
| `pose_roll_pitch` | +0.469 | +0.023 | +16.7 | 9.6 | 0 | NO_FALL_GUARD |
| `jump_stand` | -0.047 | -0.006 | -20.9 | 34.0 | 0 | NO_FALL_GUARD |
| `hop_running` | +1.117 | +0.140 | -144.4 | 179.9 | 2133 | FALL |
| `hop_early_0_55` | +1.488 | +1.080 | +271.0 | 179.3 | 834 | FALL |
| `clip_limits` | +0.069 | +0.141 | +64.3 | 15.4 | 0 | NO_FALL_GUARD |

## Critical A/B observations

- **Reverse is non-monotonic:** `vx=-.08` triggered a fall at ~5.924s, `-.12` and `-.16` completed 8s without a fall trigger, `-.20` triggered a fall at 7.200s. A no-heading-hold run at `-.20` completed 9s but wandered by -109.1° yaw, so disabling course hold alone does not solve directional control.
- Run speed `.50` reached large displacement before falling at 7.284s; `.52` fell at 4.328s, `.55` at 3.436s, `.60` at 3.984s. `hop_early_0_55` scheduled a hop at 2.50s and later fell at 4.668s. `hop_running` (longer) already fell at 3.472s *before* the queued 4.75s hop, so it cannot be counted as a stable running-hop success.
- `clip_limits` requested `(vx=3, vy=1, wz=3)`; emitted `(0.49,0.49,2.0)`, demonstrating exact sequential trained bound + walk-cap filtering.
- **No claim of velocity tracking:** these are actual world-frame net displacements; robot-frame step velocity is not inferred from final XY alone, especially during rotation. Yaw is unwrapped continuously from the physically simulated quaternion.
- Rapid joystick scenario applies 14 commands in 10s, including changes every 0.52s without resetting the model; fall guard never fires. This establishes responsiveness in one modeled trajectory, not general reliability.

## Reproduce from PR #75 (stacked on #74)

```bash
python -m unittest discover -s research/stanford_pupper -p 'test_full_controls.py' -v
python research/stanford_pupper/full_controls.py --suite extended --output-dir /tmp/pupper_all_commands
```

Use the existing `research/stanford_pupper/default_policy.json` and `upstream/pupper_v3.xml` pinned assets from PR #73 and the restored CPU environment. `--suite core` runs the 16 principal scenarios, `--case reverse` picks one case. The source code, this report, and original evidence receipts are also packaged in a Drive ZIP. The raw rollout JSON files are evidence and are *not* committed to the GitHub source repository.

**Next scientific gate:** test exact forward inference against native Stanford C++ and MJLab; diagnose yaw overshoot using an explicit body-frame response model; validate a controlled command envelope on perturbed mass, friction, initial tilt and actuator delay. Leave PR Draft and the actual robot untouched.
