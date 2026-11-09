# EN01 v4 — original jump command sequence, uncalibrated physics

The source jump commands now produce two qualified brief takeoffs in the declared surrogate battery. **No case achieves a clean two-second stable landing.** These are source-command sensitivity experiments, not a calibrated reproduction of the original robot.

## What is established from the author

Pinned upstream `fuwei007/Navbot-EN01@3139f2fb32b38d10611feb905e16f8aa012f2f2e`; firmware `Src/wl_pro_robot/wl_pro_robot.ino`, Git blob `c5c69e515d7d2a22edc3779667fc3edfe4045ae9`. Exact firmware, servo writer and web UI blobs were checked locally; original files are not copied into this research commit.

The exact source `jump_loop()` function was extracted, compiled in a small C++ harness and compared with the Python command state machine for 480 iterations, including a held button and a second STOP-to-JUMP edge. Every flag and actual servo write agrees. A fresh jump extends at relative iteration 0, returns at iteration 29 and resumes normal height at iteration 190. The author calls wheel balance before the leg/jump update, so the wheel gate sees the previous flag.

| Command | Left encoder | Right encoder |
| --- | ---: | ---: |
| Normal source height 38 | 2110 | 1986 |
| Jump extension (height expression 80) | 2463 | 1632 |
| Jump return (height expression 40) | 2127 | 1968 |

The source sends Speed=0 and ACC=0 to its servo protocol writer. These bytes are preserved as source evidence; they do not establish a measured servo velocity, acceleration or internal PID. Source web UI initializes height to 38. The main loop is not run by a fixed-period jump timer. Serial, IMU, FOC, web and other work affect its wall-clock duration. There is no source-proven millisecond delay.

The README defines encoder 2048 as the fully squatted mechanical-limit pose, but it does not give that pose's CAD hinge angle. For an assumed zero CAD angle at height 38, the command increments are +0.541495/+0.543029 rad for extension and +0.026078/+0.027612 rad for return. The previous v3 jump's timing and +0.20-rad extension were different.

## Predeclared experiment

`source_jump_protocol.json` declares three assumed CAD anchors at source height 38: -0.04, 0 and +0.04 rad; four assumed main-loop periods: 1, 2, 5 and 10 ms; and a matched no-jump baseline for each combination. These are hypotheses, not measured tolerance bands or a random-seed success estimate. All 24 cases last six seconds and use 6,000 genuine CPU MuJoCo 3.14.0 steps with 300 saved frames. The jump command occurs at t=1.5s.

Geometry, contact family, gravity, 0.322-kg assumed mass, 0.14709975-Nm hip ceiling, assumed 11.6355-rad/s hip no-load derating, and 0.030-Nm wheel ceiling are unchanged from v3. No spring, extra lift force, stall-torque substitution, changed gravity or pose animation is added. Nonzero anchors initialize analytical closed-loop joints and place both wheel bottoms on the floor. The zero-anchor baseline preserves the frozen v3 initial state; its 1-ms trajectory differs from v3 stand by at most 2.6645352591003757e-14 in qpos.

Only the command state machine is ported exactly. The simulation retains our v3 bounded torque-PD servo and wheel-controller adaptation. Source flag/speed gates and distance-reference reset are applied at the declared outer-loop period. SimpleFOC voltage gains are not relabelled as Nm. The existing 35-degree latched wheel cutout remains a diagnostic guard, not a complete firmware port.

## Physical results

For the zero-anchor hypothesis, all four no-jump baselines remain upright for six seconds, with about 5.07cm maximum x displacement.

| Assumed loop period | Extension before return | Max minimum wheel clearance | Flight interval | Qualified upward takeoff | Stable landing |
| --- | ---: | ---: | ---: | --- | ---: |
| 1 ms | 29 ms | Contact retained | None | No | 0 s |
| 2 ms | 58 ms | Contact retained | None | No | 0 s |
| 5 ms | 145 ms | 3.709 mm | 47 ms | Yes | 0.157 s |
| 10 ms | 290 ms | 3.181 mm | 63 ms | No: COM already descending | 0 s |

With the +0.04-rad anchor and 5-ms loop, qualified flight lasts 64ms and maximum minimum wheel clearance is 7.098mm; clean upright two-wheel support lasts only 0.138s after landing. The -0.04-rad anchor's baselines are already unstable, so its jump trials cannot establish a valid supported takeoff.

Qualified takeoff requires preceding supported upright stance for 100ms, no ground contact by **any** robot geometry during flight, both finite-cylinder wheel bottoms clear, positive COM vertical speed above 0.05m/s at takeoff, at least 20ms flight and at least 1mm peak minimum wheel clearance. Wheel clearance accounts for cylinder tilt and half-width. A wheel-only lift while the torso touches the floor is rejected. A completed jump additionally requires at least two seconds of clean upright two-wheel support after touchdown; body impact invalidates that landing. The 10-ms cases show why wheel clearance alone does not establish a jump.

No complete jump/landing succeeds: **0/12 jump hypotheses**. There are two qualified takeoff hypotheses, not a demonstrated hardware success rate. Full traces, including unstable controls and negative cases, are retained. See `source_jump_v4_results.json` for all 24 outcomes.

## Integrity and replay

The final-source 24-case battery was repeated in full: 144,000 physical steps per batch. All raw traces, compact replays and manifests reproduce byte-for-byte. Export reconstructs all 7,200 saved qpos states through the existing independent FK exporter; maximum pose difference is zero. Eight focused tests pass, including actual original C++ comparison, absent calibration, false-flight/landing rejection and forged-body-pose rejection. Actual Three.js math checks 525,600 original CAD part transforms with maximum residual 4.440892098500626e-16 m. This is numerical validation, not a WebGL render.

The existing replay pipeline packages 24 immutable review records, is idempotent and refuses `--publish`. Its disposable review catalog preserves all 62 prior live entries and contains 86 entries. The 73-part original CAD and shared player assets are reused. No large CAD/raw traces are committed to GitHub. The live Site/catalog, prior recordings, main, and research PR #107 remain unchanged by this work. WebGL/camera/Android touch validation is not claimed. No MP4/GIF, GPU, new runner, workflow or hardware purchase.

## Reproduce

Activate a Python environment with MuJoCo 3.14.0 and numpy. From the project root:

```bash
python -m research.navbot_en01.source_jump_battery --out /path/to/fresh/dataset
python scripts/package_en01_source_jump.py --dataset /path/to/dataset --catalog /path/to/verified/library.json --out /path/to/fresh/review
```

`experiments/replay-pipeline/en01-source-jump-v4-*.json` contains all 24 existing-pipeline recipes. Source-C++ verification needs g++ and separately retrieved exact pinned upstream files; set `EN01_UPSTREAM_SOURCE_DIR` to the folder containing firmware.ino, servo.cpp and basic_web.h. Supply `EN01_JUMP_RAW_TRACE` for the optional independent-FK forgery test. Run `python -m unittest discover -s tests -p test_en01_source_jump.py -v`.

Next research barrier: actual servo/CAD calibration and MCU timing, followed by landing recovery under declared actuator assumptions. A contact-aware landing controller would be a separate adaptation and must be compared with this preserved source-sequence baseline.

Run ledger: https://github.com/Dmitry-dev-pet/truthrail/issues/314

Dataset: [24 raw physical records and existing-player replays](https://drive.google.com/file/d/1REeM7FhVl_CcE4rS_Z5v8Yz0I7TYkxmI/view?usp=drivesdk), 8,811,019 bytes; SHA-256 `43aea694c3704a59ad659674650e77268131f58ff1203e3e1e8b965a875dbd8d`. Drive metadata readback confirms ZIP type, size and target folder.
