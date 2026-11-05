# Live Locomotion V1 — six-legged CAD30 velocity control

**Issue:** https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/62  
**Date:** 2026-11-05 · **Status:** first local physical smoke PASS for *slow* commands only. Full arbitrary-velocity or hardware robustness is **not** proven.

## Components and provenance

- `hexlab/live_gait.py` — our independent pure-Python non-blocking **wave/tripod** phase generator; dynamically updates body twist `(vx,vy,wz)` each tick; joystick timeout 0.5 s, acceleration limits and foot target slew rate. **No vendor controller code copied.**
- `experiments/cad30/live_mujoco.py` — physical MuJoCo adapter for pre-existing 30-mesh CAD30 model; reuses `hexlab.stand_v04`, `hexlab.stand_geometry_v04` and `hexlab.step_one_leg_v04` IK; free-body dynamics, 500 Hz `mj_step`, bounded PD torque, 25 Hz gait/IK, 12.4-s *continuous* run. Actual contact normal reactions measured by `mj_contactForce`, not planned foot support.
- `tests/test_live_gait.py` and `tests/test_live_mujoco_contract.py` — deterministic contracts independent of third-party CAD or MuJoCo, **10/10 passed**.
- Owner-only source scene: `/source/mjcf/scene_local30_nominal.xml` and 30 original Freenove STL; **never copy** these third-party assets to GitHub. Offline runtime MuJoCo 3.14.0 recovered from owner's Drive archive, checked against SHA256 `6d14d0c17294bac5436e1167ca4a97c30061da8efaa0a2b904c171bcac4a7b13`.
- Original open-source references, **not vendored**: [Freenove](https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi/blob/b7d228cc870b0a802d3768bfcf742f85fa8695f6/Code/Server/control.py) (CC BY-NC-SA 3.0), [wk-hexapod](https://github.com/WayneKennedy/wk-hexapod/blob/9b3154cc801fdc5f910adcf54eb7d9a00d535d7d/ros2_ws/src/hexapod_controller/hexapod_controller/controller.py) (Apache 2.0 project), [OpenSHC](https://github.com/csiro-robotics/syropod_highlevel_controller) (CSIRO BSD/MIT variant).

## Reproducible commands

Run from repository root with Python 3.13, numpy, scipy, MuJoCo 3.14 and verified private CAD bundle unpacked into `experiments/cad30/source`:

```sh
python -m unittest discover -s tests -p 'test_live*.py' -v
python experiments/cad30/live_mujoco.py --source-dir experiments/cad30/source --output results/cad30_live_v1.json
# Fast negative baseline (same scripted commands, full scale and larger lift):
python experiments/cad30/live_mujoco.py --source-dir experiments/cad30/source --speed-scale 1 --foot-lift .012 --max-stride .035 --output results/cad30_live_fast_negative.json
```

The scripted commands are generated at physical t=0,3.10,6.20,9.30 seconds: forward→left→counterclockwise yaw→stop. These are **not period boundaries** (gait period 2.4 s). At each 40ms control tick, the velocity generator accepts a new command; 500Hz physics never stops or resets. The simulation need not run at wall-clock realtime, and the browser **does not yet** connect to this runner.

## Actual outcomes: single nominal CAD30, friction μ=1.1, preliminary motor caps

| Metric | Fast baseline | Slow research configuration (default) |
|---|---:|---:|
| Command scales | 1.0 | 0.5 |
| Lift / max stride | 12 / 35 mm | 7 / 22 mm |
| Forward stage root displacement | +23.395 mm | +5.645 mm |
| Left stage root displacement | +17.783 mm | +6.079 mm |
| Yaw stage change | +0.13020 rad | +0.05830 rad |
| Stop stage forward drift | +6.868 mm | +3.975 mm |
| Actual minimum loaded tibia contacts (Fz≥0.2N) | **2** | **3** |
| Maximum tilt | 0.785° | 0.706° |
| Highest raw motor torque | 0.700 Nm (clamped) | 0.653 Nm |
| Saturated physics steps (of 6200) | **593** | **0** |
| IK rejected ticks | 0 | 0 |
| 12.4 s completed, no root resets | yes | yes |
| Predeclared v1 gates | **PHYSICAL_FAIL** | **PHYSICAL_PASS** |

Full 12.4-second physical traces (every 100ms with real root pose, yaw and six contact normal loads) are in the private owner's Drive archive linked in the PR; GitHub receives compact receipts only.

**Important limitations:** This is slow motion on flat ground. PASS uses ≥3 actual loaded feet, not the stricter five-load wave criterion from Issue #59. The final stop still drifts ~4mm and no direct joystick/browser transport has been built yet. Servo masses, toe friction and motor limits have not been calibrated on hardware. Do **not** send these `qpos` values as PWM to real Freenove without separate calibration and safety checks. For any claim of robust arbitrary real-time walking, repeat with varying direction changes, recovery, obstacles, friction and true interactive command latency.

## Next gates

1. Integrate measured-state feedback to preserve ≥3 real loaded supports under larger commands and limit motor saturation; add force-loss abort, joint limit guard, delayed-command watchdog and test stop drift/velocity.
2. Add WebSocket/local endpoint for live commands and telemetry; connect it into the **existing** Hexapod Lab viewer from PR #24 (not a new viewer); maintain offline replay intact.
3. Run repeated command sequences, speed sweeps, different μ and masses, and document stable operating region before hardware experiments.

**Main branch is unchanged; no PR merge is authorized.**
