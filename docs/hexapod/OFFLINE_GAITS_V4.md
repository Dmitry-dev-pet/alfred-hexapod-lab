# Hexapod Lab — Offline joystick V4: 3 gaits × 3 stride lengths

Issue [#62](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/62), viewer draft PR [#24](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/24). Date: 2026-11-05.

## Why
The old V2/V3 used only `wave`, with **22 mm maximum stride** and actual body-command basis **6 mm/s × 2.4 s = 14.4 mm** per cycle. V3 increased cadence, not horizontal excursion. V4 increases **horizontal foot excursion per cycle** and exposes other conventional hexapod phase schedules.

## What changes on the SAME Hexapod Lab 3D screen

**Mode:** `Джойстик · 3 походки` (`/hexapod/live-v4-gaits/`), no separate website.

* `Wave` — 1 of 6 legs scheduled in swing (nominal five in stance).
* `Ripple` — 2 in swing (nominal four in stance).
* `Tripod` — 3 in swing (nominal three in stance).

**Foot horizontal excursion:** 15, 25, or **35 mm** (default); foot height above nominal stance during swing: up to **16 mm**. `Темп`: 0.5–8×, default 4×. At full forward stick, commanded root preview speed is `step_mm × tempo / 2.4 s`, e.g. 35 mm × 4 / 2.4 s = **58.33 mm/s**, compared with the previous default ~24 mm/s in V3 (and 3.6 mm/s V2). **These are preview commands, not measured actual speeds.**

Selectors are inside the **existing** canvas, with virtual touch stick at bottom left and turning, gait, step, tempo, Stop at bottom right. The previous V1 real MuJoCo recording and V2/V3 preview versions remain immutable in the library.

## How the tables were made

`scripts/build_live_v4_gaits.py`: from pinned Freenove FNK0052 CAD30 scene and the previously generated V2 reference stance, computes 3 gait schedules × 3 step sizes × 6 direction/yaw axes × 48 phases = **2,592 distinct joint-target frames**. Each frame solves the six legs' 3-DOF IK **OFFLINE** with actual MJCF axes; 18 legs joints + 2 still head joints. Offline kinematic fit max position error from the generated receipt is under 0.4 mm (max 0.3758 mm, for 35 mm lateral tripod target). Upper bound on joint magnitude ±1.70 rad; some lateral 35 mm targets reach that bound, so **larger steps are not enabled without revised stance/limits**.

Realtime device JavaScript uses only table interpolation, phase scheduling, joint-angle rate-limited transitions and forward kinematics for 27 CAD bodies. No MuJoCo/Drake/ROS/remote realtime physics; uses cached original Freenove STL and shared player shell.

### Gait transitions

When user selects another gait **while moving**, the controller queues the change until the next gait-cycle boundary, then blends joint targets for 0.42 s. When standing, it applies the selection immediately. Stride length changes interpolate smoothly over time. For combined XY+yaw commands, requested strengths are normalized when their L1 sum exceeds 1.6 to avoid direct summation of three full joint excursions. This is a numerical guard, **not** a proof of safe contact.

## Limits / scientific honesty

**This is KINEMATIC PREVIEW ONLY**. The animated root XY/yaw is commanded integration, *not* actual measured displacement. Swing patterns are planned, **not Fz contacts**. Combined commands, phase transitions and high cadence have not been physically validated. Older tripod and wave MuJoCo baselines are separate experimental evidence and do not validate the new selector combinations. The robot's real PWM and servo rate remain uncalibrated; do not connect this preview directly to hardware.

## Tests

From the restored local CAD assets and offline MuJoCo package:

```bash
PYTHONPATH=/path/to/mujoco/site:/path/to/CAD30 python -m unittest discover -s tests -v
node --experimental-default-type=module tests/test_gaits.mjs
node --experimental-default-type=module --check docs/hexapod/live-v4-gaits/app.js
node --experimental-default-type=module --check docs/hexapod/live-v4-gaits/kinematic-controller.js
```

Build (only offline research machine, **not tablet**):

```bash
PYTHONPATH=/path/to/mujoco/site:/path/to/CAD30 python scripts/build_live_v4_gaits.py \
  --source-dir /path/to/verified/CAD30/source \
  --base-kit /path/to/live-v2-offline-kit.json \
  --output docs/hexapod/data/live-v4-gaits-kit.json
```

No third-party CAD/STL or verified physical raw data is committed to the public repository. Large input CAD original remains on owner Drive. The viewer's existing local update server checks all SHA-256 and sizes, repoints HTML imports to vetted local Three.js and STL URLs from the previous installed cache.

## Tablet

1. Open current `http://127.0.0.1:8808/hexapod/` on Samsung Tab S7.
2. Tap the title → `Обновить библиотеку` → **`Джойстик · 3 походки`**.
3. Choose `Wave`, `Ripple`, `Tripod`; choose 15/25/35 mm step and 0.5–8× tempo. Try 35 mm at 4× first.
4. For camera, drag the 3D stage outside the virtual joystick. `СТОП` zeros motion.

GitHub `main` is untouched; Android WebGL/touch runtime requires device confirmation.
