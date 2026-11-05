# Hexapod Lab — Offline Motion Controls V5

Date: 2026-11-05 · Issue #62 / player draft PR #24 · Freenove FNK0052 CAD30.

## What changed

A single **joystick overlay inside the existing Android Hexapod Lab 3D viewer** gains four modes:

| Mode | Controls | Intended geometrical limits |
|---|---|---|
| **Ходьба** | 2D joystick: forward/back/sideways/diagonal; hold ↶/↷ to turn simultaneously; gait wave/ripple/tripod, stride 15/25/35 mm, tempo ×0.5–8 | Unchanged **Offline V4** gait tables; no running physics |
| **Корпус** | Joystick: horizontal torso shift; sliders: height, roll, pitch, yaw | forward/left ±12 mm, z ±12 mm, roll/pitch ±5°, torso yaw ±7° |
| **Нога** | Select one of six CAD legs, joystick repositions selected toe, slider lifts it | toe forward/back ±18 mm, sideways ±12 mm, lift 0–13 mm |
| **Голова** | Joystick rotates/pitches the head | yaw ±18°, pitch ±16° (two model hinge joints) |

Touch release centres the joystick. **СТОП / НУЛЬ** zeros requested commands and all non-walking pose sliders; the controller ramps back to neutral. Mode switches also zero targets. Tab hide/blur zeros commands. Previous player library/replays remain unchanged, including physical V1 and original gait V4.

## Source and math

- New **motion kit** `docs/hexapod/data/live-v5-motion-kit.json` built **offline** from the same pinned owner CAD30 MuJoCo scene (30 verified STL, original legs and two head joints), using SciPy bounded IK. No third-party original CAD/STL files copied into repo.
- For each single body axis ± max, solve all six leg joint angles to keep each original toe in its original *world* position while moving the body reference transform. Head yaw/pitch are direct rotations of the two *model* head hinges.
- For each CAD leg and three foot axes, solve only its three joints to place one foot relative to the original body. Other leg targets remain unchanged. The overlay interpolates these precalculated joint tables in the same 27-body FK renderer as V4.
- Existing `live-v4-gaits-kit.json` remains the only gait kit; V5 references it, does not duplicate it in the new library manifest. Existing rig CAD metadata and local STL cache are reused.
- Runtime code runs only JS interpolation and rigid transforms. **No MuJoCo/Drake/ROS, remote simulation, background service or live physical dynamics.** The existing static local Python server serves files; it does not run physics.

## Verification — local 2026-11-05

1. `CAD30_SCENE_PATH=/path/to/scene_local30_nominal.xml PYTHONPATH=/path/to/mujoco/site:/path/to/cad30 python -m unittest discover -s tests -p 'test_*.py' -v` — **6/6 PASS with verified owner CAD30 and V4 kit (via environment)** (CAD30 FK-vs-precomputed IK, active leg isolation, data integrity and UI structure).
2. `node --experimental-default-type=module tests/test_motion_modes.mjs` — **39 mode/control cases PASS**, plus invalid input guards; walk/stop/return, all six body axes (±), all six legs (forward/lateral/lift), two head joints, mode switches.
3. JS source syntax `node --check` and 3D player HTML static structure. Android touchscreen/WebGL user acceptance **not** independently tested in this session.
4. On actual pinned CAD30, maximal isolated IK error **0.01504 mm** and maximum absolute single-axis leg angle **1.603996 rad** (software limit 1.7 rad). Actual contacts/motor torques **not** calculated for these new motions. Combinations and complex transitions are linear joint-space previews, NOT guaranteed to preserve foot positions or static/dynamic stability.

## Critical caveats

- **Do not call this a physically verified hexapod walking/body/leg controller.** There is no dynamics, contact loading, support polygon validation, actuator timing, friction, foot slippage or real robot hardware calibration. Lifting a support foot may destabilize a real robot. The absolute 18 motor PWM zero/sign calibration is not verified.
- The value displayed for walking travel is **integrated commanded displacement**, not measured distance. Body pose modes move the virtual reference root only. Unsupported mixed settings are clipped to a conservative joint-angle envelope but are **not safety certified**.
- Third-party CAD30 original STL remain in the owner's local verified cache/Drive; not redistributed in GitHub or ZIP. Two missing Freenove parts remain visual proxies.
- Runtime for V5 depends on already installed and hashed V4 controller/kit plus V1 CAD metadata, shared player-shell CSS/JS, and 30 local mesh assets. Android installed updates are through the existing on-demand **«Обновить библиотеку»**; no Android reinstallation should be required.

## Open in existing player

`http://127.0.0.1:8808/hexapod/` → title → «Обновить библиотеку» → **«Джойстик · движения V5»**.

The player branch remains a draft PR; **do not merge into main without explicit owner authorization**.
