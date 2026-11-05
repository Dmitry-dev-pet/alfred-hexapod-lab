# Hexapod Lab — Offline Locomotion V2 · onscreen joystick

Date: 2026-11-05 · Issue #62 · project FNK0052/CAD30.

## Owner decision

**The joystick must be on the existing Hexapod Lab 3D player.** Do **not** run MuJoCo, Drake, WebSocket physics or ROS on the tablet or in the background. The current shared library and existing physical-replay entries must remain available and immutable.

This release adds `/hexapod/live-v2-offline/` as **another selectable mode in the current viewer**, using its current Three.js 3D shell, camera, local Freenove CAD mesh assets, title/library sheet and on-demand browser refresh. The joystick is a visible overlay on the 3D robot, not a separate web app. Onscreen stick controls planar `(vx,vy)` and held turn buttons control yaw. Release/Stop return velocity/joints toward neutral with slew/acceleration guards; loss of tab focus zeros commands. Optional keyboard: WASD/QE. **No live physics.**

## Evidence and provenance

- New `scripts/build_live_v2_kinematics.py` uses the pinned owner's `scene_local30_nominal.xml` offline, `hexlab/live_gait.py` from draft PR #63, `hexlab/stand_v04.py`, `hexlab/step_one_leg_v04.py`, scipy IK and MuJoCo **only as offline inverse/forward kinematics**.
- Generate six basis cycles, 48 phases each: forward/back ±6 mm/s, sideways ±5 mm/s, yaw ±0.075 rad/s. Wave gait phase period 2.4s, swing height 7mm and max stride 22mm. Each row consists of 20 joint angles; 18 leg joints are actually driven, 2 head joints remain still.
- Offline metadata includes the original 48-body kinematic tree and measured-from-model hinge axes and anchors in the reference pose. The browser applies joint rotations through parent/child rigid transforms, not fake screenshots. FK was independently compared with `mujoco.mj_forward` on 24 × 47 sampled bodies: position and rotation tolerances validated.
- Simultaneous, continuous joystick values are **linear blends in joint space** of these separately compiled single-axis cycles. Root travel is **integrated from requested speed**, never represented as measured COM motion. Browser tick only does light interpolation + FK in JS.
- Original 30 STL are loaded from the **existing local Android viewer cache** via the already-installed and verified rig JSON; the refresh server rewrites Three.js CDN import maps to the offline npm copy. No new STL binaries are committed.
- The old `Live V1 · меняем направление` replay is still an **actual physical MuJoCo experiment**. This V2 controller is a **kinematic preview with no calculated loads, friction, motor torque or support guarantees**. Offline compiling a single-axis envelope cannot prove mixed-command physical stability.

## Verification

Local tests:

```sh
PYTHONPATH=/path/to/mujoco/site:/path/to/cad30/source python -m unittest discover -s tests -v
node --experimental-default-type=module tests/test_kinematic_controller.mjs
node --check docs/hexapod/live-v2-offline/app.js
node --check docs/hexapod/live-v2-offline/kinematic-controller.js
```

Independent FK-vs-MuJoCo test requires owner's local pinned CAD/STL and should not be run in an environment without authorized assets. No Actions physics, no merge into `main`. Device FPS/controls must be tested on Samsung Tab S7.

## How to open on the tablet

Open existing `http://127.0.0.1:8808/hexapod/`, tap the title, press `Обновить библиотеку`, then select **`Джойстик · 6 ног`**. The joystick is at lower left of the same 3D stage, turn buttons at lower right. Physical replays remain separate selections.

## Limitations

- No proof of dynamic/static stability outside the previously tested scripted PR #63 commands. Kinematic generated motion must not be described as "MuJoCo physical PASS".
- Real servos are **not connected**; 18 joint signs/servo PWM calibration remain unverified.
- Geometry has 30 original CAD STL plus existing two visual proxy objects; no redistribution of third-party meshes.
- The Android UI and offline cold-start should be checked on the actual tablet. No statement of confirmed device behavior until user reports it.
