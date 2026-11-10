# Hexapod Lab · Offline Joystick V3 FAST (2026-11-05)

Issue #62 / draft PR #24. A versioned replacement for `live-v2-offline`, preserving all existing immutable player library entries. No MuJoCo, Drake, MPC, servers, new CAD STL, or physics calculations at runtime. Existing on-screen virtual joystick + turn buttons + STOP, same Three.js viewer.

## Reason for new version

The original V2 model integrated commanded translation at just `vx≤6mm/s`, `vy≤5mm/s`, `yaw≤0.075rad/s`, with a 0.2–1.0 gain slider. Thus its default 0.6 gave ≈3.6mm/s. The gait cycle was fixed 2.4 s. That was very slow on the tablet.

V3 keeps the **same six verified-offline IK basis tables** and speeds **both** the phase progression (foot stepping) and the synthetic commanded root path by the same user-selectable **tempo multiplier** (0.5–8). It also scales the preview's joint slew rate so the visible legs can follow the accelerated cycle. No physics or IK rerun during interaction.

| Tempo | Commanded planar forward preview | Commanded yaw preview | Gait cycle |
|---|---:|---:|---:|
| ×0.5 | 3 mm/s | 2.15 °/s | 4.8 s |
| ×1 | 6 mm/s | 4.30 °/s | 2.4 s |
| ×4 (default) | 24 mm/s | 17.19 °/s | 0.6 s |
| ×8 (maximum) | 48 mm/s | 34.38 °/s | 0.3 s |

These are **commanded visualization speeds**, not physical estimates verified against measured MuJoCo or hardware. Faster preview may violate real motor velocity/torque/contact constraints. Mixed XY and yaw and rapid transitions also unverified physically. Do not present V3 as measured walk.

## GitHub deployment

Files are additive (no rewrites) on `feature/android-local-offline-viewer-20261104`:
- `docs/hexapod/live-v3-fast/{index.html,app.js,kinematic-controller.js}`
- `docs/hexapod/library.json` increments revision 10→11, new immutable `live-v3-fast` entry.
- Browser gets already registered `docs/hexapod/data/live-v2-offline-kit.json`, `docs/hexapod/data/cad-rig-live-v1-slow.json` and `docs/hexapod/live-v2-offline/pilot.css` from existing library entries; Three.js STL remain cached locally on tablet.
- Existing V1 physical replay and V2 slow preview preserved.

## Tests

`node --experimental-default-type=module tests/test_fast_joystick.mjs`

Tests verify tempo ×1/4/8 proportional physical-*commanded* translation/yaw, nonblocking direction changes, joint bounds, pointer-safe STOP decay, invalid tempo rejection. JS syntax and HTML route checked. No Android WebGL/actual thumb speed benchmarks performed by agent; user needs to tap **Обновить библиотеку** and open **Джойстик · быстрый**.
