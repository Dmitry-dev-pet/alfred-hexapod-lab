# EN01 — STEP audit + reproducible MuJoCo, 2026-11-09

**Status: exploratory, not hardware-validated.** EN01 is P0. No original CAD binary redistribution, no MP4/GIF, no new viewer, no main merge.

## Reproduced baseline

Restored the owner's offline **MuJoCo 3.14.0 / Python 3.13** wheelhouse from private Google Drive (bundle SHA-256 `6d14d0c17294bac5436e1167ca4a97c30061da8efaa0a2b904c171bcac4a7b13`). Ran four 6-second, 3,000-step genuine `mj_step` simulations in Linux CPU. Each raw trace matched the 2026-11-08 archive **byte for byte**:

| Scenario | Max |x| | Torso-ground physics steps | Raw SHA-256 |
|---|---:|---:|---|
| stand | 0.33801 m | 0 | `a5d0cad75d4c4cf1e78c245ebfeda4dc23e9431addcec0741ecf8a8196cdf273` |
| crouch | 0.38682 m | 0 | `ac2f053b44353f0f15066fa9908f54afb6a5f3e73f432c1f550cfd82b5ab43cc` |
| jump | 0.24550 m | 417 | `608cfa5778bf96d4578b36ec65408e64e54dd07d72366ff5faf8b058137174d3` |
| jump_fast | 0.45105 m | 1893 | `98920e20e4506a2f644288cc4c4083c7b264d38207700bf42fca2d571220b62c` |

Both jumps FAIL (no qualified full-body jump and stable landing). No claimed self-righting, real hardware, trained policy or deployed behavior.

## Source STEP semantic audit

Original upstream `fuwei007/Navbot-EN01@3139f2fb32b38d10611feb905e16f8aa012f2f2e`, `Hardware/RebotModel/OriginalRobotModel.stp`, Git blob SHA-1 `9156683a0bb694c84dc541a3e26b5f6b08db1a41`, **12,358,579 bytes**. Read and analyzed from GitHub without storing the binary in this repository.

Measured **172,707 STEP entities**, **46 product labels**, **62 solids**, **41 linked BREP part representations**, **1,584 cylindrical surfaces**, **76 assembly-usage transformations**. Product labels include `STS3032`, `QUANUM GIMBAL MOTOR 2208 BASE/OUTRUNNER`, thigh, calf and `2208电机轮胎` (wheel tire). The tire part contains **18.3/18.5 mm local cylindrical radii**. This does **not** prove final wheel rolling radius, as-sold motor model, joint axes, mass or inertia. The existing MJCF is an approximation (24 mm wheel), not a conversion of the original CAD.

Firmware `Src/wl_pro_robot/wl_pro_robot.ino` includes `jump_loop()` where the motion from height value 80 to 40 is controlled by **loop counter thresholds**, not specified timing in seconds. Our 1.85s/0.15s probe timing is an experimental assumption, not firmware parity.

## 12-run diagnostic sensitivity experiment

Only wheel cylinder radius and initial torso z were changed in the existing approximate model to preserve wheel-floor contact; all gains, motor assumptions and masses were held fixed. **12 × 3,000 real MuJoCo steps**:

| Radius hypothesis | Stand max |x| | Crouch max |x| | Jump | Fast jump |
|---|---:|---:|---|---|
| 24.0 mm baseline | 0.3380 m | 0.3868 m | FAIL | FAIL |
| 21.0 mm | 0.2949 m | 0.3409 m | FAIL | FAIL |
| 18.5 mm | 0.2588 m | 0.3022 m | FAIL | FAIL |

These are **not** improvements to the controller, proof of correct CAD dimensions, or validated realistic performance. Radius 21 mm fast-jump showed >4 m max x displacement and is a failure, not successful motion.

## Existing Android replay pipeline

The four recorded baseline traces were checked and packaged with `scripts/navbot_en01_replays.py --skip-execution` (PR #96 / shared PR #91 pipeline). Isolated source catalog revision **35 / 56 entries → revision 39 / 60 entries**, then second run added **0** entries and preserved revision 39. Entries: stand/crouch RESEARCH, jump/jump-fast FAILED. These are review-only interactive 3D states, not synthesized movement.

**Publication remains blocked:** final CAD joint transforms/inertias and exact current motor SKU/torque-speed/voltage are not confirmed; Android WebGL and offline tablet playback untested. Source catalog rev35 is a snapshot, not guaranteed current. Do not overwrite the owner viewer branch or live library. Big raw traces and runnable review ZIP are provided separately in the conversation.

**Next:** full STEP assembly-placement reconstruction, reliable actuator limits and source-controller timing; then repeat balance/drive/crouch and both jump protocols with held-out friction/mass/lag variants. No auto-merge, purchase or GPU.
