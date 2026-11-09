# MicroDuck forward roll: higher steps — 2026-11-09

Highest successful tested height: **20 mm**, four times the original 5 mm step. It succeeds from initial root-to-edge gaps of 8 and 18 cm, but fails from 12 cm. The 8 cm case includes a full forward turn; the 18 cm case uses a shorter rotation and standing recovery. This demonstrates sensitivity to placement, not a universal robot height limit.

| Height | Start gaps tested | Stable upper-platform goal |
|---|---|---|
| 5 mm | 12 cm, reused exact baseline | yes |
| 10 mm | 12 cm | yes |
| 20 mm | 8 / 12 / 18 cm | yes / no / yes |
| 30 mm | 8 / 12 / 18 cm | no / no / no |
| 50 mm | 8 / 12 / 18 cm | no / no / no |
| 75 mm | 12 cm | no |
| 100 mm | 12 cm | no |

There are 13 saved traces: the exact prior 5 mm reference, six initial height cases at 12 cm, and six supplementary 20/30/50 mm cases at 8/18 cm. Each new case is a single deterministic run. The follow-up gaps were added after the fixed-gap result; the sample is not a complete height/distance grid or a reliability percentage. The boundary between 20 and 30 mm has not been exhaustively searched, and 75/100 mm were only tested at 12 cm. Different starts, timing, control or retraining could change the result.

## Fixed physical protocol

Same original v8 roulade network, official trigger and 2 s simulator timer, then v8 velstand recovery. Trigger at t=1 s; original pre-inference timer ordering returns at t=2.98 s. Every trace lasts 10 s, physics 1 ms, policy 50 Hz, 500 saved frames. No injected velocity, root lift, external push or scripted joints. All 13 initial qpos streams agree before the trigger; none has a loaded step contact before t=1 s.

Original robot MJCF, BAM M6 XL330, encoder-through-backlash observations/feedback, Kp 200, 7.4 V, sag gain .1, voltage floor 6 V. Original masses, inertia, joint ranges, mesh vertices and 70 visible parts are unchanged. Platform footprint 1.2 x 1.2 m; friction [1,.005,.0001]; all contact solref [.005,1] as in the preceding firmer-contact experiment. Only step height and specified gap change.

RL commit bae39f5900b318f7e87fc485c30a31213cfa14eb, BAM commit 62bd8ce12154340be97e06f7f41a0ca8f116d967. Roulade model set v8 commit 665ae4eb20bfe13bca1ef4ea6d4bb6b4a096e448, SHA-256 3d60da08fc13f29c1b57f41977aa898132c0d60042100149d8e775affcbca32b. Velstand SHA-256 2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492. MuJoCo 3.10.0, ONNX Runtime 1.24.4 CPU/one thread; source blobs/installed BAM/policy hashes verified.

## Outcome meanings

Height-study `status` is PASS only when `step_goal_reached` is true: both complete sole meshes beyond edge+1 mm, both carrying >.05 N top contact, tilt <20 degrees, root speed <.06 m/s, angular speed <.7 rad/s and root z between .09+h and .18+h throughout the last 1 s. No numerical warning/reset is allowed. All four successful entries have <1.91 mm peak contact penetration. None of the 13 cases has a warning or reset.

`strict_roll_status` preserves the original rollout's supported-rotation gate; the raw trace summary's `status` still refers to that stricter roll criterion. It is separate from the height goal. The inherited `standing_final_1s` checks height relative to the upper platform, so a failed high-step attempt can stand quietly on the lower floor and still have this flag false. The UI labels it as stance at platform height. `full_rotation_completed` remains the independent total-turn/head-pivot flag.

Contacts remain a model limitation. Successful 20 mm cases have brief loss of support and peak simulated contact forces about 251-270 N. Robot/hardware calibration and real-world step reliability are unverified. The failed high cases are failures under this fixed protocol, not proof of physical impossibility.

## Verification / reproduction

Independent contact/complete-sole/final-posture checks agree with all 13 goal labels. Quaternion increments independently verify integrated rotation (<1 degree across all saved frames). Repeated 20 mm / 8 cm success and 20 mm / 12 cm failure are byte-identical. Five client tests verify all tracks/70 parts, hash/truncation/header rejection, offline cache reuse and retention of failed heights. Float32 export error <1e-6. Export compares robot parameters and meshes to the flat base model.

Restore the pinned runtime/source workspace using earlier Drive archive 1S7FgYnz_Y31OedO98oUUC_Gp1loifPqz (SHA-256 633f948b3340fb6c63087e84a25b33e6f69d6025ae63a4c5384f09befdb9a10e), then extract this archive. It supplies height runner, original rollout/terrain/benchmark helpers, rollout weights/receipt, raw traces and replay tracks. Run `python microduck-heights/height_cpu.py`, `python microduck-heights/verify_heights.py`, then `python microduck-heights/export_heights.py`. MICRODUCK_WORKSPACE selects the restored root; repository tests accept MICRODUCK_HEIGHT_DIST pointing at the restored Site dist. No training or GitHub-hosted Actions were dispatched.

Browser loading/selection/seeking are checked; actual rendering remains unverified because managed Chrome has WebGL disabled. The scene uses one exact-size platform and the shared original 38-STL package, with the visible floor aligned to physical z=0. Existing Site route /hexapod/microduck-heights-v1/. Geometry: Pollen Robotics CC BY-NC-SA 4.0; original source/policies Apache-2.0.
