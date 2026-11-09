# MicroDuck: original roulade over one 5 mm step — 2026-11-09

The step goal was reached in all six firm-contact step trials: both complete soles ended on the upper platform and the robot stood quietly throughout the last second. With the 2 s simulator timer, all three distances completed a full forward body rotation. With the 1 s shipped-manifest timer, motion was a partial roll followed by the standing policy's recovery. These are six deterministic cases, not a reliability percentage.

| Ground | Contact profile | Timer | Max total forward rotation | Supported forward progress | Full turn | Quiet standing / upper platform |
|---|---|---:|---:|---:|---|---|
| Flat | upstream / 5 ms | 1 s | 270.4° | 248.3° | no | yes / n.a. |
| Flat | upstream / 5 ms | 2 s | 364.2° | 336.4° | yes | yes / n.a. |
| Flat | firm / 1 ms | 1 s | 280.7° | 227.2° | no | yes / n.a. |
| Flat | firm / 1 ms | 2 s | 357.4° | 319.3° | yes | yes / n.a. |
| Edge 8 cm | firm / 1 ms | 1 s | 286.7° | 234.4° | no | yes / yes |
| Edge 12 cm | firm / 1 ms | 1 s | 287.1° | 233.9° | no | yes / yes |
| Edge 18 cm | firm / 1 ms | 1 s | 280.4° | 225.9° | no | yes / yes |
| Edge 8 cm | firm / 1 ms | 2 s | 363.6° | 323.0° | yes | yes / yes |
| Edge 12 cm | firm / 1 ms | 2 s | 363.5° | 321.2° | yes | yes / yes |
| Edge 18 cm | firm / 1 ms | 2 s | 355.1° | 315.4° | yes | yes / yes |

## What was executed

Original `roulade.onnx` from Pollen Robotics policy set tag v8, immutable Hub commit 665ae4eb20bfe13bca1ef4ea6d4bb6b4a096e448, LFS-verified SHA-256 3d60da08fc13f29c1b57f41977aa898132c0d60042100149d8e775affcbca32b (793685 bytes). Recovery uses the already pinned v8 velstand policy, SHA-256 2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492. The official manifest says 1 s; pinned `scripts/infer_policy.py` defaults to 2 s. Both existing timings are compared, without tuning the network.

RL source bae39f5900b318f7e87fc485c30a31213cfa14eb; BAM 62bd8ce12154340be97e06f7f41a0ca8f116d967; MuJoCo 3.10.0, ONNX Runtime 1.24.4 CPU/one thread. Model source blobs and policy bytes are verified before execution. Original robot geometry, masses, inertia, joint ranges and all 70 visual instances are unchanged. BAM M6 XL330, Kp 200, 7.4 V, sag gain .1, voltage floor 6 V, paired passive backlash and output-encoder observations.

Each case starts in the official STAND keyframe with zero velocity. The ordinary standing controller settles for 1 s, then the official `trigger_behavior('roulade')` / `update_behavior` implementation swaps policies. The timer update precedes inference just as in upstream's loop: actual hand-backs occur at 1.98 or 2.98 s absolute. There are no initial velocity injections, root lifts, external impulses or scripted joint tracks. A zero velocity command remains active after return to velstand. Each physical trajectory lasts 10 s, recorded at 50 Hz (500 frames).

The platform is 5 mm high, 1.2 m deep and wide, front edge x=.08/.12/.18 m measured from the initial root origin. A static box is appended; only the environment is added. Firm cases use geom solref [.005,1] and 1 ms physics; upstream cases retain original contacts and 5 ms physics. Both contact parameters and timestep change together, so their effects are not isolated and neither profile is calibrated against real hardware.

## Separate outcome definitions

- `full_rotation_completed`: max signed body-link-y angular integral >=330°, with the head-top contact latch. This measures an actual nearly complete turn; independent quaternion increments agree to <0.185° across every saved frame stream.
- `roll_complete` / `status`: the stricter supported-progress gate >=330°, plus head pivot and final standing (and upper-platform support for step cases). Supported progress counts rotation only during ground contact and discounts lateral motion using upstream's sagittal gate. This strict result is PASS only for upstream flat/2 s, and FAIL in the other nine cases. It must not be read as failure to reach the step.
- `step_goal_reached`: both sole meshes entirely beyond edge+1 mm, both carrying >.05 N top support, tilt <20°, root speed <.06 m/s, angular speed <.7 rad/s, trunk height between .09+h and .18+h throughout the final 1 s, with no numerical warning/reset. All six step cases meet this goal.

The 2 s step rolls briefly lose support for 7.3–7.7% of the active policy interval (force threshold .05 N); supported progress is 315–323°, so they fail the strict 330° supported criterion despite the full body turn and successful upper-platform stance. Peak model contact forces are 266–310 N. This is a bouncy maneuver in this simulation, not a verified smooth or hardware-safe transfer. Contact calibration still limits physical interpretation.

## Validation and reproduction

The four flat physical state/contact streams repeat exactly. The 12 cm / 2 s firm step trace repeats byte-for-byte. All ten trace hashes and final goal classifications are checked; angular measurements use `mjOBJ_XBODY` (body link frame), not the differently oriented `mjOBJ_BODY` inertia frame. Independent quaternion integration and a native free-joint velocity cross-check guard this distinction. Five local client tests decode all ten complete 70-part tracks, reject corrupted bytes, verify cached offline reuse, and preserve the distinction between rotation and step goal. Float32 export absolute error <1e-6. No hosted Actions or training jobs were dispatched.

Restore the preceding pinned source/runtime workspace from Drive archive 1S7FgYnz_Y31OedO98oUUC_Gp1loifPqz (SHA-256 633f948b3340fb6c63087e84a25b33e6f69d6025ae63a4c5384f09befdb9a10e), then extract this archive at its root. It includes the new policy bytes, receipt, all 10 raw traces, benchmark/terrain helpers, client and binary tracks. Run `python microduck-roll/roll_cpu.py`, `python microduck-roll/verify_roll.py`, then `python microduck-roll/export_roll.py`. Set MICRODUCK_WORKSPACE explicitly when using repository copies; MICRODUCK_ROLL_POLICY can point to the archived roulade.onnx. Repository replay tests accept MICRODUCK_ROLL_DIST pointing to the restored Site dist.

Browser loading, scenario switching, seeking and visible goal/rotation labels are checked. Actual 3D rendering remains unverified in managed Chrome because WebGL is disabled. The original mesh package is reused (38 STL / 70 instances); no proxy robot replaces it. Geometry: Pollen Robotics CC BY-NC-SA 4.0, upstream code/policies Apache-2.0. Existing Site route: /hexapod/microduck-roll-v1/.
