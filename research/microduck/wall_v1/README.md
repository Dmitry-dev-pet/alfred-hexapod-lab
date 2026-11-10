# MicroDuck: stand close and fall backward onto the obstacle

Owner-requested continuation, 2026-11-09. Same motor command and original v8 recovery as the previous backward experiment; the changed experimental variable is obstacle placement. All eight episodes are preserved.

## Observed outcomes

| Placement | Height | Rear-body load during command + 0.5 s | Final stance | Both complete soles on top |
| --- | --- | --- | --- | --- |
| Close | 10 mm | 34 ms, on upper surface | Stood on edge; one sole overhangs | No |
| Close | 20 mm | 323 ms, on upper surface | Remained lying | No |
| Close | 30 mm | 388 ms, on upper surface | Remained lying | No |
| Close | 50 mm | 444 ms, on upper surface | Remained lying | No |
| Close | 100 mm | 895 ms, including 92 ms with face-normal support | Stood below | No |
| Close | 250 mm wall | Only 5 ms of face contact | Stood below | No |
| Previous placement | 10 mm | 21 ms | Stood below | No |
| Previous placement | 20 mm | 62 ms | Stood below | No |

This supports the contact part of the proposed maneuver: the rear body can load the obstacle. The 10 mm close placement achieves a stable edge stance with both feet loading the upper surface, but the left sole extends 18.296 mm beyond the edge. This is partial climbing progress; it fails the predeclared full-sole criterion. In these fixed-command trials moving closer also prevents recovery at 20–50 mm. The existing get-up sequence did not finish with both complete soles beyond the edge. The edge-stance outcome is retained separately from that stricter goal. No universal obstacle limit or reliability estimate follows from this sweep.

## What “close” means

Distance is measured from the original robot surface, not from the free-joint origin. The obstacle face is +world X; initial robot yaw pi means this is behind the robot. For each height, intersect all original visible and collision mesh convex hulls with the obstacle's 0..height vertical slab. Take the furthest X over the initial STAND keyframe plus the first 50 reference stance states, and place the face 2 mm beyond it. This is conservative with respect to nonconvex visual parts and checks all original exterior parts, not just the collision proxies.

Before the trigger the resulting measured gap is 2.000 mm at 1/2/3/5 cm and 2.006 mm at 10/25 cm. The free-joint-to-face distances are about 22.12, 29.44 and 49.57 mm, respectively. For low steps, feet/leg geometry limits proximity; at taller walls, the rear battery does. The previous placement is face X=80 mm from the free-joint's initial origin, **not** an 80 mm surface gap. Its actual pre-trigger surface gaps are about 59.9 and 59.9 mm at 1/2 cm.

All eight simulations start with the obstacle already present. They have identical first-second qpos states, no initial loaded obstacle contact or penetration, and no runtime root repositioning or moving terrain. The independent verifier uses linear programming over mesh-vertex convex combinations to check the final 2 mm gap; it does not reuse the hull-edge clipping algorithm. Physical collision contacts are monitored every 1 ms; exterior geometry is conservatively sampled at 50 Hz for the initial layout.

## Control and scoring

- Settle in STAND for 1 s. Ramp the previous backward hip-pitch command (+0.65 rad left / opposite sign right) over 0.25 s, hold until t=2.2 s, then use the existing pinned v8 `velstand.onnx`.
- Original BAM model, masses, inertia, sparse collision masks, joint ranges, voltage profile, 1 ms integration and 20 ms control remain unchanged. No injected force, root momentum, kinematic animation, learned-policy replacement or timing search.
- Clamp all authored and recovery targets 2 degrees inside rotor joint ranges. Feed the actually sent motor target into ONNX last-action observations. Source and installed BAM Git blobs are checked before execution.
- Monitor loaded contacts on root body 1 (rear battery collision mesh) against obstacle 82. Report total, vertical-face and upper-surface durations separately. Count each loaded physics substep once, even if it has several contact points. “Back into obstacle” requires >=10-degree backward lean and >=20 ms rear-body contact during the command plus 0.5 s, with clean initial placement and no warnings/resets. The 25 cm wall has a real but briefer contact and fails this sustained-contact criterion.
- For steps, success requires both entire sole meshes beyond face+1 mm, both feet loading the top surface >0.05 N, and quiet standing for the final second. For the tall-wall control, a sustained rear-body contact followed by quiet standing is scored separately; no claim of scaling a 25 cm wall.
- Same quiet-stance definition as the previous experiment: tilt <20 degrees, trunk height 0.09..0.18+obstacle_height m, speed <0.06 m/s, angular speed <0.7 rad/s, both feet loading the environment >0.05 N throughout the last 50 frames.

## Verification and limits

The edge-stance annotation is derived from the final 50 saved foot loads and sole bounds; it does not change the strict outcome or physical traces. Both previous-placement physical streams (qpos/qvel/ctrl/targets) match the already preserved trials exactly. Close 20 mm and close 100 mm repeated byte-for-byte. All eight trajectories are finite with zero physics resets and warnings. All commands respect the 2-degree target margin; maximum soft rotor-limit overshoot is 0.403 degrees. Original sparse self-collision penetration reaches **6.277 mm in the 50 mm case**, so that failed trajectory must not be treated as proof of mechanically valid motion. Mesh contacts are approximate; hardware and noise robustness are untested.

The full 70-part original-STL transform tracks contain 4,000 frames at 50 Hz; float32 export error <1e-6. The 3D replay camera views the robot from the open floor side so the tall block does not obscure it. Data loading, selection and seeking are checked; cloud-browser WebGL is unavailable, so actual rendered motion and Android multitouch are not visually verified here.

## Reproduce

Keep the relative archive layout. Restore the pinned source archive https://drive.google.com/file/d/1S7FgYnz_Y31OedO98oUUC_Gp1loifPqz/view (SHA256 `633f948b3340fb6c63087e84a25b33e6f69d6025ae63a4c5384f09befdb9a10e`) beneath `microduck-work/`, then overlay this archive's helpers, policy and previous reference traces. Install `requirements-bam-cpu.txt` in a Python 3.12 environment and use the BAM source pinned there. Run from workspace root:

```
python microduck-wall/measure_clearance.py
python microduck-wall/wall_cpu.py
python microduck-wall/verify_wall.py
python microduck-wall/export_wall.py
node --test microduck-publish/site/tests/test_microduck_wall_replays.mjs
```

RL commit `bae39f5900b318f7e87fc485c30a31213cfa14eb`; BAM `62bd8ce12154340be97e06f7f41a0ca8f116d967`; stand policy SHA256 `2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492`. Original model package SHA256 `e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b`.

Geometry: Pollen Robotics, CC BY-NC-SA 4.0. Upstream code/policy: Apache-2.0. Retain pinned upstream notices. Interactive output is physical replay, not an MP4 or live physics session.
