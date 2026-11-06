# XGO-Lite3 — first CPU physics and pretrained actor diagnostic (2026-11-06)

**Status: exploratory success in execution, NOT native MJLab validation.**
Companion to Issue #68 and draft PR #69. Original external meshes and trained weights are in the owner's **private Drive**, not GitHub.

## Verified asset transport and provenance

- One-shot Mac M4 workflow (private runner, zero GPU): https://github.com/Dmytry-lab/runner-smoke/actions/runs/42243825347 — success.
- Upstream model: LuwuDynamics/luwu_mjlab @ 447945fd40fb5585271dce4ea8cc82567916032a; raw lite3.xml pinned git blob 9c9699c5fc3c5f4ed4c69ebf516c346ee6a57026.
- Pretrained checkpoint: robomotic/XGO @ 1646bc65ed37feff3698a43524a65cb3cd5b6981; model_1499.pt SHA256 cca3a020163309790f8dc8ecb7553ff0e98e75d02e78a1773ad3734ca535a67c; policy.onnx SHA256 773a1bbd0fee45ec38604d979d5fd6fb339ae15d13bfb5d6e2056a171d2a3422.
- Source and binary ZIP (12,229,950 bytes) stored privately: https://drive.google.com/file/d/1uzAHREDGxxp2ehs8jEeJp6PcujCFvueQ/view?usp=drivesdk
- Experiment code, raw measured traces, manifest and zero baseline: https://drive.google.com/file/d/1BTsdVakufVYqgohUZCl3KvwIvluxmmyC/view?usp=drivesdk ; ZIP SHA256 c100e1178c06c84e4abecfce8fd9675cd7b8c3dcaeab1f2567b69da6b31e948f.

## Actual executed physics

1. Original STL-backed scene_lite3.xml loaded in MuJoCo **3.14.0** on Python 3.13.5, CPU only. Fixed-position 2-second diagnostic: 1,000 real 2ms mj_step ticks, 14 actuators, 965 ticks with actual contacts. Original bare lite3.xml does **not** contain floor; use scene_lite3.xml when reporting contacts.
2. Torch checkpoint loaded with weights_only=True; actor normalization (53 inputs), MLP 512→256→128 ELU and 14 output actions reconstructed. This is **actual trained actor weights**, not an invented motor trajectory; ONNX Runtime not yet run.
3. Independent approximate MJLab adapter (not upstream runtime): original scene + 4-foot collision filter, 4ms MuJoCo physics, actor 20ms = 50Hz, action scale .25. 8-second deterministic sequence: stand (0–2s), forward vx=.2m/s (2–4s), yaw wz=.35rad/s (4–6s), stop (6–8s). Fixed pose/zero action evaluated under identical sequence.

| Metric | Trained checkpoint actor | Zero-action reference |
| --- | ---: | ---: |
| World root ΔX over 8s | +47.80 mm | −3.34 mm |
| World root ΔY over 8s | +7.64 mm | ~0 mm |
| Highest body tilt | 17.42° | 0.07° |
| Lowest body height | 86.62 mm | 92.95 mm |
| Steps with at least one actuator reaching simulated force cap | 393 / 2,000 | 0 / 2,000 |
| Fall criterion (tilt > 60° or root z < 30mm) | Not triggered | Not triggered |

- Actor x changed from +10.2mm at 2s to +53.1mm at 4s: **+42.9mm** in the commanded forward interval, much smaller than nominal 400mm for vx=.2m/s ×2s.
- Actor yaw changed ~13.1° at 4s to ~18.6° at 6s: +5.5° while commanded 0.35rad/s (40.1°/2s). Tracking poor.
- This is genuine MuJoCo free-body integration and physically measured contact force; renderer movement not fabricated.

## Scientific limitations (important)

- Not upstream mjlab 1.2.0 play (vendor GPU-oriented Warp pipeline). Action/joint order, actuator delays, observation timing, noise and normalization-to-MJLab parity **not yet independently verified**.
- Training-like shank contact filter is an independent approximation; actor/servo mapping could be wrong. Modified floor flag is necessary for actual shank contacts; exact full simulator matching requires a source/semantic audit and ONNX parity.
- Only one 8s episode, not repeatable stability across seeds. Real XGO-Lite3 hardware actuation / power curves / dynamics are uncalibrated.
- The raw trained actor moved the model but **this is not a confirmed stable walking controller** and not a demonstrated command-tracking PASS. High force saturation and weak velocity tracking are open blockers.

## Next gates

- Validate actor output numerically against pinned policy.onnx on identical observation vectors; check exact action and joint indexing and defaults.
- Reproduce observation pipeline and contact geometry semantics from mjlab pinned source; compare parameter sets.
- Re-run repeated zero/actor 10+ seeds; record command tracking, Fz, slip, saturation, fall criteria. Then optionally export only genuine measured replay to existing Android Hexapod Lab viewer.
- Do not merge main or drive motors without approval.
