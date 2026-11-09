# MicroDuck kinematic controls — 2026-11-09

Existing Alfred Robot Player: /hexapod/microduck-control-v1/.

Two independent pointer-capture joysticks: left forward/back/turn, right head yaw/pitch. Three authored fall transitions connect to recorded BAM+backlash recovery traces (side90, roll180, pitch180, seed 20261109). Stop pauses an action; Continue resumes; Reset clears it. Keyboard: WASD, arrows, F/G, Space, R.

This is a kinematic preview. The browser blends prepared qpos cycles and integrates commanded XY/yaw without collision dynamics or a live policy. Root speeds .12/.08 m/s and .6 rad/s are preview settings, not measured performance. Backward is explicitly a time-reversal of the forward gait, because the measured negative command did not walk usefully. Falls begin with a .35-second authored blend; six-second get-ups are recorded MuJoCo states. Arbitrary blended sequences are not physically validated.

Geometry: 38 original STL meshes, 70 visual instances, shared content-addressed package e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b. Exact JS FK is checked against 24 independent MuJoCo fixtures, all 70 poses, tolerance 1e-8. No proxy geometry.

Pinned sources: pollen-robotics/microduck_rl bae39f5900b318f7e87fc485c30a31213cfa14eb; BAM 62bd8ce12154340be97e06f7f41a0ca8f116d967; v8 velstand policy SHA-256 2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492. MuJoCo 3.10.0, numpy 2.3.5, onnxruntime 1.24.4, scipy 1.17.0, Python 3.12.4. Source Git blobs and installed BAM are checked before generation. Original meshes: CC BY-NC-SA 4.0, Pollen Robotics; source code upstream Apache-2.0.

The Drive reproducibility ZIP mirrors the workspace layout. Extract source tarballs into microduck-work with their pinned names, install the stated dependencies and BAM, then run `python microduck-control/prepare_motion_kit.py`. Run `node --test microduck-control/test_controls.mjs` after unpacking. Raw current and unsuccessful lower-command trials, 3 source fall/recovery traces, policy, model package, fixtures and client are retained.

Repository test: unpack the ZIP, set MICRODUCK_CONTROL_DATA to its absolute microduck-control directory, and run `node --test tests/test_microduck_controls.mjs` from the player source checkout. The generator in research/microduck mirrors the archive generator and expects the described workspace layout.

Browser QA: controls/data state and UI checked in managed Chrome. Cloud Chrome has WebGL disabled, so the final 3D rendering and physical Android multitouch are unverified here. Desktop kinematic logic, fall/get-up continuity, stop/reset and independent pointer semantics pass six local tests. No hosted Actions were dispatched.
