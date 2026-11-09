# EN01: publication of existing diagnostic recordings

Owner instruction, 2026-11-09: publish what already exists on the existing ChatGPT robot player. This explicitly permits these diagnostic recordings to be viewed before completion of the mechanics audit; it does not qualify the models, merge the research PR, or authorise a robot purchase.

One new catalog entry `/hexapod/en01-experiments-v1/` contains four scenarios (stand, crouch, jump, jump_fast) for each of v0 and v1. Default: v1 stand. Existing records remain unchanged. The Site's deployed live-catalog bridge delivers the entry; a Site rebuild is unnecessary.

## Inputs and representation

- Research source: `b492d4273a694b60f6d57136e6aa8295c35eb5e7`, draft PR #96.
- v0 source package `EN01_reproducible_physics_and_3D_preview_20261109.zip`, SHA-256 `d73f680bb6384ca35b7d3b6cb684c5d2d7be26ac78c3858947c765c59ef38d4c`.
- v1 source package `EN01_v1_CAD_pose_diagnostic_2026-11-09.zip`, SHA-256 `e9feb2b29572f522676f761685dacd732fab36c1145e9a58cb7faa1e761c1ca0`.
- Source raw traces remain in their existing private packages. Only compact measured replay states are published, with original trace and model SHA-256 in each JSON.
- 8 x 300 frames, 6 seconds each, from previous MuJoCo 3.14.0 calculations. No new physics was executed for publication.
- The exporter preserves every recorded time, position, quaternion and independent joint coordinate. Contact flags are displayed discretely; interpolation between physical poses only smooths playback.
- Render dimensions and joint placements are read separately from the corresponding source MJCF, including the smaller v1 wheels and offset legs.

v0 is a hand-built approximate model. v1 uses CAD-derived nominal placement but is still a surrogate: the closed linkage, actuator parameters, mass distribution and real-world performance remain unvalidated. Both jump scenarios fail in both versions. The UI states these limitations and does not claim CAD-accurate motion or a successful jump.

## Verification

- Verified original package checksums and all v0 manifest files; v1 model hashes match raw-trace metadata.
- All 2,400 frames round-trip against the original raw qpos, without changing a value.
- All 57 prior catalog entries remain identical; one new entry advances revision 36 to 37. Registration is idempotent. All 11 assets have checked size and SHA-256.
- `node --check` passed for the replay module.
- In the browser preview, all eight model/scenario selections loaded the expected measurements and verdicts. Shared playback controls and data-driven telemetry are present.
- The cloud browser cannot create a WebGL context. Actual 3D rendering/camera motion and Android touch remain unverified; the page reports WebGL failure explicitly while keeping scenario selection and recorded measurements available. Do not claim a visual pass from this test.

No MP4/GIF, new Site, Site access change, original CAD redistribution, research PR merge, or modification of main.
