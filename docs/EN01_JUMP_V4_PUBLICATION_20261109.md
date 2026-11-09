# EN01 v4 source-jump recording publication

Owner explicitly requested publication of all 24 existing records. Existing Site/project and audience are preserved.

New route: https://alfred-robot-player.curlyeave.chatgpt.site/hexapod/en01-jump-v4/

The page reuses the existing nine-body/73-part original-CAD renderer and common player controls. Three selectors cover jump/control, four assumed pulse durations and three assumed initial CAD anchors. All 12 jump trials remain failed because none achieves a two-second clean stable landing. Two cases have qualified brief upward takeoff; the default zero-anchor / 5-ms-clock case shows 3.709mm / 47ms / 0.157s landing support. The alternative +0.04-rad anchor shows 7.098mm / 64ms / 0.138s. Clock periods, servo/CAD zero and dynamics remain uncalibrated hypotheses.

Source research: draft PR #113, commit 53a714ee202f36b5547da6ad189676cdc5c1fc82. Exact source archive is 8,811,019 bytes, SHA-256 43aea694c3704a59ad659674650e77268131f58ff1203e3e1e8b965a875dbd8d; [verified raw/replay archive](https://drive.google.com/file/d/1REeM7FhVl_CcE4rS_Z5v8Yz0I7TYkxmI/view?usp=drivesdk). No physics is rerun.

All archive manifest members are verified. All 24 exact compact JSON replays are hosted on demand under `/assets/robot-replays/en01-jump-v4/`, with per-file size, SHA-256, trace/model/firmware/condition binding and viewed-record caching. All 7,200 visual frames compare exactly with archived body poses and contact/tilt/closure telemetry. The original CAD hash remains 8b034155dc50bb6c4e94a55bfa4ebca7b1fb742f2f38fc5c6c9f158c0cae313b. Raw traces and original binary CAD are not added to GitHub.

The small route/index and reproducible publication code are in GitHub. Catalog revision 43 → 44 appends one entry and preserves all previous 64 entries byte-for-byte as JSON objects. Existing physical and joystick records are retained. Research PR #113 and main are not merged.

Validation: all 24 files decode through the actual browser loader; four focused integrity/cache tests plus six existing service-worker tests pass; unsafe/incomplete/forged files are rejected. JavaScript syntax passes. Preview loads default and alternative flight metrics; pause/seek to 6.00s and all three selectors work. Cloud Chrome lacks WebGL, so actual rendered 3D, camera and Android physical touch are unverified. No overall CI pass is claimed; commits use [skip ci] to avoid legacy hosted workflows.

Reproduce with `python scripts/publish_en01_jump_v4.py --archive /path/to/EN01_v4_source_jump_20261109.zip --site-dist /path/to/existing/site/dist` and `node tests/test_en01_jump_v4_publication.mjs . /path/to/existing/site/dist`. Source code hashes, byte counts and source bindings are fixed in the script/index.

Run ledger: https://github.com/Dmitry-dev-pet/truthrail/issues/316
