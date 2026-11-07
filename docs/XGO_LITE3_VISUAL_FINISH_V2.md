# XGO-Lite3 appearance V2 — private-local viewer restoration

6 November 2026. New immutable version `/hexapod/lite3-finish-v2/` in existing Hexapod Lab player, leaving V1 and actual simulated physics untouched.

## Evidence

- 17 pinned original STL meshes (vendor `LuwuDynamics/luwu_mjlab` commit `447945fd40fb5585271dce4ea8cc82567916032a`) and 4 original 8/9.5-second MuJoCo CPU/ONNX replays. Their mesh/dataset bytes remain unchanged from V1.
- Official XGO-lite3 full STEP assembly v2026.09.18, MD5 `f413d18e4546e7d7255ce1a4976ba887`, SHA256 `edccad2b4ba188fe1dc53d820d9e63aeb23f97f5edf292b4411e0fdbd1d151bf`; transported by [Mac runner](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42261439835). The CAD has 259 COLOUR_RGB-style assignments but **one and only one RGB value**: approximately `#CAD1EE`. This is not a realistic material assignment. No UV image textures or bitmap artwork found in this STEP file.
- Official vendor photo references: [Luwu store image](https://shop.xgorobot.com/cdn/shop/files/2S.85.png?v=1785986727&width=3840) and [manufacturer page](https://www.xgorobot.com/en/lite3.html), showing dark graphite body, metallic limb mechanisms, frontal display and articulated gripper.

## Changes

- Per-face **procedural vertex colors** on the original STL triangles (no movement or geometry edits), fitted to a dark graphite body, silver joints/arms, darker feet.
- Runtime-only **CanvasTexture** for front display with purple eyes and reconstructed XGO lite3 side markings. These are illustrative, not a copy of real vendor firmware output or a manufacturer texture atlas.
- Screen and logo follow the measured MuJoCo body position and orientation from each replay frame. The 17 original rigid-body placements still come from measured MuJoCo FK; no browser physics or fake gait.
- Reuses the exact existing private STL cache and four original data JSON files. The browser needs no extra external texture URLs.

## Reproducibility and limitations

Local: `node --check docs/hexapod/lite3-finish-v2/app.js` passed. Five Python source/contract tests passed (source STEP MD5, reference counts, unchanged meshes/data, body attachment, no new external texture loader). Automated Chromium WebGL smoke was attempted but this runtime blocks navigation with `ERR_BLOCKED_BY_ADMINISTRATOR`; **Samsung Tab S7 device verification remains pending**. This is **visual reconstruction only**; material colors, side markings, display graphics and their exact positions are not known to match factory hardware. Source evidence STEP remains private on Google Drive, not public repo.

## Install

Within the existing Android Termux player at `http://127.0.0.1:8808/hexapod/`, tap “Обновить библиотеку” and select “XGO-Lite3 · материалы V2”. The first open uses previously cached original STL files. No changes to original physics or `main`, no merge without owner approval.
