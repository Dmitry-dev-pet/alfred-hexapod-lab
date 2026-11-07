# Hexapod Lab — Freenove V10 startup fix

Fix for user-reported frozen **«Готовим заводские траектории Freenove…»** screen in V9.

## Confirmed cause

V9 `app.js` imported `./dual-stick-input.js` but no such file is present in V9 directory or its immutable 4-file manifest. The real shared class lives in `../freenove-dual-v8/dual-stick-input.js`. The browser aborts ESM module loading before calling `boot()`, leaving the placeholder forever.

Second independent bug: V9 `cameraReset()` reads `pilot.x`/`pilot.y` off CombinedPilot, but these belong to `pilot.walk` and the returned pose; adding undefined to camera coordinates can cause NaN. V10 uses `pose?.x??0`, `pose?.y??0`.

V10 is a **new versioned route** `/hexapod/freenove-combined-v10/` because local Termux refresh treats previously installed assets as immutable and rejects in-place edits. It has exactly two new assets, `index.html` and `app.js`; it imports unchanged V9 `combo-controller.js`, unchanged V8 `dual-stick-input.js`, V9/V8 CSS, V7 Freenove factory controller and offline precomputed V7/V5/V4 kits. No new physics, no change to OEM gait amplitudes and no change to robot motors.

Additional error listener in V10 HTML makes future uncaught ESM/module load failures visible, instead of leaving the placeholder unchanged.

## Tests

- Python dependency graph recursively resolves local ESM imports and CSS links to actual files; catches V9 defect.
- Python camera regression checks `pose`-derived coordinates and error listener.
- Node syntax checks V10 app, unchanged V9 composite and V8 input.
- Node `test_combo_v9.mjs` and `test_dual_stick.mjs` use the same original OEM kit, testing walking, head/body commands and multi-touch independent pointer state.

Device WebGL/real touch remains unverified. `main` not modified, do not merge PR without owner approval.
