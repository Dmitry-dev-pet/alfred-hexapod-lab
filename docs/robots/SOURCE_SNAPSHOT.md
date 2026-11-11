# Provenance of the historical robot library in main

The one canonical library interface remains
[docs/robots/catalog.json](catalog.json) +
[docs/robots/library.js](library.js). It already describes 79 historical
cards and dynamically combines them with 3 data-only entries from
[docs/replay/catalog.json](../replay/catalog.json). This change does
**not** create another catalog implementation.

## Recovered source snapshot

The original, publicly accessible source tree
docs/hexapod/ from commit
ac1ae1ac90f10640ed2ea1b0ff4525580c130510
(feature/android-local-offline-viewer-20261104) was grafted to main
as a **single Git tree snapshot**, without importing the previous
381 experimental commits or changing project physics.

- Git tree SHA: d7af8dbf1c1eeaa78c47dd234386460afd33daaa
- Old catalog: docs/hexapod/library.json, revision 61, 77 cards, unchanged blob
  9e411d2f2c73fd276a3ad81d84f24a458e225559
- All 334 manifest-listed file paths and byte sizes were checked against
  the original feature tree, then again against the new branch tree.
  Every file retains exactly the same Git blob object (89,288,219 bytes
  across the 334 referenced assets).
- The complete docs/hexapod snapshot consists of 362 already public
  HTML, CSS, JS, JSON and Markdown files (around 94 MB total). It
  includes old experiment source/recording data, **not new STL/STEP binaries**.

This is a source-preservation step; it does not convert individual
historical studies to the shared .robotreplay format.

## Two published EN01 pages lacking original source

EN01 v8 (legs then impulse) and v9 (physics torque) exist on the
current public ChatGPT Site, and their cards are included in the
main docs/robots/catalog.json index. They were not present in the
restored feature tree. See [site-only-file-manifests.json](site-only-file-manifests.json)
for their ten published file paths and exact historic sizes/SHA-256.

**Those ten files are not yet independently recovered into main.**
Do not claim full GitHub source custody for these two pages. The
Site still serves the pages, and their links are preserved.

Four other entries (MicroDuck step goal, MicroDuck single support,
OpenBeni recovery, colored OpenBeni) have source files in the restored
GitHub snapshot but returned HTTP 404 on the current Site. The main
canonical index marks them pending rather than falsely playable.

## Deployment and tests

The existing Site has a separate publication lifecycle.
Copying files into GitHub main does not change the Site or
verify WebGL playback. Publish additional Site assets only through
a reviewed update to **the existing Alfred Robot Player**, not by
creating another website.

Run:
- node tests/robot-library.test.mjs — existing canonical 79+3 catalog checks
- node tests/robot-source-snapshot.test.mjs — source snapshots, exact
  byte-size and SHA-256 checks for all 334 referenced files

New compatible records continue to use one standalone .robotreplay
file and a change to docs/replay/catalog.json in main; no legacy
HTML/JS page should be generated for each new robot.
