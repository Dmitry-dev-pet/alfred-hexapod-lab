# Canonical Alfred Robot Library — migration 2026-11-11

One **visible** library is assembled from GitHub **main**, at one pinned
commit, by [catalog.js](catalog.js). It is a compatibility layer, not a
third copy of the catalog and not another website.

| Source in main | Cards | Purpose |
|---|---:|---|
| [../hexapod/library.json](../hexapod/library.json) | 77 | Historical simulations with per-study HTML/JS/JSON and verified file manifests |
| [site-only-legacy.json](site-only-legacy.json) | 2 | Published EN01 v8/v9 cards whose complete source assets were not present in the historical source branch |
| [../replay/catalog.json](../replay/catalog.json) | 3 | Shared data-only .robotreplay recordings |
| **Unified view** | **82** | 12 robot/model families, not 82 different robots |

The two EN01 site-only cards preserve their observed public Site URLs and
their original file size/SHA-256 descriptors. **Those ten source assets
have not been recovered into main**; their existence on the Site does not
prove they can be reproduced from GitHub alone.

Four historical pages (microduck-step-goal-v1,
microduck-single-support-v1, openbeni-recovery-v1,
openbeni-colored-v1) are **present in GitHub source but currently return
404 on the deployed Site**. They appear in the unified catalog as
site_status: not-published and the old library UI disables their links
until a real Site asset publication or migration makes them available.
Do not silently label them playable or erase the saved experiments.

## Why the historical files were imported

A single reviewed Git tree snapshot brought docs/hexapod/ from
feature/android-local-offline-viewer-20261104 at commit
ac1ae1ac90f10640ed2ea1b0ff4525580c130510 to main,
without merging its 381 experimental commits. The original legacy
manifest remains byte-for-byte unchanged: 77 records, revision 61,
334 integrity-listed assets totaling 89,288,219 bytes. The entire
subtree includes existing JSON, HTML, JavaScript, CSS and documentation,
but no new binary STL/STEP files. Git already contained those objects.

Run node tests/robot-library.test.mjs to validate the three source
manifests and independently hash every one of the 334 preserved files.

## How the current Site uses this

The existing /hexapod/ bottom-sheet library shell now first imports
../robot-library/catalog.js, reads the GitHub main SHA and loads
all three manifests from **that same commit**. It then shows the
combined list, including /replay/?recording=ID routes for data-only
recordings. If GitHub cannot be reached, the existing Site-local
library remains its fallback. The source includes no new publisher,
worker, polling daemon, database or site.

**Publishing GitHub main does not automatically publish a ChatGPT Site.**
The existing Alfred Robot Player must be updated **once**, without
creating a new site, to include the latest
/hexapod/player-shell.js, the new /robot-library/catalog.js,
and approved legacy assets. The common /replay/ player remains
unchanged. Verify the actual touch UI, replay selection, pausing,
seeking, and that the four unpublished legacy routes are not offered
as working pages before declaring the Site migration finished.

New compatible experiments should publish a .robotreplay file and
update **only** docs/replay/catalog.json in GitHub main. The
unified reader composes the next updated catalog without any duplicate
manifest edit or Site redeployment. Legacy content can be converted
gradually, preserving exact recorded data and provenance; delete or
retire a legacy page only after an independently verified replacement.
