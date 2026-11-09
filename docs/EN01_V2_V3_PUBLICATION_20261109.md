# EN01 v2 / v3 publication

Owner requested publication of existing recordings on the existing ChatGPT Site.

- Existing Site: https://alfred-robot-player.curlyeave.chatgpt.site
- New route: /hexapod/en01-v2-v3/
- Six v2 plus 31 v3 records; 11,100 archived frames, each 9 physical body poses and contact/tilt/closure telemetry. Raw-state bodies and contact values compare exactly. No physics rerun.
- Reuses the original 73-part CAD and original four-bar adapter transforms from PR #106/#107. Original CAD SHA-256: 8b034155dc50bb6c4e94a55bfa4ebca7b1fb742f2f38fc5c6c9f158c0cae313b.
- Exact archived replay JSON bytes hosted on the existing Site, fetched on demand and checked by SHA-256/size/source binding. Viewed recordings and CAD are browser-cached. Raw qpos/contacts and source archives stay in their existing private stores.
- Small route/index plus source packaging code and hashes in GitHub. Catalog 39 → 40; all prior 60 entries preserved. The old eight EN01 v0/v1 recordings remain unchanged.
- Selector groups all 37 records by version and scenario, including every negative crouch/jump/fall and heavy-mass outcome. Default v3 gated stand shows 5.07cm max displacement, 13.24° max pitch. Upright for 6s is a diagnostic label, not hardware success.

## Validation

Both complete source archive SHA-256 values and all archive-manifest members verified. All 11,100 visual frames match raw body poses and telemetry exactly; unit quaternions and source model/STEP hashes checked. All 37 files decode through the actual browser loader. Four focused tests validate malformed/forged records, unsafe or incomplete index, and viewed-record offline caching; six existing service-worker tests pass. JavaScript syntax checks pass. No broad CI pass is claimed.

Preview browser loaded both versions and a failed v2 jump; pause and seek to 6.00s verified. Cloud Chrome does not offer WebGL: actual rendered 3D, camera gestures and Android touch remain unverified. No MP4/GIF, new Site, audience change, main or research PR merge.

## Reproduce

Run `python scripts/publish_en01_recordings.py --archives /path/to/archives --site-dist /path/to/existing/site/dist` on the exact v2/v3 archive filenames and SHA-256 values recorded by the script. Existing catalog registration is immutable and idempotent. Run `node tests/test_en01_recordings.mjs . /path/to/existing/site/dist`.

Run ledger: https://github.com/Dmitry-dev-pet/truthrail/issues/312

## Deployment receipt

Existing Site version 7, source commit `1e94446522736b8abf213665c55f648e841424fd`, deployment `appgdep_6af1ff51c20881918ff5a61c824d4639` succeeded at 2026-11-09T17:14:19Z. Audience unchanged.
