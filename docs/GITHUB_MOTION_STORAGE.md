# Motion recordings in GitHub

All 377 motion payloads formerly deployed in Alfred Robot Player version 33 are preserved byte-for-byte under `recordings/site-v33/`, pinned at commit `ba71e97eaaacabb5aa940e364042ace371fbd168`: 244,287,539 bytes. `recordings/site-v33/manifest.json` records their original paths, sizes, SHA-256 and Git blob IDs. This includes historical motion data and all three standalone `.robotreplay` files. The owner explicitly confirmed permission to publish the embedded HexArth CAD in public GitHub before transfer.

The Site keeps its existing player, separate model packages, vendor assets, videos and discovery metadata. It no longer bundles motion payloads from `assets/robot-replays/`, `assets/robot-motions/`, `hexapod/data/`, or `.robotreplay` files. Separate model geometry is not a motion recording and is not moved by this migration.

## Playback and compatibility

`docs/recording-store.js` verifies and loads immutable raw GitHub URLs. The existing Site's `github-live-boot.js` installs the shared transport before replay modules run, including a first visit before service-worker control. Only allowlisted same-origin recording paths are remapped. Existing legacy loaders and their source/pose checks remain intact. Requests omit Site credentials, reject redirects, bound sizes and check SHA-256 before use. Previously verified bytes can be read from browser cache offline; a corrupted cache entry is rejected.

The installed `docs/robots/recording-assets.json` maps historical Site paths to the same immutable GitHub snapshot. It keeps existing loaders working while the canonical main catalog is reviewed. Replay catalog revision 5 points all three standalone recordings directly to immutable GitHub URLs. This does not change recording IDs or their bytes, sizes or SHA-256.

The canonical main catalog and the Site have separate publication cycles. New compatible recordings still use one self-contained `.robotreplay` file. This migration does not introduce per-robot player variants or recalculate physics. The new root transport and boot are copied into the existing Site build; `docs/` alone still needs the Site's established vendor/worker assets to run.

## Verification

All 377 immutable raw responses were downloaded again after upload, with exact size/SHA-256 and `Access-Control-Allow-Origin: *` verified. Actual MicroDuck binary and EN01 gzip loaders also decoded selected recordings through the new live GitHub transport.

Tests:

- `node --experimental-vm-modules --test tests/recording-store.test.mjs`
- `node --test tests/recording-separation.test.mjs`
- Existing catalog and replay format tests.

Run `python3 scripts/check_recording_separation.py --root <Site checkout>` as a required publication step. It rejects bundled payloads and non-GitHub standalone URLs. Negative tests cover each prohibited payload location.

Browser/WebGL rendering and physical touch remain separate checks. Work browser access was disabled in the migration session; source adapters and HTTP tests are not represented as a visual pass.
