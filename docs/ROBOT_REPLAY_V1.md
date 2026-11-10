# One file per recording or collection

The existing player opens `/replay/`. JavaScript, Three.js and camera/transport
controls are bundled in the player. A recording is **data only**, with no HTML,
JavaScript, runtime imports, external meshes or required sidecar files.

Open a local `.robotreplay` file, drop it on the player, or supply an HTTP(S) URL
using the link form or `/replay/?file=<URL-encoded file URL>`. Cross-origin URLs
must allow browser CORS. Local files are processed on the device, without upload.
Catalogs are optional discovery: the only playback input is the package.
The server hosting a URL can observe its download. No physics is recomputed.

## Container

UTF-8 JSON with `schema: robot-replay/v1`, optionally gzip-compressed. The preferred
extension is `.robotreplay`. Standard gzip and base64 keep the format portable.
`scripts/robot_replay.py` provides shared binary block and package writers;
`scripts/pack_hexarth_replay.py` is a source-data adapter, not player code.

Required top-level keys: `schema`, `metadata`, `scene`, `nodes`, `meshes`,
`clips`, `default_clip`. Unknown versions are rejected. A package may contain
several clips that share one model. The first version supports indexed triangle
meshes, solid colors, PBR roughness/metalness, rigid world transforms, flat floor
planes and boxes. Texture images, skeletal deformation and live physics are not
part of v1.

* `metadata`: title and description; optional source/commit/hash provenance.
* `scene`: `units: m`, `up_axis: Z|Y`, background `#rrggbb`, camera follow-node
  index and offset in Three.js Y-up coordinates; hemisphere/directional lights.
* `nodes`: ordered unique `{id}` objects. Pose tracks store **world**, not parent-
  relative transforms, so no simulator-specific kinematics runs in the player.
* `meshes`: node index, vertex/triangle counts, positions, indices, uniform scale,
  local translation in metres, material color/roughness/metalness. Each vertex is
  first multiplied by scale, then translated, then transformed by its node pose.
* `clips`: unique id, title, duration_s, frame_count, frame_stride, frames,
  environment, optional result/metrics/source hashes and measured normal contacts.
  Environment items contain kind (`plane|box`), size, position and color; box size
  is full extent, not half extent. Planes are horizontal in the declared up axis.
* A frame contains `[time, node0.x, y, z, qw, qx, qy, qz, node1.x, ...]`.
  Stride is `1 + 7 * nodes.length`. Time strictly increases; quaternions must be
  normalized. Playback interpolates positions and uses quaternion slerp.
* A binary block has `encoding: f32le-gzip-base64|f64le-gzip-base64|u32le-gzip-base64`,
  element `count`, SHA-256 of the **uncompressed** bytes, and base64 `data`.
  Mesh positions use f32, indices use u32, frame/force values use f64. Recorded
  values are not rounded during packing.
* Contact values are frame-major f64 normal forces in newtons with `labels`.

The reader bounds compressed/decompressed sizes and decoded allocations,
verifies each block hash, mesh indices, frame lengths/times and quaternions,
and rejects corruption before replacing the active scene. Provenance strings
and metadata render as text, never markup or executable code.

## Export contract

Every completed simulation should export this package in the same process that
saves its measured states, including failed experiments. Prepare meshes once
per model version and embed them in each standalone package (or once per
multi-clip collection). Exporting existing states does not rerun integration.
Publish the file and optionally its catalog descriptor. No per-robot app/page
or Site deployment is required for later compatible recordings.

Existing legacy modules remain available. This is an additive common reader,
not a claim that all earlier experiments have already been migrated. The HexArth package is hosted at its owner-authorized public Site path. CAD
payloads are not placed in the public research source repository.


## Automatic library opening

The normal flow is library selection → shared `/replay/?recording=<id>` → one
verified package download → immediate playback. File selection is optional.
The main library merges `robot-replay-catalog/v1` descriptors with legacy entries;
legacy catalogs, modules and service-worker constraints are preserved.

`dist/replay/catalog.json` is a fast bundled fallback. The common catalog reader
also reads `docs/replay/catalog.json` from the existing working GitHub branch,
pinning the branch commit. Newer validated catalog revisions update automatically.
Each entry contains id, title, detail, duration_s and a recording reference with
schema, public URL, exact size and SHA-256 of the whole package. Package hosts
must permit CORS for cross-origin reads. The player fetches no per-recording JS.

The owner explicitly confirmed a public file path on 2026-11-10. The complete
HexArth package is now staged for the existing public Site at
`/replay/records/hexarth-1abc147874c9a488.robotreplay`; no CAD payload is placed
in the public research GitHub repository. Catalog metadata may be public.


## External recording storage (owner correction, 2026-11-10)

The Site contains the common player and small catalog fallback only. Standalone
`.robotreplay` payloads MUST NOT be copied to `dist/`. Run
`python scripts/check_recording_separation.py` before Site packaging. New
registrations require an external public HTTPS URL. Site-relative URLs and
chatgpt.site recording hosts are rejected by the registration helper.

The player resolves the current GitHub branch and reads the catalog at its
immutable commit before falling back to cached or bundled metadata. New entries
are discovered without a Site deployment. Each file is requested without Site
credentials and verified against its exact size and SHA-256 before decoding.

The prepared read-only file service is
`https://alfred-robot-recordings.vscode3kilo.workers.dev/records/<sha256>.robotreplay`.
It uses independent immutable storage, not Site build assets. Its reusable
source is `scripts/recording-worker.mjs`; it contains no provider credential
and accepts no public writes. Object manifests name 8 MiB chunks, allowing
files up to the common reader's 100 MiB limit.

Producer steps: publish the verified standalone file in the authorized external
store, verify its public URL and unchanged digest, then register its descriptor
and commit the small catalog. No renderer, per-recording JavaScript or Site
publication belongs in this path. Legacy modules remain compatible while their
formats are migrated separately.

Migration admission: transfer of the two existing HexArth payloads to this exact
new public host was blocked by automatic approval review. The source files and
production Site v30 remain intact; deploy this prepared version only after the
owner explicitly authorizes that destination, the transfer finishes and public
size/SHA-256 readback passes.
