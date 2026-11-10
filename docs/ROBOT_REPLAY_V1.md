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
not a claim that all earlier experiments have already been migrated. The large
HexArth CAD package is delivered separately and is not committed to this public
Site or public research source repository.
