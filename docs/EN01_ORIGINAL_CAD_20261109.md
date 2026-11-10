# EN01 original CAD geometry in the existing ChatGPT player

The owner confirmed that the eight EN01 recordings work and requested original STL geometry. This adds `/hexapod/en01-original-cad-v1/` to the same live Site library. All existing pages, catalog entries, eight recorded trajectories and research branches remain unchanged.

## Source and conversion

The full assembly comes from `fuwei007/Navbot-EN01` at `3139f2fb32b38d10611feb905e16f8aa012f2f2e`, file `Hardware/RebotModel/OriginalRobotModel.stp`, 12,358,579 bytes, Git blob `9156683a0bb694c84dc541a3e26b5f6b08db1a41`. The author identifies this as the robot model and provides related STL printing files in the same source repository.

OpenCascade XCAF reads component occurrences with composed assembly transforms, including duplicate names. The assembly contains 73 non-empty component occurrences and 157 solids, including the detailed bodies of bearings and motors. The exported mesh retains every component, its source name/path, assembly placement and available original colour. No primitive replacement, guessed dimensions or rescaling is used.

The source CAD frame (millimetres, Y vertical) is mapped by the proper rotation `(X, -Z, Y)` and a documented translation into the replay frame. A finer STL package uses 0.06 mm linear / 0.2 rad angular tessellation. The mobile visual mesh uses 0.15 mm / 0.45 rad tessellation: 237,519 triangles, 5,717,216 bytes of JSON including source metadata and binary typed arrays. This changes tessellation detail, not the CAD dimensions. The original STEP and binary STL payloads are not committed to Git.

The STL download package `EN01_original_CAD_STL_20261109.zip` contains 73 binary STL parts in original global assembly coordinates (mm), a source/name/checksum manifest and README. These are exports from the authentic STEP assembly; they are not claimed to be byte-identical to the author's separate print-part STL files.

## Motion boundary

This is a replacement of the visual geometry, not a new physical model. It uses the existing unchanged v0/v1 root pose and four recorded joint angles. Original leg components follow each recorded hip angle as a rigid diagnostic group; wheel rotor/hub components follow the wheel angles. The passive four-bar articulation is not solved here. Source geometry is preserved even where it differs from v0's dimensions. Consequently contacts, forces and drift remain measurements of the earlier surrogate model, not validation of the rendered CAD shell. The UI states this explicitly.

## Verification

- Source byte count and Git blob SHA checked before import.
- All 73 source components retained; each STL export has a checksum in the private download package.
- Checked proper coordinate rotation, finite vertex/index buffers, index bounds, triangle counts, original extents and normal generation.
- With actual Three.js geometry and group transforms, the assembled rest positions match the original CAD-derived vertices: maximum coordinate residual `8.29e-10 m`.
- An independent CPU render of the full assembly was inspected and shows the original chassis, servos, legs, wheel hubs and small hardware.
- JavaScript syntax check passed. Browser preview loaded 73 parts and all eight model/scenario combinations with the expected measurements and FAIL verdicts.
- Cloud Chrome cannot create a WebGL context. Actual GPU rendering/camera and Android touch are not claimed verified in this environment. The owner's successful device check applies to the preceding replay page, not automatically to this new geometry.

Catalog registration advances revision 37 -> 38 and entries 58 -> 59, preserves every previous entry, checks all four new assets' byte counts/SHA-256, and is idempotent. No Site rebuild, access change, MP4, invented physics or merge into main.

## Rebuild

Place the pinned source STEP at `research/navbot_en01/cad_visual/source/OriginalRobotModel.stp`, then run `python research/navbot_en01/cad_visual/build_original_meshes.py --mobile` with the installed CadQuery/OpenCascade runtime. The accompanying inspection manifest checks component occurrence order/names before conversion. Run without `--mobile` for the finer STL package.
