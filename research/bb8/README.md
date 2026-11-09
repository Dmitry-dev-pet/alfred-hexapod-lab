# BB-8 CAD and private-player research

**Status, 2026-11-09:** source inventory and static STEP import tested; the full authentic HWeigo BB-8 mechanical assembly has **not** been imported or simulated and no BB-8 replay has been uploaded to ChatGPT Sites.

## Verified source inventory

The audited [sources.lock.json](sources.lock.json) pins **76 upstream CAD files** to exact Git commits, Git blob SHA-1 values, and byte lengths: [jrbedard/BB8-3D-Printed](https://github.com/jrbedard/BB8-3D-Printed), 15 STL; [HWeigo/BB8_Robot](https://github.com/HWeigo/BB8_Robot), 42 SolidWorks/native CAD/STL; and [PatchBOTS/bb8head](https://github.com/PatchBOTS/bb8head), 19 STL. Third-party binaries are **not** bundled into our repository.

The [dimension probe](stl_dimension_probe_20261109.json) records 12 actual upstream STL bounding boxes in native, unknown length units. HWeigo's head-base STL spans about 120×120×11 source units; PatchBOTS' outer head-ring about 285×285×41. This does not establish physical compatibility without checking source units and assembly transforms.

## Full SolidWorks assembly pathway

The original-drive candidate is [HWeigo/BB8_Robot/Model](https://github.com/HWeigo/BB8_Robot/tree/master/Model) at commit 7318411cfbfbffdbd68f75e3734a50c3933969e5. The 2018 SolidWorks assembly is **Model/初装.SLDASM**, with pinned Git blob SHA-1 bcfead6e92329575aa474fb4be05c24191ac5294. It references separate parts. Individual STLs are not an already articulated robot.

An experimental converter is [djbarrow/freecad_solidworks_plugin](https://github.com/djbarrow/freecad_solidworks_plugin) (LGPL-2.1-or-later). Its GUI importer handles SLDASM through importSolidWorks.py and uses assembly placements. **Important:** as inspected on 2026-11-09, its sw2fc/cli.py standalone entry point primarily handles individual parts, not a verified whole-assembly SLDASM conversion. Do not assume that a CLI SLDASM-to-STEP call works. Import the assembly inside FreeCAD and export STEP only after checking all component instances, positions, missing parts and conversion diagnostics.

We could not retrieve the large original SolidWorks assembly through the current connected GitHub binary-reading interface, and this container has no FreeCAD installation. Thus native assembly conversion remains **unverified**.

After independently obtaining a real assembly-derived STEP, our tested [step_ingest.py](step_ingest.py) exports distinct static solids with already applied world positions. It refuses mismatching source hashes and requires explicit length units.

~~~bash
python research/bb8/step_ingest.py \
  --step /private/verified-bb8-assembly.step \
  --source-sldasm /private/HWeigo/Model/初装.SLDASM \
  --expected-sldasm-git-sha1 bcfead6e92329575aa474fb4be05c24191ac5294 \
  --units mm \
  --out /private/bb8-step-cache
~~~

The **mm flag is an example** requiring independent verification from the CAD provenance and expected sizes. STEP solid traversal may flatten components; one connected solid is not necessarily one actuator or one named CAD part. Generated STL exports are *static geometry only*. Physical joints, axes, limits, motors, contact materials, masses and inertias still need independent reconstruction.

The converter was actually tested here using a synthetic two-part STEP assembly (not HWeigo's original): each part stayed separate and its X placement of ±50 mm was maintained, scaled explicitly into metres. **4/4 importer tests pass; 9/9 combined BB-8 local tests pass** in Python 3.13.

## MuJoCo physics: diagnostic only

MuJoCo 3.14.0 was restored from the checked offline runtime. Four earlier CPU mj_step experiments on a simplified proxy sphere and internal pendulum are stored in [proxy/receipt_20261109.json](proxy/receipt_20261109.json). These are **not source-CAD-derived BB-8 dynamics** or measurements on a real robot. The original BB-8 drive, steering and head mechanics have not been reproduced.

## Private player and licenses

The owner confirmed the existing ChatGPT robot player is **private**. Private interactive review is distinct from public redistribution; public release requirements should not block the physics research. A hosted private service still copies upstream meshes, and the public GitHub repositories did not show clear reuse licences during this audit. Preserve attribution and private storage; clarify permitted uploading before placing third-party meshes in a cloud-hosted player. Independent geometry can test the private preview plumbing in the interim.

Use the existing [replay pipeline](../../docs/hexapod/REPLAY_PIPELINE.md) when authentic physics is ready; BB-8 must remain review-only until original geometry, articulation and replay data integrity are verified. Do not create a new site, require MP4, or alter main.

## Reproducible commands

~~~bash
python research/bb8/cad_audit.py inventory
python research/bb8/cad_audit.py fetch --repo 'HWeigo/BB8_Robot' --path 'Model/*.STL'
python research/bb8/cad_audit.py inspect --repo 'HWeigo/BB8_Robot' --path 'Model/*.STL'
python -m unittest discover -s tests -p 'test_bb8_*' -v
~~~

Downloads go to private cache under ~/.cache/alfred-bb8-cad, never into the project repo. Hash and size are checked. Research priority remains a coherent **single-source HWeigo assembly**, not a visually convincing but physically invented mix of parts.
