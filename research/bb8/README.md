# BB-8 CAD audit — source-first research

**Status 2026-11-09: CAD inventory + verified downloader and STL dimension inspector prepared.**
No full upstream CAD binary was downloaded in this execution environment, no parts have been assembled, no actual MuJoCo \`mj_step\` was executed, and **nothing has been published to either Android or ChatGPT Sites robot player**. This draft intentionally leaves \`main\` and \`docs/hexapod/library.json\` unchanged.

## What we verified from upstream GitHub repository trees

| Source | Pinned records | Intended use |
|---|---:|---|
| [jrbedard/BB8-3D-Printed](https://github.com/jrbedard/BB8-3D-Printed) | 15 STL files: eight outer-body segments, dome, neck, spine, gears | Visual shell, subject to scale/reuse rights |
| [HWeigo/BB8_Robot](https://github.com/HWeigo/BB8_Robot) | 42 CAD files, predominantly SolidWorks 2018, including an SLDASM assembly and 3 STL files | The most promising complete **single-source** BB-8 internal mechanics |
| [PatchBOTS/bb8head](https://github.com/PatchBOTS/bb8head) | 19 STL files | Optional magnetic head, mount, roller/caster design |

All source files are pinned by upstream **commit SHA**, exact Git blob SHA-1, path, and byte count in [sources.lock.json](sources.lock.json). The lock is a manifest, **not** an archive of third-party parts. Source code and STL content is not copied into this repository.

**Private player vs public redistribution:** the existing ChatGPT robot player is **private**, as the owner has confirmed. An internal access-controlled preview of the CAD simulation should **not** be blocked by public-site publication criteria. Continue building and testing geometry and physics privately. However, a private hosted service still creates copies of models: absence of a license does not automatically grant permission to upload or redistribute third-party binaries. Keep source attribution and access restrictions; check any applicable rights before uploading upstream CAD into a hosted service. Do not publish third-party STL to public GitHub, and do not make a public asset bundle without appropriate permission. For a private preview where third-party copying is not authorized, use independently created geometry and recorded physical trajectories. [RoundRover](https://github.com/EmmeEffe/RoundRover) is GPL-3.0 and remains an alternative with explicit license terms.

## Tool

Run from repository root (Python 3.10+ / stdlib, no MuJoCo required):

~~~bash
python research/bb8/cad_audit.py inventory
python research/bb8/cad_audit.py fetch --repo 'HWeigo/BB8_Robot' --path 'Model/*.STL'
python research/bb8/cad_audit.py inspect --repo 'HWeigo/BB8_Robot' --path 'Model/*.STL'
python research/bb8/cad_audit.py verify
python -m unittest discover -s tests -p 'test_bb8_cad_audit.py' -v
~~~

Downloaded files live **outside the Git checkout** under \`~/.cache/alfred-bb8-cad/<repo>/<commit>/<path>\`. The fetcher refuses incorrect byte count and Git blob hashes. Use \`--cache PATH\` to override storage. STL inspector reports native coordinate extents and explicitly does not assert millimetres, true masses or original CAD assembly positions.

**Validation performed in the current execution environment:** 5/5 pure Python unit tests passed on synthetic STL files; no live network fetch or genuine STL geometry inspection succeeded. This distinction must be kept in later updates.

## Physics/replay workflow — not yet executed

1. Obtain HWeigo SolidWorks assembly and its referenced parts into a private local cache (one coherent original design, not an arbitrary cross-source mashup). Check assembly loading in a capable SolidWorks/FreeCAD converter and **preserve all local transforms and joint axes**; a folder of separate STLs is not an assembled robot.
2. Measure native shell diameter, wall thickness, internal-clearance budget and frame placement. Then evaluate whether jrbedard outer-body STLs or PatchBOTS head can be adapted without structural interference, while retaining source credits and redistribution restrictions.
3. Specify accurate component masses/inertias (estimate vs verified), motor torque, internal wheel/shell contact, floor friction, head magnet model, camera weight, and solver tolerances. Do not silently swap original STL for invented geometry in the final visualization.
4. Execute CPU MuJoCo \`mj_step\` under static/drive/brake/turn cases, record raw qpos, qvel, time, actuator command and contact diagnostics. Preserve failed runs as failed.
5. Export verified physical replays through the existing [replay pipeline](../../docs/hexapod/REPLAY_PIPELINE.md) on a research branch with `publish_policy: review_only` while original assembly transforms, contact behavior and playback are unvalidated. A private preview is distinct from a public distribution, but license constraints on third-party mesh copies still need attention.
6. After physical validation, prepare an **internal preview** for our existing **private** ChatGPT robot player through a supported integration. No new site, no public release, no MP4 requirement, no merge to `main` without approval. The player should replay actual MuJoCo states, not staged motion.

### Current hard blockers

- Execution container does not have MuJoCo and outbound network requests to GitHub/PyPI fail; we have not obtained the upstream binary CAD here.
- \`HWeigo/BB8_Robot\` publishes a native SolidWorks assembly, not a ready-to-run MuJoCo MJCF; assembly-to-joints conversion has not been done.
- The player is **private**, not public; public redistribution of CAD is therefore not the active task. Still confirm whether the chosen private hosting arrangement permits copies of unlicensed third-party assets. This is separate from, and should not block, the physics research.

This is the first audited import milestone, **not a finished simulation**.

