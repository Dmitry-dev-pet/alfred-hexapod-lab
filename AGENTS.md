# AGENTS.md — Alfred Robot Lab

## Scope and authoritative sources
- This repository covers **many robots**, not only Freenove hexapods: HexArth, MicroDuck, EN01, BB-8, Beni, XGO, Stanford Pupper and related designs.
- Start with the current `README.md`, `docs/ROBOT_REPLAY_V1.md`, live `main`, project Issues and relevant PRs; never infer an up-to-date state from an old local checkout.
- Before new implementation, inspect existing experimental/stacked PRs and branch dependencies. Continue or extract existing verified work rather than duplicating it.
- Keep simulation, replay reader, GitHub catalog, the **published private ChatGPT Site** and physical robot observations as distinct layers. Main commits do **not** redeploy the Site.

## Physics and experimental claims
- Reproduce **actual measured simulation states** and report model version, source SHA, body mass/inertia assumptions, joint/actuator limits, contacts, test conditions and outcome.
- Explicitly distinguish OEM measurements, estimates, numerical simulation, real hardware experiments and unverified hypotheses. No uncalibrated simulation is evidence of actual hardware capability.
- Record failed trials and constraints (self-collisions, floor contact, penetration, IK or torque violations); do not count a pose change as a completed walk, climb, jump or self-righting episode.
- Prefer bounded local/session compute when verified available; Actions is for appropriate CI or specifically approved execution. Never pretend a dependency, GPU, Mac runner or physical device is available merely because it worked before.
- Store minimal reproducible scripts, parameters, digests and small receipts in GitHub. Keep heavy CAD/STEP/STL, raw traces and large rendered outputs in approved external storage with verified provenance, permissions and size/hash. Respect upstream CAD and model licenses.

## One player, data-only recordings
- The canonical universal replay contract is [`docs/ROBOT_REPLAY_V1.md`](docs/ROBOT_REPLAY_V1.md): one self-contained `.robotreplay` file for each record/collection; geometry, materials, terrain and measured motion are data, not executable JS.
- The existing shared `docs/replay/` reader owns playback, seek, camera and catalog loading. Do **not** create per-robot HTML/JS player variants for new compatible records.
- `docs/replay/catalog.json` on **current main** is the canonical v1 catalog. Never replace it with an older feature-branch catalog; preserve stable IDs, all existing entries, monotonic revision, immutable URL, exact size and SHA-256.
- New files need an **owner-approved** public HTTPS host with actual CORS and SHA-256 verification; no automatic Site rebuild or new storage deployment follows from merely adding a descriptor.
- The larger `docs/hexapod/library.json` is a legacy collection from research branches. Preserve its historical recordings and source links during migration; its card count is not the number of robots or validated physical outcomes.
- The established Alfred Robot Player Site is a **separate existing product**. Do not create a second Site or change publication/access settings without exact owner approval. Browser/WebGL/seek/touch verification must be real and reported separately from source CI.

## Review and safety gates
- Changes go through reviewed branches/PRs and appropriate existing tests; no bulk merge of old stacked or research PRs. Keep source licenses, failed experiments and unique research evidence recoverable.
- Before merging, compare against current main, check tests/CI and exact owner approval; after merging, verify the resulting source. For hosted behavior, independently check the Site before claiming deployment.
- Do not commit private files, credentials, restricted CAD, personal material or unlicensed third-party assets to this public repository.
- Current consolidation queue is tracked in [Issue #146](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/146). Refresh its state rather than treating any saved counts as timeless truth.
