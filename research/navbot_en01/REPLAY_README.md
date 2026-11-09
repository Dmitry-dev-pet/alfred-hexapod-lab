# NavBot EN01 · 3D replays through existing viewer pipeline

This branch extends **PR #91** and imports the four-axis MuJoCo EN01 baseline from **PR #95** without modifying either draft PR or main. No new player. **No MP4 / GIF is produced.**

Physical evidence: 4 × 3000 verified mj_step calls at 2 ms = 6 seconds per scenario, 300 recorded qpos/contact frames each. The pre-existing recorded runs are available in the owner-facing chat archive with SHA256 and the EN01 receipt in research/navbot_en01/results. Never infer success from a contact gap alone: both jump scenarios FAILED. Standing/crouching remain provisional with 0.34–0.39 m drift.

The four declarative jobs use the already implemented scripts/replay_pipeline.py; the EN01-specific exporter verifies source SHA, model geometry/provenance, qpos, joint states, monotonically increasing time, contact, mode, and status before producing the 16-channel 3D replay. Browser renderer shows both independently controlled hips and both wheels; geometry is schematic and explicitly **not original STEP**.

From repository root with MuJoCo 3.14.0+numpy installed:

~~~sh
python scripts/replay_pipeline.py run experiments/replay-pipeline/navbot-en01-stand-20261108.json
python scripts/replay_pipeline.py run experiments/replay-pipeline/navbot-en01-crouch-20261108.json
python scripts/replay_pipeline.py run experiments/replay-pipeline/navbot-en01-jump-20261108.json
python scripts/replay_pipeline.py run experiments/replay-pipeline/navbot-en01-jump-fast-20261108.json
~~~

If the exact previously recorded raw JSON is already restored in results/navbot_en01/, pass --skip-execution. All jobs are publish_policy=review_only, so --publish refuses them. After physical verification and a review of original CAD and Android Three.js, use the existing viewer library update/owner-managed branch; do **not** merge or publish automatically. Big raw traces remain outside GitHub with exact SHA. No GitHub Actions physics or paid GPU.

## Owner-selected complete EN01 pipeline — Nov 9, 2026

**EN01 is the P0 robot.** SeBaJu is a benchmark, not the main hardware target. The existing one-command route is `scripts/navbot_en01_replays.py`.

Run the actual CPU physics and build four deterministic interactive 3D replays (requires local MuJoCo 3.14.0 and numpy):

```sh
python scripts/navbot_en01_replays.py
```

If the original **Nov 8 CPU evidence ZIP with four full raw JSON traces** is available locally, restore the four sources from their **exact SHA-256 checksums** and build the same 3D replays **without re-running physics**:

```sh
python scripts/navbot_en01_replays.py --archive /path/to/navbot_en01_mu3140_cpu_20261108.zip
```

The importer matches *bytes by digest*, not ZIP paths. It reads only the four receipt-authorized traces, refuses missing/duplicate/modified files and preserves existing differing source files. It does not download anything, trust ZIP paths, bypass physics checks, or generate videos. `--skip-execution` remains supported for manually restored raw JSON. All four jobs remain `review_only`: the command creates *local review entries*, **not a publication on the owner's Android branch**, and never touches `main`.

Test importer (stdlib, safe on CI):

```sh
python -m unittest discover -s tests -p 'test_navbot_en01_archive.py' -v
```

**Before publication:** reconstruct moving parts from the pinned original STEP rather than the approximate XML; reconcile purchased wheel motor, servo electrical supply and torque-speed limits; rerun measured `mj_step` simulations; prove replay hashes, contact semantics and device-side WebGL on Samsung Tab S7. No physical success, hardware parity or new animation may be inferred from the archival 3D entries.


## Original STEP audit gate (without fabricating an articulated model)

The `model_v0.xml` is still approximate. First inspect the authentic local upstream `Hardware/RebotModel/OriginalRobotModel.stp` using CadQuery/OpenCascade:

```sh
python research/navbot_en01/audit_step.py \
  --step /path/to/OriginalRobotModel.stp \
  --receipt results/navbot_en01/original_step_audit.json
```

The receipt records SHA-256, imported solid count, per-solid bounds/volumes/centres and an explicit **geometric-audit-only** status. A successful import is **not** identification of the hip/wheel moving groups, masses, inertias or motor constraints, and is not permission to replace MuJoCo physics with a static animation. Do not commit third-party original STEP until its redistribution rights are established. Source: `fuwei007/Navbot-EN01@3139f2fb32b38d10611feb905e16f8aa012f2f2e`.



## Original STEP source-derived checkpoint — 9 November 2026

- [Pinned semantic STEP evidence](results/original_step_semantic_probe_20261109.md): source Git blob `9156683a0bb694c84dc541a3e26b5f6b08db1a41`, 46 PRODUCT entities, 62 manifold solids, mm units. This is STEP-entity analysis, **not** a verified articulated model.
- Part `2208电机轮胎` has two outer cylindrical faces of radius **18.5 mm** (candidate outer tire geometry), versus **24 mm** assumed in `model_v0.xml`. Never silently patch the old model or classify old replays as hardware-accurate.
- The CAD names include `QUANUM GIMBAL MOTOR 2208` and `STS3032`; the exact motor as sold and servo wiring remain uncertain.

To repeat the source-pinned entity audit on a machine with the official STEP locally present:

```sh
python research/navbot_en01/step_semantic_audit.py \
  --step /path/to/OriginalRobotModel.stp \
  --out results/navbot_en01/step_semantic_audit.json
python -m unittest discover -s tests -p test_navbot_en01_step_semantic_audit.py -v
```

The script first verifies the exact upstream Git blob SHA1 then records original file SHA256, named parts, selected axis/radius observations and explicit limitations. **It does not run MuJoCo.** A new versioned joint/COM/inertia/contact model and real `mj_step` are still required before a new EN01 physics replay can be published.
