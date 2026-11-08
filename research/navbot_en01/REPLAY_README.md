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
