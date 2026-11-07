# SeBaJu — two-wheel balance, jump and self-righting diagnostics

This is an **experimental adapter**, not a new robot and not a claimed successful self-righting controller.

## Provenance and copyright

- Author/source: Chukwuemerie Kennedy Chukwuma, University of Leeds MSc project.
- Original repository: https://github.com/k3nkreate/SeBaJu_BOT_5845
- Immutable source commit: `7bbc38f4b26b8ecce7ac0ee37e06164e851dc600`
- Controller: `SeBaJu_Ctrl_BalJumpAchieve.py`; physical model: `SeBaJu_BOT(II).xml`.
- Upstream README reports **45 s** nominal balance, **0.202 s** flight, **0.045 m** minimum-wheel-clearance peak and **4.632 s** settling after landing.
- Those values are *upstream-reported*, **not independently reproduced by this adapter yet**.
- Upstream currently has **no explicit LICENSE file**. Do not copy/redistribute its code or MJCF into this public repository or viewer. Clone privately at execution time, and publish only factual recorded physical state, logs and receipts from our experiments.

## One-step physics runs (Mac M4 or Linux, no GUI)

Use the existing Python + MuJoCo 3.14.0 environment. The first run clones and pins the public source into the user's local cache; subsequent runs use that exact commit.

```bash
python research/sebaju/run_headless.py --trial jump --out .cache/sebaju/jump
python research/sebaju/run_headless.py --trial tip --out .cache/sebaju/tip
```

Optional: `--upstream /path/to/checked-out/repo` works fully offline when the pinned checkout already exists.

Each trial produces:
- `physical_trace.json`: **actual** per-frame MuJoCo `qpos`, torso pose, joint anchors, wheel centres, wheel contact force and clearance at 50 Hz.
- `receipt.json`: engine/source IDs, step count, flight and recovery evidence, and explicit success/failure/inconclusive classification.
- `source_controller.log`: unchanged original controller diagnostics.

### Trial 1 — jump

Runs the source's 500 Hz feedback controller for 13 s, without opening a GUI. The viewer hook does **not** step physics on its own. Measured jump evidence requires both wheels unloaded, at least 5 mm clearance and at least 20 ms of consecutive true flight. After touchdown, the script tests whether upright, grounded, slow-wheel conditions persist for 0.3 s.

### Trial 2 — external tipping/self-righting attempt

Runs the same upstream controller for 18 s. At 12 s the adapter applies an **external roll torque to the torso for 0.15 s** (8 N·m by default) via `xfrc_applied`; there is no teleportation and no extra recovery policy. A physical tip is confirmed only when measured torso tilt reaches 60°. A successful self-right requires subsequent measured tilt ≤10° with wheel contact and low wheel speed for 0.5 s.

Three distinguishable outcomes:
- **Tip confirmed, no self-right:** the source controller demonstrably does not recover under that disturbance.
- **Tip confirmed, righted:** measured self-recovery, requiring further repeatability trials.
- **Tip not confirmed:** the disturbance was insufficient; outcome is *inconclusive*, never counted as a success.

Neither source code nor our experiment currently establishes self-righting from **every** fallen position.

## Acceptance and publication

1. Standard-library classification tests pass: `python -m unittest discover -s tests -p 'test_sebaju_monitor.py' -v`.
2. Native MuJoCo runs complete without NaNs, report source SHA and version.
3. Inspect the raw physics receipt rather than copying upstream performance values.
4. Only after a successful measured recording, generate a separate browser replay using physical joint/torso/wheel poses. Never invent missing frames or label a sketch as a MuJoCo result.
5. Keep the viewer `feature/android-local-offline-viewer-20261104` branch intact until results are independently verified; no automatic merge into `main`.
