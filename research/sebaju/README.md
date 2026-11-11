# SeBaJu — two-wheel balance, jump and self-righting diagnostics

This is an **experimental adapter**, not a new robot and not a claimed successful self-righting controller.

## Provenance and copyright

- Author/source: Chukwuemerie Kennedy Chukwuma, University of Leeds MSc project.
- Original repository: https://github.com/k3nkreate/SeBaJu_BOT_5845
- Immutable source commit: `7bbc38f4b26b8ecce7ac0ee37e06164e851dc600`
- Controller: `SeBaJu_Ctrl_BalJumpAchieve.py`; physical model: `SeBaJu_BOT(II).xml`.
- Upstream README reports **45 s** nominal balance, **0.202 s** flight, **0.045 m** minimum-wheel-clearance peak and **4.632 s** settling after landing.
- Those values are upstream-reported. Our independent local CPU simulation now confirms a genuine jump with **0.180 s strictly contact-free flight** and **43.95 mm** peak wheel clearance (non-identical flight criteria). See [2026-11-07 measured results](results/chatgpt-cpu-physical-20261107.md).
- Upstream currently has **no explicit LICENSE file**. Do not copy/redistribute its code or MJCF into this public repository or viewer. Clone privately at execution time, and publish only factual recorded physical state, logs and receipts from our experiments.

## One-step physics runs (ChatGPT Linux CPU sandbox, no GUI or external compute)

Use the locally restored Python 3.13 + MuJoCo 3.14.0 environment. Fetch the two pinned upstream files separately and verify their GitHub blob hashes. The **offline snapshot mode does not call GitHub or Mac**; it works entirely inside the ChatGPT sandbox. Do not vendor upstream's unlicensed code into this public repository.

```bash
python research/sebaju/run_headless.py --offline-snapshot --upstream /path/to/two-pinned-upstream-files --trial jump --out .cache/sebaju/jump
python research/sebaju/run_headless.py --offline-snapshot --upstream /path/to/two-pinned-upstream-files --trial tip --tip-torque 12 --out .cache/sebaju/tip
```

Optional: a pinned full Git checkout works without `--offline-snapshot` if already present, or git clone can run where network is allowed. The research environment in this experiment **did not clone at runtime**.

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

## Verified physical outcomes (2026-11-07)

- **Jump:** 6500 MuJoCo ticks (13 s), 0.180 s strictly contact-free flight, peak wheel clearance 43.95 mm; observed touchdown and return to stable stance.
- **Side tip:** genuine 12 N·m roll torque for 0.15 s at 12 s; physical tip, final 111.03° at 18 s; original controller did **not** self-right.
- **Extended:** 20 N·m / 0.15 s, 30 s of real physics; robot remained inverted at 179.95°, with torso touching ground.
- 8 and 10 N·m did not produce a fall in these trials; see the full sensitivity table, hashes and limitations in [the result report](results/chatgpt-cpu-physical-20261107.md).
- The adapter had to fix flat `xmat` indexing and distinguish an airborne whole robot from a tipped body resting on the ground. The 9 unit tests now pass.
- The Mac-specific GitHub Actions workflow was removed **from this research branch**. No external compute was used for these results.

## Acceptance and publication

1. **9/9 tests pass**: `python -m unittest discover -s tests -p 'test_sebaju_monitor.py' -v`.
2. **Native MuJoCo runs complete** without NaNs; source model/controller blob SHA verified and engine version 3.14.0.
3. Inspect the raw physics receipt rather than copying upstream performance values.
4. Only after a successful measured recording, generate a separate browser replay using physical joint/torso/wheel poses. Never invent missing frames or label a sketch as a MuJoCo result.
5. Keep the viewer `feature/android-local-offline-viewer-20261104` branch intact until results are independently verified; no automatic merge into `main`.
