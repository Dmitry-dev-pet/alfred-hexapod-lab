# Mac M4 — first real Stanford Pupper PPO smoke, 2026-11-07

Status: **VERIFIED_PPO_4096_MAC_CPU / NOT_LEARNED_WALKING**. Research draft PR #76; main unchanged.

## Verified execution

- GitHub: [successful run #42379724161](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42379724161) on existing private isolated `M4-ci-private` self-hosted runner (`ci-runner`, non-admin, arm64). One-off workflow in `Dmytry-lab/runner-smoke` task branch; **no new runner**. Two earlier failures are documented in [Issue #77](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/77).
- Exact executed source commit: `6d22db2a119fb9cf7fb2cab180bd748d7830b445` (one-line fix of missing `math` import after local-preflight commit `6bb491a...`).
- Environment: macOS 26.6.2, Python 3.13.16, MuJoCo 3.14.0, PyTorch 2.10.0, NumPy 2.3.5, Gymnasium 1.2.2, SB3 2.9.2. **CPU**. `torch.backends.mps.is_available() == True`, but **MPS not trained/tested**.
- Physics: pinned source Stanford Pupper v3 MJCF Git blob `7771e176c42b3aed1d48cc62434771b0fceb7c8c`, 12 position-controlled servos, 250Hz MuJoCo and 50Hz policy control. No real robot or scripted root motion.
- **15/15 Mac tests PASS**, including both previously unexecuted Gymnasium integration tests.
- PPO: 4 parallel CPU Gym environments, **4096 total environment steps**, 6.244 seconds training wall, **655.968 environment steps/s** (excludes dependency-install time); train/checkpoint step actually completed, not just an env smoke.
- **Checkpoint**: `policy_final.zip`, 7,599,673 bytes, SHA-256 `acc4f766ee1b7ab82bdfb46fbe2878ccbb54c0af7022ab31c69b8f132a6edff4`, byte hash independently checked after downloading private Actions artifact.
- Evaluated checkpoint for **20 seconds / 5000 physical steps / 1000 policy steps** on scripted forward, lateral, turn, reverse and stop route: XY speed RMSE **0.13371 m/s**, yaw-rate RMSE **0.16680 rad/s**, 0 fall-guard ticks, root world end XYZ (−0.013111,+0.003183,+0.155694) m. **No evidence yet of useful velocity-tracking skill**: earlier zero-action baseline was XY RMSE 0.13368 m/s, yaw RMSE 0.16658 rad/s. Stable stance is not learned walking.

## Permanent evidence

- [Owner's private Google Drive complete Mac artifact](https://drive.google.com/file/d/1ZCopT_bk2_eCV1NDEKyw8hcWjbxZUfCi/view?usp=drivesdk): source of saved checkpoint, full fixed-route physical evaluation JSON, training receipt, log. Archive SHA-256 `6a7018fa2a8142b28d71828a2c8a2fc04ed54ed5651c92308c1e0333023ba03f`, 6,095,458 bytes, verified before Drive upload and Drive file metadata read back.
- [Private Actions artifact (7-day retention)](https://github.com/Dmytry-lab/runner-smoke/actions/runs/42379724161/artifacts/13576564983).
- [Issue #77 execution receipt](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/77#issuecomment-6707970830).

## Follow-on, not performed

100k+ PPO training; same-seed CPU/MPS A/B throughput; reward shaping, fresh generalization seeds, perturbations and a real velocity-tracking policy. Do not infer these from this smoke. Stanford-native MJLab/RSL-RL numerical parity and deploy JSON export remain separate open questions. **No main merge, real hardware actuation or automatic player publication.**
