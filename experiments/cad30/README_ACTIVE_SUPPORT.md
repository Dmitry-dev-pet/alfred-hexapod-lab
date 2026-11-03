# CAD30 — local active stance support: paired negative result (2026-11-03)

## Scientific question

Can the six **independent** leg controllers of the Freenove FNK0052 CAD30 MuJoCo model reduce transient loss of floor contacts if a loaded leg about to lift asks its two neighbors to *preload the ground*? This experiment tests an intentionally simple local heuristic, **not** neuroWalknet, not a validated robot controller, and not force control.

## Data and method

- Simulator: MuJoCo 3.14.0, 30 verified external Freenove STL assets (2 missing meshes replaced for visualization only), via the pinned CAD30 reference model in the project, 20 modeled joints.
- Nominal **assumed** mass 1.804868 kg; all bodies are freely dynamic; six actual tibia–ground collision meshes (convex-hull solver contacts); no fixed body/feet.
- Identical commanded 35-mm step, 20-mm lift, local period 2.4s, 8s experiment duration plus startup settle, PD servo `Kp=7`, `Kd=.32`, modeled |τ| <=0.7 N·m. This torque bound is engineering assumption, **not** measured MG996R capacity.
- Fixed paired initial phases and tie-break states by random seed: μ=1.1 seeds [11,23,37,51,97], μ=0.6 seeds [11,23,37], μ=0.3 seeds [11,23]. **10 conditions × 2 policies = 20 MuJoCo simulations**, no adaptive seed selection.
- Baseline: `neighbor_force_gate_supported` (own heavy leg inhibited, both neighbors require minimal load); new: `neighbor_active_support`, same inhibition plus a local stance-foot downward position offset (up to 4 mm at 20 mm/s). Local foot pressure demand depends on neighbors' stance progress and 50-ms EMA normal force. Each leg can use only own / two ring neighbors' sensor states; no central gait clock, global mass sum or explicit COM planner.
- Force signal is **solver-computed**, not real sensor. This is *open-loop stance-foot position bias*, which may raise or lower reaction forces; it is not measured force tracking. No noise, sensor latency, servo speed/torque curve, compliant foot, battery effect or measured hardware properties.
- Contacts checked on **every physical 2-ms step**; 50-Hz contact/force diagnostic trace and 9-Hz actual MuJoCo `qpos` saved for every run. Animation is a transparent joint skeleton projected from `mj_forward` on recorded simulator `qpos`; it is not a reconstructed real-world video.

## Observed results (group means; 8s)

| μ | Policy | Root advance cm | Fraction of 2-ms steps with <3 ground contacts | Runs with <3 contacts | Mean peak tilt ° | Mean peak drive torque N·m |
|---|---|---:|---:|---:|---:|---:|
| 1.1 | baseline | 11.8350 | 0.00095 | 2/5 | 0.5868 | 0.37978 |
| 1.1 | active support | 11.8344 | 0.00105 | 2/5 | 0.6678 | 0.38458 |
| 0.6 | baseline | 12.6933 | 0.01158 | 3/3 | 0.7437 | 0.45899 |
| 0.6 | active support | 12.6497 | 0.01383 | 3/3 | 0.8537 | 0.54781 |
| 0.3 | baseline | 12.5150 | 0.01550 | 2/2 | 1.0715 | 0.66226 |
| 0.3 | active support | 12.7645 | 0.01575 | 2/2 | 1.8355 | 0.66844 |

Both controllers had periods with **zero** tibia–ground contacts in 2/2 runs at μ=0.3. All 20 runs completed the allotted 8s, but the absence of an ultimate fall does **not** mean continuous stable support. Values are conditioned on small seed sets and the simplified contact/servo model; no population-level statistical claim.

**Result: negative** for the defined stability endpoint. Adding a local downward displacement does not reliably decrease loss-of-support and increases mean peak tilt for every μ group. At μ=0.3, mean distance improves but with larger tilt and still full-contact loss. Do not tune thresholds post hoc on these same seeds and report it as generalization.

## Next hypothesis to test

The 4-mm foot displacement may be poorly coupled to useful load transfer: it is not an allocation of desired support forces and may destabilize the chassis. Compare **active stance-foot pressure** against **geometric stability margin / actual COM projection / net foot wrench**, while preserving the *local* coordination algorithm, and collect independent holdout seeds. First validate contact dynamics and partial ground penetration, then explore proprioceptive force feedback, delays, slip sensing, soft feet and terrain. If global COM estimator is used as a comparator, clearly label it as a centralized **oracle**, not a biologically local algorithm.

## Reproduction

Dependencies: Python 3.13, `mujoco==3.14.0`, NumPy, SciPy, pinned CAD30 external STL material restored to `source/` from project's Google Drive archive; project `hexlab/` code. Paths below assume scripts live in `experiments/cad30` and tests in repo-level `tests`:

```bash
PYTHONPATH=.:experiments/cad30 python -m unittest discover -s tests -p 'test_*load*.py' -v
PYTHONPATH=.:experiments/cad30 python experiments/cad30/active_load_transfer.py --mode neighbor_active_support --seed 11 --friction 0.6
PYTHONPATH=.:experiments/cad30 python experiments/cad30/run_active_experiment.py
PYTHONPATH=.:experiments/cad30 python experiments/cad30/render_active_skeleton.py
```

Assets and 20 complete JSON trajectories, 2-videos and checkpoint images are stored in heavy ZIP on Drive. Small cross-condition summary tracked at `results/active_local_support_paired_summary_2026-11-03.json`; actual CAD/STL not copied into public GitHub. The original condition list and the negative result must be kept intact. `PR #4` remains **draft**; do not merge without user approval. No GitHub Actions were used for this experiment; 13 local tests passed.
