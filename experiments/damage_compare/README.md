# Damage adaptation comparison on CAD30: stage 0 (2026-11-04)

**Research status: only an independent IT&E-*inspired* small-repertoire pilot and MuJoCo sensor/actuator boundary have been executed. The original neuroWalknet and IT&E algorithms have NOT been run or ported.** This is not an efficacy comparison of the two published methods. Do not merge into `main` without review.

Tracking: [Issue #10](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/10). Baseline: [PR #9](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/9), whose `research/leg-loss-cad30-20261104` branch is the parent of this staged branch. The `research/v04-physical-reconstruction` work remains [PR #4](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/4) (draft).

## Examined original publications/code

- Schilling & Cruse (2020), [neuroWalknet](https://github.com/hcruse/neuro_walknet), pinned repo commit `608f6e728b95dc8a173b092319f1206d7ee4eb71` (GPL-3.0). The code requires Hector / ODE, BioFlexBus, custom leg network classes. `controller/neuro_walknet/NeuroWalknet.py` runs the neural network ~1 kHz inside a controller called at ~100 Hz; it sends **joint velocity** commands. The local coordination rules act on neuronal variables, PEP and load signals. Hector `Hector/LegF.py` distinguishes *inputPosition* and *outputPosition* of compliant joints. CAD30 currently has one rigid simulated joint angle and no calibrated servo electrical dynamics. This is a nontrivial model mismatch; MUST NOT fake Hector motor sensors and call it a faithful port.
- ResiBots [pyite](https://github.com/resibots/pyite), pinned commit `b43400740908cbc8be692fe2dc079f3f36928e8a` (source headers state CeCILL); [ite_v2](https://github.com/resibots/ite_v2), commit `fa4d86dcfac8a144085033d6069d683374968238` (CeCILL). The released hexapod example uses a **six-dimensional behavior descriptor** and **36-dimensional controller** with an offline MAP-Elites repertoire from a DIFFERENT morphology. pyite uses a Matérn-5/2 GP on residuals of physical minus predicted map fitness, and selects with an upper-confidence criterion; PyBullet and submodules are required. The C++ v2 implementation requires Limbo/DART. Do not reuse original archives as if calibrated for Freenove.
- [Nature article, Cully et al. (2015)](https://www.nature.com/articles/nature14422): offline behavior map + online damaged-reward observations; number of online trials matters. Original paper reports tests on different damage conditions, not our robot.

No upstream third-party source files were copied or vendored. The present code in this folder is an independently written experiment and interface; licenses for any future direct import must be reviewed.

## Stage 0 completed locally

- Original physical CAD30 source: Freenove FNK0052, 30 pinned meshes out of 32, 2 placeholders strictly **visual**. Actual six tibia convex hull colliders. Source CAD archive kept in **private Drive**, not in this repository.
- Model: MuJoCo 3.14.0 on Python 3.13, approx 1.804868 kg, nominal floor friction μ=1.1, assumed 0.7 N·m cap, fixed 2ms physics timestep. Not calibrated against real servos or force sensors.
- Failure: at t=3 s, all three motor torques of CAD leg `fl` are forced to zero and its tibia-floor collision disabled, **mass stays**. CAD `fl` maps to Freenove hardware/front-right due to mirrored labels. Not an actual limb amputation.
- Intact map: exactly SIX **hand-designed** stride/lift/period parameter combinations, each evaluated at intact seed 97. This is *not* MAP-Elites; no quality diversity search was done.
- Search: initial best undamaged expected reward, then up to four different damaged-robot trials selected by Matérn-5/2 Gaussian-process residual model + UCB acquisition. Our numerical constants are local prototype settings, not claimed equal to the original pyite experiment. All trials are fresh episodes (the damaged robot is reset). No within-episode adaptation is implemented.
- Post-damage scoring: `progress_cm(t=3..8) - 8 * fraction(samples with <3 contacts) - 20 * fraction(samples with 0 contacts) - 50 * indicator(fall or incomplete)`; fixed BEFORE the new trials. A single score does not prove static or dynamic stability.
- Existing neighbor-inhibition controller without parameter search is the reference, using identical simulated dynamics and initial state; winner of 4 trials uses **4 times as many damaged evaluations** as reference, so improvement does not mean greater sample efficiency. We do NOT claim these are budget-equivalent tests.

## Numerical receipts

Test seeds: 23 (prior PR #9 scenario) and 211, 307, 409 (additional seeds). Each uses 4 online damaged trials, fixed intact map from seed 97. Values are the defined *score*, not cm. **The experiment is exploratory: 4 seeds are not a statistically strong test.**

| Damaged seed | Fixed neighbor-inhibit | Best of 4 IT&E-style selections | Selected design |
|--|--:|--:|--|
| 23 | 1.4282 | 0.7579 | long_slow |
| 211 | 1.9030 | 5.0806 | long_slow |
| 307 | 3.2730 | 3.2730 | existing_baseline |
| 409 | 4.9336 | 5.5111 | long_slow |
| Mean | 2.8845 | 3.6557 | *NOT a fair equal-budget superiority estimate* |

**Negative findings:** In seed 23, search performs worse than the fixed controller. The trial for `short_slow` loses *all foot contacts briefly*. At least one candidate in seed 307 also loses all foot contacts. Even the best designs commonly briefly have only one or two supports. No controller is demonstrated safe. And only 6 designs, not a MAP-Elites repertoire.

Numbers: `results/small_repertoire_ite_summary_2026-11-04.json` and `results/small_repertoire_ite_smoke_seed{23,211,307,409}_2026-11-04.json` (prior map scores, all acquisition decisions and per-trial metrics). These files contain summaries, **not** full saved 50 Hz traces for the new stage; reproduction requires rerunning the scripts. Full baseline failure traces remain in the PR #9 Drive archive.

## Code, validation, reproducing locally

- `ite_selector.py`: standalone numpy Matérn-5/2 GP residual selector (original independent code, not a port of upstream library).
- `small_repertoire_cad30.py`: makes six parameterized variants, runs the real CAD30 physics, creates a small intact map, and performs trial selection on failed model.
- `cad30_observation.py`: six-leg physical readout (joint position/velocity, estimated tibia ground contact/normal force, toe marker, body orientation) and *torque-bounded* output boundary. This is only a starting interface: upstream neuroWalknet expects **velocity commands**, compliant Hector joints, and neuronal substeps, NONE of which has been implemented. Torque limits are simulated assumptions.
- `tests/test_ite_selector.py` and `tests/test_cad30_observation.py`: **12 tests passed** locally (8 kernel/selection tests, 4 sensor/actuator smoke tests on a real stabilized CAD30 model). No Actions were used.

From the CAD30 research checkout, after restoring private STL assets into the expected `source/` paths (see PR #4 and its manifests):

```bash
python -m unittest discover -s experiments/damage_compare/tests -v
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python experiments/damage_compare/small_repertoire_cad30.py --test-seed 23 --trials 4
```

For a blind comparison, use *new seeds and different failure patterns*, not the four published exploratory seeds. Store full physical traces, service frequencies, parameter hashes and exact model hashes. No use of GitHub Actions for computation/rendering.

## What must happen before a true neuroWalknet vs IT&E comparison

1. **Neural fidelity:** run original neuroWalknet with its Hector/ODE simulator as upstream baseline first. Implement Hector→MuJoCo inputPosition/outputPosition and 3D coordinate conversion, neuronal 1ms substepping, neural coordination, motor velocity→bounded torque conversion with measured servo dynamics. Compare undamaged leg phase and disturbance responses against original Hector before naming a port neuroWalknet. Respect GPL-3.0.
2. **IT&E fidelity:** replace 6 handcrafted candidates with a reproducible MAP-Elites diversity archive generated on intact CAD30 using physically admissible parameterized controllers; report archive coverage/size/QD score. Retain a disjoint test set. Compare to random archive search and greedy prior under **same number of online trials**, not a single fixed baseline episode. Respect CeCILL where using the original implementation.
3. **Fair physical test:** predetermined >=20 unseen seeds, all six single leg failures then all 15 pairs, varied motor limits and μ, failure patterns (zero torque vs lost contact vs geometry removal), synchronized observation rates, realistic delay/current constraints, true fall and slip metrics, energy/load fatigue. Require contact/support safety criteria, don't count 'no fall threshold' as verified balance.
4. Preserve PR #9 intact; do not merge draft work without review and a clearly documented result.