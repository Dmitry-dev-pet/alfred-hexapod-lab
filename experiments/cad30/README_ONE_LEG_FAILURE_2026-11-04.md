# Loss of one supporting leg during a decentralized gait — CAD30 FNK0052

**Status:** Completed **locally** in ChatGPT's Python 3.13 / MuJoCo 3.14.0 environment on 2026-11-04; exploratory simulation only. No robot hardware tested. No GitHub Actions used for computation. This is a new diagnostic experiment, not evidence of full neuroWalknet, genuine leg self-repair, or a safe physical gait.

## Question

If one front leg suddenly loses all its actuator torque and tibia-floor support while the hexapod is walking, will the pre-existing independent leg-state-machine + neighbor-inhibition controller keep moving without any explicit notification of which leg is faulty? Does a new locally sensed foot-pressing rule improve the outcome?

## Setup

- Model: CAD30-derived Freenove FNK0052, 30 pinned original STL, missing two visual-only CAD STL represented by boxes, six original tibia convex contact hulls. Nominal 1.804868 kg. Source upstream commit `5c9a84c16335b30036e451c691d2c544abf4bc72`.
- Ground / tibia simulated friction coefficient μ=1.1; estimated PD Kp=7, Kd=0.32, simulated absolute motor torque limit 0.7 N·m.
- Each of six legs runs its own stand/swing state machine, with neighbor inhibition and no global tripod order or body movement commands. Period 2.4s, swing fraction 0.18, nominal 35 mm step, 20 mm lift. Foot contact is reported by MuJoCo.
- After 1s six-legged stabilization, simulate 8s movement. Sudden failure at t=3.0s. Physical intervention: tibia collision for CAD code `fl` disabled, all three leg motor torques forced to zero; link/servo masses still retained. **It is not physical amputation**, and actual vendor motor label for this CAD leg is **FR (front right)** due to mirrored CAD/vendor nomenclature.
- No explicit fault-id flag enters the controller state machine or lift/stance decision. It detects only ordinary per-leg contact and two-neighbor messages; all 3 arms use the same seed, geometry, friction and period. No retuning after observing outcomes.
- Three paired initial-state seeds: 11, 23, 37. A = healthy neighbor-inhibit. B = same controller + unilateral loss. C = same failure + additional local rule: a neighboring leg absent for >0.55s triggers a 4mm foot lowering target while other neighboring-leg contact is considered before lifting. This is a **preprogrammed recovery heuristic**, not spontaneous learning.
- Physics dt=2 ms, saved 50 Hz trace, video from recorded MuJoCo `qpos`. Static-fall threshold: body origin z <75mm or tilt >50°. Failing to cross threshold is NOT proof of stability.

## Measured results

Distances **between t=3s and t=8s**, in centimeters; not whole-run displacement:

| Seed | Six legs, A | Loss + original controller, B | Loss + extra local rule, C | Loss B compared with A |
|---|---:|---:|---:|---:|
| 11 | 7.790 | 2.442 | 2.285 | -68.7% |
| 23 | 7.772 | 1.970 | 1.804 | -74.7% |
| 37 | 7.893 | 6.609 | 6.806 | -16.3% |
| **Mean** | **7.818** | **3.674** | **3.632** | **-53.0% (means)** |

All 9 runs completed 8 simulated seconds without crossing the above fall threshold. **Post-failure minimum observed contact legs were 2, 1, 1** across seed 11,23,37 in B, insufficient for a static-stability claim. The baseline A remained at minimum 3/3/4 contacts after t=3s (seed 23/37/11). Damage in seed 23 increased max tilt from 0.680° to 5.671°. B retained limited advancement, but in seed 23 postfailure average normal load on `fr` rose from 3.25 N (healthy) to 5.68 N (damaged), while failed `fl` fell to 0.0 N.

C did not show a consistent incremental benefit; in seeds 11 and 23 progress worsened, in seed 37 progress improved slightly. Mean C < mean B. Three seeds are not enough for robust significance claims; we retain all outcomes including the negative results.

## Reproduction

In the existing research branch containing `hexlab/`, `experiments/cad30/locomotion_cad30.py`, and pinned assets restored from the private Drive CAD30 archive **without** checking binary mesh sources into GitHub:

```bash
python -m pip install mujoco==3.14.0 numpy scipy pillow matplotlib
python build_30meshes.py  # when running from the standalone archived original base
python experiments/cad30/failure_one_leg_cad30.py
python experiments/cad30/plot_leg_failure.py
xvfb-run -a env MUJOCO_GL=glfw python experiments/cad30/render_failure_comparison.py
```

The standalone original archive also includes `locomotion_cad30.py` at its root; place these scripts in `experiments/` under that archive for local replay, or restore the upstream repo layout. `failure_one_leg_cad30.py` scans ancestor folders for `hexlab/`, so both layouts are supported. Read the Python source and results for complete implementation details.

## Interpretation and next step

This shows that a contact-aware neighbor-inhibition controller *continues to function* after one support disappears, but not that it compensates effectively: forward progress often collapses, the surviving contacts may fall to one, and the outcome depends strongly on the initial step states. Naive 4mm local preload did **not** fix the failure.

Next: test each of six leg failures and all fifteen two-leg pairs across ≥20 randomized states, conservative motor/friction sensitivity, body lean and explicit weight-transfer strategies; evaluate fall and support margin, energy, contact force, and progress; compare a fixed-phase control. Real robot claims need physical motor torque-speed/current/temperature and contact calibration.

**Data lineage:** Full per-run traces and actual physics replay are stored in the experiment archive; GitHub retains script, protocol, compact summary and immutable archive checksums. No third-party CAD binary is included in this source-only commit.