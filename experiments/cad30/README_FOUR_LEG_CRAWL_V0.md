# Four-leg locomotion attempt v0 — **negative result** (4 November 2026)

## Question
Can the FNK0052 walk continuously on **four** of its six legs with the front pair acting as raised forelimbs? Does a conventional fixed-phase wave gait achieve this, before implementing an insect-inspired reflex?

**Answer from this diagnostic experiment: not yet.** The two middle support legs stay on the ground; the rear pair lifts a few millimeters, but the chassis does not make meaningful sustained forward progress. Retain this failure in all reporting and do not present video as demonstrated 4-leg locomotion.

## Frozen setup

- Base model: pinned Freenove FNK0052 CAD30 with 30 genuine mesh assets and 2 *visual-only* stand-ins, six original tibia convex-hull colliders. The Freenove CAD assets and license are separate; do not publish the source STL/MJCF with this PR.
- Physically free torso: no pinned root or toe, no scripted root translation. MuJoCo 3.14.0, Python 3.13.5, integration step 0.002 s.
- Body masses: lower / nominal / upper scenarios, approx 1.644 / 1.805 / 2.080 kg, **unmeasured**. Friction μ=1.1, 0.6, 0.3, **assumed**.
- PD controls from existing project: Kp=7, Kd=.32, absolute torque limited to **assumed** 0.7 N·m; no real RC servo current/voltage/thermal/speed calibration.
- Stabilize six-legged stand for 1 s; then shift all four supporting toe targets +37 mm along body Y over 2 s. Raise two front toes +105 mm and forward +50 mm during next 1.2 s. Hold until 4 s. Next 8 s: fixed-phase sequential four-support cycle with period **4.0 s**, stride **12 mm**, lift **8 mm**, swing fraction **15%**, one-quarter-cycle phase offsets. Offline IK solves each leg trajectory independently. **No real-time reflex or learned policy**.
- Frames are original MuJoCo joint states; videos are diagnostic and labeled **FAILED GAIT**.

## Prespecified scientific success gate

Require all simultaneously: (i) experiment completes, (ii) both front tibias stay contact-free throughout the scored period, (iii) **all four** support legs show at least **2 mm** lift at some point in their scheduled swing, (iv) >=3 support legs touching throughout scoring, (v) no motor torque saturation, (vi) **net forward body progress >=1 cm** during gait. A torso shift caused only by preparing the initial balance pose does not count as walking.

## Results — 9 independent physical simulations, no cherry-picked successes

| Scenario mass | μ | Front toes touch floor? | Min support contacts | Middle legs lift? | Net body progress during gait |
| --- | ---: | --- | ---: | --- | ---: |
| lower | 1.1 | no | 3 | no | -0.36 mm |
| lower | 0.6 | no | 2 | no | -1.16 mm |
| lower | 0.3 | yes | 1 | no | +0.84 mm |
| nominal | 1.1 | no | 3 | no | -0.45 mm |
| nominal | 0.6 | no | 1 | no | +0.35 mm |
| nominal | 0.3 | yes | 2 | no | +2.61 mm |
| upper | 1.1 | no | 3 | no | -0.55 mm |
| upper | 0.6 | yes | 1 | no | +27.68 mm (failed support condition) |
| upper | 0.3 | yes | 2 | no | +3.14 mm |

All nine simulations finished their configured time horizon, but **0/9** passed the full gait success criterion. In the nominal μ=1.1 scenario, front feet remain airborne and the remaining four control commands run, yet middle tibias never demonstrate positive mesh clearance. Rear tibia maximum clearance is approximately 3.7/5.2 mm. Net motion -0.45 mm. Thus this is an informative **negative** finding, not autonomous four-legged walking. At low friction, any positive displacement may represent slip or loss of controlled support and must not be counted as walking.

## Geometry and safety-gate interpretation

The earlier one-rear-leg experiment (stacked PR #7) showed a short 9.3-mm rear-foot step with three ground supports, but a near-static two-middle-leg swing was not attained. At the four-contact raised-front pose, the MuJoCo COM projection is outside the three-foot support triangle for either middle leg. The read-only `four_leg_support_gate.py` returns a *geometric oracle* flag: middle `ml/mr` would fail the prospective margin test while rear `bl/br` pass. **This oracle is not available on the physical robot as a ready-made sensor**. It is deliberately isolated from the controller and must not be smuggled into a future distributed policy.

Next experimental options: (1) actively reshape the mid/rear contact polygon and redistribute normal force before attempting a middle-leg swing; (2) allow dynamic balance, while explicitly monitoring momentum and recovery, as a distinct controller; (3) use IMU, estimated joint state and motor current / contact measurements to build a physical state observer. Do **not** silently lower the success criterion by letting a middle foot drag or letting a front foot return to the ground.

## Run, tests, provenance

```bash
cd experiments/cad30
python four_leg_crawl_v0.py --step .012 --lift .008 --shift .037 --period 4 --seconds 8 --mode wave --record --out results/four_leg_crawl_wave_nominal_trace.json
python -m unittest test_four_leg_crawl_v0.py -v
xvfb-run -a env MUJOCO_GL=glfw python render_four_leg_crawl_v0.py results/four_leg_crawl_wave_nominal_trace.json ../four_leg_cycle_media
```

The full 3 mass × 3 friction series is preserved in `results/four_leg_crawl_v0_sweep_2026-11-04.json`. The nominal `qpos` trace and diagnostic MP4/GIF live on project Drive with SHA-256 manifest; they do not contain Freenove STL assets. No GitHub Actions were used for physical calculations. Draft PR only; **do not merge** without explicit owner approval.
