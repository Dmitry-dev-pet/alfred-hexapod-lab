# Hexapod Lab: Mink contact-plan preflight + force-feedback recovery

**4 November 2026 · NOMINAL MuJoCo RESEARCH, not verified four-leg gait.**

Issue #26 (math-first lookahead) / Issue #21 (four-legged gait). This experiment reuses upstream **Mink 1.4.0** for IK, **DAQP** for its QP, and the MuJoCo 3.14.0 original FNK0052 CAD30 model with 30 meshes. No mesh binaries in this branch. Existing custom IK/WBC is retained as a benchmark, not expanded.

## Physical starting point and caveat

The starting snapshot `results/cad30_safe_pose_B_nominal.json` is the **result** of a prior physically simulated B→safer-four-support rebalance, held until t≈5.05 s. Its 18 leg joints are within the **provisional research limit ±1.7 rad**. The historical original B state during the **preceding** 6→4 procedure contained joints outside ±1.7 rad. Therefore these results do **not** establish a full six-to-four transition that always respects that provisional limit; actual servo travel/current/torque limits are also uncalibrated.

The four starting contacts are the **real MuJoCo tibia-floor contacts**. They are transformed to massless tibia-local `site` markers solely for kinematic planning. Neither body mass/inertia nor actuator ordering changes. All four support coordinates are in the world frame (+Y forward).

## A. Virtual planning and force preflight

For each candidate rear leg `bl` or `br`, the math script:

1. Raises the torso by 24 mm in **12 virtual waypoints** of 2 mm, while preserving all 4 foot sites using Mink and joint limits ±1.7 rad.
2. Lifts the selected rear foot by 10 mm in 4 waypoints, slides it 20 mm forward in 4 waypoints while raised, and lowers it in 4 waypoints. All non-swing feet are held in the world frame via Mink (`FrameTask`, `DofFreezingTask`, `ConfigurationLimit`, `VelocityLimit`).
3. At **all 24 waypoints**, checks ≥5 mm 3-/4-support COM margin, hinge travel, toe target error <0.2 mm, and a conservative *quasistatic* LP: total wrench equilibrium, gravity of all bodies, positive support reactions Fz≥0.2 N, inner 32-face Coulomb cone (assumed μ=1.1), and smallest feasible fraction of all model actuator torque limits (must be ≤0.9).

| Case | All 24 math waypoints | Smallest predicted COM support margin | Largest |joint| | Largest required model torque fraction |
| --- | --- | ---: | ---: | ---: |
| `bl` (rear left) | PASS | 15.4093 mm | 1.65464 rad | 0.57156 |
| `br` (rear right) | PASS | 15.4092 mm | 1.65466 rad | 0.57156 |

**These are kinematic/static certificates for the sampled poses, NOT a dynamically stable walking cycle or a verified path between samples.** The torso is virtually positioned in the optimizer but **never teleported** during the independent physics replay; only the computed joint targets drive the physical robot model.

The full-four-step lookahead is **NOT** completed. As a diagnostic illustration, placing both rear feet just 20 mm ahead leaves a hypothetical middle-foot lift with a negative initial geometric margin ≈−87 mm, requiring ≈92 mm COM movement to recover a 5 mm triangle margin **if the remaining feet are kept fixed**. This does not prove impossibility with other foot placements or torso rotations.

## B. Free-base MuJoCo replay of ONE rear-foot step

The second script follows each planned joint-target waypoint using a clipped PD controller (Kp=7, Kd=0.32, assumed torque bounds ±0.7 N·m for leg channels). It integrates the **unrestrained floating base** with `mj_step` at 2 ms, reads actual tibia-floor contacts and measured normal forces on **every physics timestep**, measures COM support polygons from real contacts, and independently verifies swing/landing. A 0.18 s servo ramp is used per waypoint plus an end hold.

| Dynamical observation | Left rear `bl` | Right rear `br` |
| --- | ---: | ---: |
| Physical candidate duration | 4.82 s | 4.82 s |
| Actually measured swing-foot travel forward | 19.258 mm | 19.260 mm |
| COM moves forward (not full 4-step target) | 4.694 mm | 4.705 mm |
| Minimum true loaded supports at any 2 ms timestep | 3 | 3 |
| Minimum true signed support margin | 11.613 mm | 11.602 mm |
| Highest nominal actuator torque command | 0.5325 N·m | 0.5325 N·m |
| Output status | `DYNAMIC_VETO_NEXT_STEP` | `DYNAMIC_VETO_NEXT_STEP` |

**Crucial negative result:** after landing, the *other* rear foot has effectively zero normal force. The model holds on 3 legs, so both cases **FAIL** the full 4-support touchdown completion gate. No next autonomous step is authorized. The static preflight by itself did not predict this contact force redistribution.

## C. Feedback to restore the unloaded fourth contact

Starting from the **same observed final dynamic qpos/qvel** of the left rear step, compare a matched unchanged-command hold with Mink-calculated increments to the *unloaded right rear foot* (`br`) of 0.25, 0.5, 1.0, or 2.0 mm downward. The rest of the actuator targets are **not reset**; the correction is added to the previous commanded target (to avoid previously observed artificial transients). All variants simulate 0.8 s from **identical physics states** under the same PD constants, μ and model torque caps.

| Extra target change | Final right rear `br` normal force | Final four contacts loaded? | Maintained four contacts on every timestep in last 0.2 s? | Physics-gated recovery |
| --- | ---: | --- | --- | --- |
| 0 mm, matched hold | 0.0244 N | No | No | NO |
| −0.25 mm | 0.1405 N | Yes by 0.08 N detection | Yes | NO: Fz <0.2 N reserve |
| −0.5 mm | 0.2579 N | Yes | Yes | **PASS** |
| −1.0 mm | 0.4913 N | Yes | Yes | **PASS** |
| −2.0 mm | 0.9270 N | No: `bl` becomes unloaded | No | NO |

The first two passing corrections do not lose three-support stability, do not exceed the provisional ±1.7 rad angles, and have no actuator-command saturation in the nominal simulation. A correction that is too large transfers load away from the previously landed foot. **Feedback must control force distribution, not merely contact/no-contact.**

This demonstrates **one** candidate swing/placement plus contact recovery, **not** an entire 4-foot cycle. Full MPC, terminal viability, hardware calibration, friction/torque sensitivity and the strict 6→4 initial transition are all still outstanding. **Autonomous execution of a next step remains explicitly blocked.**

## Reproduction

Requires the user-owned CAD30 XML/STL from [Drive model archive](https://drive.google.com/file/d/1qN8KAgekhXm5RasMXSHNxHAsMxivpZ44/view) and the Mink CPython 3.13 Linux wheelhouse from [Drive runtime archive](https://drive.google.com/file/d/1hnDaarQNdj5_Y94BJ9aNA4j2xRQ-kowv/view). Do not put original third-party meshes in GitHub.

In the research branch, from the repository root:

```bash
export CAD30_MODEL_XML="$PWD/source/mjcf/scene_local30_nominal.xml"
export CAD30_SAFE_STATE="$PWD/results/cad30_safe_pose_B_nominal.json"
export CAD30_DYNAMIC_RESULT="/path/to/dynamic_one_rear_actual.json"
export PYTHONPATH=experiments/cad30

python experiments/cad30/mink_contact_preflight.py --model "$CAD30_MODEL_XML" \
  --safe-state-json "$CAD30_SAFE_STATE" --out results/local_preflight.json
python experiments/cad30/mink_dynamic_contact_probe.py --model "$CAD30_MODEL_XML" \
  --safe-state-json "$CAD30_SAFE_STATE" --leg bl --out "$CAD30_DYNAMIC_RESULT"
python experiments/cad30/mink_contact_recovery.py --model "$CAD30_MODEL_XML" \
  --safe-state-json "$CAD30_SAFE_STATE" --dynamic-result "$CAD30_DYNAMIC_RESULT" \
  --out results/local_recovery.json
python -m unittest discover -s experiments/cad30 -p 'test_mink_contact_preflight.py' -v
```

Store full 50-ms sample logs and physical states in Drive; keep compact machine-readable receipts, code, tests and links in private/draft research GitHub. **No GitHub Actions as MuJoCo numerical compute** (the Mac runner was used only for one-time wheel download). Main must remain unchanged; draft PR requires owner review.