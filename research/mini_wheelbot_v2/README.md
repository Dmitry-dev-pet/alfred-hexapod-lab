# Mini Wheelbot V2 — original CAD + physically constrained self-righting

**Status (2026-11-07): VERIFIED_SOURCE_AND_PRELIMINARY_MUJOCO_NEGATIVE.** Eight physical MuJoCo runs completed; hardware-validated self-righting is not established; no new tablet-library entry. Previous `wheelbot-self-righting-v1` remains unchanged, and
its near-vertical trajectory must **not** be reused for any V2 success claim.

## Provenance / upstream

- [Official Mini Wheelbot](https://github.com/wheelbot/Mini-Wheelbot) at
  commit `73815e49371c5fb5f49eec59d364e4d9d1b57b10`.
- [Original 3D meshes](https://github.com/wheelbot/Mini-Wheelbot/tree/73815e49371c5fb5f49eec59d364e4d9d1b57b10/hardware/mechanical/meshes):
  `mini-wheelbot.stl`, `body.stl`, `actuator_rotor.stl`.
- [Original STEP](https://github.com/wheelbot/Mini-Wheelbot/tree/73815e49371c5fb5f49eec59d364e4d9d1b57b10/hardware/mechanical/step):
  frame, reaction wheel, lid.
- [Identified parameters](https://github.com/wheelbot/Mini-Wheelbot/blob/73815e49371c5fb5f49eec59d364e4d9d1b57b10/simulation/python/examples/mini_wheelbot_sysid_parameters.json).
- [Official acados ODE and example](https://github.com/wheelbot/Mini-Wheelbot/tree/73815e49371c5fb5f49eec59d364e4d9d1b57b10/simulation/python):
  useful dynamics reference, **not** a validated side-to-vertical MuJoCo contact
  simulation or self-righting policy.
- Hardware BOM lists two T-Motor MN4006 KV380 motors. The BOM alone does not
  establish tested torque/speed limits under battery load.

The upstream sysid model has `m_B=0.22470645904855896 kg`,
`m_WR=0.154800009999768 kg` **per wheel**,
`r_W=0.032 m`, `l_WB=0.065 m`, and identified principal inertias
for wheels and body. The calculated total mass is `0.5343064790480949 kg`.
STL *visual* geometry does not replace measured inertias or contact validation.

## Gate S0: verified original source bytes (implemented, not yet executed here)

On an internet-connected trusted Mac / Linux host, from this branch:

```sh
python research/mini_wheelbot_v2/audit_original_cad.py --cache "$HOME/mini-wheelbot-original-cad"
python -m unittest discover -s tests -p test_wheelbot_original_cad.py -v
```

Script pins all download URLs to upstream commit; validates original file length
and Git blob SHA-1; calculates SHA-256, raw STL triangle bounds, and total mass.
Writes `source-audit.json` **only to the local cache**, without adding binary
upstream meshes to our public GitHub.

**S0 PASS requires** real observed output, successful tests, dimensions matching
official CAD and an explicit mm-to-m scale with documented body/wheel transforms.
Do not infer a transform from the previous approximation.

## Gate S1: mechanically faithful model and actuation

1. Align body, drive wheel, flywheel and ground contact coordinates against
   original STEP/STL. The complete `mini-wheelbot.stl` is useful for visual
   assembly but is **not** one rigid part if its wheels must turn independently.
2. Create MuJoCo bodies/joints for drive and reaction wheel with the source
   masses/inertias, source-grounded wheel radius and center-of-mass offsets.
   Document how source roll/pitch/yaw axes map to MuJoCo/Three.js.
3. Use visual meshes for rendering and separately validated compact collision
   geoms for stable physical contact; record contact force, penetration and
   energy. Compare at least three initial orientations, including 90° on side.
4. Source a realistic drive/flywheel torque-speed envelope **with citations**,
   battery voltage, motor drive/current limitation. An invented constant torque
   limit or arbitrary flywheel speed cannot pass S1. At every step log actual
   wheel angular speed, torque command and saturation. Use CPU MuJoCo on the
   existing Mac; no GPU/Colab/robot hardware required.
5. Record and compare a passive no-torque baseline. The published V1 model
   reached ~4983.7 rad/s, unvalidated for hardware; it is **negative evidence**
   about V1 realism and must not become a target for the controller.

## Gate S2: new real physics, no cinematic replay

- Run `mj_step` for the entire maneuver with at least 1 ms physics integration
  and deterministic seed/input. Export actual independent joint state, root
  pose, wheel speed, motor torque, contact state, ground penetration.
- Test a bounded controller under verified motor limits, including realistic
  side starts, impulse, capture, and sustained balance. Avoid directly
  teleporting orientation or splicing v1 poses into v2.
- Measurable success: root body within 10° of upright and stable for >= 1 s,
  with no forbidden collision or motor speed/torque limit violation.
  Otherwise record `FAILED` or `INCONCLUSIVE` and publish the failed replay
  as such. Include time-series plots and complete model/experiment hashes.
- Save heavyweight source CAD, logs and complete trajectories outside the
  public Git history; leave a SHA256 receipt and reproducible script here.

## Gate S3: original mesh in tablet player

- Add **a new** immutable player route only after S1/S2. Keep
  `/hexapod/wheelbot-self-righting-v1/` intact.
- Show the actual official CAD assembly, with separate real rotating parts,
  not a scaled box/cylinder stand-in. Validate visual alignment against upstream
  photographs and mesh bounds. Reject silently inferred model units.
- The tablet is an offline replay player, not a physics simulator. Implement a
  pinned upstream download to local cache with integrity verification, or a
  license-reviewed derived display mesh. **Do not publish upstream binary CAD
  into this repo without confirming redistribution rights.**
- Catalog revision increases only after real Android install/replay tests,
  verified route, hashes, manifest integrity and success/failure labeling.
  Do not change `main` or auto-merge.


## 2026-11-07 verified execution — 8×8 s of MuJoCo (S0 PASS; S1/S2 pending)

A **temporary nonpersistent compute sandbox** (Linux amd64, 1 vCPU, 2GB RAM)
successfully installed MuJoCo 3.14.0 and Trimesh, fetched the pinned *official*
CAD and parameter JSON, and verified all Git blob hashes/sizes. Four original
S0 fixture unit tests passed. On the source body mesh, 255,098 triangles
exceeded MuJoCo's 200,000-triangle binary STL decoder limit: Trimesh converted
it into OBJ **without decimating any triangles**. The upstream files are
not modified.

The original identified chassis inertias violate the triangle inequality for
physical rigid bodies:
`I_Bx + I_Bz = 3.95e-4 < I_By = 4.50e-4 kg·m²`.
MuJoCo refused them. The source code applies and reports the nearest
Euclidean admissible projection, **not** an assertion of revised real
hardware parameters:
`[0.00012832667, 0.00043167333, 0.00030334667] kg·m²`.

Actual `mj_step` physics was executed for 8 runs (60°/90° initial roll ×
0/0.03/0.10/0.20 N·m **assumed** torque caps), 8 seconds each, timestep 1ms
(**64,000 physics integration steps**). Source CAD is used as authentic
visual meshes; collisions are still approximate cylinders/box. Drive and
reaction wheels are distinct actuated rigid bodies. Hypothesized 22.2V and
MN4006 KV380 no-load speed are used for a provisional 883.6 rad/s limit.

| Initial roll | Max torque assumed | Minimum tilt observed | Final tilt |
|---|---:|---:|---:|
| 90° | 0 N·m | 90.00° | 90° |
| 90° | 0.03 N·m | 87.03° | 90° |
| 90° | 0.10 N·m | 81.76° | 90° |
| 90° | 0.20 N·m | 79.40° | 90° |
| 60° | 0 N·m | 60.00° | 90° |
| 60° | 0.03 N·m | 60.00° | 90° |
| 60° | 0.10 N·m | 60.00° | 90° |
| 60° | 0.20 N·m | 60.00° | 90° |

All eight runs failed to reach and hold near-vertical. Contact penetration
peaked at ~3.6–5.4 mm, highlighting the need to calibrate contact/solref
and source-backed geom placements. **This is NOT evidence of failure for
the real Mini Wheelbot**, whose own researchers report successful self-righting.
The controller is an ad hoc PD law, not upstream recovery policy.

The reproducible scripts and [compact result receipt](results/2026-11-07-mujoco-probe-receipt.json)
are checked into this research branch. Raw full frame JSON traces were
produced in the temporary compute sandbox only and are not yet archived
to Drive; their full SHA-256 identifiers can be regenerated from the code.

### Re-run on a connected CPU environment (not requiring Mac or GPU)

```sh
python3 -m pip install "mujoco==3.14.0" "trimesh" "numpy"
python3 research/mini_wheelbot_v2/audit_original_cad.py --cache "$HOME/mini-wheelbot-original-cad"
python3 -m unittest discover -s tests -p test_wheelbot_original_cad.py -v
python3 research/mini_wheelbot_v2/simulate_mujoco_probe.py \
  --cache "$HOME/mini-wheelbot-original-cad" \
  --out "$HOME/mini-wheelbot-v2-probe" --seconds 8
```

**Unfinished validation gates:** original CAD/STEP frame alignment; drive wheel
rim location and contact calibration; source-tested torque-speed-current
envelopes; real righting controller; visual replay acceptance on Samsung Tab S7.
No new library entry or merge is authorized by a provisional negative rollout.

## 2026-11-07 continuation: measured STEP axle axes + original control sequence

**SOURCE-STEP / GATE S0 STEP PASS** in a temporary Linux amd64 MuJoCo 3.14.0
sandbox. Original STEP assets verified by Git blob and SHA-256. Actual
`frame.stp` has two perpendicular **motor seats with radius 15 mm**:

| STEP frame feature | Axis reference point (mm, CAD) | Axis direction (CAD) |
|---|---|---|
| lower, Y = -32.5 mm | (10.5, -32.5, 0) | (-1, 0, 0) |
| upper, Y = +32.5 mm | (0, +32.5, -10.5) | (0, 0, -1) |

Their shaft separation along CAD Y is exactly **65 mm**. The CNC
`reactionwheel.stp` has radii 22.2, 27 and 31 mm. Axle identity is
**inferred**, not labeled by STEP: this probe treats the lower seat as the
drive axle, upper as the reaction axle. Source CAD Y-up rotates to MuJoCo
Z-up with a +90° X rotation. These are **stronger source-derived constraints**
than the arbitrary 43-mm axle offset used in the first test.

The source C++ `RollStandupController` implements:
- 1.0 s idle.
- Ramp of reaction torque to -0.6 × 0.55 N·m over 3.6 s.
- Sign reversal to +0.55 N·m for 1.5 s.
- Return zero; actual real-world stabilization is handed to a controller
  container/AMPC. The simple state-feedback sensitivity tests performed
  here are **not** the official complete C++ AMPC port.

See `audit_step_axes.py`, `simulate_step_source_roll.py`, tests
`test_wheelbot_original_cad.py` and `test_wheelbot_step_roll.py`.
**7/7 stdlib unit tests PASS**. MuJoCo ran 11 independent 8-second
experiments (the original open loop with two contact profiles, then
stiffer-contact LQR-sign variants). Details and SHA256 for all 11 replay
trajectories in
[results/2026-11-07-step-axes-source-controller.json](results/2026-11-07-step-axes-source-controller.json).

Selected **stiffer-contact** findings (not hardware-reliable):
- 90° start, passive: final 90°, no standup.
- 90° start with original open-loop sequence and *assumed* no-load speed
  cap ~883 rad/s: minimum tilt **0.111°** for an instant, upright interval
  **0.066 s**, final **90°**.
- Same source control, source LQR gains after 6.1 s with one assumed state
  sign: final 90°, brief upright interval 0.021 s.
- Reversed mapping sign: final **49.13°** for 90° start, upright only
  **0.010 s**. This is a sensitivity test, **not verified parity**.
- Runs without a motor speed cap became **numerically unstable** near
  5.85–6.00 s with extreme nonphysical RPM, and are marked numerical
  failures (never successes).
- Preliminary contact depth improved from ~19 mm to **~2.17 mm** with
  explicit stiffer contact `solref/solimp`. This is a controlled solver
  sensitivity, not calibration against measured force data.

**Result: NONE of the 11 cases meets the sustained 1-second upright
criterion.** The original CAD meshes were not pushed into this repo;
simulation loads and verifies them from the pinned upstream revision.
Motor voltage/current/torque-speed behavior, COM calibration, exact
controller estimator mapping and nonconvex contact geometry remain
uncertain. No conclusion about the real wheelbot's ability to self-right
is justified; the upstream authors demonstrate that ability.

To reproduce after installing MuJoCo 3.14 and Trimesh in an internet-
connected CPU environment:

```sh
python research/mini_wheelbot_v2/audit_original_cad.py --cache "$HOME/mini-wheelbot-original-cad"
python research/mini_wheelbot_v2/audit_step_axes.py --cache "$HOME/mini-wheelbot-step"
python -m unittest discover -s tests -p test_wheelbot_step_roll.py -v
python research/mini_wheelbot_v2/simulate_step_source_roll.py \
  --cache "$HOME/mini-wheelbot-original-cad" \
  --step-cache "$HOME/mini-wheelbot-step" \
  --out "$HOME/wheelbot-step-source-trials" \
  --contact-profile stiff --include-lqr --duration 8
```
