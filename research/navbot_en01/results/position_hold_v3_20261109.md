# EN01 v3: source parameter audit and wheel-position feedback

Research continuation of #106, 9 November 2026. The nominal source-derived v2 four-bar MJCF, 0.322-kg mass assumption, 0.030-Nm wheel ceiling, 0.14709975-Nm hip ceiling, friction and leg schedule are frozen. This work adds a wheel-encoder position term and mode-dependent gates; it does not calibrate hardware or train a policy.

## Source findings

Pinned author source: `fuwei007/Navbot-EN01@3139f2fb32b38d10611feb905e16f8aa012f2f2e`.

- README states that encoder **2048 is the fully-squatted mechanical-limit pose**, with left servo ID 1 and right ID 2. The corresponding absolute CAD joint angle is not given. `servo_mapping.cad_angle` refuses an absent mechanical-zero offset rather than inventing zero.
- `wl_pro_robot.ino`, blob `c5c69e515d7d2a22edc3779667fc3edfe4045ae9`, assigns signed integer commands before clamping: left 2110–2510, right 1586–1986. For height 38 both outputs clamp to 2110/1986. For height 40 the ordinary outputs are 2115/1980. Jump initial/return outputs are 2463/1632 and 2127/1968. Integer truncation creates a one-tick mirrored difference. Conversion is 4096 counts per revolution, supported by the manufacturer specification. This pure mapping is not a full firmware emulator.
- The jump counter advances in the main loop; its return condition is counter 31–34. No fixed wall-clock period is established. We retain our explicitly experimental timing rather than call it the source timing.
- The author's `lqr_balance_loop` includes wheel-distance feedback, in addition to angle, gyro and speed terms, and suppresses distance/speed feedback during its jump/wheel-lift states. Our previous diagnostic controller omitted distance feedback. Source SimpleFOC runs in **voltage** torque-control mode; its gains cannot be copied directly into Nm.
- Original BOM, Git blob `f91c763c29d0182c4d5018cd622a569bca2f483a`, has 22 part rows. It names two 2208 90KV gimbal motors; the author's [build guide](https://frankfu.blog/embodied-ai-robot/building-a-tabletop-wheel-legged-robot-from-scratch/) instead names TMCM-6210. The actual sold motor/torque curve remains unresolved.
- No BOM row explicitly names a spring. CAD cover/anchor names are insufficient to establish an installed spring, preload or stiffness. Presence is unconfirmed; no invented spring model is added.
- The [official STS3032-C001 sheet](https://www.feetechrc.com/Data/feetechrc/upload/file/20200611/6372749220559698399729459.pdf) gives 20g ±0.2, 0.09s/60° and 4.5kg.cm stall at 6V. The [manufacturer catalog](https://www.feetechrc.com/Data/feetechrc/upload/file/20240706/2024%E9%A3%9E%E7%89%B9%E5%AE%A3%E4%BC%A0%E5%86%8C.pdf), marked 20230718V1, lists ST-3032-C001 as 20.6g and rated 1.5kg.cm/400mA at 6V. Exact installed revision is not established; neither weight substitutes for whole-robot mass. The already-used 0.14709975-Nm nominal ceiling is retained; stall torque is not continuous output.
- The pinned upstream simulator JS calls a `1455f4b95a042c0990ab.module.wasm` physics resource, which is absent from the pinned repository tree. No original simulator physics/URDF or mass values were recovered from its JS wrapper.

See `actuation_evidence.json`, `bom_evidence.json` and `servo_mapping.py`. Whole-robot mass/COM, wheel electrical limits, servo/CAD zero, loaded rolling radius and any spring parameters remain unmeasured.

## Controlled comparison

New wheel-position feedback is **our diagnostic adaptation**, not a port of the author's voltage controller:

`tau_wheel = clip(0.15*pitch + 0.05*gyro + 0.008*(mean_wheel_rate - desired_rate) + Kx*(0.0185*mean_wheel_angle - x_reference), -0.03, 0.03)`.

It uses IMU and wheel encoders, not privileged base position. Base position is used only for evaluation. Wheel-encoder odometry assumes the current 18.5-mm radius and does not correct slip or leg motion. Position reference is zero for stand/push and a 0.10-m/s ramp during t=1–4 for drive. Four declared gains Kx=0/0.03/0.1/0.3 Nm/m were evaluated on stand and crouch; Kx=0.3 minimizes stand drift among upright trials. This is development selection, not a held-out success test.

| Stand gain, Nm/m | Maximum x displacement, m | Maximum pitch, degrees | Upright for 6s |
| --- | ---: | ---: | --- |
| 0.00 | 0.250009 | 11.537 | True |
| 0.03 | 0.201089 | 11.701 | True |
| 0.10 | 0.127802 | 12.088 | True |
| 0.30 | 0.050707 | 13.243 | True |

Thus nominal stand drift changes from 25.00 to 5.07 cm (4.93 times smaller), while peak pitch increases from 11.54 to 13.24 degrees. This improves position holding under the frozen surrogate, not every aspect of balance or hardware behavior. Zero added gain reproduces every frozen v2 nominal stand qpos exactly.

## Gates and evaluation outside gain selection

The guarded candidate omits distance **and speed** terms while a diagnostic jump command is active (from t=1.5), or when mean wheel speed exceeds 25 rad/s. These thresholds/schedule are our bounded adaptation. At pitch or roll beyond 35 degrees, wheel output latches to zero for the rest of the rollout. Roll protection and latched reset behavior extend the source idea; they are not presented as identical firmware. Hips retain their bounded diagnostic position control. Cutout is a stop, not self-righting.

| Gated case (Kx=0.3) | Max x, m | Max pitch, degrees | Non-wheel floor-contact steps | Upright 6s |
| --- | ---: | ---: | ---: | --- |
| gated-friction-035-stand | 0.050772 | 13.254 | 0 | True |
| gated-g30-crouch | 1.624160 | 13.243 | 348 | False |
| gated-g30-drive | 0.363143 | 15.495 | 0 | True |
| gated-g30-fall | 0.011041 | 179.991 | 5633 | False |
| gated-g30-jump | 0.363197 | 13.243 | 3280 | False |
| gated-g30-push | 0.057513 | 17.339 | 0 | True |
| gated-g30-stand | 0.050707 | 13.243 | 0 | True |
| gated-mass-090-stand | 0.049577 | 12.481 | 0 | True |
| gated-mass-110-stand | 2.572461 | 11.782 | 596 | False |

The selected gain also reduces displacement after the nominal 0.3-N/0.1-s push and for the light (-10% mass) and low-friction (0.35) stand variants. The heavy (+10% mass, 0.3542kg) model remains unstable with non-wheel ground contact. These perturbations were not used to choose Kx. They are sensitivity assumptions, not measured manufacturing tolerances.

All crouch trials still fail. Nominal gated jump fails and contacts the floor with non-wheel geometry; no qualified flight plus stable two-second landing is shown. The ungated position term worsens failed jump/fall displacement; these negative runs are retained. Gating cuts the sideways-drop wheel drive for all 6,000 steps and limits x displacement to 1.10cm, but the robot remains fallen. Drive is upright under nominal assumptions; tracking RMSE/autonomous navigation is not established.

## Reproduction, integrity and limits

31 final-source rollouts, 6,000 CPU MuJoCo 3.14.0 steps each, total **186,000 steps and 9,300 saved frames**. All final raw records contain their controller-source SHA-256, model profile, generated model SHA-256 and 4-motor commands. Early prototype summaries were superseded before packaging; final source is the reproducibility reference. Code and model inputs are unchanged during the final batch. `feedback_model.py` allows only nominal, mass-090, mass-110 and friction-035; only the stated mass/friction field is perturbed.

The exporter reconstructs each declared model profile before checking its hash and recomputes all body poses from every recorded qpos. Maximum residual is zero. The existing four-bar renderer and player shell are reused, with the existing original 73-part CAD asset. All 31 records pass replay packaging, idempotency and the review-only publish block. The disposable review catalog retains its 65 previous records (59 viewer entries plus six v2 review entries), then contains 96. The live Site/catalog is not modified.

Actual Three.js r169 numeric checks pass for 83,700 body transforms and 3,645,538 vertex reconstructions. Eleven focused tests pass with the research dependencies; seven pure tests pass and four physics tests explicitly skip in the no-dependency profile. No new workflow or remote compute is used for research. Broad repository CI is separate and is reported from its actual run.

Visual QA remains unverified: the current AF_UNIX stream-socket probe raises PermissionError errno 1, matching the previous Chromium launch blocker. No WebGL/camera/Android touch pass is claimed. Original geometry remains a visual shell over simplified collision capsules; mass/inertia, torque-speed model, encoder/CAD zero and physical safety are unvalidated.

Run a declared review job in a disposable copy, for example:

```bash
python scripts/replay_pipeline.py run experiments/replay-pipeline/en01-v3-gated-g30-stand-20261109.json
```

Use Python with MuJoCo 3.14.0/numpy. Run `python -m unittest discover -s tests -p 'test_en01_*.py' -v`. For visual review, reuse the original CAD and player-shell/vendor assets from the viewer branch. Do not use --publish or infer that a review catalog is the published library.

Dataset: [EN01 v3 position feedback](https://drive.google.com/file/d/1oHhFVQppdS9NYeCw8hsPDpFzGFBvSfQy/view?usp=drivesdk), 10,524,003 bytes; SHA-256 `5562ebd511ed3784e733531e70a4334848750997b6046b4f77d631360f34f550`. Contains all 31 final physical records and replays.
