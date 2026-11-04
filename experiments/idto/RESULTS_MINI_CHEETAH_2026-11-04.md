# IDTO Mini Cheetah: baseline and 80 mm torso-COM shifts (2026-11-04)

Status: **successfully executed dynamics in Drake 1.30.0**, independent of the project CAD30/MuJoCo physics. Upstream IDTO commit: `de0629c7811aa9b330e56c4385629005b09495f0` (TRI).

## What was executed

The upstream C++ `mini_cheetah` was built and run **natively without Docker** on the local Linux x86_64 container, using verified Drake 1.30.0 + native Jammy dependency bundle delivered through the dedicated Mac runner and Google Drive. The default `mpc: true` configuration ran 10 seconds simulated time with a controller update interval 1/60 s.

To avoid conflating desired pose and real motion, a local experiment-only **telemetry patch** on `examples/example_base.cc` records the actual Drake MultibodyPlant state and point-pair contact results at 50 ms intervals. The IDTO numerical optimizer and feedback control logic were **not modified**. Each run produced a CSV of actual simulated states, a contact CSV, a solver log and an exit code. Data below are from a *matched* 3-case study using this telemetry instrument. Baseline measurements from the earlier independent recording can differ slightly; compare only matched-case values here.

## Measured results (201 samples in each run)

| Model | Net forward dx (m) | Net lateral dy (m) | Maximum observed base tilt | Number of samples with 2 / 3 / 4 point-pair contacts |
| --- | ---: | ---: | ---: | --- |
| Unmodified baseline | 3.466846 | +0.014446 | 8.867° | 168 / 32 / 1 |
| Torso COM +80 mm in X | 3.188351 | −0.072737 | 12.144° | 110 / 85 / 6 |
| Torso COM +80 mm in Y | 3.509714 | +0.152890 | 7.527° | 164 / 36 / 1 |

All three simulator processes exited 0 after 10 simulated seconds. No case showed an observed body inversion in the 50 ms recorded orientation samples. This does **not** establish global real-world robustness, continuous stability, or slip-free contact at every 1 ms simulator step.

The perturbation replaces only the `<origin xyz="..."/>` inside the torso `<inertial>` section of `models/mini_cheetah_mesh.urdf`; torso mass stays **3.3 kg**, its rotational inertia tensor *about its COM* remains unchanged, while its body-frame COM moves +80 mm in X or Y. This is a controlled **model-side thought experiment**, not a validated physical payload attached to the real Mini Cheetah. Inertial origin edit does not itself measure the actual distribution of mass of a manufactured robot.

Original model SHA-256: `f7034bd97084e594b2a32a0bfc1447afa9b9a839743046eab30e850517a65db1`.  
Modified +X model SHA-256: `264e3b921c44b518b0a91c61debf04b597226ca6f41530266ab7a71722926233`.  
Modified +Y model SHA-256: `e466c31e816d0faa576fb360253a1eec30bfa672d40a80c8ddd6549b2dd36caa`.

## Scientific interpretation

- A ready-made *contact-implicit* MPC clearly works for a quadruped under the specified model, including these two changes of torso COM. It is **not** a generic guarantee for arbitrary loads, geometry, ground contact, or motors.
- **Not our target contact policy:** the Mini Cheetah example routinely reports just **two point-pair contacts** at sample times. Contact-pair count at 50 ms is not a proof that two distinct loaded feet existed at *every* physics step; it certainly does not certify `>=3` load-bearing feet continuously.
- Our CAD30 goal (#26) is not merely movement: two front legs remain raised, four other legs perform a 4-step cycle, **at least three real supporting legs** during each swing, continuous support/load/torque/slip constraints, future-step viability and `>=20 mm` verified forward progress.
- We should **not** claim the published IDTO automatically chooses a static crawl. Testing a 3+-stance policy needs contact-mode/swing scheduling, feasibility constraints and model-specific integration; a scalar gait-config toggle was **not** found or verified.
- Next discriminating experiment: on the existing IDTO Mini Cheetah first test a low-speed `>=3 loaded feet` crawl with actual contact checks and known COM shift. Only then transfer verified optimizer/controller principles to the real 6-legged Freenove CAD30; retain independent MuJoCo validation.

## Evidence and reproducibility

- Binary provenance, verified source/Google Drive chain and native toolchain: `experiments/idto/DRIVE_TRANSFER_2026-11-04.md`, associated #31.
- Telemetry source patch, actual sampled state/contact CSVs, solver logs and per-model receipts were produced in the session working directory `/mnt/data/idto-measured-run`, `/mnt/data/idto-contact-baseline`, and `/mnt/data/idto-mass-offset-study`. These temporary container paths are **not** durable artifacts and must not be misrepresented as shareable links. Before full reproducibility is claimed, commit measurement-only source and upload the raw research traces/receipts to Drive with checksums.
- Keep this on **draft** PR #33. Do not merge into main without owner approval.
