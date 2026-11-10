# HexArth: first OEM CAD trial, 2026-11-10

The factory STEP can be articulated into a chassis and 18 moving links using
the axes inferred from the paired metal servo horns. This first CAD-native
diagnostic stands, walks and crosses a 10 mm step in MuJoCo. It stalls at a
20 mm step with the same 12 mm foot lift. These are simulated outcomes for
the assumptions below, not measured hardware performance or the robot's limit.

| Case | Duration | Observed result | Forward travel after settling |
| --- | ---: | --- | ---: |
| Stand | 10 s | Stable, no non-foot ground contact | <0.1 mm |
| Flat tripod walk | 10 s, including 2 s settling | Stable | 187.5 mm |
| 10 mm step | 24 s, including 2 s settling | All six feet reach the top; chassis crosses | 487.8 mm |
| 20 mm step | 24 s, including 2 s settling | Stalls at the riser; remains upright | 41.2 mm |

Across the four cases the largest applied torque is 0.9114 N m, below the
0.980665 N m nominal limit, and the largest joint speed is 3.7501 rad/s.
No MuJoCo numerical warnings, self penetration, or non-foot ground contact
were recorded. Maximum exterior contact penetration is 0.354 mm with the
configured soft contacts. A 0.5 ms rerun of the 10 mm step agrees on full
crossing and differs by 0.101 mm in forward travel from the 1 ms calculation.

## What is reconstructed

- OEM STEP geometry is tessellated without replacing the robot with primitives.
- 18 joint axes come from the two coaxial CAD metal horns of each servo.
- 445 exported part/group records form 19 rigid visual bodies. Tiny fasteners
  use nearest-component assignment and still need mechanical review.
- Collision uses 94 individual exterior CAD parts with MuJoCo convex hulls.
  Original full visual geometry is retained separately; its chassis exceeds
  MuJoCo's 200,000-face STL input limit and is not loaded into physics as a
  single visual mesh.
- Saved `qpos`, `qvel`, applied torque, targets, contacts and 19 body poses
  come from actual `mj_step` integration. The replay uses the existing flat
  pose-array layout. The PNG is a Blender render of an actual saved state.

The primary dimensions measured from the axes are 32.970–32.981 mm from yaw
to shoulder and 65.950 mm from shoulder to knee. The knee-to-shoe-center
distance is 106.495–106.509 mm. OEM firmware uses `hypot(95.353,3.257)` =
95.409 mm for its final IK link. A shoe center and an IK endpoint may be
different reference points: this finding is not evidence of a firmware defect.
The endpoint, electrical zero, sign and tibia offset must be reconciled
before claiming that this simulation executes the factory gait.

## Explicit assumptions and scope

Total mass is a 2.1 kg scenario: chassis 0.9 kg, and each leg's coxa/femur/
tibia 0.09/0.08/0.03 kg. Per-link COM and box inertias are estimates.
All joints use a local +/-20 degree diagnostic range around the CAD pose;
these are **not** verified hardware joint limits. The firmware tick clamps
are recorded independently in `oem-sources.json` and are not silently mapped
onto this model.

Actuation is a bounded PD torque model with nominal 10 kgf cm, stall 30 kgf cm,
and no-load 60 degrees/0.222 s. The envelope is the lesser of nominal torque
and a linear signed-speed DC estimate. Friction is 0.8. Backlash, electrical
and thermal dynamics, contact compliance and actual servo firmware are not
calibrated. The gait is a CAD-native tripod (20 mm stride, 12 mm lift, 1.6 s
cycle) with numerical IK. It is not an execution of the OEM firmware.

No self-righting, obstacle perception, or online foot-height adaptation was
tested. A 20 mm failure with a 12 mm lift does not establish maximum obstacle
height. The next useful experiment is an explicit lift-height change, followed
by reconciliation of the OEM endpoint/servo convention and mass calibration.

## Reproduce

Use the two original OEM downloads pinned by SHA-256 in `oem-sources.json`.
Their archive names are not immutable versions, so reject a hash mismatch.
Do not commit downloaded CAD/STL or firmware archives to the public repository.
The private experiment package contains these originals, generated meshes,
raw traces, replays, PNG and compact receipts.

```sh
python -m venv --system-site-packages .venv
.venv/bin/pip install -r research/hexarth/requirements.txt
.venv/bin/python research/hexarth/inspect_cad.py --step oem.step --output hierarchy.json
.venv/bin/python research/hexarth/extract_cad.py --step oem.step --hierarchy hierarchy.json --output hexarth_assets
.venv/bin/python research/hexarth/baseline.py --assets hexarth_assets --output hexarth_trial --duration 24
.venv/bin/python research/hexarth/check_timestep.py --assets hexarth_assets --trial hexarth_trial
.venv/bin/python research/hexarth/verify_results.py hexarth_trial
blender -b -t 4 -P research/hexarth/render_still.py -- hexarth_assets hexarth_trial walk-step-10mm hexarth_trial/preview-step10.png
```

`receipt.json`, `convergence-receipt.json` and `environment.json` below are
small copies of the executed evidence. No Site, player library catalog or
existing replay was changed or published by this trial. OEM asset
redistribution terms remain unconfirmed; generated STL stays private.
