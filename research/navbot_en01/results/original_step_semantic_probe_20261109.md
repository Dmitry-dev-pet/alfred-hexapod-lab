# EN01 — original STEP semantic probe (2026-11-09)

**Classification: SOURCE STEP ENTITY ANALYSIS / NOT YET ARTICULATED CAD / NO NEW PHYSICS.**

## Source and reproducibility

- Upstream `fuwei007/Navbot-EN01@3139f2fb32b38d10611feb905e16f8aa012f2f2e`
- Path: `Hardware/RebotModel/OriginalRobotModel.stp`
- Git blob SHA-1 (provided by GitHub contents API): `9156683a0bb694c84dc541a3e26b5f6b08db1a41`
- Returned STEP text length: **12,358,579 characters** (do not confuse this with an independently verified raw-file SHA-256).
- Method: read pinned Git blob through the connected GitHub API; traverse ISO-10303-21 entity references: product → formation → definition → definition shape → shape representation → BREP → shell → face → cylindrical surface → axis placement. This is semantic/topological source inspection, **not** an OpenCascade import or verification of all component assembly transforms.
- Explicit length unit: `SI_UNIT(.MILLI.,.METRE.)`, i.e. mm.

## Extracted structural evidence

| Source entity count | Value |
| --- | ---: |
| Total parsed `#id=` entities | 172,707 |
| `PRODUCT` entities | 46 |
| `MANIFOLD_SOLID_BREP` entities | 62 |
| `CYLINDRICAL_SURFACE` entities | 1,584 |
| `ADVANCED_BREP_SHAPE_REPRESENTATION` links | 39 |
| Links to named products resolved through this traversal | 27 |

The pinned STEP contains named components **STS3032**, **QUANUM GIMBAL MOTOR 2208 ASM**, **QUANUM GIMBAL MOTOR 2208 BASE**, **QUANUM GIMBAL MOTOR 2208 OUTRUNNER**, the tire component `2208电机轮胎`, mirrored thigh components (`大腿`) and lower legs (`小腿_2208电机版本`). A 2208-labelled *CAD component* is consistent with the upstream BOM but **does not establish what motor is shipped**; the linked assembly guide mentions a different SKU.

### Wheel/tire source geometry

Product `2208电机轮胎` → BREP `#4714` → shell `#63625` → 74 faces, of which 46 are cylindrical surfaces. Its largest coaxial cylindrical-face radii are:

| Radius | Count of faces | Local axis point | Local direction |
| --- | ---: | --- | --- |
| **18.5 mm** | 2 | (0, 0, 0) mm | (0, 1, 0) |
| **18.3 mm** | 1 | (0, 6, 0) mm | (0, -1, 0) |
| 13.875 mm | 20 | (0, 0, 0) mm | (0, 1, 0) |
| 13.0 mm | 3 | (0, 12, 0) mm | (0, 1, 0) |

**Important:** A cylindrical face of radius **18.5 mm** is a grounded *candidate outer tire envelope*, **not yet a calibrated effective rolling radius**. The current approximate `model_v0.xml` sets `wheel radius=0.024 m` (24 mm), about 30% larger than this candidate (24/18.5 − 1). A tire feature must not be confused with a *thigh* curved surface: the source includes 58–59.5 mm cylinders on the named thigh components. Do **not** use those as wheel radii.

## Required before promotion to source-constrained physics

1. Verify the original STEP's *assembled* world transforms and distinguish tire, rotor, hub, leg, thigh, spring and chassis bodies. Derive left/right wheel and hip joint axes in one documented coordinate system; do not substitute the raw local axis (0,1,0) as the final world axis.
2. Independently validate the tire outer profile and effective contact radius, then create **a new versioned EN01 model**, preserving existing `model_v0.xml` results unchanged. Do not silently overwrite past 24-mm physics.
3. Establish part masses, COM, inertia and collision geometry. STEP volumes alone are not mass.
4. Confirm actual supplied wheel motor SKU and torque/voltage/current/speed, STS3032 wiring and supply regulation. Source names are not motor validation.
5. Run fresh MuJoCo `mj_step` against this updated model for stand, crouch, drive, jump, fall, with contact and torque sanity checks. Compare with 2026-11-08 approximate-v0 results (stand x-drift 0.338 m, crouch 0.387 m, both jump FAIL).
6. Only then export measured `qpos/qvel/contacts` through PR #91/#96 to review-only 3D entries. No MP4/GIF, fabricated root motion or new viewer.

## What this work did **not** establish

- No executed CAD-accurate articulated MuJoCo model or new physical trajectory.
- No original STEP import via CadQuery/OpenCascade in this run; entity-level inspection used the connected pinned Git blob. The existing `audit_step.py` can independently inspect the actual file when locally accessible.
- No inferred joint transformations from just one local cylindrical axis.
- No actual physical hardware, new Android player entry, source CAD redistribution or `main` change.

Issue: [#93](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/93), source model [PR #95](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/95), existing pipeline [PR #96](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/96).
