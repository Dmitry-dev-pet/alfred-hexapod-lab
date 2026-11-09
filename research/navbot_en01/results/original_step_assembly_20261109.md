# EN01 — original STEP assembly placements (9 Nov 2026)

**Class: source-CAD placement evidence only. Not reconstructed kinematics, CAD-accurate MuJoCo, or sim-to-real.**

## Input and reproducibility

Original [fuwei007/Navbot-EN01](https://github.com/fuwei007/Navbot-EN01) commit 3139f2fb32b38d10611feb905e16f8aa012f2f2e, file Hardware/RebotModel/OriginalRobotModel.stp, Git blob SHA-1 9156683a0bb694c84dc541a3e26b5f6b08db1a41, 12,358,579 bytes.

The original source has 172,707 parsed entities and **76 assembly usage links**. The auditor follows NEXT_ASSEMBLY_USAGE_OCCURRENCE → PRODUCT_DEFINITION_SHAPE → CONTEXT_DEPENDENT_SHAPE_REPRESENTATION → relationship transformation → ITEM_DEFINED_TRANSFORMATION → paired AXIS2_PLACEMENT_3D. The source entity graph was inspected directly and all 76 pairs resolved with normalized orthogonal frame axes.

To reproduce with a locally acquired, unchanged original STEP:

    python research/navbot_en01/step_assembly_audit.py --step /path/to/OriginalRobotModel.stp --out results/navbot_en01/assembly_20261109.json
    python -m unittest discover -s tests -p 'test_navbot_en01_step_assembly_audit.py' -v

The first command rejects a different upstream blob, malformed or incomplete placement frames. It creates a JSON receipt with both frames for every instance and no embedded STEP binary. **5/5 synthetic regression tests passed** on Python 3.13. Do not claim the new Python CLI has been run on the actual binary yet; real STEP parsing here was performed via the repository source blob reader with equivalent link/axis checks.

## Raw assembly placement evidence

All locations below are **source frame-B origins in CAD mm**, not verified physical joint axes or wheel axle positions:

| Instance pair | STEP IDs | Origin A | Origin B | Euclidean distance |
|---|---|---|---|---:|
| QUANUM GIMBAL MOTOR 2208 ASM:1/ASM:2 | #426/#427 | (-2.8903,-20.9145,65.075) | (-2.8903,-20.9145,-1.075) | **66.15 mm** |
| STS3032:1/STS3032:2 | #424/#425 | (-187.8768,-152.2758,55.75) | (-187.8768,-152.2758,8.25) | **47.50 mm** |
| Hub:1/Hub:2 | #471/#472 | (-2.8903,-20.9145,-5.15) | (-2.8903,-20.9145,69.15) | **74.30 mm** |

The STS3032 part model origins are very far from the robot centre; substituting these origins as hip pivots would be invalid. Tire part cylindrical faces in the previous semantic audit have candidate radii 18.3/18.5 mm, but that is **not** proof of the effective rolling radius. The existing approximate MJCF uses 24 mm wheel radius and hip anchors at ±43 mm (86 mm nominal separation); the new 66.15 mm motor-assembly placement separation is **not directly comparable** with that 86 mm without resolving the correct kinematic reference frames.

## Scientific gate

Do not make a new *CAD-accurate v1 MJCF* by simply changing geometry numbers. Next, use OpenCascade on the original STEP to resolve real part geometry, shafts/joints, rolling surfaces and articulated transforms; reconcile motor SKU (BOM 2208 90KV versus guide TMCM-6210), torque-speed, power, mass/inertia and controller timing. Only then build a versioned v1 model (preserving v0), run real CPU MuJoCo stand/drive/crouch/jump/fall with qpos/qvel/contacts and SHA receipts, and route review-only results into the existing Android viewer after hardware WebGL testing. No MP4, auto-merge, auto-publication or purchase.

**Publication gate: BLOCKED — actual robot kinematics and actuation not yet identified.**
