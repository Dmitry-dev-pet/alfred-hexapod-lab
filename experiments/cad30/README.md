# FNK0052 Freenove — CAD30, MuJoCo local experiment, 3 November 2026

**Experimental model, NOT physically calibrated and NOT full 32/32 CAD.**

## Provenance and verification
- Community source: `MattiaGrigoli/autonomous_hexapod`, MJCF PR #1 commit `5c9a84c16335b30036e451c691d2c544abf4bc72`.
- 30 original STL verified by size and **Git blob SHA-1** against `data/fnk0052_cad_assets_lock.json`.
- Two missing original STL: `topplate.stl` (1,057,284 bytes) and `bigservo1.stl` (3,590,184 bytes). Their 19 instances use **visual-only** box stand-ins and have no physical contact or mass. Do **not** claim identical visuals to the original CAD model.
- Original 20 actuated CAD joints; 18 leg joints with six real Freenove tibia STL mesh contact hulls. The original CAD masses/inertial placements are retained and corrected with **estimated** servo/group masses and additional battery/Pi/accessory masses.
- Freenove retains copyright to original CAD; Mattia Grigoli obtained written permission for publishing open-source ROS2 simulation assets; see `https://github.com/MattiaGrigoli/autonomous_hexapod/blob/main/LICENSE-3RD-PARTY.md`. These assets are **not MIT-licensed**. This bundle is for private research; do not redistribute the CAD assets without checking permitted terms.

## Run locally (no GitHub Actions)
A compatible Python 3.13 environment needs `mujoco==3.14.0`, `numpy`, `scipy` and for video `Pillow`, `ffmpeg`, Xvfb or EGL.

```bash
python experiments/cad30/build_30meshes.py
python experiments/cad30/run_local30_stand.py
python experiments/cad30/locomotion_cad30.py --gait tripod --mass nominal --seconds 8
python experiments/cad30/locomotion_cad30.py --gait wave --mass nominal --seconds 8
xvfb-run -a env MUJOCO_GL=glfw python experiments/cad30/render_cad30_gaits.py
python -m unittest discover -s experiments/cad30 -p "test_*.py" -v
```

Directory `source/meshes` has original 30 pinned STL, `source/mjcf` original pinned XML and derived cases, and `results/` JSON physics receipts and original joint-state video replays. Video is rendered from genuine recorded MuJoCo qpos snapshots; **the body is never kinematically translated**.

## Main measured *in simulation* outcome
8 seconds of free-root simulation after 1 second of standing stabilization, scenario masses 1.644 / 1.804868 / 2.080 kg. Parameters: 35 mm commanded step, 20 mm commanded foot lift, 2.4 s period, PD torque limited to assumed 0.7 N·m/leg; stable flat ground, simulated friction.

| Mass | Tripod body advance | Wave body advance |
|---|---:|---:|
| 1.644 kg | 12.05 cm | 10.13 cm |
| 1.805 kg | 11.92 cm | 10.07 cm |
| 2.080 kg | 11.68 cm | 9.92 cm |

Notably, the nominal-wave experiment occasionally had only three ground-contact legs despite a nominal 5-leg support duty, and the light tripod briefly had just two contact legs. **Small body tilt and no fall within 8 s do not prove robust stability.**

The maximal absolute joint angle (around 1.68 rad at 35-mm commanded step) nearly reaches the local numeric 1.7 rad search bound; actual servo travel and PWM zero/sign remain uncalibrated. A shorter 20-mm step and 12-mm foot lift reduced max angle to ~1.59 rad but roughly halved travel; supplementary nominal JSON receipts are preserved.

These are **centrally phased tripod and wave controllers**, not an insect-inspired distributed controller or hardware evidence. Next: missing assets if full visuals are needed, hardware motor calibration, friction sensitivity, perturbations, obstacle stance, and insect-inspired local feedback coordination.

**GitHub code path**: experiments/cad30/; original Freenove STL and MJCF are never committed, but shipped in the private Drive ZIP. Extract `source/` from that ZIP into the repository root before running. The standalone ZIP also contains scripts runnable from its own root.
