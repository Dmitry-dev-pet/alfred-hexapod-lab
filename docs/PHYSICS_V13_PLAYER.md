# Hexapod Lab: V13 · MuJoCo V12 — actual dynamics replay

Date: 2026-11-06. Status: **53 physics experiments recorded offline, research only, not hardware qualification**.

This is a **new versioned entry in the existing Android Hexapod Lab player**, *not* a separate app and *not* real-time physics on the tablet.

## What the player actually shows

- 53 free-base MuJoCo `mj_step` experiments from physical V12, rebuilt with the **same source integrator, same 29 original V11 JS command traces and original CAD30**. Four variants: 29 nominal (mass 1.805 kg, friction 1, leg torque 0.7 Nm), 10 heavier (2.08 kg), 7 low friction (0.35) and 7 weak motors (0.35 Nm).
- **Actual measured `d.qpos`** for all 27 coordinates: root world position xyz (3), root quaternion wxyz (4), and joint coordinates (20); sampled at **50 Hz** from 500 Hz MuJoCo physics. This is *not* the command trajectory, not approximate planar GroundLock, not browser pseudo-physics. The actual qpos root joint origin may differ by ≤~1 mm from the V12 report's `base_link_2` body-frame `d.xpos` due to link offset.
- Six physically loaded foot contacts and `Fz` from original `mj_contactForce` snapshot (leg threshold >0.05 N). Highlighted CAD30 toe dots: green for >0.5 N, yellow for light load, muted blue for swing. HUD shows number of loaded feet and **maximum tangential contact speed** in mm/s, not an estimated accumulated slip distance. No fake contact force readings.
- Screen: variant, experiment, playback/pause, speed, timeline/scrub, reset camera; full Three.js CAD30 mesh. 3D positions and joint poses interpolate between true physical samples for smooth viewing; no changes to recorded physics.
- The viewer reuses 30 user-cached CAD STL meshes, original `cad-rig-live-v1-slow.json`, V4 skeletal coordinate references and V11 verified tibia-tip positions. It does **not** redistribute originals (CAD/XML/STL) or fetch model binaries from GitHub.

## V12 fidelity checks

The complete V12 evidence archive is stored on [owner Google Drive](https://drive.google.com/file/d/1I8BSmqe2FtHrP_bb-oYQjQ7RJ6vwMCqa/view), and main physical report/source are in [draft PR #70](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/70). All 53 regenerated runs have the same sample timestamps, per-leg Fz, contact count and maximum loaded-contact speed as the **original V12 records**; original per-case SHA-256 stored in every replay case. Re-generated root **joint** origin is compared to the original report's `base_link_2` body position with ≤1.2 mm tolerance due to their frame offset. All required qpos quaternions normalized, and full recordings are 50 Hz.

Original scene SHA-256 (nominal): `fe9d7c292b199f19b11da220e74f826e05fb5433a43c5b46c1bc620eb34f6df3`. Upper mass scene: `5f7b4b35a7f28ce58f6e44b315e1bf700a03b3f4973cbaaa214e3b8354e5a2e2`. Original V11 factory q target and model checksums captured in V12 receipts.

**Warnings:** a simulation that stands up or does not fall in a brief run is not proof of physical robot stability. OEM tripod and wave have contact dropouts, foot slip, inaccurate yaw and torque saturation in this model. Real PWM/servo limits and friction are not calibrated. This viewer displays the measured MuJoCo states only, without any actuator control interface.

## Reproduction

The builder (`research/physics_v13/build_replays.py`) uses a byte-for-byte copy of the V12 runner (`research/physics_v13/reference_v12_runner.py`), adding observation-only full qpos recording through one identified `snapshot()` return insertion. It must **never change or patch the integrator dynamics**. Only measured states are emitted. To regenerate, locally restore `source/mjcf/` and `source/meshes/` original CAD30 from the owner's private Drive; unpack V12 [evidence ZIP](https://drive.google.com/file/d/1I8BSmqe2FtHrP_bb-oYQjQ7RJ6vwMCqa/view) to `research/physics_v12/` including `data/targets_v11.json` and `results/`. The original OEM V7 + V4 JSON are already in Hexapod Lab. Set Python 3.13, MuJoCo 3.14.0 and NumPy. Then run:

```bash
for variant in nominal_full upper_mass_full friction_035_full torque_035_full; do
  python research/physics_v13/build_replays.py --variant "$variant"
done
python -m unittest discover -s tests -p 'test_physics_viewer_v13.py' -v
node tests/test_replay_controller_v13.mjs
```

Outputs are immutable `docs/hexapod/data/physics-v12-*.json` plus four small local viewer files in `docs/hexapod/physics-replay-v13/`. **No GitHub main merge without owner approval.** Freenove Big Hexapod OEM gait algorithm retains original project attribution (CC BY-NC-SA 3.0).
