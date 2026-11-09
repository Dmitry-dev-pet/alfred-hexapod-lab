# EN01: the folding poses are mechanically inadmissible

The user's report is confirmed. The v2-v4 surrogate allows the original visual
leg to enter the chassis. The previous brief numerical takeoffs do not establish
a physically admissible EN01 jump. Neither replay clipping nor a different camera
would repair this model defect.

## Evidence

The frozen hip range is **-0.22..+0.65 rad** (-12.61..+37.24 degrees). These are
diagnostic choices, not measured mechanical stops. Knee and rod hinges have no
separate limits; four-bar closure alone does not prevent solid interference.
Every robot geom has contype=2/conaffinity=1, so all robot/robot pairs are disabled
by the masks. Floor contact remains enabled.

A saved-frame audit of all 24 frozen v4 traces finds declared-limit excursions
in 12 records. The worst saved hip angle is about -0.25576 rad, **0.03576 rad
(2.05 degrees) beyond the declared lower limit**. MuJoCo's compliant joint-limit
response therefore also needs checking under load. These are sampled extrema,
not a bound on every unsaved physics step.

The exact original STEP was retrieved at the same pinned upstream commit and
validated against Git blob `9156683a0bb694c84dc541a3e26b5f6b08db1a41`
and SHA-256 `4715cf940718af95872bd2a771fdde61de3376bb748936e2299f94fa76416947`.
OpenCascade BRep intersections use the analytical closed four-bar branch and
preserve the original lateral part placements. This independently checks solid
geometry instead of inferring interference from a picture or the surrogate capsules.

| Hip angle relative to CAD pose | Exact intersection witness |
| --- | --- |
| 0 rad | No intersection in the tested subset |
| -0.04 rad (-2.29 degrees) | Calf 62 / side plate 43: 2.471 mm³ |
| -0.08 rad | Calf / bottom plate: 45.285 mm³; calf / side plate: 61.901 mm³ |
| -0.18 rad | Calf / bottom plate: 105.640 mm³; calf / battery: 560.165 mm³; calf / side plate: 184.749 mm³ |
| -0.22 rad | Calf / battery: 891.486 mm³ |

Ten discrete angles were checked for right-side calf/tire/rod parts 62/65/28
against fixed parts 40/41/42/43/53/56/72. Five angles have intersections larger
than the 0.01-mm³ reporting threshold. No interval is certified collision-free:
the positive test poses and this partial pair set cannot establish complete
mechanical stops. The source README's encoder-2048 squatted stop still lacks an
absolute CAD-angle calibration. **Do not replace -0.22 with -0.04 and call it a
measured safe limit.**

## Changes and verification

`model_validity.py` checks the exact trace/model hash, recorded hinge excursions
and collision masks. The source-jump exporter attaches the audit and keeps
status `failed`; no source-jump replay can be promoted from a flight metric to a
physical success with this model. Recorded frames and source hashes are preserved.
Frozen raw files, old replays and the live Site are unchanged by this draft.

`cad_interference.py` reproduces the pinned-STEP witnesses. Receipts contain all
ten pose results and the 24 trace audits with raw SHA-256 bindings. Four new
validity/export tests and eight existing source-jump tests pass, including the
actual compiled upstream command sequence and independent-FK forgery rejection.
The new exporter test confirms that every measured frame remains unchanged.
This is a geometry/audit change; no new physical experiment is claimed.

Reproduce from the repository root with MuJoCo 3.14.0, CadQuery 2.7.0 and the
separately retrieved pinned STEP:

```bash
python -m research.navbot_en01.cad_interference --step /path/OriginalRobotModel.stp --out /tmp/interference.json
python -m research.navbot_en01.model_validity --trace /path/a-zero-p5-jump_raw.json --out /tmp/validity.json
python -m unittest discover -s tests -p test_en01_model_validity.py -v
```

The export integration test requires `EN01_JUMP_RAW_TRACE` pointing to the frozen
trace; the existing C++ comparison uses `EN01_UPSTREAM_SOURCE_DIR`. Raw data remain
in the [frozen v4 archive](https://drive.google.com/file/d/1REeM7FhVl_CcE4rS_Z5v8Yz0I7TYkxmI/view?usp=drivesdk).

## Required physical repair

Determine the continuous admissible range using all relevant original solids
and the author's encoder stop; calibrate the servo/CAD offset. Add appropriate
self-contact geometry and explicit mechanical-stop response, with exclusions
only for intended joint mating surfaces. Verify that applied torque and impact
cannot drive the assembly through those stops. Re-run stand, crouch and jump
with the declared mass/actuator assumptions and generate new measured replays.
Do not alter the old raw trajectories to make them look admissible.

This draft **diagnoses and flags** the defect; it does not yet repair/calibrate
the simulator or replace the published clips.

Run: https://github.com/Dmitry-dev-pet/truthrail/issues/318
