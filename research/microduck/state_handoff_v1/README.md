# MicroDuck: handoff from roll to get-up on a step

The frozen roll and velstand policies can reach the upper platform after a
partial forward rotation. In the nominal 1 cm case, handing over at a measured
135 degrees leads to both complete soles supported on top and a quiet final
one-second stance, with peak forward rotation about **158.6 degrees**. The
same handoff on 2 cm reaches the platform at about **172.8 degrees**, while
retaining the earlier joint-range quality failure. This is a new coordination
of existing skills, not new training or a proven robust climbing controller.

## Nominal comparison

| Step / handoff | Platform goal | Fixed motion-quality gate | Peak forward rotation | Motor-range excursion |
| --- | --- | --- | ---: | ---: |
| 1 cm / original 2 s timer | Reached | Pass | 364.5 deg | 0.974 deg |
| 1 cm / 135 deg state trigger | Reached | Pass | 158.6 deg | 0.974 deg |
| 2 cm / original 2 s timer | Reached | Fail | 364.3 deg | 1.538 deg |
| 2 cm / 135 deg state trigger | Reached | Fail | 172.8 deg | 1.538 deg |

The state-triggered nominal handoffs occur 0.48 s / 0.58 s after the roll
trigger, compared with 1.98 s for the original timer. The first complete top
support is later, not earlier: 1.88 s / 1.66 s after triggering, versus
1.18 s / 1.22 s with the full roll. The result is less forward rotation;
there is no speed, energy, impact-tolerance or hardware-safety claim.

Forward rotation uses the existing integrated body-local pitch angular
velocity definition for comparison. Goal achievement independently requires
the full original sole meshes inside the platform, both feet carrying top
support, and quiet stance for the final 1 s. The two endpoint facts are not
substituted for a claimed complete somersault.

## Bounded search and sensitivity

All **42** primary trajectories are retained: 24 original-timer/angle/contact
comparisons, eight targeted early-handoff/head-target follow-ups, four combined
posture/handoff probes, and six initial-pose sensitivity checks. This is a
parameter search, not 42 independent reliability trials. Of these, 26 reach
the platform goal and nine also pass the unchanged motion-quality gate.

The early 75/90-degree transfers stop the roll near the edge. Later transfers
on 2 cm inherit the head-pitch range excursion, which peaks approximately
0.38 s after triggering, before handback. Restricting only the head-pitch
target either reduces that excursion or prevents the necessary motion; it
does not produce an accepted 2 cm climb in this series. Blending the two
networks over 60/150 ms is not consistently an improvement.

Three +/-0.005 rad initial-joint perturbations were tested for a 1 cm,
135-degree transfer with a larger 0.14 rad command margin, and for the 2 cm,
135-degree transfer at the original margin. Each reaches the platform in
two of three trials; neither has a trial passing both goal and quality.
The nominal 1 cm default uses the original 0.07 rad margin; those three
perturbations do not estimate reliability of that different setting.
Two nominal 135-degree trajectories repeat byte-for-byte.

## Fixed model and action boundary

Same model and inputs as `../step_goal_v1`: original RL source
`bae39f5900b318f7e87fc485c30a31213cfa14eb`, BAM source
`62bd8ce12154340be97e06f7f41a0ca8f116d967`, original v8 ONNX binaries and
70 visual mesh instances. Robot masses, inertias, joint ranges, XL330 M6
parameters, voltage/sag and output-encoder backlash remain unchanged.
Uses the corrected complete environment-contact coverage, sparse original
self-collision coverage, 0.5 ms physics and 20 ms control.

Only policy switching, target blending and explicitly logged command margins
change. Blending evaluates both frozen networks from the same current
observation; the actual applied target is fed back as action history.
There is no runtime root repositioning, velocity injection, external force,
changed neural weight, extra joint or increased motor torque.

Angle switching uses a measured rotation threshold. Contact switching uses a
foot's measured upper-platform contact, with a 3 s fallback timeout. The
latter is a simulation-state criterion; real obstacle/contact sensing and
hardware deployment are not implemented here.

Quality remains: no physics warning/reset, environment and represented
self-contact penetration <1 mm, and motor-joint overshoot <1 degree. This is
an experimental tolerance, not a manufacturer rating. Achieved ascent and
quality are recorded and displayed separately. All rejected trials remain.

## Verification and reproduction

The source gate checks 259 upstream RL blobs, 112 BAM blobs, installed BAM
bytes, policy checksums and seven pinned local helper blobs. Both original
timer controls reproduce all prior qpos/qvel and applied motor targets exactly.
The independent audit rebuilds sole bounds and final support from original
CAD for all 20,263 saved poses, checks target limits and causal switching,
queries all 70 visible convex shapes against the platform, rejects fabricated
sole geometry and illegal targets, and checks two exact numerical repeats.

Saved-state distance auditing additionally bounds a negative convex-distance
result by exact world mesh-vertex / platform-box separation. A signed distance
cannot imply greater overlap than the world-axis translation that separates
the vertex bounds. Corrections retain the raw result and bounding vertices;
this audit safeguard never modifies physics or the motion-quality gate.

Use Python 3.12 and the requirements here, plus the restored BAM source
installed with `pip install --no-deps -e PATH_TO_BAM`. The input workspace is
in the previously verified [source archive](https://drive.google.com/file/d/1q_CmK4XggV4fv_21_Pa75-B9tKNp-que/view?usp=drivesdk),
SHA-256 `881977efcf04af626928c779be39f99c79e59091f2a19250ad58ad947cd16208`.
From the repository root:

```bash
python research/microduck/state_handoff_v1/probe.py --workspace /absolute/workspace --cases research/microduck/state_handoff_v1/cases-all.json --out /absolute/traces
python research/microduck/state_handoff_v1/verify.py --workspace /absolute/workspace --traces /absolute/traces
```

`cases-repeat.json` reruns the two selected nominal cases into a separate
directory; pass that directory with `verify.py --repeat`. The independent
original-control comparison additionally accepts `--baseline-traces` pointing
to the prior archive's `experiment/traces`. Full data and their checksum are
referenced by `storage.json`. `summary.json` and `verification.json` are compact
receipts. No independent CI or physical robot test is claimed.

New original-STL recordings are published through the existing GitHub player
library, with no Site rebuild. Main is not merged. Research coordination:
[Truthrail #336](https://github.com/Dmitry-dev-pet/truthrail/issues/336).
