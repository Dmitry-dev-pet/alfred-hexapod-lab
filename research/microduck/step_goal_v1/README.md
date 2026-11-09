# MicroDuck: step-goal timing and recovery tuning, 2026-11-09

The frozen v8 forward-roll and velstand policies can put both complete feet on a
corrected 1 cm platform in a nominal motor-driven approach. The selected trace
repeats byte-for-byte. The same timing reaches the 2 cm platform but exceeds the
unchanged joint-limit quality gate. The tested 3 and 5 cm cases fail the platform
goal. Small initial-pose perturbations also fail the joint-limit gate, so this
is not a demonstrated robust climbing controller.

## Goal and fixed model

Success requires both entire sole meshes inside the upper platform, both feet
carrying vertical top support, and a quiet standing posture for the final 1 s.
Full forward rotation is a separate measured result. A partial edge stance does
not satisfy the goal.

The additional quality gate is unchanged throughout: no MuJoCo warnings or
physics resets; environment and represented self-contact penetration below
1 mm; actual motor-joint overshoot below 1 degree. Targets remain inside the
original ranges. The 1 degree tolerance is an experimental simulation gate,
not a manufacturer specification. A successful visual endpoint can still be
rejected by this gate.

Uses the corrected contact model from alfred-hexapod-lab commit
`471418514b7ad89ccdbca82985f4fce5327d1bd2`, PR #125: environment collision coverage
for all 70 original visible mesh instances, unchanged robot masses/inertia,
joint ranges, motors and policy weights. MuJoCo step 0.5 ms, policy 20 ms, BAM
M6 with output-encoder backlash, 7.4 V supply and existing voltage-sag model.
Self-collision coverage remains the original sparse approximation. The checks
do not establish hardware motion or hardware impact tolerance.

No root momentum injection, runtime root/terrain repositioning, or external
force is used. Optional jitter is applied only before simulation starts.
The two documented helper edits only resolve workspace paths; reversing these
edits reproduces the pinned Git blob SHA-1 values.

## Bounded search and result

All 37 main/follow-up outcomes are preserved, together with the two initial
smaller-margin smoke trials. Trial counts describe parameter search, not a
statistical reliability rate.

| Setting | Measured outcome |
| --- | --- |
| Flat original-roll control | Complete forward turn, stable endpoint, quality gate passes |
| 1 cm, velocity command 0.30 m/s, trigger toe gap 3 cm | Roll timers 1, 2 and 3 s pass the platform and quality gates |
| Selected 1 cm, 2 s timer, target margin 0.07 rad | About 364.5 degrees peak forward rotation; 3.96 s continuous complete top support; final 1 s stable |
| Same selected timing, 2 cm | Platform goal reached, rejected: actual joint overshoot about 1.54 degrees |
| 2 cm, larger target margins 0.105/0.14/0.21 rad | Platform goal reached, rejected: joint overshoot about 1.42/1.34/1.07 degrees |
| 3 cm / 5 cm, selected timing | Platform goal not reached; peak forward rotation about 87 / 78 degrees |
| Three +/-0.005 rad initial-joint jitters, selected 1 cm | All reach the platform goal; all rejected: joint overshoot about 1.08–1.24 degrees |
| Final target margins 0.28/0.35 rad, 1 and 2 cm | Quality gate passes; platform goal fails |

The selected approach uses an ordinary learned gait. Its command is 0.30 m/s;
measured forward velocity at the roll trigger is only 0.04478 m/s. The maximum
forward approach velocity is recorded separately. Commanded speed is not the
robot's measured speed.

Four recovery-assistance trials request 0.12/0.24 m/s after the roll. Both feet
are already supported on top before handback, so the goal-aware stop condition
suppresses that assistance. These are duplicate physical controls and provide
no evidence that an extra recovery command improves the climb.

The larger-margin trials keep the original joint ranges and original binary
quality thresholds. They constrain motor targets more strongly; they do not
change the physics to admit a result. The two initial 0.035 rad target-margin
smoke trajectories remain under `traces-original-margin/` in the archive.

## Verification

`verify_source.py` reverses only the two workspace path edits and verifies all
seven helper Git blobs against the pinned repository tree. The existing source
gate checks 259 upstream RL blobs, 112 BAM blobs, installed BAM source bytes,
and the stand-policy checksum; the roll-policy checksum is independently pinned.

`verify_results.py` independently rebuilds original mesh sole bounds from all
saved generalized positions, derives top support from recorded contact IDs,
normals and loads, checks the final standing window and target limits, queries
all 70 visible mesh convex distances to the step, and rejects deliberately
tampered sole bounds and illegal motor targets. Per-physics-step extrema are
retained separately from sampled mesh checks. The selected nominal trace is
rerun and compared byte-for-byte. `verification.json` contains the evidence.

`microduck-step-recovery.png` is an actual MuJoCo/EGL rendering of three saved
states with the original STL geometry. It does not synthesize motion.

## Reproduction from the archive

The archive contains `experiment/` and `workspace/`, the full original pinned
RL/BAM file sets, policy binaries, all traces and their hashes. There are no
credential or runtime-session files. Use an appropriate Python environment:

```bash
python -m pip install -r experiment/requirements.txt
python -m pip install --no-deps -e workspace/microduck-work/bam-62bd8ce12154340be97e06f7f41a0ca8f116d967
python experiment/verify_source.py
```

To verify the saved data, pass the absolute extracted `workspace` directory:

```bash
python experiment/verify_results.py --workspace /absolute/path/to/workspace
```

For a fresh numerical search, use a new empty output directory:

```bash
python experiment/tune_recovery.py --workspace /absolute/path/to/workspace --out /absolute/path/to/fresh-traces
```

The two bounded follow-up scripts operate on `experiment/traces/summary.json`;
the supplied archive already includes their results. To reproduce the full
sequence from scratch, retain a copy of the saved data, generate the initial
search into `experiment/traces`, then run `refine_margin.py` and
`refine_limits.py` with the same workspace argument in that order.

This experiment tunes commands and switching conditions with fixed neural
weights. It has not trained a terrain-aware recovery policy. The higher-step
failures arise before a complete turn, which gives a concrete future training
target: use contact with the edge to carry the body and feet onto the platform,
while retaining actual motor and contact constraints.

Source and results are proposed in a separate research PR. The main branch and
the existing player are not modified by this experiment; the baseline run-up
publication is separately coordinated by Truthrail issue #326. This run is #327.
