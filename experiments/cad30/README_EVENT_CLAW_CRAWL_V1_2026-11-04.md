# Issue #21 — event-controlled front claws, dynamic CAD30 probe v1

**Status: partial positive, full four-legged gait NOT achieved.**

This is an original experiment extending the mathematical LP/QP oracle from the
existing CAD30 research. A central controller selects **which actual leg** to
move only after checking the live contacts and a model-based static wrench
balance; no fixed wave/tripod order is supplied. It is not neuroWalknet,
MPC, or evidence of self-organizing neural locomotion.

## What was physically executed (MuJoCo 3.14.0, Python 3.13.5)

- Freenove FNK0052 CAD30, 30 original third-party STL imported from the
  previously verified private Drive bundle, 2 visual mesh proxies, 20 modeled
  actuator channels. No root teleport, body weld, forced contacts, or
  artistically generated gait.
- Settle in a six-legged stand, then command a 35 mm shift of four stance
  foot targets so the free body actually moves backward under gravity.
- A static LP/QP oracle checks the four-support configuration before raising
  the two front claws; the front tibias stay above the floor during the events.
- For each proposed stance swing, enumerate all four remaining legs, measure
  the actual contacts and COM, reject a hypothetical liftoff with COM outside
  the remaining triangle or with insufficient model torque reserve. Test
  reachable foot IK, then command a smooth, real MuJoCo swing and touchdown.
- For zero eligible legs, search a feasible preparatory COM shift using the
  triangles (IK plus actual ground dynamics) and abort rather than pretending
  a successful walk if it cannot be reached.

The resulting v1 still uses central, instantaneous **quasistatic** LP/QP
planning, and a handcrafted foot-swing primitive. Dynamic path safety is
checked after executing MuJoCo, not proved by the oracle in advance.

## Fixed 10-second measured runs

| Model friction coefficient | Actual result | Min contacts at every 2ms | Fraction of 50Hz samples with negative support margin |
|---|---|---:|---:|
| μ=1.1 | Both front claws raised; rear CAD `bl` then `br` lifted and landed; middle legs vetoed | 3 | 0% |
| μ=0.6 | Same rear-lift motions, **but brief two-support intervals** | 2 | ~0.60% |
| μ=0.3 | Four-support claw pose not sustained, **no qualifying rear lifts** | 0 | ~1.80% |

In the nominal μ=1.1 run: `bl` tibia max clearance **2.59 mm**, `br`
**2.91 mm**, with contact genuinely absent for 904 ms / 938 ms during their
swings. **Both steps commanded ~8 mm BACKWARDS**, not a forward walking cycle.
Measured forward displacement while the claws were airborne is **0.41 mm**.
The middle-leg candidate COM margins were ~**−91 to −95 mm**; after both rear
swings, no admissible IK body-prepositioning target was found.

Max model torque reached the **assumed** ±0.7 Nm cap for ~0.02% of the
simulation, so this also fails the strict no-saturation criterion. Initial
mass estimate is 1.804868 kg, not weighed hardware; friction coefficients
and servos are uncalibrated.

**Strict #21 success? NO:** both middle legs never took steps; forward
translation is far under 20 mm; one or more samples showed model torque
saturation. We do not interpret rear toe motions as walking on four legs.

## Files, evidence, and reproducibility

- `event_claw_crawl_v1.py`: actual physics controller and event log
- `tests/test_event_claw_crawl_v1.py`: local mathematical/logic regression
  (5 unit tests passed)
- `render_event_claw_crawl_v1.py`: two-camera rendering directly from saved
  `qpos_10hz`, no animation of poses or artificial translation
- `results/event_claw_crawl_mu1p1.json`: 10s trajectory, 50Hz contacts, 10Hz
  recorded qpos; analogous compact logs for μ=0.6 and μ=0.3
- `results/event_claw_demo/`: evidence MP4/GIF (7.4s excerpt) and keyframe
- all complete files including SHA-256 retained in the project archive on
  private Google Drive; no Freenove STL binaries are redistributed.

Run from root of an authorized local workspace **with the original meshes**:

```bash
python -m unittest experiments.cad30.tests.test_event_claw_crawl_v1 -v
python experiments/cad30/event_claw_crawl_v1.py --seconds 10 --mu 1.1 --out results/event_claw_crawl_mu1p1.json
MUJOCO_GL=egl python experiments/cad30/render_event_claw_crawl_v1.py --trace results/event_claw_crawl_mu1p1.json
```

## Next physical hypothesis

A fixed COM location cannot safely remove each of four supports in turn.
To lift a middle leg we need a *planned multiphase body-and-foot reposition*
that places COM inside the other three support contacts **and** stays below
velocity-dependent motor torque limits throughout the movement. Continue with
rear-foot repositioning + trunk lateral shift; score full 3-support movement
and safe touchdown, not just positive static start-state margins. Keep this
v1 as a negative control for any claimed four-legged gait.