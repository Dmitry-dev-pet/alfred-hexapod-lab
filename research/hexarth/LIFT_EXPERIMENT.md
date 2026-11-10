# HexArth: a 20 mm step with increased foot lift

The existing CAD-native tripod crossed the 20 mm step after changing only its
foot-lift amplitude from 12 to 22 mm. All six feet were supported on top during
the last second, with the chassis beyond the riser. The robot travelled 487.6 mm
after settling. This is a simulated result for the diagnostic model, not a
calibrated hardware limit.

| Commanded lift | Outcome over 24 s | Travel | Quality check |
| --- | --- | ---: | --- |
| 12 mm | Stops at riser, upright | 41.2 mm | Accepted diagnostic |
| 18 mm | Stops at riser, upright | 41.6 mm | Accepted diagnostic |
| 20 mm | Stops at riser, upright | 41.5 mm | Accepted diagnostic |
| 22 mm | All six feet and chassis cross | 487.6 mm | Accepted diagnostic |
| 23 mm | Crossing observed | 490.7 mm | Excluded: 6.04 rad/s peak speed and 1.14 mm sampled contact penetration |
| 24 mm | No physics run | — | IK residual 0.042 mm exceeds the fixed 0.005 mm tolerance |
| 30 mm | No physics run | — | IK residual 5.58 mm exceeds tolerance |

At 22 mm, the maximum actual joint angle is 18.285 degrees within the unchanged
local +/-20 degree range. Maximum torque is 0.980665 N m (the unchanged nominal
cap), peak joint speed 3.832 rad/s, peak sampled exterior penetration 0.434 mm,
and peak chassis tilt 4.125 degrees. There are no recorded self contacts,
non-foot ground contacts, or MuJoCo numerical warnings.

An independent 0.5 ms calculation retains the full crossing verdict and differs
in forward travel by 0.113 mm from the 1 ms calculation. The saved-trace audit
checks 7,200 frames across six physical calculations, exact replay poses,
contacts, torques, targets, actual joint ranges, hashes and numerical finiteness.
Contacts and poses are sampled at 50 Hz; joint speed is tracked every step.

The 2.1 kg mass scenario, CAD assets, per-part convex collision hulls, friction,
estimated inertias, torque/speed envelope, 20 mm stride, 1.6 s cycle, settling
and initial pose are retained from PR #139. No body lift, changed joint limits,
new actuator strength, kinematic chassis displacement or obstacle sensing is used.
The 24/30 mm targets are rejected rather than clipped into an apparently
successful path. The fixed path's IK restriction does not establish a mechanical
maximum obstacle height.

## Reproduce

Use the private baseline CAD asset package referenced by `artifact.json`; source
geometry must match `oem-sources.json`. Use the same MuJoCo 3.3.7 runtime as the
baseline. Source code here is independent of the private mesh bytes.

```sh
python research/hexarth/lift_sweep.py --assets hexarth_assets --output results/lift-sweep --lifts 12 18 24 30
python research/hexarth/lift_sweep.py --assets hexarth_assets --output results/fine-sweep --lifts 20 22 23
python research/hexarth/lift_sweep.py --assets hexarth_assets --output results/convergence --lifts 22 --timestep .0005
python research/hexarth/verify_lift_sweep.py results
```

The four accepted 1 ms cases use the common Robot Replay v1 reader. Their
geometry is copied byte for byte from the authorized baseline package, while
every pose and contact sample comes from the new physical traces. The 23 mm
case remains in the research evidence with its failed quality checks; it is
not included in the normal player comparison.
