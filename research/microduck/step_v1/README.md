# MicroDuck: one 5 mm step — 2026-11-09

Result: reliable ascent is NOT demonstrated. With upstream contacts/5 ms physics, 1 of 3 trials ascended and stopped, 2 stalled at the edge. The previously successful initial state failed with a firmer-contact / 1 ms profile. There were no falls or numerical warnings/resets in any of the five trials. A flat control walked and stopped.

| Trial | Outcome | Duration | Stop command |
|---|---|---:|---:|
| Upstream, seed 20261109 | FAIL: blocked | 15.00 s | none |
| Upstream, seed 20261110 | FAIL: blocked | 15.00 s | none |
| Upstream, seed 20261111 | PASS in this contact model | 6.98 s | 3.96 s |
| Firm / 1 ms, seed 20261111 | FAIL: blocked | 15.00 s | none |
| Flat control, seed 20261109 | walked/stopped | 7.76 s | 4.74 s |

Configuration: original pinned MicroDuck MJCF, original v8 velstand.onnx (no retraining), BAM M6 XL330, output-encoder backlash semantics, 7.4 V, sag gain .1, voltage floor 6 V, Kp 200. Policy updates at 50 Hz. The step starts at x=.18 m, is .005 m high, 1.2 m deep and 1.2 m wide, friction [1,.005,.0001]. A static box is appended without changing original robot bodies, joints, masses, inertia or mesh vertices. Original mesh IDs and all 70 visual instances are preserved.

Protocol: 1 s standing, then velocity command [.30,0,0]. Once both complete sole meshes are beyond x=.185 and at least one sole has top support, command zero and hold 3 s. Otherwise run to 15 s. A fall (>65 degrees) would switch to zero command and permit 6 s of natural policy recovery; none occurred. No root lift, teleport, external push or scripted joint trajectory is applied after initialization. Nominal seed 20261109 uses the original stand pose; the other two use deterministic +/- .005 rad joint perturbations.

Acceptance: both complete soles beyond the edge (>x=.181), both carrying >.05 N top contact, tilt <20 degrees, root speed <.06 m/s and angular speed <.7 rad/s throughout the final 1 s; crossed before stop; no fall, warning or simulation reset. Initial step contact by one foot is not ascent.

Sensitivity: upstream contact penetration reached 3.30–4.02 mm, large relative to a 5 mm step. The firm profile changes geom solref to [.005,1] and physics timestep from 5 ms to 1 ms; max penetration became 1.048 mm, and the previously successful state stalled. Both parameters changed together: their individual causal effects are not isolated. The small sample is not a real-robot reliability estimate. Do not advance to higher stairs on the strength of the single nominal PASS.

Reproducibility: `MICRODUCK_WORKSPACE=/absolute/workspace python microduck-step/step_cpu.py --flat-control --firm-check`, then `python microduck-step/export_step.py`. Use the pinned source/runtime workspace from the preceding reproducibility archive (Drive ID 1S7FgYnz_Y31OedO98oUUC_Gp1loifPqz; ZIP SHA-256 633f948b3340fb6c63087e84a25b33e6f69d6025ae63a4c5384f09befdb9a10e). This archive supplies step source, both benchmark helper modules, raw traces, actual client/binary tracks and verification receipt. Upstream sources, policy and original geometry stay pinned to the same hashes as that archive. No GitHub-hosted Actions were dispatched.

Validation: all four initial trial frame streams repeated exactly; the firm trace repeated byte-for-byte. Five local client tests cover complete decoding of all five tracks and all 70 instances, corruption rejection, offline cache reuse and preservation of negative outcomes. Export checks original robot mass/inertia, joint ranges and mesh vertices against the flat model; Float32 pose error <1e-6. Browser checks verify loading, selection, seeking and the explicit mixed results. Actual WebGL rendering is unverified because managed Chrome has WebGL disabled.

Original 3D: Pollen Robotics, CC BY-NC-SA 4.0; original code Apache-2.0. Route: /hexapod/microduck-step-v1/ on the existing Alfred Robot Player.
