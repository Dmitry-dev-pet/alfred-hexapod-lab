# Wheel-Legged Robotics · P0 protocol

**Status:** DESIGN ONLY — no new model training or physics experiment was run by this protocol.  
**Decision:** 2026-11-08 · owner approval.  
**Issue:** [#93](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/93).  
**Repository:** keep existing Alfred Hexapod Lab; no separate project, no merge into `main` until review.

## Research objective

Test whether a *single deployable learned controller* can control a two-wheel articulated-legged robot through balancing, commanded wheeled travel, squat/height control, vertical jump, landing and self-righting, without a fixed pre-scripted task-by-task state machine. This is a falsifiable research question, **not an achieved capability**.

### Choose the physical testbed

1. **SeBaJu first**, exact upstream `k3nkreate/SeBaJu_BOT_5845@7bbc38f4b26b8ecce7ac0ee37e06164e851dc600`; original nonlinear MuJoCo model and 6 actuated joints (hips, knees, wheels). Pinned baseline already replicated in [PR #90](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/90): 0.180 s whole-body flight; 0.04395 m wheel clearance; recovery after landing; **no** self-righting after side fall.
2. **XGO-Rider2 independent comparator**, not a substitute SeBaJu policy: [#66](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/66). Published RL balance policy currently holds the two legs fixed, so do **not** claim demonstrated learning of multi-joint legs.
3. **Upkie future corroboration / hardware candidate**: https://github.com/upkie/upkie. The upstream Gymnasium example uses PyBullet; a MuJoCo port is **not established**. Pin a commit before using this source in an experiment.
4. **Stanford Pupper** remains a separate quadruped RL reference; do not delete or mislabel old work.

## W0 — independently reproducible baseline (first implementation)

**Do not change the upstream controller when reproducing it.** Keep `source baseline` and `new experimental controller` in distinct records.

For each run, record source Git SHA, code SHA, MuJoCo version, physics `dt`, simulator flags, XML hash, solver and contact settings, initial qpos/qvel, action frequency, random seed, command schedule and complete output manifest. Preserve the original time-indexed numeric trajectory.

| ID | Controller | Scenario | Expected diagnostic; NOT a pass claim |
| --- | --- | --- | --- |
| W0-A | source controller | nominal stand, >=20 sim seconds | time balanced, roll/pitch drift, wheels displacement, torque limits |
| W0-B | source controller | original baseline jump | actual **whole-body** no-contact interval; touchdown; 2 s post-touchdown stable guard |
| W0-C | source controller | side-torque perturbation 12 Nm × 0.15 s at t=12 s | measured tip, actual torso contact, recovery status |
| W0-D | source controller or verified source drive mode | forward/backward/stop | run only if command interface is source-verified; otherwise mark `NOT_RUN_UPSTREAM_MODE_UNVERIFIED` |
| W0-E | explicitly labelled no-control or zero-leg-command comparator | repeat matched states | rule out a fake easy success due to passive dynamics |

Use a verified deterministic baseline replay before introducing randomised settings. For robustness, use at least **three distinct perturbed starts** after documenting the exact perturbations. Multiple seeds without any stochastic perturbation are not independent trials.

**Failure guards:**
- Jump = every ground contact on **wheels, body, and articulated legs** absent for >=20 ms; verify positive clearance and upward COM motion, not just wheel clearance.
- Landing = contact regained, finite velocities and forces, then 2 s of bounded angle and horizontal speed without tip / torso scraping; report settling time separately.
- Self-right = documented fallen configuration with measured body-ground contact and large tilt, then >=2 s truly upright, not merely transient passage through vertical.
- Physical plausibility = finite state, feasible joint ranges, torque/rate limits, contact penetration and solver warnings logged. Unexpected penetration or numerical warning never counts as success.

Capture `qpos`, `qvel`, actual joint controls, torso pose/quaternion, wheel speeds, all external contacts and `Fz`, actuator saturation, COM, roll/pitch/yaw, wheel displacement, takeoff/landing events, fall reason and simulator timings. Choose an explicit coordinate convention in the code and test it.

**W0 DONE when:** a deterministic source replay matches the archived #90 baseline to documented tolerances; negative side-fall remains correctly classified; raw traces + SHA256 manifest are preserved; CI/static tests pass; one reviewer can rerun from pinned sources. Otherwise mark `INCOMPLETE` or `FAIL`, not `PASS`.

## W1–W4 (design gates; NOT STARTED)

- **W1:** define common interface `command=(vx, yaw_rate, body_height, jump_request)`; observation limited to realisable IMU, wheel & joint states and past commands/actions. Simulation privileged signals belong to *teacher/debug only*. Action controls *both* wheels and hip/knee joints, with explicit saturations.
- **W2:** independent baseline + trained policy on balance/drive/squat. Start with minimal CPU smoke, then if genuinely needed use existing authorised machine. Save checkpoints, deterministic evaluation and training curves separately.
- **W3:** add jump/landing via curriculum. Same independent physical test for learned policy and source hybrid; compare actual full-body airborne time, landing stability, torque and contact, no edited animations.
- **W4:** add recovery from diverse physically reachable falls. Evaluate withheld starts, actual time to steady upright, 2 s sustained post-recovery, ground-contact rules, false positives and failure fraction. If the geometry makes recovery impossible, document that as a negative result rather than cheating the start pose.
- **W5:** >=10 held-out perturbed starts, mass/friction/latency perturbations, and (if a source-aligned model exists) cross-engine evaluation. Never call it sim-to-real evidence without hardware calibration.

## Reproducibility and publishing

- **GitHub:** issue/protocol/script/short receipts/sha/checkpoints metadata. **Private Drive:** binary weights, large raw traces, videos, authentic third-party meshes subject to original license.
- **Replay:** only measured states from physics, not a scripted visualization. Reuse the existing Android viewer and the reviewed import pipeline [PR #91](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/91). No new player, no automatic publication.
- **Cost and scope:** do not create new Mac runners, initiate paid GPU jobs, run physical hardware, alter `main`, or auto-merge PRs. Pure code tests may use CI; physics should run in a real verified local/runtime simulation outside free Actions quota.
- This file does **not** claim any new run, learned gait, working self-righting, or Android/WebGL verification.

## Hardware acquisition constraint — owner decision 2026-11-08

**All-in budget ceiling: €250 delivered to Portugal, including taxes, delivery, hardware, printed structure, battery, charger (if required), PCB fabrication, wiring, fasteners and a modest spare/repair allowance.** This replaces the earlier exploratory €150 suggestion. Budget does **not** authorize placing an order.

Shortlist for sourcing and physics evaluation, **not yet validated as delivered ≤€250**:

1. [Skythinker616 FOC wheel-legged robot](https://github.com/Skythinker616/foc-wheel-legged-robot/blob/master/README_en.md) — source author reports ¥549 historical component cost **inside China**, including 4×4010 leg motors, 2×2804 wheel motors, 6× driver electronics, controller, battery, printed/acrylic parts and bearings. No current Portugal checkout quote, unknown assembly effort/PCB manufacturing cost, **no documented successful full-body jump**. Favor for open hardware and six BLDC actuators, *conditional* on physical jump feasibility.
2. [KudoC0nan Balance_On_Wheels](https://github.com/KudoC0nan/Balance_On_Wheels) — open CAD, ESP32, N20 geared-wheel motors and MG90 leg servos, video `Jumping_Bot.mp4`. Authors report servo gear damage during jump tests. An initial cheap build is plausible, **durable repeatable jumps unproven**; cannot call €70–130 a current validated Portuguese delivered price.
3. [ToolKnox Wheeliebot](https://github.com/ToolKnox/Wheeliebot-w-Code) — creator claims real jumping with two leg STS3032 bus servos and gimbal wheel motors; **controller PCB v2 still pending** per author warning. BOM+part prices and manufacturing before deciding; not automatically build-ready.

### Purchasing gate

- [ ] Itemized BOM with exact vendor SKU, quantity, voltage/current, torque/speed, VAT applicability, shipping to Portuguese postcode and availability; links to **current checkout prices**, not historical China BOM.
- [ ] Whole project total ≤€250, including identified hidden costs (PCB fab/assembly, LiPo-safe charger, connectors, printed part quotation, replacement gears/servo/motor).
- [ ] Independent real-model MuJoCo probe of full-body lift-off + landed recovery, explicitly separating demonstrated physics from extrapolated actuator specs.
- [ ] Make explicit whether **jumping is mandatory hardware capability** or a research target needing redesign; if proof unavailable, document risk, do not claim a purchase candidate can jump.
- [ ] Owner approval before spending. No auto-order, no automatic source/firmware updates.
