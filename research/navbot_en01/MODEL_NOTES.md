# NavBot EN01 — actuator-grounded MuJoCo prototype (P0)

Status: **APPROXIMATE MODEL / CPU PHYSICS EXERCISED, HARDWARE NOT VALIDATED** (2026-11-08).
Owner project: [Alfred Hexapod Lab](../../README.md), independent of the SeBaJu W0 baseline in [#93](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/93).
Do not merge into main without owner review. Do not publish a replay until it comes from a measured MuJoCo trajectory.

## Source provenance

- EN01 upstream fixed at `fuwei007/Navbot-EN01@3139f2fb32b38d10611feb905e16f8aa012f2f2e`:
  [BOM.xlsx](https://github.com/fuwei007/Navbot-EN01/blob/3139f2fb32b38d10611feb905e16f8aa012f2f2e/Hardware/RebotModel/BOM.xlsx),
  [firmware](https://github.com/fuwei007/Navbot-EN01/blob/3139f2fb32b38d10611feb905e16f8aa012f2f2e/Src/wl_pro_robot/wl_pro_robot.ino),
  [original STEP geometry](https://github.com/fuwei007/Navbot-EN01/blob/3139f2fb32b38d10611feb905e16f8aa012f2f2e/Hardware/RebotModel/OriginalRobotModel.stp).
- Manufacturer [Feetech STS3032 6 V datasheet](https://pages.switch-science.com/comparison/files/feetech/serial-sts/STS3032_datasheet.pdf), dated 2020-06-08.
- [SimpleFOC BLDC reference](https://docs.simplefoc.com/bldc_motors) describes 2208 90 KV gimbal motor, 39 g, 13–16 ohm phase resistance.
- [Frank Fu EN01 product page](https://frankfu.blog/product/navbot-en01/) lists the 322 g overall mass and 12 x 9.5 x 9 cm envelope.
- [Frank Fu assembly guide](https://frankfu.blog/embodied-ai-robot/building-a-tabletop-wheel-legged-robot-from-scratch/) calls the wheel motor TMCM-6210, not 2208 90 KV. **The actual assembled SKU is not verified (UNVERIFIED hardware configuration).**
- Upstream has no project-wide explicit license. This folder contains an original analytical approximation; third-party CAD, firmware, meshes and binaries are **not copied**.

## Four actuated axes

Two direct-driven wheels + two independent hip/leg-axis serial servos. **This is not a two-joint-per-leg SeBaJu model.**

| Element | Provenanced spec | Model value / status |
| --- | --- | --- |
| Complete robot | total mass 0.322 kg | 0.322 kg (exact sum of assumed parts) |
| Wheels | BOM: 2x 2208 90KV BLDC; SimpleFOC: each 39 g, 13–16 ohm | wheel+hub mass 0.047 kg per side; radius 0.024 m **assumed** |
| Wheel motors | firmware SimpleFOC voltage/torque mode, driver supply configured 8 V | motor torque limit +/-0.030 Nm **unverified probe parameter**, not a manufacturer rating |
| Leg servo | STS3032 6 V: rated 1.5 kgf cm = 0.1471 Nm | dynamic baseline continuous torque <=0.1471 Nm |
| Leg servo stall | 4.5 kgf cm = 0.4413 Nm at locked rotor | experimental short peak ceiling <=0.4413 Nm, **NOT rated torque**, no assertion of safe duration |
| Leg servo speed | 0.09 s / 60 deg no load = 11.64 rad/s | approximate speed-dependent torque envelope; NOT a measured motor curve |
| Electronics | ESP32, L6234 wheel driver, MPU6050, two AS5600 | IMU and joint sensors in simulation, not validated sensor noise/latency |

**Power caveat:** the firmware sets `driver1.voltage_power_supply=8` and `driver2.voltage_power_supply=8` for BLDC wheels. The STS3032 datasheet specifies 4–7.4 V absolute input range and 6 V test conditions. The build guide calls for a 7.4 V LiPo (full 2S can exceed 7.4 V). The servo power rail/regulation is **not verified**; never infer safe direct wiring or high-voltage servo torque.

**Geometry/inertia caveat:** The XML geometry (torso dimensions and COM, 50 mm leg reach, 24 mm wheel radius, 43 mm side offsets) is a *placeholder*, not reconstructed from STEP or per-part mass measurements. Distribution 0.180 + 2x0.024 + 2x0.047 = 0.322 kg. A numerical success with this XML would **not** prove the purchased EN01 can perform it.

## Source jump command (not a measured jump)

The real EN01 firmware does contain `jump_loop()` at lines ~504–534.
It first sends mirrored positions `2048 +/- (12 + 8.4*(80-32))` = approx `2048 +/- 415.2` ticks, then `2048 +/- (12 + 8.4*(40-32))` = `2048 +/- 79.2` ticks.
At 4096 counts/revolution these are **0.637 rad** and **0.121 rad** from 2048; right-side servo direction is mechanically mirrored.
The source advances phases by *loop iterations* (30+, 200+), not clocked milliseconds. Its original physical timing cannot be inferred from that code alone.
Our `probe.py` reproduces the two commanded angular setpoints with **assumed timings**, tuned PD gains, motor limits and a wheel stabilizer; it is not the original controller. Slow preparation exists as a diagnostic comparator, not a claim of upstream behaviour.

## Reproducible gate

Pure stdlib static tests:
```sh
python -m unittest discover -s tests -p 'test_navbot_en01_model.py' -v
```

With an **existing authorised MuJoCo 3.x runtime**:
```sh
python research/navbot_en01/probe.py --mode stand --duration 6 --out .cache/navbot/stand.json
python research/navbot_en01/probe.py --mode crouch --duration 6 --out .cache/navbot/crouch.json
python research/navbot_en01/probe.py --mode jump --duration 6 --out .cache/navbot/jump.json
python research/navbot_en01/probe.py --mode jump_fast --duration 6 --out .cache/navbot/jump_fast.json
```

Do not run physics in GitHub Actions. No new Mac runner/GPU is needed. **Physics has now been run** in the restored offline Linux CPU MuJoCo 3.14.0. Results and failures are recorded in [CPU physical probe receipt](results/cpu_mu3140_20261108.md).

Probe outputs full `qpos`, `qvel`, actuator commands, rigid-body pitch, wheel clearance, whole-body ground contacts, servo peak flag, and source/assumption markers. Flight is reported only for whole-body no-floor-contact intervals >=20 ms after the command, followed by contact and a separate >=2 s landing-stability assessment. Any result without contact/velocity and source proof is incomplete.

**Next:** confirm *as-sold* wheel motor SKU and servo voltage rail; extract articulated axes / wheel radius / part masses from original STEP; run parameter sweeps across wheel torque, hip torque and friction; compare a real contact-based jump to the source claim. Neither a motor's stall torque nor a visually plausible animation is jump evidence.
