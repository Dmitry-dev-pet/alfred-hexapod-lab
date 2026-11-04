# External reference baseline: Toyota Research Institute IDTO / Mini Cheetah

**Status, verified 2026-11-04:** Drake 1.30.0 native C++ IDTO compiled and original Mini Cheetah **10-second closed-loop MPC actually executed**. Separate instrumentation measured body state and foot contact forces for baseline and two known inertial COM shifts. Full evidence, limitations and SHA-256 are in [REPORT_MPC_2026-11-04_RU.md](REPORT_MPC_2026-11-04_RU.md). Tracked by [#31](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/issues/31). **This is not a CAD30 four-support gait.**

## What is being reproduced

Upstream: https://github.com/ToyotaResearchInstitute/idto ; paper: https://arxiv.org/abs/2309.01813

Pinned upstream commit: `de0629c7811aa9b330e56c4385629005b09495f0`  
Original binary: `./build/examples/mini_cheetah/mini_cheetah`  
Original YAML: `examples/mini_cheetah/mini_cheetah.yaml`  
Original C++ example: `examples/mini_cheetah/mini_cheetah.cc`

Checked at the pinned commit: `mpc=true`, `time_step=0.05 s`, `num_steps=20` (1 s optimization horizon), `controller_frequency=60 Hz`, `sim_time=10 s`, `sim_time_step=1 ms`. The desired end-of-horizon X offset is `0.4 m`, **not** a measured success distance. The simulation uses **Drake**, not MuJoCo. The published Dockerfile is based on Ubuntu Jammy and Drake 1.30.0. The intended native target is Linux x86_64; an Apple silicon host may attempt Linux/amd64 emulation, which is not yet verified.

## Preferred: native build, no Docker

The upstream [Local Installation](https://github.com/ToyotaResearchInstitute/idto#local-installation) explicitly supports CMake builds against a **native Drake installation**. Its tested recommendation is **Ubuntu 22.04 + Drake 1.30.0**; use of other OS / Python / Drake combinations must be treated as an experiment, not as verified compatibility.

Install the native dependencies following the official Drake and upstream IDTO instructions. For example, with Drake extracted to `$HOME/drake`, CMake, a C++20 compiler, Git, and gflags available, run from the **Alfred Hexapod Lab** repository root:

```bash
export DRAKE_ROOT="$HOME/drake"
bash experiments/idto/run_native.sh preflight
bash experiments/idto/run_native.sh prepare
bash experiments/idto/run_native.sh configure
bash experiments/idto/run_native.sh build
bash experiments/idto/run_native.sh run
```

The script does **not** use Docker, does not install system packages or Drake automatically, and does not modify upstream physics or YAML. It checks for the Drake CMake package, pins the IDTO source commit, builds the `mini_cheetah` C++ executable, and writes a separate native run log and metadata. Set `IDTO_LAB_WORKDIR` to change the local workspace and `IDTO_BUILD_JOBS` to control build parallelism.

**Execution update (2026-11-04):** the assistant container is Debian 13 x86_64 and lacks direct internet, but the **download problem was solved** via self-hosted Mac M4 GitHub Actions transport → temporary Actions artifact → persistent Google Drive → local materialization. Drake 1.30.0, Python 3.10 and 17 Ubuntu Jammy dependency packages were extracted to an isolated prefix. CMake configured, GCC 14.2 compiled and linked the original C++ executable, `--test` (MPC disabled) passed, and the original **10-second MPC** exited 0. State and actual contact results were then measured in separately instrumented runs; see [full report](REPORT_MPC_2026-11-04_RU.md) and [Drive evidence](https://drive.google.com/file/d/1W8ODUcZCnoywwzPRVmAAV1QbFowj4vhH/view). This proves this specific environment worked, not generic Ubuntu-binary compatibility with every Debian system.

## Alternative: Docker-based build on a machine with Git, Docker and internet access

From this repository's root:

```bash
bash experiments/idto/run_reference.sh preflight
bash experiments/idto/run_reference.sh prepare
bash experiments/idto/run_reference.sh build
bash experiments/idto/run_reference.sh run
```

This wrapper clones the **pinned** upstream repository into `$HOME/.cache/alfred-hexapod-idto/idto`, uses the official upstream Dockerfile, then runs the unmodified quadruped example. To choose another directory, set `IDTO_LAB_WORKDIR=/path/to/workspace`. It prints the output location.

The upstream example serves Meshcat on `http://localhost:7000` while active (Docker port forwarded only to loopback on the Docker host). The script writes the upstream revision, a SHA-256 of the checked-out YAML, host and Docker metadata, stdout and an execution status into a separate run directory. A zero exit status is labeled **`exit_0_unverified`**, not `reproduced`: the dynamics, foot contacts, achieved displacement and solver success still require independent validation. If building or running fails, retain the logs and mark the attempt failed. This wrapper does not change the official YAML, URDF or C++ example.

On Apple silicon, Docker's linux/amd64 emulation may be slow or unsupported for this numerical stack. Prefer a genuine x86_64 Linux host for baseline measurements. **Termux/Android is a viewer, not the compute host for Drake.**

## Acceptance evidence before claiming `reproduced`

- Confirm the recorded git SHA and the build environment (native toolchain/Drake version or Docker image ID); save build log and runtime log.
- Verify from actual simulator output / trajectories that the **closed-loop dynamic** 10-second run occurred, not just an initial planned trajectory or Meshcat animation. Extract and report achieved base displacement, support transitions and contact forces when reliable telemetry is available. If telemetry is not available, state `metric_unavailable`; don't infer it from video alone.
- Record optimizer failures, solver timings, contacts, unexpected falls and every negative trial. Keep exact initial conditions; do not tune physical parameters in baseline.
- For a **separate** controlled experiment, alter mass/COM/inertia in a mechanically consistent way and compare X/Y perturbations to the untouched baseline; save the full model/config diff. Avoid merely translating the initial body pose and calling it shifted COM.
- Transfer only verified concepts to CAD30, using existing contact-planner → MPC → WBC → independent MuJoCo test. The Freenove model remains six-legged with raised front pair; its joint ranges, mass distribution, actuator limits and contacts are **not** Mini Cheetah's.

## Storage / project rules

Small source, YAML/JSON receipts and summaries belong in GitHub. Large simulation traces, videos and CAD archives belong in Google Drive with checksums. Do not commit third-party Drake/IDTO binaries or Freenove CAD. Do not run physics on GitHub Actions, do not merge into `main` without review, and never claim working CAD30 gait or no slipping based only on an IDTO Mini Cheetah demonstration; keep source and measurement provenance separated.
