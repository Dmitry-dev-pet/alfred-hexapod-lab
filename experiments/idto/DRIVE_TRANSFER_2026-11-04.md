# Drake 1.30.0 binary — verified Mac → Actions → Drive → container transfer

**Date:** 2026-11-04 UTC. **Transport:** verified successful. **IDTO physics:** not run.

## Source and execution provenance

- Original binary: https://github.com/RobotLocomotion/drake/releases/download/v1.30.0/drake-1.30.0-jammy.tar.gz
- Intended binary target: Ubuntu 22.04 (Jammy), x86_64; **not** macOS ARM64.
- GitHub Mac transfer: https://github.com/Dmytry-lab/runner-smoke/actions/runs/41913299133
- Workflow path on source branch: `.github/workflows/drake-130-bundle-m4.yml`; branch `task/drake-130-jammy-m4-20261104`; workflow commit `23fd135a32f5f9f4caffd2d50df0ac25725e64a8`.
- Actual runner reported: `mac-m4-org-02`, non-admin `ci-runner`; transfer job succeeded. No Drake execution or physical simulation on Mac.
- GitHub Actions artifact ID: `13362669245`; name `drake-1.30.0-jammy-linux-x86_64-41913299133`. GitHub artifact is temporary (3-day retention), **not durable source**.
- Durable binary folder in Google Drive: https://drive.google.com/drive/folders/1GSmx1yTPAhid_UbkvHwrZuxYx7HYNGUz; file `drake-1.30.0-jammy.tar.gz`, receipt and SHA-256 separate.
- Binary byte size: **71,550,973**.
- SHA-256: `76d74c13f46397161f305e95f76d1c433584d394608ff666cb549ceb87e1d810` (recorded by Mac, confirmed on the unzipped GitHub artifact, and re-verified after independent materialization from Google Drive).
- Confirmed: `gzip -t` successful; archive expands to `drake/` and exposes `lib/cmake/drake/drake-config.cmake`; unpacked runtime approx. 241 MB.

## Native experiment setup and current blockers

Local execution container: Debian 13 x86_64, Python 3.13.5, GCC 14.2, CMake 3.31.6. Drake binary unpacked into `/mnt/data/drake-runtime/drake/`.

A standalone CMake `find_package(drake CONFIG REQUIRED)` probe was **actually executed** with `CMAKE_PREFIX_PATH=/mnt/data/drake-runtime/drake`; the configure step failed because **Eigen3 development CMake package** is missing. `ldd libdrake.so` additionally reports `libfmt.so.8`, `libspdlog.so.1`, `libdmumps_seq-5.4.so` missing. Original Drake installation targets Python 3.10; container only has Python 3.13. IDTO adds `gflags` and expects a Drake-compatible Python toolchain. The originally recommended native platform is Ubuntu 22.04, **not** this Debian 13 environment.

These are **resolvable dependency/ABI prerequisites to investigate**, not evidence that Drake is impossible to install and not evidence of a successful IDTO compile. No Docker or GitHub-hosted Actions compute was used; `main` remains unchanged. Next experiment: supply matching dependencies via verified offline bundle / compatible runtime, then build original IDTO at fixed commit and measure the dynamic Mini Cheetah simulation.
