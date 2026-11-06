# Codex handoff: install existing PR, do not develop it

**Assignment:** Install draft PR #72 on the owner's Mac mini M4 using a clean isolated checkout, then invoke its installer. **Codex must not implement or edit any code, create another PR, add a GitHub runner, modify main, or alter the S2 physics benchmark.** The installer runs the benchmark itself. No GitHub Actions physics.

## One local installation procedure

Use an isolated checkout; do not overwrite the owner's existing repositories:

~~~bash
mkdir -p "$HOME/alfred-robotics" &&
git clone --branch research/lite3-performance-benchmark-20261106 --single-branch \
  https://github.com/Dmitry-dev-pet/alfred-hexapod-lab.git \
  "$HOME/alfred-robotics/hexapod-lab-bench" &&
cd "$HOME/alfred-robotics/hexapod-lab-bench" &&
bash scripts/research/install_lite3_mac.sh
~~~

If the isolated checkout already exists and is clean, verify its origin and branch and use the same installer; never clone over local changes. This exact research branch is the head of draft PR #72, stacked on research PR #69.

The installer creates a persistent isolated environment in ~/alfred-robotics/lite3-benchmark/ with Python 3.13 and pinned native macOS ARM64 wheels. If Python 3.13 is missing and Homebrew is available, the shell wrapper installs that Python version without sudo. On other dependency failures, stop with the exact log; do not change the benchmark to make it pass.

### Private model source, resolved by installer

If valid private assets are already installed, reuse them. Otherwise the installer tries the existing private artifact from successful GitHub runner run 42243825347 via the currently authenticated local gh CLI. **This downloads existing data and does not dispatch an Actions job.** This artifact is byte-identical to the archived owner ZIP:

[Private original model and ONNX policy in Google Drive](https://drive.google.com/file/d/1uzAHREDGxxp2ehs8jEeJp6PcujCFvueQ/view?usp=drivesdk).

If gh is unauthenticated or the artifact expires, **stop and report the blocker**. Do not ask for credentials in GitHub or download unpinned replacement files. The owner can transfer the Drive ZIP to ~/Downloads/XGO-Lite3-original-model-and-PPO-policy-2026-11-06.zip and rerun the same installer. To use a different owner-local path, set LITE3_BUNDLE_ZIP or pass --bundle /path/to/archive.zip.

Bundle verification: exact pinned archive SHA256, internal SHA256 manifest, original scene XML, ONNX policy, and 17 original STL meshes. All third-party binaries stay outside GitHub.

### Outputs (local files, no automatic uploads)

~~~text
~/alfred-robotics/lite3-benchmark/results/mac_environment.json
~/alfred-robotics/lite3-benchmark/results/mac_m4_s2_perf.json
~/alfred-robotics/lite3-benchmark/results/mac_m4_s2_perf.log
~/alfred-robotics/lite3-benchmark/results/comparison_receipt.json
~~~

Successful benchmark: 60 × 9.5s, 142500 physical steps, unchanged S2 perturbations, same ONNX/scene SHA256. Linux ChatGPT reference: 30.961651129s. Comparison is reference_seconds / measured_mac_wall_seconds. Numerical differences in physics results across architectures must be reported separately.

Optional --install-only installs environment and verifies original model + policy load without running the 60-episode benchmark. Default installer performs both installation and benchmark in one invocation. Repeated runs reuse environment and assets.

## Codex acceptance boundary

- Only check out approved PR and run its existing installer. Return the actual command, exit code, relevant output, and result paths. Do **not** write code, amend the PR, or make an extra branch.
- Never merge main, change the Android viewer, access robot hardware, or flash firmware.
- Never publish private vendor assets, GitHub tokens, or Drive credentials.
- If unable to install, report the reason. Mark Mac benchmark NOT_RUN until a real receipt exists.
