# MuJoCo wheelhouse — VERIFIED 2026-11-03

Target: Linux x86-64, CPython 3.13, MuJoCo 3.14.0.
GitHub Actions successful build: https://github.com/Dmitry-dev-pet/truthrail/actions/runs/41674000645

The GitHub-built wheelhouse was recovered and verified here: all 10 wheels passed SHA-256
checks, including mujoco, numpy, pyopengl, glfw, etils and dependencies.
Fresh venv offline install using `--no-index --find-links` passed.
MuJoCo smoke fall test passed in both GitHub and this container.

The approximate Freenove 18-DoF model loaded and executed:
- stand: 1000 steps, 2 seconds
- tripod: 5000 steps, 10 seconds (not useful walking yet).
- headless EGL frame rendered; cleanup produces non-fatal PyOpenGL EGL warnings.

Google Drive contains the independent wheelhouse tar.gz, its sha256, and project release zip.
Neither these wheels nor the model are validated on actual Freenove hardware.
No persistent runtime process is guaranteed across sessions.