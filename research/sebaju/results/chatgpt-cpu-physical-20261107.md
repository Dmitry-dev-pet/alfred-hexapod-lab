# SeBaJu: independently measured local MuJoCo jump and side-tip audit (2026-11-07)

**Execution location:** ChatGPT's local Linux x86_64 CPU sandbox only. **No Mac / GitHub Actions / Colab / Kaggle compute.** No real robot commands. No changes to `main`.

## Provenance

- Original public [SeBaJu](https://github.com/k3nkreate/SeBaJu_BOT_5845) pinned at `7bbc38f4b26b8ecce7ac0ee37e06164e851dc600`.
- Original model `SeBaJu_BOT(II).xml`: git blob `c0abe8d5f9663813e2782fbd44fb38c40d7ea048`.
- Original controller `SeBaJu_Ctrl_BalJumpAchieve.py`: git blob `42397bb5e613a12ecbe6074030bb917f3d03533a`.
- Independently SHA-verified both exact files before offline import; original content is **not vendored** into this repository nor the evidence ZIP (upstream currently has no explicit license).
- MuJoCo **3.14.0**, physics timestep **0.002 s**, local Python **3.13**; offline wheelhouse bundle SHA256 `6d14d0c17294bac5436e1167ca4a97c30061da8efaa0a2b904c171bcac4a7b13`.
- Our adapter and test git blobs: `0858679c867176a35d87b39a03670d2140a8dc41` / `b291ac709ef5ac05cfc31870114687952e3af730`.
- Full [owner's Drive evidence ZIP](https://drive.google.com/file/d/1L7hflDMr7oYUTeUiU9TSL1Es863tUwDJ/view?usp=drivesdk) with original measured traces, source controller logs, receipts, visualization MP4, independent SHA-256 per file, manifest, and runnable adapter/tests; ZIP SHA256: `ec015e829b3b22e382b761d8419f922ae32efed22eb379934c63fbfff026757c`.

## Actual physical simulation, not an invented player animation

13-s unperturbed baseline: **6500 MuJoCo steps**. The original controller reports ~0.200 s flight; our stricter measurement (neither wheel below 5mm clearance and **no ground contact for any robot body**) measures **0.180 s**. Maximum minimum wheel clearance **43.95 mm**. We observed touchdown at ~6.012 s and an upright stable hold by 8.15 s. The nominal ~0.202 s / 45 mm upstream numbers are closely consistent but not identically defined.

In tipping runs, the exact unmodified controller first balances and jumps. Starting at **t=12 s**, an actual *world-frame roll torque* is applied to the torso for **0.15 s**, then removed. Same MuJoCo physics with no qpos teleport:

| Torque (N·m) | Max tilt | Final tilt at 18 s | Tip >=60°? | Self-right after real tip? |
|---:|---:|---:|:---:|:---:|
| 8 | 8.47° | 4.35° | no | inconclusive |
| 10 | 16.24° | 4.23° | no | inconclusive |
| 12 | 175.87° | 111.03° | yes | **no** |
| 14 | 179.95° | 179.95° | yes | **no** |
| 18 | 134.85° | 111.03° | yes | **no** |
| 20 | 180.00° | 179.98° | yes | **no** |
| 30 | 177.16° | 74.67° | yes | **no** |

A separate **30-second** continuous experiment at 20 N·m / 0.15 s ends upside-down at **179.95°** with the torso physically in contact with the floor, not the wheels. No self-right was measured. The observed 10→12 N·m distinction is **not a globally validated threshold** and these results do not establish mechanical impossibility of a new purpose-built recovery controller.

## Bugs found and corrected in our adapter (not upstream)

1. `MjData.xmat[torso]` is a flat array of 9 numbers in this runtime: reshape(3,3) before reading z-axis.
2. Original classifier incorrectly treated wheels off floor as `in_flight` even when a tipped torso/leg rested on the ground. It now checks **all ground contacts** and restricts flight to source jump window, records contact geometry per sampled frame.
3. Added `--offline-snapshot`, requiring exact pinned **git blob SHA** for both original model and controller, without git clone/network.
4. **9/9** Python unit tests PASS. Local full-physics replay also PASS. Removed the Mac-specific GitHub Actions workflow from **this research branch**.

## Reproduction on a compatible CPU environment

```bash
python -m unittest discover -s tests -p test_sebaju_monitor.py -v
python research/sebaju/run_headless.py --offline-snapshot --upstream /path/to/verified-upstream --trial jump --out results/jump
python research/sebaju/run_headless.py --offline-snapshot --upstream /path/to/verified-upstream --trial tip --tip-torque 12 --out results/tip12
```

The physical trace ZIP and MP4 are **derived measurements** of the approximate MuJoCo model, *not validated real-hardware performance*. Real servo torque and friction differ. The Android viewer's existing build should be updated only after review; no browser-player publication was performed by this branch.
