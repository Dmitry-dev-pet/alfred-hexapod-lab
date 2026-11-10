# MicroDuck single-support recordings in the existing player

Publishes all 14 canonical recordings from [study PR #135](https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/135)
through `docs/hexapod/library.json` on the existing player feature branch.
The collection includes the standing control, three perturbed walking controls,
and all ten unsuccessful slow transfer/hold attempts. It does not rerun physics.

The existing shared original-STL package is reused: 38 meshes and 70 visible
instances. Every one of 5,540 saved poses is reconstructed with `mj_forward`.
The maximum float32 export error is below 0.5 micrometres. Fourteen independent
JSON track packages use the existing reversible XOR/byte-shuffle/gzip format,
each below 0.7 MB. The complete entry has 19 allowlisted files.

Walking and failed holds are clearly separated in the selector and outcome
labels. Load, whole-sole clearance and trunk tilt come from the recorded
50 Hz frames; the 0.12/0.14 s walking intervals are sampled observations,
not continuous-contact guarantees or long single-foot standing. Playback
starts at half speed. A `?clip=<id>` link opens a specific recording.

Validation: five focused tests verify all 14 outcomes, all decoded frames,
checksum/truncation rejection, offline reopening and serving all 19 files
through the **unchanged** deployed GitHub worker fixture. JS syntax and
catalog idempotence/preservation checks pass. No Site build, global shell,
service worker, model package, physics, joint range or neural weights change.
Live browser rendering remains subject to the browser's WebGL availability.

Reproduce from this repository plus the immutable source-study checkout and
restored source workspace from the archive referenced in PR #135:

```bash
python research/microduck/single_support_publication_v1/export.py --study-root /absolute/study/research/microduck/single_support_v1 --workspace /absolute/workspace --traces /absolute/canonical-traces
node --test tests/test_microduck_single_support_replays.mjs
```

Trace/source archive: [MicroDuck_single_support_20261109.zip](https://drive.google.com/file/d/1S6ZdRII1jX8_88p0BYOZVUqWZhb9v_B6/view?usp=drivesdk),
SHA-256 `e0f1d3527879007fc5c438935524dc8fdaccd90f90ceb698e0209ac0a6d0ae97`.
Publication run: [Truthrail #334](https://github.com/Dmitry-dev-pet/truthrail/issues/334).
