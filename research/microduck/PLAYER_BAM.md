# MicroDuck BAM comparison replays

The existing Alfred Robot Player now serves all 63 saved CPU benchmark episodes at `/hexapod/microduck-bam-v2/`: three actuator variants, seven cases, three initial-condition seeds. The previous seven recordings remain separate.

Each clip has 300 original 50 Hz frames and 70 original visual mesh instances. `player_export_bam.py` applies MuJoCo forward kinematics to every saved qpos/qvel/ctrl state. The little-endian Float32 format retains every frame (MDPOSE2 header, 300 rows, stride 498); maximum absolute numeric conversion error is 2.29e-7. Each track has an immutable SHA-256 URL and is checked before playback. Viewed clips are cached for offline reuse.

The 38-mesh robot package is reused from the existing Site, preserving original STL bytes and CC BY-NC-SA 4.0 attribution. Pose binaries are hosted with the existing Site and backed up separately, rather than committed into the public source repository. The four small route files are registered in the live GitHub catalog. Archive: https://drive.google.com/file/d/1fvS0B7C7HtFmXvqzCVyWd8DF7_t271mX/view?usp=drivesdk ; SHA-256 648c5eec148731f14076057901c2eb95be68a90669fde176b7e993b61bbed576 . Original physical traces/provenance remain in PR #108.

Reproduce with Python 3.12, the pinned upstream RL and BAM sources from the benchmark, the existing immutable model package under `<site-dist>/assets/robot-models`, and the SHA-verified original trace archive:

```bash
python research/microduck/player_export_bam.py --rl-root /path/to/microduck_rl --traces /path/to/battery-final --site-dist /path/to/site-dist
MICRODUCK_SITE_DIST=/path/to/site-dist node --test hosting/test_microduck_bam_replays.mjs
```

Validation: all 63 files decoded and all 18,900 frames checked, including unit quaternions for 70 parts; corrupt/truncated/header-tampered records rejected; viewed-clip offline cache verified. Four focused replay tests plus six existing service-worker tests pass. Browser preview verifies three selectors, actual metric changes, pause and seek to 6.00s. Cloud Chrome disables WebGL, so image rendering and physical Android touch remain unverified for this new route. The page retains a truthful WebGL fallback while its data/controls continue to work.

PASS means the existing final-one-second stability gate, not bounded drift or safe hardware loads. The BAM/backlash push case reaches 53.3 cm drift; displacement/contact peak are visible in the info panel. No physics was rerun or controller retuned for these clips.

Publication authorized by the owner request `добавь ролики` on 2026-11-09. Run: https://github.com/Dmitry-dev-pet/truthrail/issues/310 . No main/EN01 change or new Site.
