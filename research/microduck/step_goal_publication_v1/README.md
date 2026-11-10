# MicroDuck step-goal recordings

All 37 previously verified study trajectories are available through the existing live GitHub library. No new Site version or deployment is required. The published Site bridge is unchanged.

The collection uses five existing-style replay client files plus 15 SHA-256-bound JSON track packages. All 16,752 original-STL frames and 70 mesh poses per frame are preserved. Identical tracks are shared; byte shuffle, temporal XOR and gzip are exactly reversible. Every package is below the bridge file-size bound and the collection has 20 allowlisted files.

The source study and limits remain in PR #128. The selected default is the nominal accepted 1 cm climb; rejected higher steps, joint perturbations and command margins remain visible. Goal achievement and the quality gate are separate in the UI.

Validation: five focused client/actual unchanged-worker checks pass, including all frames, integrity rejection and offline reopening. Six existing worker regressions also pass. 487,980 interpolated torso/step distance samples have maximum overlap 0.532 mm. Browser selection and seeking were checked; cloud WebGL is disabled, so rendered 3D on Android is not claimed verified.

Native Site publication was attempted in error, failed its expanded archive limit and was stopped. Unpublished task-specific source additions were removed while preserving concurrent changes. This release changes the GitHub player library only.
