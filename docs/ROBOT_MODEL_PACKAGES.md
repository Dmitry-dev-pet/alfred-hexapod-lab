# One geometry package per robot in Alfred Robot Player

The hosted MicroDuck adapter downloads one 10,340,979-byte JSON package,
instead of 38 upstream STL requests totalling 21,593,292 bytes. The package
contains the original gzip-compressed STL bytes, their pinned manifest,
70 articulated visual-instance links and the upstream model licence notice.
All seven six-second physical traces are unchanged and load separately.
EN01 already uses one `original-meshes.json` for its 73 CAD parts.

New hosted STL models should use the shared `model-package.js` loader and
`robot-model-package/v1` envelope, with an immutable content-addressed URL,
SHA-256, byte size, source revision, mesh count, instance count and data licence
in `robot-model-package-ref/v1`. The package is cached only after checking its
outer digest and every decompressed original STL. Requests in one page share
one promise. A fresh page can read the verified browser cache without network.
Oversized responses/expansions and corrupt files fail closed.

## Reproduce the hosted MicroDuck adapter

```sh
python scripts/build_microduck_model_package.py \
  --output /tmp/microduck-model.json --cache /tmp/microduck-stls \
  --descriptor /tmp/microduck-descriptor.json
python scripts/install_microduck_model_package.py \
  --site /path/to/existing-site/dist --package /tmp/microduck-model.json
node --experimental-vm-modules --test tests/test_model_package.mjs
```

The source repo stores the exporter, loader, adapter, tests and small descriptor;
the heavy package is a Site asset and is backed up in the project Drive folder:
https://drive.google.com/file/d/1TpLb-Wm-AETN7xgDurnDTzF2mjsGDt6B/view

The installer preserves the existing Site manifest, media, vendor libraries,
UI and live GitHub catalog. Four exact MicroDuck adapter routes bypass the
catalog bridge and use the reviewed Site bundle. All other replay routes keep
their existing manifest verification. The bundle is checked again by the loader.
This hosting overlay does not change the working replay branch/tablet loader
or claim that all older robots have been migrated. Do not merge the source PR
without arranging the model package on the destination host.

## Attribution and terms

Copyright Pollen Robotics. Original models:
https://github.com/pollen-robotics/microduck_rl/tree/bae39f5900b318f7e87fc485c30a31213cfa14eb/src/mjlab_microduck/robot/microduck/assets

Models and this data package: CC BY-NC-SA 4.0,
https://creativecommons.org/licenses/by-nc-sa/4.0/ .
The package includes the complete upstream `LICENSE-MODELS.md` notice.
The change is gzip packaging and inclusion of the manifest; original STL bytes
and existing MuJoCo-to-Three transforms remain unchanged. The general code
licence in upstream does not replace the separate model licence.

Freenove, Lite3 and Pupper legacy loaders retain their current source/rights
boundaries. This change does not grant redistribution rights to their models.
