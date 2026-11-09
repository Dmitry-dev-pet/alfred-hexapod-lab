"""FreeCAD headless smoke test on synthetic CAD, not the original BB-8."""
import hashlib
import json
import os
from pathlib import Path

import FreeCAD as App
import Part
import Mesh

output = Path(os.environ.get('BB8_FREECAD_SMOKE_OUT', '/tmp/alfred-bb8-freecad-smoke'))
output.mkdir(parents=True, exist_ok=True)
doc = App.newDocument('Alfred_BB8_Smoke')
try:
    body = doc.addObject('PartDesign::Feature', 'SyntheticCylinder')
    body.Shape = Part.makeCylinder(10, 30)
    doc.recompute()
    step = output / 'synthetic.step'
    stl = output / 'synthetic.stl'
    Part.export([body], str(step))
    Mesh.export([body], str(stl))
    assert step.is_file() and stl.is_file(), 'Missing STEP/STL export'
    reimport = Part.read(str(step))
    assert reimport.isValid() and reimport.Volume > 0
    assert abs(reimport.Volume - body.Shape.Volume) < 1e-6
    report = {
        'schema': 'alfred-bb8-freecad-smoke/v1',
        'freecad_version': '.'.join(App.Version()[:3]),
        'headless_ok': True,
        'step_reimport_ok': True,
        'synthetic_only': True,
        'bb8_source_cad_imported': False,
        'step_sha256': hashlib.sha256(step.read_bytes()).hexdigest(),
        'stl_sha256': hashlib.sha256(stl.read_bytes()).hexdigest(),
        'body_volume_native_units3': round(reimport.Volume, 4),
    }
    (output / 'receipt.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print('PROBE_RESULT', json.dumps(report), flush=True)
finally:
    App.closeDocument(doc.Name)
