"""No-third-party test of XGO Lite3 V2 material-only viewer overlay."""
from pathlib import Path
import hashlib,json,re,unittest

ROOT=Path(__file__).resolve().parent
if not (ROOT/'docs').is_dir():ROOT=ROOT.parent
PAGE=ROOT/'docs'/'hexapod'/'lite3-finish-v2'

class VisualFinishTests(unittest.TestCase):
    def test_profile_and_provenance(self):
        p=json.loads((PAGE/'visual-finish.json').read_text())
        self.assertEqual(p['schema'],'xgo-lite3-visual-finish-v2')
        self.assertEqual(p['reference']['official_step_md5'],'f413d18e4546e7d7255ce1a4976ba887')
        self.assertEqual(p['reference']['step_rgb_unique'],1)
        self.assertEqual(p['reference']['step_rgb_colored_solids'],259)
        self.assertIn('reconstruction',p['reference']['confidence'])
    def test_existing_mesh_and_trajectories_reused(self):
        s=(PAGE/'app.js').read_text()
        self.assertIn("const ORIGINAL_MANIFEST='/hexapod/lite3-original-v1/asset-manifest.json'",s)
        self.assertIn('const VENDOR_COMMIT=\'447945fd40fb5585271dce4ea8cc82567916032a\'',s)
        for key in ('standard-onnx','standard-zero','midstride-onnx','midstride-zero'):
            self.assertIn('lite3-'+key+'-seed20261106-original-v1.json',s)
        self.assertIn('for(let i=0;i<17;i++)',s)
        self.assertIn('function validateReplay(obj)',s)
    def test_materials_only_and_rigid_attachment(self):
        s=(PAGE/'app.js').read_text()
        self.assertIn("geometry.setAttribute('color'",s)
        self.assertIn('new THREE.CanvasTexture(canvas)',s)
        self.assertIn('decorationGroup.position.copy(modelMeshes[0].mesh.position)',s)
        self.assertIn('decorationGroup.quaternion.copy(modelMeshes[0].mesh.quaternion)',s)
        self.assertNotIn('new THREE.TextureLoader',s)
    def test_internal_library_resources_only(self):
        doc=(PAGE/'index.html').read_text()
        self.assertIn('/hexapod/lite3-finish-v2/app.js',doc)
        self.assertNotIn('XGO-lite3_Assembly_v2026.09.18.step',doc)
    def test_sealed_entry_bytes(self):
        entries=json.loads((ROOT/'new_lite3_finish_v2_entries.json').read_text())
        self.assertEqual(len(entries),1)
        entry=entries[0]
        self.assertEqual(entry['id'],'lite3-photo-finish-v2')
        for x in entry['files']:
            data=(ROOT/'docs'/x['path']).read_bytes()
            self.assertEqual(x['size'],len(data))
            self.assertEqual(x['sha256'],hashlib.sha256(data).hexdigest())

if __name__=='__main__':unittest.main(verbosity=2)
