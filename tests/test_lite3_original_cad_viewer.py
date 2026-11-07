"""Verification of exact original STL ↔ MuJoCo body coordinate replay and catalog."""
import hashlib,importlib.util,json,struct,unittest
from pathlib import Path
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree

SCRIPT=Path(__file__).with_name('export_lite3_original_cad_viewer.py')
spec=importlib.util.spec_from_file_location('export_lite3_original_cad_viewer',SCRIPT)
exp=importlib.util.module_from_spec(spec)
spec.loader.exec_module(exp)

class OriginalSTLViewerTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.model=mujoco.MjModel.from_xml_path(str(exp.MODEL))
  cls.manifest=json.loads((exp.ROOT/'docs/hexapod/lite3-original-v1/asset-manifest.json').read_text())

 def test_original_stl_matches_mujoco_compiled_vertices(self):
  # Raw STL vertices must be in body-local coordinates to be attached directly
  # under the sampled mj_forward body world transform; compare vertex positions.
  m=self.model
  for name in ['body','FL_hip','RR_thigh','R_claw']:
   mid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_MESH,name)
   path=exp.MESHES/(name+'.STL')
   with path.open('rb') as stream:
    stream.seek(80);face_count=struct.unpack('<I',stream.read(4))[0]
    dt=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('unused','<u2')])
    raw=np.fromfile(stream,dtype=dt,count=face_count)['v'].reshape(-1,3)
   tree=cKDTree(raw)
   compiled=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
   w,x,y,z=m.mesh_quat[mid]
   rotated=Rotation.from_quat([x,y,z,w]).apply(compiled)+m.mesh_pos[mid]
   nearest=tree.query(rotated[::max(1,len(rotated)//1000)],k=1)[0]
   self.assertLess(float(np.max(nearest)),5e-7,name)

 def test_geometry_manifest(self):
  m=self.manifest
  self.assertEqual(len(m['meshes']),17)
  self.assertEqual({x['body'] for x in m['meshes']},set(exp.BODY_NAMES))
  self.assertEqual(m['commit'],exp.SRC_COMMIT)
  self.assertEqual(sum(x['size'] for x in m['meshes']),19917828)
  for x in m['meshes']:
   path=exp.MESHES/x['mesh']
   self.assertEqual(path.stat().st_size,x['size'])
   self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),x['sha256'])

 def test_physical_json_frames(self):
  for protocol,mode,seconds in exp.CASES:
   path=exp.ROOT/'docs/hexapod/data'/f'lite3-{protocol}-{mode}-seed20261106-original-v1.json'
   obj=json.loads(path.read_text())
   self.assertEqual(obj['format'],'xgo-lite3-original-stl-mujoco-replay-v1')
   self.assertEqual(obj['layout']['stride'],162)
   self.assertEqual(obj['layout']['body_names'],list(exp.BODY_NAMES))
   self.assertAlmostEqual(obj['frames'][-1][0],seconds,places=5)
   self.assertTrue(all(len(x)==162 and all(np.isfinite(x)) for x in obj['frames']))
   self.assertTrue(all(obj['frames'][i][0]<obj['frames'][i+1][0] for i in range(len(obj['frames'])-1)))
   self.assertEqual(obj['source']['original_trace_sha256'],exp.digest(exp.SOURCE/('onnx_corrected' if protocol=='standard' else 'midstride_corrected')/f'{mode}_20261106.json'))

 def test_first_mesh_body_pose_matches_original_mujoco_fk(self):
  m=self.model
  d=mujoco.MjData(m)
  for protocol,mode,seconds in exp.CASES:
   src=json.loads((exp.ROOT/'docs/hexapod/data'/f'lite3-{protocol}-{mode}-seed20261106-original-v1.json').read_text())
   for f in [src['frames'][0],src['frames'][len(src['frames'])//2],src['frames'][-1]]:
    d.qpos[:]=f[1:22]
    mujoco.mj_forward(m,d)
    for i,name in enumerate(exp.BODY_NAMES):
     off=31+i*7
     bodyid=m.body(name).id
     np.testing.assert_allclose(f[off:off+3],d.xpos[bodyid],rtol=0,atol=6e-7)
     np.testing.assert_allclose(f[off+3:off+7],d.xquat[bodyid],rtol=0,atol=6e-7)

if __name__=='__main__':unittest.main()
