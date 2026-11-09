import copy,json,math,os,tempfile,unittest
from pathlib import Path
import mujoco
import numpy as np
from research.navbot_en01.getup_model import create
from research.navbot_en01.getup_probe import stable_gate
from research.navbot_en01.getup_export import validate_outcome
from research.navbot_en01.linkage_export import build
from research.navbot_en01.contact_model import create as v5

ROOT=Path(__file__).resolve().parents[1]
MESH=Path(os.environ.get('EN01_CAD_MESH','docs/hexapod/en01-original-cad-v1/original-meshes.json'))
DATA=ROOT/'results/en01_getup_v7/final'


class GetupTests(unittest.TestCase):
    def test_complete_original_floor_envelopes_preserve_mechanics(self):
        before=mujoco.MjModel.from_xml_string(v5(MESH));after=mujoco.MjModel.from_xml_string(create(MESH))
        for name in ('body_mass','body_inertia','body_ipos','jnt_range','actuator_ctrlrange'):
            np.testing.assert_array_equal(getattr(before,name),getattr(after,name))
        for i in range(73):
            g=mujoco.mj_name2id(after,mujoco.mjtObj.mjOBJ_GEOM,'floorcad_'+str(i))
            self.assertGreaterEqual(g,0)
            self.assertEqual(after.geom_conaffinity[g],1)
            self.assertEqual(after.geom_priority[g],1)
        for g in range(after.ngeom):
            name=mujoco.mj_id2name(after,mujoco.mjtObj.mjOBJ_GEOM,g)
            if name and name.endswith('_geom'):
                self.assertEqual(after.geom_contype[g],0)
                self.assertEqual(after.geom_conaffinity[g],0)

    def test_gate_rejects_airborne_single_wheel_inverted_and_sliding(self):
        a=json.loads((ROOT/'research/navbot_en01/getup_protocol.json').read_text())['acceptance']
        self.assertTrue(stable_gate(0,[1.5,1.5],0,0,0,a))
        for args in [(0,[0,0],0,0,0),(0,[0,3],0,0,0),(math.pi,[1.5,1.5],0,0,0),
                     (0,[1.5,1.5],0,.1,0),(0,[1.5,1.5],1,0,0),(0,[1.5,1.5],0,0,1)]:
            self.assertFalse(stable_gate(*args,a))

    def test_false_success_rejected_from_measured_tail(self):
        record=json.loads((DATA/'raw/back-extend-backward_raw.json').read_bytes())
        self.assertEqual(validate_outcome(record)['saved_final_stable_frames'],0)
        record['summary']['getup_pass']=True
        record['summary']['checks']={k:True for k in record['summary']['checks']}
        with self.assertRaisesRegex(ValueError,'contradicts'):validate_outcome(record)

    def test_incomplete_trace_rejected(self):
        record=json.loads((DATA/'raw/back-idle_raw.json').read_bytes());record['frames'].pop()
        with self.assertRaisesRegex(ValueError,'Incomplete'):validate_outcome(record)

    def test_forged_visual_root_pose_rejected(self):
        record=json.loads((DATA/'raw/back-idle_raw.json').read_bytes())
        record['frames'][0]['body_poses'][0][2]+=.1
        with tempfile.TemporaryDirectory() as td:
            src=Path(td)/'raw.json';src.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError,'disagree'):build(src,Path(td)/'replay.json',model_xml=(DATA/'model.xml').read_text())


if __name__=='__main__':unittest.main()
