import hashlib
import json
import os
import tempfile
import unittest
from pathlib import Path

import mujoco

from research.navbot_en01.linkage_model import create
from research.navbot_en01.model_validity import audit_trace, require_physical_claim


class ModelValidityTests(unittest.TestCase):
    def trace(self):
        xml = create()
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        return xml, model, {'summary': {'model_sha256': hashlib.sha256(xml.encode()).hexdigest()},
                            'frames': [{'t': .001, 'qpos': data.qpos.tolist()},
                                       {'t': .021, 'qpos': data.qpos.tolist()}]}

    def test_folded_trace_and_disabled_collision_cannot_be_called_valid(self):
        xml, model, trace = self.trace()
        hip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'hip_left')
        trace['frames'][1]['qpos'][int(model.jnt_qposadr[hip])] = -.256
        result = audit_trace(trace, xml)
        left = next(x for x in result['joints'] if x['name'] == 'hip_left')
        self.assertAlmostEqual(left['max_limit_excursion_rad'], .036)
        self.assertEqual(left['first_violation_s'], .021)
        self.assertEqual(left['violating_saved_frames'], 1)
        self.assertEqual(result['mask_enabled_robot_pairs'], 0)
        with self.assertRaisesRegex(ValueError, 'claim blocked'):
            require_physical_claim(result)
        self.assertEqual(trace['frames'][1]['qpos'][int(model.jnt_qposadr[hip])], -.256)

    def test_in_range_trace_still_has_no_mechanical_certification(self):
        xml, _, trace = self.trace()
        result = audit_trace(trace, xml)
        self.assertFalse(result['physical_jump_claim_allowed'])
        with self.assertRaises(ValueError):
            require_physical_claim(result)

    def test_wrong_model_and_nonfinite_position_are_rejected(self):
        xml, _, trace = self.trace()
        with self.assertRaisesRegex(ValueError, 'hash mismatch'):
            audit_trace(trace, xml+'\n')
        trace['frames'][1]['qpos'][7] = float('nan')
        with self.assertRaisesRegex(ValueError, 'Invalid recorded'):
            audit_trace(trace, xml)

    @unittest.skipUnless(os.environ.get('EN01_JUMP_RAW_TRACE'), 'Frozen raw trace supplied separately')
    def test_export_exposes_invalidity_and_preserves_measured_frames(self):
        from research.navbot_en01.source_jump_export import export
        from research.navbot_en01.feedback_export import export as old_export
        trace = Path(os.environ['EN01_JUMP_RAW_TRACE'])
        with tempfile.TemporaryDirectory() as td:
            first, second = Path(td)/'old.json', Path(td)/'new.json'
            old_export(trace, first)
            export(trace, second)
            before, after = json.loads(first.read_bytes()), json.loads(second.read_bytes())
            self.assertEqual(before['frames'], after['frames'])
            self.assertEqual(before['source'], after['source'])
            self.assertEqual(after['status'], 'failed')
            self.assertFalse(after['model_validity']['physical_jump_claim_allowed'])


if __name__ == '__main__':
    unittest.main()
