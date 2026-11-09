import json, math, os, unittest
from pathlib import Path
import numpy as np
from research.navbot_en01.basic_commands import command, wheel_mix
from research.navbot_en01.basic_metrics import classify

ROOT = Path(__file__).resolve().parents[1]
ACCEPT = json.loads((ROOT/'research/navbot_en01/basic_protocol.json').read_bytes())['acceptance']
DATA = os.environ.get('EN01_BASIC_DATA')
V5 = os.environ.get('EN01_V5_DATA')


class BasicCommandTests(unittest.TestCase):
    def fixture(self):
        return {'max_abs_tilt_deg':5., 'nonwheel_ground_steps':0, 'final_stop_speed_m_s':.001,
                'final_stop_yaw_rate_rad_s':.01, 'active_contact_slip_rms_m_s':.001,
                'active_slip_supported_s':3., 'hip_min_rad':.05, 'hip_max_rad':.3,
                'two_wheel_support_fraction':1., 'signed_axle_travel_m':.075,
                'yaw_change_deg':55., 'final_yaw_error_deg':2., 'axle_net_displacement_m':.01,
                'squat_height_change_m':.02, 'final_height_error_m':.001}

    def test_wheel_spinning_without_actual_translation_cannot_pass(self):
        m = self.fixture()
        m['signed_axle_travel_m'] = 0.
        self.assertFalse(classify('drive-forward', m, ACCEPT)['basic_pass'])
        m['signed_axle_travel_m'] = -.075
        self.assertFalse(classify('drive-forward', m, ACCEPT)['basic_pass'])
        self.assertTrue(classify('drive-backward', m, ACCEPT)['basic_pass'])

    def test_wrong_direction_drifting_turn_and_missing_contact_are_rejected(self):
        for edit in ({'yaw_change_deg':-55.}, {'axle_net_displacement_m':.2},
                     {'final_stop_yaw_rate_rad_s':.2}, {'active_slip_supported_s':0.}):
            m = self.fixture()
            m.update(edit)
            self.assertFalse(classify('turn-left-low', m, ACCEPT)['basic_pass'])

    def test_balance_torque_is_preserved_when_steering_saturates(self):
        left, right = wheel_mix(.029, .02)
        self.assertAlmostEqual((left+right)/2, .029)
        self.assertGreater(right, left)
        self.assertLessEqual(max(abs(left), abs(right)), .03+1e-15)
        for common in np.linspace(-.05, .05, 21):
            left, right = wheel_mix(common, .04)
            self.assertLessEqual(max(abs(left), abs(right)), .03+1e-15)

    def test_commands_stop_and_complete_a_real_squat_cycle(self):
        for t in (0., 1.5, 4.5, 6.):
            c = command('drive-turn-left', t)
            self.assertEqual(c['v_m_s'], 0.)
            self.assertEqual(c['yaw_rad_s'], 0.)
        self.assertAlmostEqual(command('squat-turn-left', 3.)['hip_q_rad'], .05)
        self.assertAlmostEqual(command('squat-turn-left', 6.)['hip_q_rad'], .3)
        self.assertGreater(command('squat-turn-left', 3.)['yaw_rad_s'], 0)


@unittest.skipUnless(DATA, 'Final baseline battery supplied separately')
class MeasuredMotionTests(unittest.TestCase):
    def rows(self):
        return json.loads((Path(DATA)/'RESULTS.json').read_bytes())

    def test_every_measured_case_uses_frozen_physics_and_retains_failures(self):
        rows = self.rows()
        self.assertEqual(len(rows), 12)
        for row in rows:
            s = row['summary']
            self.assertEqual(s['model_sha256'], '04f24a31a9c1f72e6f8599c266876ec6bdf17f2819ed79f510d1c145c82663f3')
            self.assertLessEqual(s['peak_hip_torque_nm'], .14709975+1e-12)
            self.assertLessEqual(s['peak_wheel_torque_nm'], .03+1e-12)
            self.assertEqual(row['export']['qpos_body_residual'], 0.)
            self.assertTrue(row['export']['geometry_ok'])
            if not s['basic_pass']:
                self.assertEqual(row['export']['status'], 'failed')
        self.assertTrue(any(not r['summary']['basic_pass'] for r in rows))

    def test_real_yaw_and_travel_have_correct_mirrored_signs(self):
        rows = {r['case']:r['summary'] for r in self.rows()}
        left, right = (rows[c]['metrics']['yaw_change_deg'] for c in ('turn-left-low', 'turn-right-low'))
        self.assertGreater(left, 35.)
        self.assertLess(right, -35.)
        self.assertAlmostEqual(left, -right, delta=1.)
        self.assertGreater(rows['drive-forward']['metrics']['signed_axle_travel_m'], .03)
        self.assertLess(rows['drive-backward']['metrics']['signed_axle_travel_m'], -.03)

    @unittest.skipUnless(V5, 'Frozen v5 dataset supplied separately')
    def test_zero_command_reproduces_v5_low_stance(self):
        before = json.loads((Path(V5)/'raw/stand_raw.json').read_bytes())
        after = json.loads((Path(DATA)/'raw/stand-low_raw.json').read_bytes())
        np.testing.assert_allclose([f['qpos'] for f in before['frames']],
                                   [f['qpos'] for f in after['frames']], atol=1e-9, rtol=0)


if __name__ == '__main__':
    unittest.main()
