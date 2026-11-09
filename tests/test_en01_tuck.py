import json, unittest, math
from pathlib import Path
from research.navbot_en01.tuck_commands import Sequence
from research.navbot_en01.getup_probe import stable_gate


class SequentialGetupTests(unittest.TestCase):
    def test_no_impulse_until_both_hips_stop_at_target(self):
        s=Sequence(.02,1,.15)
        for t in [0.,.3,.6,1.]:
            _,torque,phase=s.command(t,[.02,.02],[.2,0.])
            self.assertEqual(torque,[0.,0.]);self.assertIsNone(s.pulse_at)
        s.command(1.01,[.02,.02],[0.,0.])
        self.assertEqual(s.command(1.105,[.02,.02],[0.,0.])[1],[0.,0.])
        self.assertEqual(s.command(1.11,[.02,.02],[0.,0.])[1],[.03,.03])

    def test_single_pulse_ends_and_never_restarts(self):
        s=Sequence(.02,-1,.15)
        s.command(.3,[.02,.02],[0.,0.]);s.command(.4,[.02,.02],[0.,0.])
        self.assertEqual(s.command(.545,[.02,.02],[0.,0.])[1],[-.03,-.03])
        for t in [.55,.8,2.,4.]:self.assertEqual(s.command(t,[.02,.02],[0.,0.])[1],[0.,0.])

    def test_timeout_never_issues_impulse(self):
        s=Sequence(.02,1,.5)
        for t in [.3,1.25,1.5,2.]:self.assertEqual(s.command(t,[.1,.1],[0.,0.])[1],[0.,0.])
        self.assertTrue(s.timed_out);self.assertIsNone(s.pulse_at)

    def test_old_gate_rejects_brief_upright_body_support_and_speed(self):
        a=json.loads(Path('research/navbot_en01/tuck_protocol.json').read_text())['acceptance']
        self.assertTrue(stable_gate(0,[1,1],0,0,0,a))
        self.assertFalse(stable_gate(0,[1,1],1,0,0,a))
        self.assertFalse(stable_gate(math.pi/2,[1,1],0,0,0,a))


if __name__=='__main__':unittest.main()
