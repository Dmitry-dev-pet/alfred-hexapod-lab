import json,os,tempfile,unittest
from pathlib import Path
from research.navbot_en01.source_jump import JumpCommands,cad_targets,audit_source
from research.navbot_en01.source_jump_probe import JumpGate

class SourceCommandTests(unittest.TestCase):
    def test_original_command_edges_and_no_held_button_retrigger(self):
        f=JumpCommands();events=[]
        for i in range(480):
            s=f.tick(previous_stop=i in (0,250),jump=True)
            events.extend((i,e) for e in s['writes'])
        self.assertEqual(events,[(0,'extend'),(29,'return'),(190,'resume'),(250,'extend'),(279,'return'),(440,'resume')])

    def test_calibration_is_never_invented_or_clipped(self):
        with self.assertRaises(ValueError):cad_targets([2463,1632])
        with self.assertRaises(ValueError):cad_targets([2463,1632],anchor_q_rad=2.)
        q=cad_targets([2463,1632],anchor_q_rad=0.)
        self.assertAlmostEqual(q[0],353*2*3.141592653589793/4096)
        self.assertAlmostEqual(q[1],354*2*3.141592653589793/4096)

    @unittest.skipUnless(os.environ.get('EN01_UPSTREAM_SOURCE_DIR'),'Pinned upstream files provided separately')
    def test_exact_upstream_cpp_matches_all_480_iterations(self):
        p=Path(os.environ['EN01_UPSTREAM_SOURCE_DIR'])
        result=audit_source(p/'firmware.ino',p/'servo.cpp',p/'basic_web.h')
        self.assertTrue(result['all_commands_and_flags_match'])
        self.assertEqual(result['compiled_original_cpp_iterations'],480)
        with tempfile.TemporaryDirectory() as td:
            forged=Path(td)/'forged.ino';forged.write_text('void jump_loop(){}')
            with self.assertRaisesRegex(ValueError,'blob mismatch'):audit_source(forged,p/'servo.cpp',p/'basic_web.h')

class PhysicalJumpGateTests(unittest.TestCase):
    def prime(self,supported=True):
        gate=JumpGate(.001)
        for i in range(100):gate.update(t=1.4+i*.001,ground=True,nonwheel=not supported,both_wheels=supported,upright=supported,clearance=0.,com_vz=0.)
        return gate

    def flight(self,g,*,ticks=30,vz=.2):
        for i in range(ticks):g.update(t=1.5+i*.001,ground=False,nonwheel=False,both_wheels=False,upright=True,clearance=.005,com_vz=vz)
        return 1.5+ticks*.001

    def land(self,g,t,*,nonwheel=False):
        for i in range(2001):g.update(t=t+i*.001,ground=True,nonwheel=nonwheel,both_wheels=True,upright=True,clearance=0.,com_vz=0.)

    def test_clean_upward_flight_and_two_second_landing(self):
        g=self.prime();t=self.flight(g);self.land(g,t)
        self.assertTrue(g.result(t+2)['jump_pass'])

    def test_airborne_wheels_do_not_count_with_body_on_floor(self):
        g=self.prime()
        for i in range(50):g.update(t=1.5+i*.001,ground=True,nonwheel=True,both_wheels=False,upright=False,clearance=.01,com_vz=.2)
        self.land(g,1.55);self.assertFalse(g.result(4)['qualified_flight'])

    def test_short_flight_downward_fall_and_bad_pre_support_fail(self):
        for ticks,vz,support in [(10,.2,True),(30,-.2,True),(30,.2,False)]:
            g=self.prime(support);t=self.flight(g,ticks=ticks,vz=vz);self.land(g,t)
            self.assertFalse(g.result(t+2)['jump_pass'])

    def test_body_impact_invalidates_landing(self):
        g=self.prime();t=self.flight(g);self.land(g,t,nonwheel=True)
        self.assertTrue(g.result(t+2)['qualified_flight']);self.assertFalse(g.result(t+2)['jump_pass'])

class ExportIntegrityTests(unittest.TestCase):
    @unittest.skipUnless(os.environ.get('EN01_JUMP_RAW_TRACE'),'Final raw trace supplied separately')
    def test_forged_body_pose_is_rejected_by_independent_fk(self):
        from research.navbot_en01.source_jump_export import export
        d=json.loads(Path(os.environ['EN01_JUMP_RAW_TRACE']).read_text())
        d['frames'][0]['body_poses'][0][0]+=.01
        with tempfile.TemporaryDirectory() as td:
            p=Path(td);(p/'forged.json').write_text(json.dumps(d))
            with self.assertRaisesRegex(ValueError,'poses disagree'):export(p/'forged.json',p/'out.json')

if __name__=='__main__':unittest.main()
