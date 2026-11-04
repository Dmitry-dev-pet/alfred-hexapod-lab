"""Pure-geometric four-step screening examples: NOT physical gait tests."""
import unittest
import numpy as np
from horizon_contact_gate import ContactPhase, screen_four_step_horizon, select_candidate


FEET={"ml":(-.2,.2),"mr":(.2,.2),"bl":(-.2,-.2),"br":(.2,-.2)}
INITIAL=(0.,-.05)


def candidates(order=("bl","ml","br","mr"),bad_phase=None):
    feet={k:np.asarray(v,float).copy() for k,v in FEET.items()}
    phases=[]
    for k,leg in enumerate(order):
        remaining=np.array([v for c,v in feet.items() if c!=leg])
        before=np.mean(remaining,axis=0)
        if k==bad_phase:before=np.array([.4,.4])
        new_toe=feet[leg]+[0.,.04]
        after=before.copy()
        phases.append(ContactPhase(leg,tuple(before),tuple(new_toe),tuple(after)))
        feet[leg]=new_toe
    return tuple(phases)


class TestHorizonGate(unittest.TestCase):
    def test_four_future_lifts_are_screened(self):
        candidate=candidates()
        result=screen_four_step_horizon(FEET,INITIAL,candidate)
        self.assertTrue(result.admissible_necessary_geometry,result.reason)
        self.assertGreaterEqual(result.final_com_progress_m,.02)
        self.assertTrue(result.terminal_immediate_liftoff_options)

    def test_future_failure_blocks_first_step(self):
        result=screen_four_step_horizon(FEET,INITIAL,candidates(bad_phase=1))
        self.assertFalse(result.admissible_necessary_geometry)
        self.assertIn('phase 2',result.reason)

    def test_conservative_selector_prefers_complete_cycle(self):
        idx,result=select_candidate(FEET,INITIAL,[candidates(bad_phase=1),candidates()])
        self.assertEqual(idx,1)
        self.assertTrue(result.admissible_necessary_geometry)

    def test_com_progress_is_required(self):
        phases=candidates()
        result=screen_four_step_horizon(FEET,INITIAL,phases,minimum_forward_progress_m=.3)
        self.assertFalse(result.admissible_necessary_geometry)
        self.assertIn('progress',result.reason)

    def test_invalid_duplicate_leg_sequence_rejected(self):
        phases=list(candidates());phases[-1]=ContactPhase('ml',*(phases[-1].__dict__[k] for k in ['com_during_swing_xy_m','touchdown_xy_m','com_after_touchdown_xy_m']))
        with self.assertRaises(ValueError):screen_four_step_horizon(FEET,INITIAL,phases)


if __name__=='__main__':unittest.main()
