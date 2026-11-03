import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from active_load_transfer import CODES,LegState,support_pressure_targets,compute_press_delta

class ActiveSupportTests(unittest.TestCase):
    def setUp(self):
        self.states={c:LegState(c,stance_progress=.2) for c in CODES}
        self.contacts=set(CODES)
        self.loads={c:2.95 for c in CODES}
    def test_neighbor_request_generates_local_support(self):
        self.states['fl'].stance_progress=1.
        self.loads['fl']=4.45
        demands=support_pressure_targets(self.states,self.contacts,self.loads,3.)
        self.assertGreater(demands['fr'],0.)
        self.assertGreater(demands['ml'],0.)
        for c in ('br','bl'):
            self.assertEqual(demands[c],0.)
    def test_high_own_load_does_not_press(self):
        self.states['fl'].stance_progress=1.
        self.loads['fl']=4.45
        self.loads['fr']=5.
        self.assertEqual(support_pressure_targets(self.states,self.contacts,self.loads,3.)['fr'],0.)
    def test_unready_neighbor_does_not_request(self):
        self.states['fl'].stance_progress=.5
        self.loads['fl']=6.
        self.assertEqual(support_pressure_targets(self.states,self.contacts,self.loads,3.)['fr'],0.)
    def test_contact_needed_to_press(self):
        self.states['fl'].stance_progress=1.
        self.loads['fl']=4.5
        self.contacts.remove('fr')
        self.assertEqual(support_pressure_targets(self.states,self.contacts,self.loads,3.)['fr'],0.)
    def test_bad_weight_rejected(self):
        with self.assertRaises(ValueError):
            support_pressure_targets(self.states,self.contacts,self.loads,0.)

if __name__=='__main__':unittest.main()
