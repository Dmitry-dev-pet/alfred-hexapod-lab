"""Unit checks for exclusively LOCAL force transfer and bounded demands."""
import sys
import unittest
from pathlib import Path
P=Path(__file__).resolve().parents[1]
if str(P) not in sys.path:sys.path.insert(0,str(P))
from prelift_force_transfer import (CODES,NEIGHBORS,LegState,
    pre_lift_requests,transfer_pressure_update,local_force_liftoffs)

class PreliftTests(unittest.TestCase):
 def setUp(self):
  self.states={c:LegState(c,stance_progress=.5) for c in CODES}
  self.contacts=set(CODES)
  self.force={c:3.0 for c in CODES}
  self.priority={c:i for i,c in enumerate(CODES)}

 def test_single_request_requires_high_load_and_ready(self):
  self.states['fl'].stance_progress=.91
  self.assertEqual(pre_lift_requests(self.states,self.contacts,self.priority,self.force,1.),{'fl'})
  self.force['fl']=1.0
  self.assertEqual(pre_lift_requests(self.states,self.contacts,self.priority,self.force,1.),set())
  self.force['fl']=3.0
  self.contacts.remove('fl')
  self.assertEqual(pre_lift_requests(self.states,self.contacts,self.priority,self.force,1.),set())

 def test_nearby_conflict_keeps_local_priority(self):
  self.states['fl'].stance_progress=.93
  self.states['ml'].stance_progress=.93
  req=pre_lift_requests(self.states,self.contacts,self.priority,self.force,1.)
  self.assertEqual(len(req),1)
  self.assertIn('ml',req)   # CODES starts ml, lower priority id

 def test_pressure_only_direct_neighbors(self):
  requests={'fl'}
  pressure={c:0. for c in CODES}
  new=transfer_pressure_update(self.states,self.contacts,self.force,requests,pressure,
                                3.,.002,.004,.02)
  for c in CODES:
   if c in NEIGHBORS['fl']:
    self.assertAlmostEqual(new[c],.00004)
   else:self.assertEqual(new[c],0.)

 def test_pressure_cap_and_retract(self):
  v={c:.00399 for c in CODES}
  n=transfer_pressure_update(self.states,self.contacts,self.force,{'fl'},v,3.,.002,.004,.02)
  for c in NEIGHBORS['fl']:self.assertLessEqual(n[c],.004)
  n=transfer_pressure_update(self.states,self.contacts,self.force,set(),n,3.,.002,.004,.02)
  self.assertLess(n['ml'],.004)

 def test_high_force_neighbor_does_not_press_further(self):
  self.force['ml']=6.0
  v={c:.002 for c in CODES}
  n=transfer_pressure_update(self.states,self.contacts,self.force,{'fl'},v,3.,.002,.004,.02)
  self.assertLess(n['ml'],v['ml'])

 def test_force_gate_specific_to_new_mode(self):
  self.states['fl'].stance_progress=1.
  self.force['fl']=2.5
  ordinary=local_force_liftoffs(self.states,self.contacts,'neighbor_force_gate_supported',self.priority,1.,self.force,3.)
  new=local_force_liftoffs(self.states,self.contacts,'neighbor_pre_lift_transfer',self.priority,1.,self.force,3.)
  self.assertIn('fl',ordinary)
  self.assertNotIn('fl',new)
  self.force['fl']=1.4
  new=local_force_liftoffs(self.states,self.contacts,'neighbor_pre_lift_transfer',self.priority,1.,self.force,3.)
  self.assertIn('fl',new)

if __name__=='__main__':unittest.main()