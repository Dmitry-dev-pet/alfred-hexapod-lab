"""Minimal local-controller tests (no network or GitHub Actions)."""
from pathlib import Path
import sys,unittest
ROOT=next((p for p in Path(__file__).resolve().parents if (p/'decentralized_ablation.py').exists()), None)
if ROOT is None: raise RuntimeError('Missing decentralized_ablation.py in experiment folder')
sys.path.insert(0,str(ROOT))
from decentralized_ablation import LegState, CODES, NEIGHBORS, MODES, initial_progresses, decide_liftoffs

class LocalRuleTests(unittest.TestCase):
    def setUp(self):
        self.states={c:LegState(name=c,stance_progress=1.) for c in CODES}
        self.prio={c:i for i,c in enumerate(CODES)}
    def test_all_six_independent_requests_can_be_admitted(self):
        self.assertEqual(set(decide_liftoffs(self.states,set(CODES),'independent',self.prio,1.)),set(CODES))
    def test_neighbor_inhibition_prohibits_adjacent_simultaneous_lifts(self):
        for m in ('neighbor_inhibit','neighbor_touchdown'):
            accepted=set(decide_liftoffs(self.states,set(CODES),m,self.prio,1.))
            self.assertGreaterEqual(len(accepted),1)
            for c in accepted:self.assertTrue(not (set(NEIGHBORS[c]) & accepted))
    def test_no_own_contact_blocks_request(self):
        for m in MODES:
            requests=decide_liftoffs(self.states,set(CODES)-{'fl'},m,self.prio,1.)
            self.assertNotIn('fl',requests)
    def test_neighbor_currently_swinging_blocks_request(self):
        self.states['ml'].phase='swing'
        self.assertNotIn('fl',decide_liftoffs(self.states,set(CODES),'neighbor_inhibit',self.prio,1.))
    def test_touchdown_excitation_is_opt_in(self):
        for c in CODES:self.states[c].stance_progress=.78
        self.states['ml'].excitation=.5
        self.assertEqual(decide_liftoffs(self.states,set(CODES),'neighbor_inhibit',self.prio,1.),[])
        self.assertIn('ml',decide_liftoffs(self.states,set(CODES),'neighbor_touchdown',self.prio,1.))
    def test_seeded_start_states_reproducible_non_tripod(self):
        self.assertEqual(initial_progresses(23),initial_progresses(23))
        self.assertNotEqual(initial_progresses(23),initial_progresses(37))
        self.assertGreater(len(set(round(v,3) for v in initial_progresses(23).values())),4)
    def test_no_prelift_during_initial_stand_ramp(self):
        self.assertEqual(decide_liftoffs(self.states,set(CODES),'independent',self.prio,.3),[])
    def test_all_variants_share_same_actuator_model_and_plan(self):
        # These are in run(); this test is a guardrail against silent model swaps.
        import inspect
        from decentralized_ablation import run
        src=inspect.getsource(run)
        self.assertIn('get_trajectory(model,original,legs,forward,stride,lift,duty)',src)
        self.assertIn('7.*(qtarget-data.qpos[qa])-.32*data.qvel[da]',src)
        self.assertIn('np.clip(unbounded,limits[:,0],limits[:,1])',src)
        self.assertNotIn('model.body_pos[root] =',src)

if __name__=='__main__':unittest.main()