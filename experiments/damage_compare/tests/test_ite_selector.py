import pathlib
import sys
import unittest
import numpy as np

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from ite_selector import matern52, posterior_residual, select_next

class SelectorTests(unittest.TestCase):
    def setUp(self):
        self.x=np.array([[0.,0.],[.5,0.],[1.,1.]])
        self.s=np.array([1.,2.,3.])

    def test_kernel_is_psd_symmetric(self):
        k=matern52(self.x,self.x)
        np.testing.assert_allclose(k,k.T)
        self.assertGreaterEqual(np.linalg.eigvalsh(k).min(),-1e-9)

    def test_first_choice_highest_prior(self):
        idx, _=select_next(self.x,self.s,[])
        self.assertEqual(idx,2)

    def test_observations_change_posterior(self):
        mean,std=posterior_residual(self.x,[(1,-2.)])
        self.assertLess(mean[1],-1.8)
        self.assertLess(std[1],std[0])

    def test_no_repeated_trials(self):
        idx,_=select_next(self.x,self.s,[(2,-5.),(1,0.)])
        self.assertEqual(idx,0)

    def test_fully_tested_stops(self):
        with self.assertRaises(StopIteration):
            select_next(self.x,self.s,[(0,1.),(1,1.),(2,1.)])

    def test_duplicate_rejected(self):
        with self.assertRaises(ValueError):
            posterior_residual(self.x,[(0,1.),(0,2.)])

    def test_noise_positive(self):
        with self.assertRaises(ValueError):
            posterior_residual(self.x,[(0,1.)],noise=0.)

    def test_descriptor_dimension(self):
        with self.assertRaises(ValueError):
            matern52(self.x,np.ones((2,4)))

if __name__=='__main__': unittest.main()