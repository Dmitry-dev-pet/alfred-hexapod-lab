"""Adapter contract tests that do not require private CAD/MuJoCo installation."""
import unittest

from experiments.cad30.live_mujoco import COMMANDS, stage_at, contact_loads
from hexlab.live_gait import LEGS


class TestAdapterContract(unittest.TestCase):
    def test_commands_switch_midway_through_2p4s_cycle(self):
        self.assertEqual([x[0] for x in COMMANDS], [0., 3.1, 6.2, 9.3])
        self.assertEqual([stage_at(t) for t in (0., 3.09, 3.1, 6.19,
                                                  6.2, 9.29, 9.3, 12.4)],
                         [0, 0, 1, 1, 2, 2, 3, 3])
        self.assertNotAlmostEqual(3.1 / 2.4 % 1., 0)

    def test_contacts_measured_not_scheduled(self):
        class Contact:
            def __init__(self, a, b, f):
                self.geom1, self.geom2, self.f = a, b, f

        class FakeModel:
            def geom(self, name):
                self_key = type('G', (), {})()
                self_key.id = 0 if name == 'floor' else -1
                return self_key

        class FakeData:
            contact = [Contact(0, 10, 1.2), Contact(11, 0, 0.3),
                       Contact(4, 5, 999.)]
            ncon = len(contact)

        class FakeMujoco:
            @staticmethod
            def mj_contactForce(model, data, ci, force):
                force[:] = 0
                force[0] = data.contact[ci].f

        loads = contact_loads(FakeModel(), FakeData(), FakeMujoco(),
                              {name: i + 10 for i, name in enumerate(LEGS)})
        self.assertAlmostEqual(loads['ml'], 1.2)
        self.assertAlmostEqual(loads['bl'], 0.3)
        self.assertEqual(loads['br'], 0)
        self.assertEqual(set(loads), set(LEGS))


if __name__ == '__main__':
    unittest.main()
