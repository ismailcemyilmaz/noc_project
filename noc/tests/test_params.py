import unittest

import numpy as np

from noc.common import params as P


class TestHoverAllocation(unittest.TestCase):
    def setUp(self):
        self.G = P.compute_G()
        wrench = np.array([0, 0, P.MASS * P.G_ACC, 0, 0, 0])
        self.u = np.linalg.solve(self.G, wrench)  # u = w*|w|

    def test_G_full_rank(self):
        self.assertEqual(np.linalg.matrix_rank(self.G), 6)

    def test_hover_rotor_speeds_equal(self):
        self.assertTrue(np.all(self.u > 0))
        w = np.sqrt(self.u)
        np.testing.assert_allclose(w, w[0], rtol=1e-9)
        print(f"\n  hover speeds (computed, mass={P.MASS} kg): {np.round(w, 3)}")

    def test_hover_speed_within_limits(self):
        w = np.sqrt(self.u)
        self.assertTrue(np.all((w > P.OMEGA_MIN) & (w < P.OMEGA_MAX)))

    def test_derived_constants(self):
        self.assertAlmostEqual(P.MASS, 2.72)
        self.assertAlmostEqual(P.X_WALL, 2.0)


if __name__ == '__main__':
    unittest.main()
