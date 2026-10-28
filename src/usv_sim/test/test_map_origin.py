import unittest
from usv_sim.map_origin import local_xy


class OriginTests(unittest.TestCase):
    def test_equal_origin_and_observed_wind_drift(self):
        self.assertEqual(local_xy(-33.724223,150.679736),(0.,0.))
        x,y=local_xy(-33.7242275,150.6797341)
        self.assertAlmostEqual(x,-.176,delta=.002)
        self.assertAlmostEqual(y,-.499,delta=.002)

    def test_invalid_origin_rejected(self):
        with self.assertRaises(ValueError):local_xy(float('nan'),150.)
        with self.assertRaises(ValueError):local_xy(100.,150.)
