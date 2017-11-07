"""
Created on Tue Aug  8 15:10:34 2017

@author: yume
"""

import numpy as np
import unittest
import factors


class TestDistanceToObstacle(unittest.TestCase):
    def setUp(self):
        self.factor = factors.DistanceToObstacle(a=20.0, b=0.40, c=0, d_t=1)

    def check_distance_to_obstacle(self, obstacle, p, expected):
        p = np.array(p)
        obstacle = np.array(obstacle)
        actual = self.factor.distance_to_obstacle(p, obstacle)
        np.testing.assert_allclose(expected, actual)

    def test_distance_to_obstacle(self):
        self.check_distance_to_obstacle([1, 1], [0, 0], np.sqrt(2))
        self.check_distance_to_obstacle([1, 1], [-1, -1], np.sqrt(8))
        self.check_distance_to_obstacle([0, 0], [1, 1], np.sqrt(2))
        self.check_distance_to_obstacle([-1, -1], [1, 1], np.sqrt(8))
        self.check_distance_to_obstacle([-101, -101], [-100, -100], np.sqrt(2))
        self.check_distance_to_obstacle([-101, -101], [-1, -1], np.sqrt(100**2 + 100**2))
        self.check_distance_to_obstacle([-1.5, -1.2], [-0.5, -0.2], np.sqrt(2))
        self.check_distance_to_obstacle([-1.5, -1.2], [1, 0.3], np.sqrt((-2.5)**2 + (-1.5)**2))


if __name__ == '__main__':
    unittest.main()
