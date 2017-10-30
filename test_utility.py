"""
Created on Tue Aug  8 15:10:34 2017

@author: yume
"""

import numpy as np
import unittest


def distance_to_obstacle(obstacle, p):
    """
    obstacleとpはともに二次元平面座標系のx, y座標を持つtuple
    """
    distance = np.sqrt(np.sum((obstacle - p) ** 2))
    return distance


def calculation_environmental_factors(obstacle, p):
    e_o = distance_to_obstacle(obstacle, p)
    return e_o


def f_obstacle(x, a, b):
    f_o = - np.abs((a / x) ** (2*b))
    return f_o


class TestDistanceToObstacle(unittest.TestCase):

    def check_distance_to_obstacle(self, obstacle, p, expected):
        actual = distance_to_obstacle(obstacle, p)
        np.testing.assert_allclose(expected, actual)

    def test_distance_to_obstacle(self):
        self.check_distance_to_obstacle(np.array([1, 1]), np.array([0, 0]), np.sqrt(2))
        self.check_distance_to_obstacle(np.array([1, 1]), np.array([-1, -1]), np.sqrt(8))
        self.check_distance_to_obstacle(np.array([0, 0]), np.array([1, 1]), np.sqrt(2))
        self.check_distance_to_obstacle(np.array([-1, -1]), np.array([1, 1]), np.sqrt(8))
        self.check_distance_to_obstacle(np.array([-101, -101]), np.array([-100, -100]), np.sqrt(2))
        self.check_distance_to_obstacle(np.array([-101, -101]), np.array([-1, -1]), np.sqrt(100**2 + 100**2))
        self.check_distance_to_obstacle(np.array([-1.5, -1.2]), np.array([-0.5, -0.2]), np.sqrt(2))
        self.check_distance_to_obstacle(np.array([-1.5, -1.2]), np.array([1, 0.3]), np.sqrt((-2.5)**2 + (-1.5)**2))

    def check_f_obstacle(self, x, a, b, expected):
        actual = f_obstacle(x, a, b)
        np.testing.assert_allclose(expected, actual)

    def test_f_obstacle(self):
        self.check_f_obstacle(0.6, 20, 0.4, 1)
        self.check_f_obstacle(0.4, 20, 0.4, 0)


if __name__ == '__main__':
    unittest.main()
