# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:28:10 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
import unittest
import factors


class TestMovingTowardSubgoals(unittest.TestCase):
    def setUp(self):
        self.factor = factors.MovingTowardSubgoals(a=0.45, b=1.00, c=0.0, d_t=0.1)

    def check_moving_toward_subgoals(self, p, next_p, subgoal_p, expected):
        p = np.array(p)
        next_p = np.array(next_p)
        subgoal_p = np.array(subgoal_p)
        actual = self.factor.moving_toward_subgoals(p, next_p, subgoal_p)
        np.testing.assert_allclose(expected, actual)

    def test_distance_to_obstacle(self):
#        正常系
        self.check_moving_toward_subgoals([1, 1], [2, 2], [4, 4], np.deg2rad(0))
#        異常系
#        三点が同じとき
        self.check_moving_toward_subgoals([1, 1], [1, 1], [1, 1], np.deg2rad(0))
#        一直線上に三点が並ぶとき
        self.check_moving_toward_subgoals([1, 1], [2, 1], [3, 1], np.deg2rad(0))
        self.check_moving_toward_subgoals([1, 1], [1, 2], [1, 3], np.deg2rad(0))


if __name__ == '__main__':
    unittest.main()
