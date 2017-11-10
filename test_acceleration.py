# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:34:37 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
import unittest
import factors


class TestAcceleration(unittest.TestCase):
    def setUp(self):
        self.factor = factors.Acceleration(a=0.45, b=1.00, c=0.0, d_t=0.1)

    def check_acceleration(self, prev_p, p, next_p, d_t, expected):
        prev_p = np.array(prev_p)
        p = np.array(p)
        next_p = np.array(next_p)
        actual = self.factor.acceleration(prev_p, p, next_p, d_t)
        np.testing.assert_allclose(expected, actual)

    def test_acceleration(self):
#        正常系
        self.check_acceleration([1, 1], [2, 2], [4, 4], 0.1, np.sqrt(2)/0.1)
#        異常系
#        三点が同じとき
        self.check_acceleration([1, 1], [1, 1], [1, 1], 0.1, 0)
#        一直線上に三点が並ぶとき
        self.check_acceleration([1, 1], [2, 1], [3, 1], 0.1, 0)
        self.check_acceleration([1, 1], [1, 2], [1, 3], 0.1, 0)



if __name__ == '__main__':
    unittest.main()
