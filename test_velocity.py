# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:32:34 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
import unittest
import factors


class TestVelocity(unittest.TestCase):
    def setUp(self):
        self.factor = factors.Velocity(a=0.30, b=1.60, c=0.03, d_t=0.1)

    def check_m_v(self, p, next_p, d_t, expected):
        p = np.array(p)
        next_p = np.array(next_p)
        actual = self.factor.velocity(p, next_p, d_t)
        np.testing.assert_allclose(expected, actual)

    def test_m_v(self):
        self.check_m_v([1, 1], [0, 0], 0.1, np.sqrt(2)/0.1)
        self.check_m_v([0, 0], [1, 1], 0.1, np.sqrt(2)/0.1)
        self.check_m_v([1, 1], [1, 1], 0.1, 0)
        self.check_m_v([7.9, 8.9], [4.2, 5.8], 0.1, np.sqrt(13.690000000000001+9.610000000000001)/0.1)


if __name__ == '__main__':
    unittest.main()
