# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:31:13 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
import unittest
import factors


class TestRelativeVelocity(unittest.TestCase):
    def setUp(self):
        self.factor = factors.RelativeVelocity(a=0.45, b=1.00, c=0.0, d_t=0.1)

    def check_relative_velocity(self, d_me, d_you, d_t, expected):
        d_me = np.array(d_me)
        d_you = np.array(d_you)
        actual = self.factor.relative_velocity(d_me, d_you, d_t)
        np.testing.assert_allclose(expected, actual)

    def test_relative_velocity(self):
#        正常系
        self.check_relative_velocity([1, 1], [2, 2], 0.1, np.sqrt(2)/0.1)
        self.check_relative_velocity([2, 2], [1, 1], 0.1, np.sqrt(2)/0.1)
        self.check_relative_velocity([1, 1], [1, 1], 0.1, 0)
        self.check_relative_velocity([2, 2], [1, 2], 0.1, 1/0.1)
        self.check_relative_velocity([2, 1], [2, 2], 0.1, 1/0.1)
#        異常系


if __name__ == '__main__':
    unittest.main()
