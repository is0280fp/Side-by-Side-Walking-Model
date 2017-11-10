# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:29:24 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
import unittest
import factors


class TestRelativeDistance(unittest.TestCase):
    def setUp(self):
        self.factor = factors.RelativeDistance(a=0.45, b=1.00, c=0.0, d_t=0.1)

    def check_relative_distance(self, p_me, p_you, expected):
        p_me = np.array(p_me)
        p_you = np.array(p_you)
        actual = self.factor.relative_distance(p_me, p_you)
        np.testing.assert_allclose(expected, actual)

    def test_relative_distace(self):
#        正常系
        self.check_relative_distance([1, 1], [2, 2], np.sqrt(2))
        self.check_relative_distance([2, 2], [1, 1], np.sqrt(2))
        self.check_relative_distance([1, 1], [1, 1], 0)
        self.check_relative_distance([2, 2], [1, 2], 1)
        self.check_relative_distance([2, 1], [2, 2], 1)
#        異常系


if __name__ == '__main__':
    unittest.main()
