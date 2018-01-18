# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:32:34 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
import unittest
import main_ver3


class TestLogger(unittest.TestCase):
    def setUp(self):
        self.logger = main_ver3.Logger(length_step=3)

    def check_log_leader(self, s, expected):
        actual = self.logger.log_leader(s)[-1]
        np.testing.assert_allclose(expected, actual)

    def test_log_leader(self):
        self.check_log_leader(
                np.array([[0, 1], [1, 2]]), np.array([[0, 1], [1, 2]]))

    def check_log_follower(self, s, expected):
        actual = self.logger.log_follower(s)[-1]
        np.testing.assert_allclose(expected, actual)

    def test_log_follower(self):
        self.check_log_leader(
                np.array([[0, 1], [1, 2]]), np.array([[0, 1], [1, 2]]))

    def check_display(self, p, next_p, d_t, expected):
        pass

    def test_display(self):
        pass


if __name__ == '__main__':
    unittest.main()
