# -*- coding: utf-8 -*-
"""
Created on Tue Aug  8 15:10:34 2017

@author: yume
"""

import numpy as np
import unittest
import geometry
import factors


def relative_angle(next_p_me, next_p_you, next_d_you):
    # youの進行方向の絶対角度
    theta_mae = np.arctan2(next_d_you[1], next_d_you[0])
    theta_yoko = geometry.angb(next_p_me, next_p_you)
    theta = theta_yoko - theta_mae
    r_a = geometry.revision_theta(theta)
    return r_a


class TestRelativeAngle(unittest.TestCase):
    def setUp(self):
        self.factor = factors.RelativeAngle(a=1, b=1, c=1, d_t=0.1)

    def check_relative_angle(self, p_me, p_you, d_you, r_a):
        r_a_actual = self.factor.relative_angle(p_me, p_you, d_you)
        np.testing.assert_allclose(r_a, r_a_actual)

    def test_relative_angle(self):
        self.check_relative_angle([2, 2], [1, 2], [0, 1], np.pi / 2)
        self.check_relative_angle([1, 2], [2, 2], [0, 1], np.pi / 2)
        self.check_relative_angle([1, 4], [1, 2], [0, 2], 0)
        self.check_relative_angle([1, 2], [1, 4], [0, 1], np.pi)
        self.check_relative_angle([2, 2], [1, 1], [0, 1], np.pi / 4)
        self.check_relative_angle([1, 1], [2, 2], [0, 1], np.pi * 3 / 4)
        self.check_relative_angle([2, 2], [1, 1], [1, 1], 0)
        self.check_relative_angle([1, 1], [2, 2], [2, 2], np.pi)
        self.check_relative_angle(
                [6, 6], [3, 3], [np.sqrt(3), 3], np.pi * 1 / 12)
        self.check_relative_angle(
                [3, 3], [6, 6], [np.sqrt(3), 3], np.pi * 11 / 12)
        self.check_relative_angle([1, 1], [2, 2], [1, 0], np.pi * 3 / 4)
        self.check_relative_angle([1, 1], [2, 2], [-1, 0], np.pi / 4)
        self.check_relative_angle([1, 1], [2, 2], [-1, -1], 0)
        self.check_relative_angle([2, 2], [1, 1], [-0.5, -0.5], np.pi)
        self.check_relative_angle([3, 1], [2, 2], [-2, 2], np.pi)
        self.check_relative_angle(
                [2 + np.sqrt(3), 1], [2, 2], [-2, 2], np.pi * 11 / 12)


if __name__ == '__main__':
    unittest.main()
