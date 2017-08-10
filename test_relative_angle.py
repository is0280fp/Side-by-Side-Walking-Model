# -*- coding: utf-8 -*-
"""
Created on Tue Aug  8 15:10:34 2017

@author: yume
"""

import numpy as np
import unittest


def angb(next_p_me, next_p_you, next_d_you):
    theta_mae = np.arctan2(next_d_you[1], next_d_you[0])
    theta_yoko = np.arctan2(
            next_p_me[1] - next_p_you[1], next_p_me[0] - next_p_you[0])
    print("theta_mae", theta_mae)
    print("theta_yoko", theta_yoko)
    theta = theta_yoko - theta_mae
#    if 180 < theta <= 360:
#        r_a = theta - 360
#    else:
#        r_a = theta
#    if theta <= 180:
#        r_a = theta
#    else:
#        r_a = theta - 360
    if 0 <= theta <= np.pi or - np.pi <= theta <= 0:
        print("1", theta)
        print("")
        r_a = theta
    elif - 2 * np.pi <= theta < - np.pi:
        print("3", theta)
        print("")
        r_a = theta + np.pi * 2
    else:                         # 今回の場合、この条件に該当するthetaは無し
        print("2", theta)
        print("")
        r_a = theta - np.pi * 2

    return np.abs(r_a)


def calculation_relative_angle(next_p_me, next_p_you, next_d_you):
#    assert(isinstance(next_p_me, np.ndarray) and next_p_me.shape == (2,))
    r_a = angb(next_p_me, next_p_you, next_d_you)
    # relative_angle
    return r_a


class TestRelativeAngle(unittest.TestCase):

    def check_relative_angle(self, p_me, p_you, d_you, r_a):
        r_a_actual = calculation_relative_angle(p_me, p_you, d_you)
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
