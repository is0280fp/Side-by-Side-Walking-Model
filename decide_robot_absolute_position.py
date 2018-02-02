# -*- coding: utf-8 -*-
"""
Created on Tue Jan 23 16:15:09 2018

@author: yume
"""
import unittest
import numpy as np
import matplotlib.pyplot as plt


#  二点を通る直線の方程式
def linear_equation(x1, y1, x2, y2):
    a = (y1 - y2) / (x1 - x2)
    b = y1 - a * x1
    return (a, b)


#  直線上のある点からある距離の先にある点
def f(p_prev, p, r):
    assert not np.array_equal(p_prev, p)
    d = p - p_prev
    d_unit = d / np.linalg.norm(d)
    return p + r * d_unit


class Test(unittest.TestCase):
    def setUp(self):
        pass

    def check_decide_robot_absolute_position(
            self,  prev_p, p, social_distance, expected):
        prev_p = np.array(prev_p)
        p = np.array(p)
        actual = f(prev_p, p, social_distance)
        np.testing.assert_allclose(expected, actual)

    def test_decide_robot_absolute_position(self):
        #        正常系
        self.check_decide_robot_absolute_position(
                (1, 1), (3, 3), np.sqrt(2), (4, 4))
        self.check_decide_robot_absolute_position(
                (0, 0), (1, np.sqrt(3)), 2, (2, 2*np.sqrt(3)))


if __name__ == '__main__':
    unittest.main()
