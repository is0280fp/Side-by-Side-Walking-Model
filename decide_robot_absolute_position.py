# -*- coding: utf-8 -*-
"""
Created on Tue Jan 23 16:15:09 2018

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


#  二点を通る直線の方程式
def linear_equation(x1, y1, x2, y2):
    a = (y1 - y2) / (x1 - x2)
    b = y1 - a * x1
    return (a, b)


#  直線上のある点からある距離の先にある点
def decide_robot_absolute_position(prev_p, p, social_distance):
    a, b = linear_equation(p[0], p[1], prev_p[0], prev_p[1])
    x1 = p[0]
#    y1 = p[1]
#   plus
    x = x1 + social_distance / np.sqrt(a ** 2 + 1)
    y = a * (x1 + social_distance / np.sqrt(a ** 2 + 1)) + b
#   minus
    x2 = x1 - social_distance / np.sqrt(a ** 2 + 1)
    y2 = a * (x1 - social_distance / np.sqrt(a ** 2 + 1)) + b

    return ((x2, y2), (x, y))


if __name__ == '__main__':
    print(decide_robot_absolute_position((1, 1), (3, 3), np.sqrt(2)))
    print(decide_robot_absolute_position((0, 0), (1, np.sqrt(3)), 2))
