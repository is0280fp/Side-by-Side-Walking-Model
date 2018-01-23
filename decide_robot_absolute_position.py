# -*- coding: utf-8 -*-
"""
Created on Tue Jan 23 16:15:09 2018

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


def linear_equation(x1, y1, x2, y2):
    a = (y1 - y2) / (x1 - x2)
    b = y1 - a * x1
    return (a, b)


def decide_robot_absolute_position(p, d, social_distance):
    a, b = linear_equation(p[0], p[1], d[0], d[1])
    x1 = p[0]
#    y1 = p[1]
    x2 = x1 + social_distance / np.sqrt(a ** 2 + 1)
    y2 = a * (x1 + social_distance / np.sqrt(a ** 2 + 1)) + b
    return (x2, y2)


if __name__ == '__main__':
    print(decide_robot_absolute_position((3, 3), (2, 2), np.sqrt(2)))
