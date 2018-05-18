# -*- coding: utf-8 -*-
"""
Created on Thu May 17 16:16:39 2018

@author: yume
"""
import numpy as np
import matplotlib.pyplot as plt
import sympy.geometry as sg


#  2点間の距離がrより大きいか小さいか調べる
def dis_compare_r(p0, p1, r):
    dis = np.linalg.norm(p0 - p1)
    if dis >= r:
        return True
    else:
        return False


#  点p1と点p2を結ぶ直線上のp1=(x1, y1)から同じ直線上で距離r離れた点を求める
def other_p(p1, p2, r):
    center = sg.Point(p1[0], p1[1])
    radius = r

    circle = sg.Circle(center, radius)
    line = sg.Line(sg.Point(p1[0], p1[1]), sg.Point(p2[0], p2[1]))
    return np.array(sg.intersection(circle, line), np.float)


#  直線方程式y=ax+bのx=3の時のyの値
def p_on_line(a, b):
    x = 3
    y = a*x + b
    return np.array([x, y])


#  直線ax+by+c=0に垂直で、p=(x0, y0)を通る直線方程式y=ex+d
def normal(a, b, p):
    x0 = p[0]
    y0 = p[1]

    e = b/a
    d = (a*y0 - b*x0)/a
    return (e, d)


#  2点の直線式ay+bx+cを求める
def line(p1, p2):
    x1 = p1[0]
    x2 = p2[0]
    y1 = p1[1]
    y2 = p2[1]

    a = 1
    b = (y1-y2)/(x2-x1)
    c = -y1 + x1*(y2-y1)/(x2-x1)
    return (a, b, c)


if __name__ == '__main__':
    p0 = np.array([0, 0])
    p1 = np.array([1, 1])
    r = 2**0.5

    a, b, c = line(p0, p1)
    print(a, b, c)
    e, d = normal(a, b, p0)
    print(e, d)
    p2 = p_on_line(e, d)
    p = other_p(p0, p2, r)
    print(p[0], p[1])

    next_p = np.array([2, 2])
    ans = dis_compare_r(next_p, p[0], r)
    print(ans)
    ans = dis_compare_r(next_p, p[1], r)
    print(ans)
