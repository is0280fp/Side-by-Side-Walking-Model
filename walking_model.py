# -*- coding: utf-8 -*-
"""
Created on Wed May 24 16:09:19 2017

@author: yume
"""

import math
import numpy as np
import matplotlib.pyplot as plt


def walking_model(
        t=0.1, d_max=0.4, d_stance=0.33, t_swing=0.83, t_stance=0.33):
    #  秒とメートル
    '''The Detection and Following of Human Legs Through Inductive Approaches
    for a Mobile Robot With a Single Laser Range Finder
    '''
    t_cycle = t_swing + t_stance
    t = t % t_cycle
    if t < t_swing:
        d_t = - (d_max - d_stance) / 2  \
            * math.cos(2*math.pi/t_swing * t) + (d_max + d_stance) / 2
    else:
        d_t = d_stance

    return d_t


if __name__ == '__main__':
    t = 0
    d_max = 5
    d_stance = 0.4
    t_swing = 1.1
    t_stance = 0.4
    t_cycle = t_swing + t_stance
    v_max = 2
    d_t = []
    ts = np.arange(0, 7, 0.01)

    for t in ts:
        d_t.append(walking_model(t, d_max, d_stance, t_swing, t_stance))

    plt.plot(ts, d_t)  # このままでは横軸にd_tの添え字が入る
    plt.show()
