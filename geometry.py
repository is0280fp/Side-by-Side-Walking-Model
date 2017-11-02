# -*- coding: utf-8 -*-
"""
Created on Wed Nov  1 15:27:06 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


def angb(self, p1, p2):
        # p2からみたp1の相対位置ベクトルの絶対角度
        d = np.arctan2(p1[1] - p2[1], p1[0] - p2[0])
        return d


def revision_theta(self, theta):
    """
    unit test required
    """
    theta += np.pi
    theta %= 2 * np.pi
    if theta < 0:
        theta += np.pi
    else:
        theta -= np.pi
    r_a = theta
    return r_a


def m_v(self, p, next_p, d_t):
        """eq. (1)
        unit test required
        """
        m_v = np.sqrt(np.sum((next_p - p) ** 2)) / d_t
        return m_v


def m_a(self, prev_m_v, motion_v):
        """
        unit test required
        """
        m_a = motion_v - prev_m_v
        return m_a


def m_w(self, prev_p, p, next_p, d_t):
        """eq. (2)
        unit test required
        """
        ang = angb(p, prev_p)       # 時刻tのd_t(direction=向き)
        next_ang = angb(next_p, p)  # 時刻tのd_t+1(direction=向き)
        return (next_ang - ang) / d_t
