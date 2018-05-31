# -*- coding: utf-8 -*-
"""
Created on Wed Nov  1 15:27:06 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


#  リファクタリング後
def absolute_angle(p_base, p_top):
    # p_baseからみたp_topの相対位置ベクトルの絶対角度
    d = np.arctan2(p_top[1] - p_base[1], p_top[0] - p_base[0])
    return d


def revision_theta(theta):
    """
    出力：角度
    0°から360°以上を-180°から180°に範囲変換
    """
    theta += np.pi
    theta %= 2 * np.pi
    if theta < 0:
        theta += np.pi
    else:
        theta -= np.pi
    r_a = theta
    return r_a


def scope_transform(theta):
    """
    出力：角度
    -180°から180°を0°から360°以上-に範囲変換
    """
    if theta < 0:
        theta = np.pi*2 + theta
    return theta


def motion_velocity(p, next_p, d_t):
        """eq. (1)
        出力：スカラー、速さ
        """
        sq_distance = np.sum((next_p - p) ** 2)
#        assert np.all(sq_distance >= 0)
        m_v = np.sqrt(sq_distance) / d_t
        return m_v


def motion_acceleration(prev_m_v, motion_v):
        """
        出力：ベクトル
        """
        m_a = motion_v - prev_m_v
        return m_a


def motion_angular_velocity(prev_p, p, next_p, d_t):
        """eq. (2)
        出力：角度
        """
        ang = absolute_angle(prev_p, p)       # 時刻tのd_t(direction=向き)
        next_ang = absolute_angle(p, next_p)  # 時刻t+1のd_t+1(direction=向き)
        return (next_ang - ang) / d_t
