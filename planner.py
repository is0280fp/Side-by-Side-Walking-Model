# -*- coding: utf-8 -*-
"""
Created on Thu Jul 20 18:06:45 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


def decide_action(trajectory_me, trajectory_you, num_grid_x, num_grid_y,
                  search_range_x, search_range_y, k_o, k_rv, k_rd, k_ra):
    utility = []

    p_me = trajectory_me[-1]  # 真の位置
    p_you = trajectory_you[-1]
    next_p_you, d_you = linear_extrapolation(trajectory_you)  # 予測した次の位置と変化量
    next_d_you = next_p_you - p_you  # 予測した次の変化量

    grid_points = making_grid(num_grid_x, num_grid_y,
                              search_range_x, search_range_y)
    # 予測位置を中心としたグリッドを作成
    midle_point, d_me = linear_extrapolation(trajectory_me)
    grid_points = grid_points + midle_point

    m_v_you, m_w_you, next_ang_you, ang_you = calculation_motion_factor_you(
            p_you, next_p_you, d_you, next_d_you, d_t)

    for next_p_me in grid_points:
        next_d_me = next_p_me - p_me
        m_v_me, m_w_me, next_ang_me, ang_me = calculation_motion_factor_me(
                p_me, next_p_me, d_me, next_d_me, d_t)
        r_d, r_a, r_v = calculation_relative_factor(
                next_p_me, next_p_you, next_d_me,
                next_d_you, next_ang_you, d_t)
        f_o, f_rv, f_rd, f_ra = calculation_utility(
                k_o, k_rv, k_rd, k_ra, r_d, r_a, r_v)
        utility.append(k_o * f_o + k_rv * f_rv + k_rd * f_rd + k_ra * f_ra)

    utility = np.array(utility).reshape(num_grid_y, num_grid_x)
    predicted_p_me = grid_points[utility.argmax()]
    plt.matshow(utility)
    plt.gca().invert_yaxis()
    plt.colorbar()
    plt.show()
    return predicted_p_me


def linear_extrapolation(trajectory):
    """位置の履歴から次の時刻の位置を等速直線運動で予測する

    Args　:
        trajectory:
            位置の記録
    Returns:
        next_p(ndarray):
            .. math:: \hat{p}_{t+1}
        d(ndarray):
            p_t - p.{t-1}
    """
    d = trajectory[-1] - trajectory[-2]
    next_p = trajectory[-1] + d
    return next_p, d


def m_v(p, next_p, d_t):
    """eq. (1)
    """
    m_v = np.sqrt(np.sum((next_p - p) ** 2)) / d_t
    return m_v


def m_w(ang, next_ang, d_t):
    """eq. (2)
    """
    return (next_ang - ang) / d_t


def angb(p1, p2):
    d = p1 - p2
    return np.arctan2(d[1], d[0])


def f(x, a=0.25, b=2.00, c=0.75):
    """
    eq. (9)
    """
    f_x = 1 / (1+(abs((x-c) / a)**(2*b))) - 1
    return f_x


def making_grid(num_grid_x, num_grid_y, search_range_x, search_range_y):
    # trajectoryの基の7*7grid作る(中心を(0,0)とする)
    x = np.linspace(-search_range_x, search_range_x, num_grid_x)
    y = np.linspace(-search_range_y, search_range_y, num_grid_y)
    x, y = np.meshgrid(x, y)
    x = x.reshape(-1, 1)
    y = y.reshape(-1, 1)
    grid_points = np.hstack((x, y))
    return grid_points


def calculation_utility(k_o, k_rv, k_rd, k_ra, r_d, r_a, r_v):
    # utilityの計算
    f_o = 0
    f_rv = f(r_v, 0.2, 1.2, 0)
    f_rd = f(r_d, 0.25, 2.0, 0.75)
    f_ra = f(r_a, 0.08, 3.0, np.pi/2)
    return f_o, f_rv, f_rd, f_ra


def calculation_motion_factor_me(p_me, next_p_me, d_me, next_d_me, d_t):
    m_v_me = m_v(p_me, next_p_me, d_t)  # 現在の位置と予測した位置に基づいた現在の速さ
    ang_me = np.arctan(d_me[1]/d_me[0])  # 現在のベクトル
    next_ang_me = np.arctan(
            next_d_me[1]/next_d_me[0])  # 予測した変化量に基づく次回のベクトル
    m_w_me = m_w(ang_me, next_ang_me, d_t)  # 現在と予測による角速度
    return m_v_me, m_w_me, next_ang_me, ang_me


def calculation_motion_factor_you(p_you, next_p_you, d_you, next_d_you, d_t):
    m_v_you = m_v(p_you, next_p_you, d_t)
    ang_you = np.arctan(d_you[1]/d_you[0])
    next_ang_you = np.arctan(next_d_you[1]/next_d_you[0])
    m_w_you = m_w(ang_you, next_ang_you, d_t)
    return m_v_you, m_w_you, next_ang_you, ang_you


def calculation_relative_factor(next_p_me, next_p_you,
                                next_d_me, next_d_you, next_ang_you, d_t):
    # (relative factor)
    r_d = np.linalg.norm(next_p_you - next_p_me)  # socialrelativedistance
    r_a = next_ang_you - angb(next_p_me, next_p_you)  # relative_angle
    r_v = np.linalg.norm((next_d_me - next_d_you) / d_t)  # relative_v
    return r_d, r_a, r_v


if __name__ == '__main__':
    trajectory_me = np.array([[1, 1],
                             [1.03, 1],
                             [1.06, 1],
                             [1.09, 1]])
    trajectory_you = np.array([[1, 2],
                              [1.03, 2],
                              [1.06, 2],
                              [1.09, 2]])
    num_grid_x = 7
    num_grid_y = 7
    search_range_x = 0.6
    search_range_y = 0.6
    d_t = 0.03
    k_o = 0.11
    k_rv = 0.01
    k_rd = 0.25
    k_ra = 0.32  # ra = relative_angle

    print(decide_action(trajectory_me, trajectory_you, num_grid_x, num_grid_y,
                  search_range_x, search_range_y, k_o, k_rv, k_rd, k_ra))

#    p_me = trajectory_me[-1]  # 真の位置
#    p_you = trajectory_you[-1]
#    prev_p_me = trajectory_me[-2]  # 一事象前の真の位置
#    prev_p_you = trajectory_you[-2]
#    utility = []
#    next_p_you, d_you = linear_extrapolation(trajectory_you)  # 予測した次の位置と変化量
#    next_d_you = next_p_you - p_you  # 予測した次の変化量

#    m_v_you = m_v(p_you, next_p_you, d_t)
#    ang_you = np.arctan(d_you[1]/d_you[0])
#    next_ang_you = np.arctan(next_d_you[1]/next_d_you[0])
#    m_w_you = m_w(ang_you, next_ang_you, d_t)

#    # 予測位置を中心としたグリッドを作成
#    midle_point, d_me = linear_extrapolation(trajectory_me)
#    grid_points = grid_points + midle_point

#    ang_me = np.arctan(d_me[1]/d_me[0])  # 現在のベクトル

#    for i in np.arange(num_grid_x * num_grid_y):
#    for next_p_me in grid_points:
#        next_p_me = grid_points[i]  # 予測した次の位置と変化量
#        next_d_me = next_p_me - p_me

        # decide_action(motion factor)
#        m_v_me = m_v(p_me, next_p_me, d_t)  # 現在の位置と予測した位置に基づいた現在の速さ
#        next_ang_me = np.arctan(
#                next_d_me[1]/next_d_me[0])  # 予測した変化量に基づく次回のベクトル
#        m_w_me = m_w(ang_me, next_ang_me, d_t)  # 現在と予測による角速度

#        # (relative factor)
#        r_d = np.linalg.norm(next_p_you - next_p_me)  # socialrelativedistance
#        r_a = next_ang_you - angb(next_p_me, next_p_you)  # relative_angle
#        r_v = np.linalg.norm((next_d_me - next_d_you) / d_t)  # relative_v

#        # utilityの計算
#        f_o = 0
#        f_rv = f(r_v, 0.2, 1.2, 0)
#        f_rd = f(r_d, 0.25, 2.0, 0.75)
#        f_ra = f(r_a, 0.08, 3.0, np.pi/2)
#        utility.append(k_o * f_o + k_rv * f_rv + k_rd * f_rd + k_ra * f_ra)
#    utility = np.array(utility).reshape(num_grid_y, num_grid_x)
#    grid_points[utility.argmax()]
#    plt.matshow(utility)
#    plt.gca().invert_yaxis()
#    plt.colorbar()
#    plt.show()
