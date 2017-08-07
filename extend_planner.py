# -*- coding: utf-8 -*-
"""
Created on Thu Jul 20 18:06:45 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
import itertools


class ExtendPlanner(object):
    def __init__(self, num_grid_x, num_grid_y, search_range_x, search_range_y,
                 k_o, k_rv, k_rd, k_ra, k_s, k_ma, k_mv, k_mw, d_t):
        self.num_grid_x = num_grid_x
        self.num_grid_y = num_grid_y
        self.search_range_x = search_range_x
        self.search_range_y = search_range_y
        self.k_o = k_o
        self.k_rv = k_rv
        self.k_rd = k_rd
        self.k_ra = k_ra  # ra = relative_angle
        self.k_s = k_s
        self.k_ma = k_ma
        self.k_mv = k_mv
        self.k_mw = k_mw
        self.d_t = d_t

    def decide_action(self, trajectory_me, trajectory_you, subgoal_p):
        utility_me = []
        utility_you = []
        utility = []

        p_me = trajectory_me[-1]  # 真の位置
        p_you = trajectory_you[-1]
        prev_p_me = trajectory_me[-2]  # 真の位置
        prev_p_you = trajectory_you[-2]

        grid_points_me = self.making_grid(
                num_grid_x, num_grid_y, search_range_x, search_range_y)
        grid_points_you = self.making_grid(
                num_grid_x, num_grid_y, search_range_x, search_range_y)

        # 予測位置を中心としたグリッドを作成
        midle_point_me, d_me = self.linear_extrapolation(
                trajectory_me)
        grid_points_me = grid_points_me + midle_point_me
        # 予測位置を中心としたグリッドを作成
        midle_point_you, d_you = self.linear_extrapolation(
                trajectory_you)
        grid_points_you = grid_points_you + midle_point_you

    #    for next_p_me in grid_points_me:
        each_other_p = list(itertools.product(grid_points_you, grid_points_me))
        for next_p_me, next_p_you in each_other_p:
            next_d_me = next_p_me - p_me
            next_d_you = next_p_you - p_you
            u_me, u_you = self.calculation_utility(
                    prev_p_me, p_me, next_p_me, d_me, next_d_me,
                    prev_p_you, p_you, next_p_you, d_you,
                    next_d_you, subgoal_p)
            utility_me.append(u_me)
            utility_you.append(u_you)
            utility.append(u_me + u_you)

        utility = np.array(utility_you)
        utility_map = utility.reshape(
            num_grid_y, num_grid_x, num_grid_y, num_grid_x).transpose(
                0, 2, 1, 3).reshape(num_grid_y**2, num_grid_x**2)

        predicted_p_me, predicted_p_you = each_other_p[utility.argmax()]
        plt.matshow(utility_map[21:28, 21:28])
        plt.gca().invert_yaxis()
        plt.colorbar()
        plt.show()
        return predicted_p_me

    def linear_extrapolation(self, trajectory):
        """位置の履歴から次の時刻の位置を等速直線運動で予測する

        Args　:
            trajectory:ｎｕｍ＿grid＿x
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

    def m_v(self, p, next_p):
        """eq. (1)
        """
        m_v = np.sqrt(np.sum((next_p - p) ** 2)) / d_t
        return m_v

    def m_w(self, ang, next_ang):
        """eq. (2)
        """
        return (next_ang - ang) / d_t

    def m_a(self, v, next_v):
        """eq. (3)
        """
        return (next_v - v) / d_t

    def angb(self, p1, p2):
        d = p1 - p2
        return np.abs(np.arctan2(d[1], d[0]))

    def f(self, x, a=0.25, b=2.00, c=0.75):
        """
        eq. (9)
        """
        f_x = 1 / (1+(abs((x-c) / a)**(2*b))) - 1
        return f_x

    def making_grid(self,
                    num_grid_x, num_grid_y, search_range_x, search_range_y):
        # trajectoryの基の7*7grid作る(中心を(0,0)とする)
        x = np.linspace(-search_range_x, search_range_x, num_grid_x)
        y = np.linspace(-search_range_y, search_range_y, num_grid_y)
        x, y = np.meshgrid(x, y)
        x = x.reshape(-1, 1)
        y = y.reshape(-1, 1)
        grid_points = np.hstack((x, y))
        return grid_points

    def calculation_utility(
            self, prev_p_me, p_me, next_p_me, d_me, next_d_me,
            prev_p_you, p_you, next_p_you, d_you, next_d_you, subgoal_p):
        m_v_me, m_w_me, next_ang_me, ang_me, m_a_me = \
            self.calculation_motion_factors(
                    prev_p_me, p_me, next_p_me, d_me, next_d_me)
        m_v_you, m_w_you, next_ang_you, ang_you, m_a_you = \
            self.calculation_motion_factors(
                prev_p_you, p_you, next_p_you,
                d_you, next_d_you)
        r_d, r_a, r_v = self.calculation_relative_factors(
                next_p_me, next_p_you, next_d_me,
                next_d_you, next_ang_you)
        e_s_me = self.calculation_environmental_factors(
                subgoal_p, p_me, next_p_me)
        e_s_you = self.calculation_environmental_factors(
                subgoal_p, p_you, next_p_you)
        # utilityの計算
        f_o = 0
        f_rv = self.f(r_v, 0.2, 1.2, 0)
        f_rd = self.f(r_d, 0.25, 2.0, 0.75)
        f_ra = self.f(r_a, 0.08, 3.0, np.pi/2)
        f_s_me = self.f(e_s_me, 0.45, 1.00, 0.0)
        f_ma_me = self.f(m_a_me, 0.2, 1.0, 0.0)
        f_mv_me = self.f(m_v_me, 0.3, 1.6, 1.10)
        f_mw_me = self.f(m_w_me, 0.7, 4.4, 0.0)
        f_s_you = self.f(e_s_you, 0.45, 1.00, 0.0)
        f_ma_you = self.f(m_a_you, 0.2, 1.0, 0.0)
        f_mv_you = self.f(m_v_you, 0.3, 1.6, 1.10)
        f_mw_you = self.f(m_w_you, 0.7, 4.4, 0.0)
        utility_me = k_o * f_o + k_s * f_s_me + k_rv * f_rv + k_rd * f_rd + \
            k_ra * f_ra + k_ma * f_ma_me + k_mv * f_mv_me + k_mw * f_mw_me
        utility_you = k_o * f_o + k_s * f_s_you + k_rv * f_rv + k_rd * f_rd + \
            k_ra * f_ra + k_ma * f_ma_you + k_mv * f_mv_you + k_mw * f_mw_you
        return utility_me, utility_you

    def calculation_motion_factors(self, prev_p, p, next_p, d, next_d):
        motion_v = self.m_v(p, next_p)
        # 現在の位置と予測した位置に基づいた現在の速さ
        ang = np.arctan(d[1]/d[0])  # 現在のベクトル
        next_ang = np.arctan(
                next_d[1]/next_d[0])  # 予測した変化量に基づく次回のベクトル
        motion_w = self.m_w(ang, next_ang)  # 現在と予測による角速度
        prev_m_v = self.m_v(prev_p, p)
        motion_a = self.m_a(prev_m_v, motion_v)
        return motion_v, motion_w, next_ang, ang, motion_a

    def calculation_relative_factors(self, next_p_me, next_p_you,
                                     next_d_me, next_d_you, next_ang_you):
        # (relative factor)
        r_d = np.linalg.norm(next_p_you - next_p_me)  # socialrelativedistance
        r_a = next_ang_you - self.angb(next_p_me, next_p_you)
        # relative_angle
        r_v = np.linalg.norm((next_d_me - next_d_you) / d_t)  # relative_v
        return r_d, r_a, r_v

    def calculation_environmental_factors(self, subgoal_p, p, next_p):
        e_s = self.angb(subgoal_p, p) - self.angb(next_p, p)
        return e_s


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
    k_s = 0.2
    k_ma = 0.01
    k_mv = 0.05
    k_mw = 0.01
    subgoals_p = (4, 4)
    planner = ExtendPlanner(num_grid_x, num_grid_y,
                            search_range_x, search_range_y,
                            k_o, k_rv, k_rd, k_ra, k_s, k_ma, k_mv, k_mw, d_t)
    print(planner.decide_action(trajectory_me, trajectory_you, subgoals_p))
