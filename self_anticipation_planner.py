# -*- coding: utf-8 -*-
"""
Created on Thu Jul 20 18:06:45 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


class StandardPlanner(object):
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
        utility = []

        p_me = trajectory_me[-1]  # 真の位置
        p_you = trajectory_you[-1]
        prev_p_me = trajectory_me[-2]  # 真の位置
        prev_p_you = trajectory_you[-2]
        next_p_you, d_you = self.linear_extrapolation(
            trajectory_you)  # 予測した次の位置と変化量
        next_d_you = next_p_you - p_you  # 予測した次の変化量

        grid_points = self.making_grid(
                self.num_grid_x, self.num_grid_y,
                self.search_range_x, self.search_range_y)
        # 予測位置を中心としたグリッドを作成
        midle_point, d_me = \
            self.linear_extrapolation(trajectory_me)
        grid_points = grid_points + midle_point
        m_v_you, m_w_you, next_ang_you, ang_you, m_a_you = \
            self.calculation_motion_factors(prev_p_you, p_you,
                                            next_p_you, d_you, next_d_you)

        for next_p_me in grid_points:
            next_d_me = next_p_me - p_me
            u = self.calculation_utility(
                    prev_p_me, p_me, next_p_me, d_me, next_d_me, next_p_you,
                    next_d_you, next_ang_you, subgoal_p)
            utility.append(u)

        utility = np.array(utility).reshape(self.num_grid_y, self.num_grid_x)
        predicted_p_me = grid_points[utility.argmax()]
        plt.matshow(utility)
        plt.gca().invert_yaxis()
        plt.colorbar()
        plt.show()
        return predicted_p_me

    def linear_extrapolation(self, trajectory):
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

    def m_v(self, p, next_p, d_t):
        """eq. (1)
        """
        m_v = np.sqrt(np.sum((next_p - p) ** 2)) / d_t
        return m_v

    def m_w(self, ang, next_ang, d_t):
        """eq. (2)
        """
        return (next_ang - ang) / d_t

    def m_a(self, v, next_v, d_t):
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

    def making_grid(
            self, num_grid_x, num_grid_y, search_range_x, search_range_y):
        # trajectoryの基の7*7grid作る(中心を(0,0)とする)
        x = np.linspace(-search_range_x, search_range_x, num_grid_x)
        y = np.linspace(-search_range_y, search_range_y, num_grid_y)
        x, y = np.meshgrid(x, y)
        x = x.reshape(-1, 1)
        y = y.reshape(-1, 1)
        grid_points = np.hstack((x, y))
        return grid_points

    def calculation_utility(
            self, prev_p_me, p_me, next_p_me, d_me, next_d_me, next_p_you,
            next_d_you, next_ang_you, subgoal_p):
        m_v_me, m_w_me, next_ang_me, ang_me, m_a_me = \
                self.calculation_motion_factors(
                        prev_p_me, p_me, next_p_me,
                        d_me, next_d_me)
        r_d, r_a, r_v = self.calculation_relative_factors(
                next_p_me, next_p_you, next_d_me,
                next_d_you, next_ang_you)
        e_s_me = self.calculation_environmental_factors(
                subgoal_p, p_me, next_p_me)
        # utilityの計算
        f_o = 0
        f_rv = self.f(r_v, 0.2, 1.2, 0)
        f_rd = self.f(r_d, 0.25, 2.0, 0.75)
        f_ra = self.f(r_a, 0.08, 3.0, np.pi/2)
        f_s = self.f(e_s_me, 0.45, 1.00, 0.0)
        f_ma = self.f(m_a_me, 0.2, 1.0, 0.0)
        f_mv = self.f(m_v_me, 0.3, 1.6, 1.10)
        f_mw = self.f(m_w_me, 0.7, 4.4, 0.0)
        utility = (self.k_o * f_o + self.k_s * f_s +
                   self.k_rv * f_rv + self.k_rd * f_rd + self.k_ra * f_ra +
                   self.k_ma * f_ma + self.k_mv * f_mv + self.k_mw * f_mw)
        return utility

    def calculation_motion_factors(self, prev_p, p, next_p, d, next_d):
        motion_v = self.m_v(p, next_p, self.d_t)
        # 現在の位置と予測した位置に基づいた現在の速さ
        ang = np.arctan(d[1]/d[0])  # 現在のベクトル
        next_ang = np.arctan(
                next_d[1]/next_d[0])  # 予測した変化量に基づく次回のベクトル
        motion_w = self.m_w(ang, next_ang, self.d_t)  # 現在と予測による角速度
        prev_m_v = self.m_v(prev_p, p, self.d_t)
        motion_a = self.m_a(prev_m_v, motion_v, self.d_t)
        return motion_v, motion_w, next_ang, ang, motion_a

    def calculation_relative_factors(self, next_p_me, next_p_you,
                                     next_d_me, next_d_you, next_ang_you):
        # (relative factor)
        r_d = np.linalg.norm(next_p_you - next_p_me)  # socialrelativedistance
        r_a = next_ang_you - self.angb(next_p_me, next_p_you)
        # relative_angle
        r_v = np.linalg.norm((next_d_me - next_d_you) / self.d_t)  # relative_v
        return r_d, r_a, r_v

    def calculation_environmental_factors(self, subgoal_p, p, next_p):
        e_s = self.angb(subgoal_p, p) - \
            self.angb(next_p, p)
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
    planner = StandardPlanner(num_grid_x, num_grid_y, search_range_x,
                              search_range_y, k_o, k_rv, k_rd,
                              k_ra, k_s, k_ma, k_mv, k_mw, d_t)
    print(planner.decide_action(trajectory_me, trajectory_you, subgoals_p))
