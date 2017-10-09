# -*- coding: utf-8 -*-
"""
Created on Thu Jul 20 18:06:45 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
from agents_ver2 import AgentState


class ExtendSelfAnticipationPlanner(object):
    def __init__(self, name, num_grid_x=7, num_grid_y=7,
                 search_range_x=0.6, search_range_y=0.6,
                 k_o=0.11, k_rv=0.01, k_rd=0.25, k_ra=0.32, k_s=0.2,
                 k_ma=0.01, k_mv=0.05, k_mw=0.01, k_cv=0,
                 d_t=0.03, relative_a=None):
        self.name = name
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
        self.k_cv = k_cv
        self.d_t = d_t
        self.relative_a = np.deg2rad(relative_a)

    def decide_action(self, trajectory_me, trajectory_you,
                      subgoal_p, obstacle_p, optimum_velocity):
        utility = []

        s_me = trajectory_me[-1]  # 真の位置
#        s_you = trajectory_you[-1]
        prev_s_me = trajectory_me[-2]  # 真の位置
#        prev_s_you = trajectory_you[-2]
        # 予測した次の位置と変化量
        next_s_me = self.linear_extrapolation(trajectory_me)
        next_s_you = self.linear_extrapolation(trajectory_you)

        # 予測位置を中心としたグリッドを作成
        grid_points = self.making_grid(
                self.num_grid_x, self.num_grid_y,
                self.search_range_x, self.search_range_y)

        grid_points = grid_points + next_s_me.p
#        m_v_you, m_w_you, m_a_you = \
#            self.calculation_motion_factors(prev_s_you, s_you, next_s_you)

        for next_p_me in grid_points:
            next_d_me = next_p_me - s_me.p
            next_s_me = AgentState(next_p_me, next_d_me)
            u = self.calculation_utility(
                    prev_s_me, s_me, next_s_me, next_s_you,
                    subgoal_p, obstacle_p, optimum_velocity)
            utility.append(u)
#            relative_angle.append(r_a)

        utility = np.array(utility).reshape(self.num_grid_y, self.num_grid_x)
#        relative_angle = np.array(relative_angle).reshape(
#               self.num_grid_y, self.num_grid_x)
        predicted_p_me = grid_points[utility.argmax()]
#        predicted_r_a = relative_angle.argmax()
#        print("relative_angle", predicted_r_a)
        plt.matshow(utility)
        plt.title(self.name)
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
        temp = trajectory[-1].p - trajectory[-2].p
        next_p = trajectory[-1].p + temp
        temp = trajectory[-1].d - trajectory[-2].d
        next_d = trajectory[-1].d + temp
        next_d = next_d / np.linalg.norm(next_d)
        return AgentState(next_p, next_d)

    def m_v(self, s, next_s):
        """eq. (1)
        """
        p = s.p
        next_p = next_s.p
        m_v = np.sqrt(np.sum((next_p - p) ** 2)) / self.d_t
        return m_v

    def m_w(self, prev_s, s, next_s):
        """eq. (2)
        """
        p = s.p
        prev_p = prev_s.p
        next_p = next_s.p
        ang = self.angb(p, prev_p)       # 時刻tのd_t(direction=向き)
        next_ang = self.angb(next_p, p)  # 時刻tのd_t+1(direction=向き)
        return (next_ang - ang) / self.d_t

    def m_a(self, v, next_v):
        """eq. (3)
        """
        return (next_v - v) / self.d_t

    def angb(self, p1, p2):
        # p2からみたp1の相対位置ベクトルの絶対角度
        d = np.arctan2(p1[1] - p2[1], p1[0] - p2[0])
        return d

    def revision_theta(self, theta):
        theta += np.pi
        theta %= 2 * np.pi
        if theta < 0:
            theta += np.pi
        else:
            theta -= np.pi
        r_a = theta
        return r_a

    def relative_angle(self, s_me, s_you):
        p_me = s_me.p
        p_you = s_you.p
        d_you = s_you.d
        # youの進行方向の絶対角度
        theta_mae = np.arctan2(d_you[1], d_you[0])
        theta_yoko = self.angb(p_me, p_you)
        theta = theta_yoko - theta_mae
        r_a = self.revision_theta(theta)
        return np.abs(r_a)

    def distance_to_obstacle(self, s, obstacle_p):
        p = s.p
        distance = np.sqrt(np.sum((obstacle_p - p) ** 2))
        return distance

    def f(self, x, a=0.25, b=2.00, c=0.75):
        """
        eq. (9)
        """
        f_x = 1 / (1+(abs((x-c) / a)**(2*b))) - 1
        return f_x

    def f_o(self, x, a, b, c):
        f_o = - np.abs((a / x) ** (2*b))
        return f_o

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
            self, prev_s_me, s_me, next_s_me, next_s_you,
            subgoal_p, obstacle_p, optimum_velocity):
        m_v_me, m_w_me, m_a_me = \
                self.calculation_motion_factors(prev_s_me, s_me, next_s_me)
        r_d, r_a, r_v = self.calculation_relative_factors(
                next_s_me, next_s_you)
        e_s_me, e_o_me = self.calculation_environmental_factors(
                subgoal_p, obstacle_p, s_me, next_s_me)
        c_v = self.calculation_control_factors(
                s_me, next_s_me, optimum_velocity)
        # utilityの計算
        f_o = self.f_o(e_o_me, 20, 0.4, 0)
        f_rv = self.f(r_v, 0.2, 1.2, 0)
        f_rd = self.f(r_d, 0.25, 2.0, 1.5)
        f_ra = self.f(r_a, 0.08, 3.0, self.relative_a)
        f_s = self.f(e_s_me, 0.45, 1.00, 0.0)
        f_ma = self.f(m_a_me, 0.2, 1.0, 0.0)
        f_mv = self.f(m_v_me, 0.3, 1.6, 1.10)
        f_mw = self.f(m_w_me, 0.7, 4.4, 0.0)
        f_cv = self.f(c_v, 0.08, 2, 0)
        utility = (self.k_o * f_o + self.k_s * f_s +
                   self.k_rv * f_rv + self.k_rd * f_rd + self.k_ra * f_ra +
                   self.k_ma * f_ma + self.k_mv * f_mv + self.k_mw * f_mw +
                   self.k_cv * f_cv)
        return utility

    def calculation_motion_factors(self, prev_s, s, next_s):
        motion_v = self.m_v(s, next_s)
        # 現在の位置と予測した位置に基づいた現在の速さ
#        ang = np.arctan(d[1]/d[0])  # 現在のベクトル
#        next_ang = np.arctan(
#                next_d[1]/next_d[0])  # 予測した変化量に基づく次回のベクトル
        motion_w = self.m_w(prev_s, s, next_s)  # 現在と予測による角速度
        prev_m_v = self.m_v(prev_s, s)
        motion_a = self.m_a(prev_m_v, motion_v)
        return motion_v, motion_w, motion_a

    def calculation_relative_factors(self, s_me, s_you):
        p_you = s_you.p
        p_me = s_me.p
        d_you = s_you.d
        d_me = s_me.d
        # (relative factor)
        r_d = np.linalg.norm(p_you - p_me)  # socialrelativedistance
        r_a = self.relative_angle(s_me, s_you)
        # relative_angle
        r_v = np.linalg.norm((d_me - d_you) / self.d_t)  # relative_v
        return r_d, r_a, r_v

    def calculation_environmental_factors(
            self, subgoal_p, obstacle_p, s, next_s):
        p = s.p
        next_p = next_s.p
        e_s = self.angb(subgoal_p, p) - \
            self.angb(next_p, p)
        e_o = self.distance_to_obstacle(next_s, obstacle_p)
        return e_s, e_o

    def calculation_control_factors(self, s, next_s, optimum_velocity):
        motion_v = self.m_v(s, next_s)
        difference = optimum_velocity - motion_v  # スカラー - スカラー
        return difference  # ベクトル


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
    k_cv = 0
    subgoals_p = (4, 4)
    obstacles_p = (3, 3)
    planner = ExtendSelfAnticipationPlanner(
            num_grid_x, num_grid_y, search_range_x,
            search_range_y, k_o, k_rv, k_rd,
            k_ra, k_s, k_ma, k_mv, k_mw, k_cv, d_t)
    print(planner.decide_action(trajectory_me, trajectory_you,
                                subgoals_p, obstacles_p))
