# -*- coding: utf-8 -*-
"""
Created on Thu Jul 20 18:06:45 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
import itertools
from agents_ver3 import AgentState
from utility_visualization import utility_changing_graph
from utility_visualization import utility_color_map


class PartnerSelfAnticipationPlanner(object):
    def __init__(self, name, num_grid_x=7, num_grid_y=7,
                 search_range_x=0.6, search_range_y=0.6,
                 k_o=0.11, k_rv=0.01, k_rd=0.25, k_ra=0.32, k_s=0.2,
                 k_ma=0.01, k_mv=0.05, k_mw=0.01, k_pt=0,
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
        self.k_pt = k_pt
        self.d_t = d_t
        self.relative_a = np.deg2rad(relative_a)

    def decide_action(self, trajectory_me, trajectory_you,
                      obstacle_p, subgoal_p):
        utility_me = []
        utility_you = []
        utility = []

        s_me = trajectory_me[-1]  # 真の位置
        s_you = trajectory_you[-1]
        prev_s_me = trajectory_me[-2]  # 真の位置
        prev_s_you = trajectory_you[-2]

        grid_points_me = self.making_grid(
                self.num_grid_x, self.num_grid_y,
                self.search_range_x, self.search_range_y)
        grid_points_you = self.making_grid(
                self.num_grid_x, self.num_grid_y,
                self.search_range_x, self.search_range_y)

        # 予測位置を中心としたグリッドを作成
        next_s_me = self.linear_extrapolation(trajectory_me)
        grid_points_me = self.making_grid(
                self.num_grid_x, self.num_grid_y,
                self.search_range_x, self.search_range_y)
        # 予測位置を中心としたグリッドを作成
        next_s_you = self.linear_extrapolation(trajectory_you)
        grid_points_you = self.making_grid(
                self.num_grid_x, self.num_grid_y,
                self.search_range_x, self.search_range_y)

        grid_points_me = grid_points_me + next_s_me.p
        grid_points_you = grid_points_you + next_s_you.p

    #    for next_p_me in grid_points_me:
        each_other_p = list(itertools.product(grid_points_you, grid_points_me))
        for next_p_you, next_p_me in each_other_p:
            next_d_me = next_p_me - s_me.p
            next_s_me = AgentState(next_p_me, next_d_me)
            next_d_you = next_p_you - s_you.p
            next_s_you = AgentState(next_p_you, next_d_you)
            u_me, u_you = \
                self.calculation_utility(
                    prev_s_me, s_me, next_s_me,
                    prev_s_you, s_you, next_s_you, subgoal_p, obstacle_p)
            utility_me.append(u_me)
            utility_you.append(u_you)
            utility.append(u_me + u_you)

        utility = np.array(utility)
        utility_color_map(utility, self.num_grid_x, self.num_grid_y, "utility")

        predicted_p_you, predicted_p_me = each_other_p[utility.argmax()]
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
            self, prev_s_me, s_me, next_s_me,
            prev_s_you, s_you, next_s_you, obstacle_p, subgoal_p):
        m_v_me, m_w_me, m_a_me = \
            self.calculation_motion_factors(prev_s_me, s_me, next_s_me)
        m_v_you, m_w_you, m_a_you = \
            self.calculation_motion_factors(prev_s_you, s_you, next_s_you)
        r_d, r_a, r_v = self.calculation_relative_factors(
                next_s_me, next_s_you)
        e_s_me, e_o_me = self.calculation_environmental_factors(
                subgoal_p, obstacle_p, s_me, next_s_me)
        e_s_you, e_o_you = self.calculation_environmental_factors(
                subgoal_p, obstacle_p, s_you, next_s_you)
#        pt_me = self.calculation_new_factors()
        pt_me = 0
        # utilityの計算
        f_o = self.f_o(e_o_me, 20, 0.4, 0)
        f_rv = self.f(r_v, 0.2, 1.2, 0)
        f_rd = self.f(r_d, 0.25, 2.0, 0.75)
        f_ra = self.f(r_a, 0.08, 3.0, self.relative_a)
        f_s_me = self.f(e_s_me, 0.45, 1.00, 0.0)
        f_ma_me = self.f(m_a_me, 0.2, 1.0, 0.0)
        f_mv_me = self.f(m_v_me, 0.3, 1.6, 1.10)
        f_mw_me = self.f(m_w_me, 0.7, 4.4, 0.0)
        f_s_you = self.f(e_s_you, 0.45, 1.00, 0.0)
        f_ma_you = self.f(m_a_you, 0.2, 1.0, 0.0)
        f_mv_you = self.f(m_v_you, 0.3, 1.6, 1.10)
        f_mw_you = self.f(m_w_you, 0.7, 4.4, 0.0)
        f_pt_me = self.f(pt_me)
        utility_me = (self.k_o * f_o + self.k_s * f_s_me + self.k_rv * f_rv + \
                      self.k_rd * f_rd + self.k_ra * f_ra + \
                      self.k_ma * f_ma_me + self.k_mv * f_mv_me + \
                      self.k_mw * f_mw_me + self.k_pt * f_pt_me)
        utility_you = (self.k_o * f_o + self.k_s * f_s_you + \
                       self.k_rv * f_rv + self.k_rd * f_rd + \
            self.k_ra * f_ra + self.k_ma * f_ma_you + self.k_mv * f_mv_you + \
            self.k_mw * f_mw_you)
        return utility_me, utility_you

    def calculation_new_factors(self):
        pass

    def calculation_motion_factors(self, prev_s, s, next_s):
        motion_v = self.m_v(s, next_s)
        # 現在の位置と予測した位置に基づいた現在の速さ
        motion_w = self.m_w(prev_s, s, next_s)  # 現在と予測による角速度
        prev_m_v = self.m_v(prev_s, s)
        motion_a = self.m_a(prev_m_v, motion_v)
        return motion_v, motion_w, motion_a

    def calculation_relative_factors(self, s_me, s_you):
        # (relative factor)
        p_you = s_you.p
        p_me = s_me.p
        d_you = s_you.d
        d_me = s_me.d
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
    k_pt = 0
    subgoals_p = (4, 4)
    obstacles_p = (3, 3)
    planner = PartnerSelfAnticipationPlanner(
            num_grid_x, num_grid_y, search_range_x,
            search_range_y, k_o, k_rv, k_rd,
            k_ra, k_s, k_ma, k_mv, k_mw, k_pt, d_t)
    print(planner.decide_action(
            trajectory_me, trajectory_you, subgoals_p, obstacles_p))
