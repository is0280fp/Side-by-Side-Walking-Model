# -*- coding: utf-8 -*-
"""
Created on Thu Jul 20 18:06:45 2017

@author: yume
"""

import numpy as np
import itertools
from agents_ver3 import AgentState
from utility_visualization import utility_changing_graph
from utility_visualization import utility_color_map
from utility import DistanceToObstacle
from utility import MovingTowardSubgoals
from utility import RelativeVelocity
from utility import RelativeDistance
from utility import RelativeAngle
from utility import Velocity
from utility import AngularVelocity
from utility import Acceleration


class PartnerSelfAnticipationPlanner(object):
    def __init__(self, name=None, num_grid_x=7, num_grid_y=7,
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
                      subgoals, obstacles):
        utility_me = []
        utility_you = []
        utility = []

        s_me = trajectory_me[-1]  # 真の位置
        s_you = trajectory_you[-1]
        prev_s_me = trajectory_me[-2]  # 真の位置
        prev_s_you = trajectory_you[-2]
        subgoal = subgoals
        obstacle = obstacles

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
                    prev_s_you, s_you, next_s_you, subgoal, obstacle)
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
            prev_s_you, s_you, next_s_you, subgoal, obstacle):
        dto_me = DistanceToObstacle(a=20.0, b=0.40)
        dto_you = DistanceToObstacle(a=20.0, b=0.40)
        mts_me = MovingTowardSubgoals(a=0.45, b=1.00, c=0.0)
        mts_you = MovingTowardSubgoals(a=0.45, b=1.00, c=0.0)
        rv = RelativeVelocity(a=0.20, b=1.20, c=0.0)
        rd = RelativeDistance(a=0.25, b=2.00, c=0.75)
        ra = RelativeAngle(a=0.08, b=3.00, c=self.relative_a)
        v_me = Velocity(a=0.30, b=1.6, c=1.10)
        v_you = Velocity(a=0.30, b=1.6, c=1.10)
        a_me = Acceleration(a=0.20, b=1.0, c=0.0)
        a_you = Acceleration(a=0.20, b=1.0, c=0.0)
        av_me = AngularVelocity(a=0.7, b=4.4, c=0.0)
        av_you = AngularVelocity(a=0.7, b=4.4, c=0.0)

        f_o_me = dto_me.calculation_f_o_utility(s_me, obstacle)
        f_o_you = dto_you.calculation_f_o_utility(s_you, obstacle)
        f_s_me = mts_me.calculation_f_s_utility(s_me, next_s_me, subgoal)
        f_s_you = mts_you.calculation_f_s_utility(s_you, next_s_you, subgoal)
        f_rv = rv.calculation_f_rv_utility(s_me, s_you, self.d_t)
        f_rd = rd.calculation_f_rd_utility(s_me, s_you, self.d_t)
        f_ra = ra.calculation_f_ra_utility(s_me, s_you)
        f_mv_me = v_me.calculation_f_mv_utility(s_me, next_s_me, self.d_t)
        f_mv_you = v_you.calculation_f_mv_utility(s_you, next_s_you, self.d_t)
        f_ma_me = a_me.calculation_f_ma_utility(prev_s_me, s_me, next_s_me, self.d_t)
        f_ma_you = a_you.calculation_f_ma_utility(prev_s_you, s_you, next_s_you, self.d_t)
        f_mw_me = av_me.calculation_f_mw_utility(prev_s_me, s_me, next_s_me, self.d_t)
        f_mw_you = av_you.calculation_f_mw_utility(prev_s_you, s_you, next_s_you, self.d_t)

        utility_me = (self.k_o * f_o_me + self.k_s * f_s_me +
                      self.k_rv * f_rv + self.k_rd * f_rd + self.k_ra * f_ra +
                      self.k_ma * f_ma_me + self.k_mv * f_mv_me + self.k_mw * f_mw_me)
        utility_you = (self.k_o * f_o_you + self.k_s * f_s_you +
                       self.k_rv * f_rv + self.k_rd * f_rd +
                       self.k_ra * f_ra + self.k_ma * f_ma_you + self.k_mv * f_mv_you + self.k_mw * f_mw_you)
        return utility_me, utility_you


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
