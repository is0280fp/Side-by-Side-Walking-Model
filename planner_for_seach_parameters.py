# -*- coding: utf-8 -*-
"""
Created on Thu Jul 20 18:06:45 2017

@author: yume

"""

import numpy as np
import itertools
from agents_ver3 import AgentState
from states import States
from utility_visualization import utility_color_map
from factors import DistanceToObstacle
from factors import MovingTowardSubgoals
from factors import RelativeVelocity
from factors import RelativeDistance
from factors import RelativeAngle
from factors import Velocity
from factors import AngularVelocity
from factors import Acceleration


class PartnerSelfAnticipationPlanner(object):
    def __init__(self, specified_rv_k, specified_rd_k,
                 specified_ra_k, specified_mv_k,
                 specified_ma_k, specified_mav_k, specified_sgd_k,
                 specified_od_k,
                 name=None, num_grid_x=7, num_grid_y=7,
                 search_range_x=0.6, search_range_y=0.6, d_t=0.1,
                 relative_a=None, scraper=None):
        self.name = name
        self.num_grid_x = num_grid_x
        self.num_grid_y = num_grid_y
        self.search_range_x = search_range_x
        self.search_range_y = search_range_y
        self.d_t = d_t
        self.specified_rv_k = specified_rv_k
        self.specified_rd_k = specified_rd_k
        self.specified_ra_k = specified_ra_k
        self.specified_mv_k = specified_mv_k
        self.specified_ma_k = specified_ma_k
        self.specified_mav_k = specified_mav_k
        self.specified_sgd_k = specified_sgd_k
        self.specified_od_k = specified_od_k
        self.relative_a = relative_a
        self.scraper = scraper

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
            next_d_you = next_p_you - s_you.p
            states_me = States(
                    prev_s_me, s_me, AgentState(next_p_me, next_d_me))
            states_you = States(
                    prev_s_you, s_you, AgentState(next_p_you, next_d_you))
            u_me, u_you = self.calculation_utility(
                    states_me, states_you, subgoal, obstacle)
            utility_me.append(u_me)
            utility_you.append(u_you)
            utility.append(u_me + u_you)

            #                1ステップの全グリッドのutilityを持つリスト

        utility = np.array(utility)

#        utility_color_map(utility, self.num_grid_x, self.num_grid_y, "utility")

        predicted_p_you, predicted_p_me = each_other_p[utility.argmax()]
        return predicted_p_me

    def linear_extrapolation(self, trajectory):
        """位置の履歴から次の時刻の位置を等速直線運動で予測する

        Args　:
            trajectory:num_grid_x
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
            self, states_me, states_you, subgoal, obstacle):

        dto_me = DistanceToObstacle(self.scraper, a=20.0, b=0.40)
        dto_you = DistanceToObstacle(self.scraper, a=20.0, b=0.40)
        mts_me = MovingTowardSubgoals(
                self.scraper, a=0.45, b=1.00, c=0.0, d_t=0.1)
        mts_you = MovingTowardSubgoals(
                self.scraper, a=0.45, b=1.00, c=0.0, d_t=0.1)
        rv = RelativeVelocity(self.scraper, a=0.20, b=1.20, c=0.0, d_t=0.1)
        rd = RelativeDistance(self.scraper, a=0.25, b=2.00, c=1.5, d_t=0.1)
        ra = RelativeAngle(
                self.scraper, a=0.08, b=3.00, c=self.relative_a, d_t=0.1)
        v_me = Velocity(self.scraper, a=0.30, b=1.6, c=1.10, d_t=0.1)
        v_you = Velocity(self.scraper, a=0.30, b=1.6, c=1.10, d_t=0.1)
        a_me = Acceleration(self.scraper, a=0.20, b=1.0, c=0.0, d_t=0.1)
        a_you = Acceleration(self.scraper, a=0.20, b=1.0, c=0.0, d_t=0.1)
        av_me = AngularVelocity(self.scraper, a=0.7, b=4.4, c=0.0, d_t=0.1)
        av_you = AngularVelocity(self.scraper, a=0.7, b=4.4, c=0.0, d_t=0.1)

        f_o_me = dto_me.score(states_me, None, subgoal, obstacle)
        f_o_you = dto_you.score(states_me, None, subgoal, obstacle)
        f_s_me = mts_me.score(states_me, None, subgoal, obstacle)
        f_s_you = mts_you.score(states_you, None, subgoal, obstacle)
        f_rv = rv.score(states_me, states_you, subgoal, obstacle)
        f_rd = rd.score(states_me, states_you, subgoal, obstacle)
        f_ra = ra.score(states_me, states_you, subgoal, obstacle)
        f_mv_me = v_me.score(states_me, None, None, None)
        f_mv_you = v_you.score(states_you, None, None, None)
        f_ma_me = a_me.score(states_me, None, None, None)
        f_ma_you = a_you.score(states_you, None, None, None)
        f_mw_me = av_me.score(states_me, None, None, None)
        f_mw_you = av_you.score(states_you, None, None, None)

        utility_me = (self.specified_od_k * f_o_me +
                      self.specified_sgd_k * f_s_me +
                      self.specified_rv_k * f_rv +
                      self.specified_rd_k * f_rd +
                      self.specified_ra_k * f_ra +
                      self.specified_ma_k * f_ma_me +
                      self.specified_mv_k * f_mv_me +
                      self.specified_mav_k * f_mw_me)
        utility_you = (self.specified_od_k * f_o_you +
                       self.specified_sgd_k * f_s_you +
                       self.specified_rv_k * f_rv +
                       self.specified_rd_k * f_rd +
                       self.specified_ra_k * f_ra +
                       self.specified_ma_k * f_ma_you +
                       self.specified_mv_k * f_mv_you +
                       self.specified_mav_k * f_mw_you)
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
