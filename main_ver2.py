# -*- coding: utf-8 -*-
"""
Created on Fri Jul 14 21:56:53 2017

@author: yume
"""

import copy
import matplotlib.pyplot as plt
import numpy as np
# import self_anticipation_planner
import partner_and_self_anticipation_planner
from agents_ver2 import AgentState
from agents_ver2 import Human


class Logger(object):
    def __init__(self, length_step, subgoals_p, obstacles_p):
        self.l_s = []  # 現在位置を格納するリスト
        self.f_s = []  # 現在位置を格納するリスト
        self.length_step = length_step
        self.subgoals_p = np.array(subgoals_p)
        self.obstacles_p = np.array(obstacles_p)

    def log_leader(self, s):
        self.l_s.append(copy.copy(s))

    def log_follower(self, s):
        self.f_s.append(copy.copy(s))

    def display(self):
        l_p = np.array([s.p for s in self.l_s])
        f_p = np.array([s.p for s in self.f_s])
        plt.plot(*l_p.T, "-*", label="a")
        plt.plot(*f_p.T, "-o", label="b")
        plt.plot(self.subgoals_p.T[0], self.subgoals_p.T[1], "x",
                 label="subgoal")
        plt.plot(self.obstacles_p.T[0], self.obstacles_p.T[1], "^",
                 label="obstacle")
        plt.xlim(0, 5)  # 表の軸を0~20に固定
        plt.ylim(0, 5)  # 表の軸を0~20に固定
        plt.grid()
        plt.legend()
        plt.gca().set_aspect('equal')
        plt.show()
#        print("leader.p", self.l_p[-1])
#        print("follower.p", self.f_p[-1])

    def savefig(self, filename):
        plt.savefig(filename)
        self.display()
        plt.draw()

    def print(self):
        print("a_s")
        print("\n".join(
                ["{}: {}".format(i, s) for i, s in enumerate(logger.l_s)]))
#  print("\n".join([str(i) + ": " + str(s) for i, s in enumerate(logger.l_s)]))
        print()
        print("b_s")
        print("\n".join(
                ["{}: {}".format(i, s) for i, s in enumerate(logger.f_s)]))
        print()
        print()


def make_trajectory(ps):
    ps = np.array(ps)
    trajectory = []
    prev_p = ps[0]
    state = AgentState(prev_p)
    trajectory.append(state)
    for p in ps[1:]:
        d = p - prev_p
        state = AgentState(p, d)
        trajectory.append(state)
        prev_p = p
    return trajectory


if __name__ == '__main__':
    # 表描画
    goal_x = 11
    relative_pos = 2
    l_v_max = 3
    f_v_max = 2

    trajectory_a = make_trajectory([[0.91, 0.91],
                                    [0.94, 0.94],
                                    [0.97, 0.97],
                                    [1, 1]])
    trajectory_b = make_trajectory([[1.41, 0.41],
                                    [1.44, 0.44],
                                    [1.47, 0.47],
                                    [1.5, 0.5]])
#    trajectory_a = make_trajectory([[2, 2],
#                                    [2.03, 2.03],
#                                    [2.06, 2.06],
#                                    [2.09, 2.09]])
#    trajectory_b = make_trajectory([[1, 1],
#                                    [1.03, 1.03],
#                                    [1.06, 1.06],
#                                    [1.09, 1.09]])

    d_t = 0.03
    num_grid_x = 7
    num_grid_y = 7
    search_range_x = 0.6
    search_range_y = 0.6
    k_o = 0.11
    k_rv = 0.01
    k_rd = 0.25
    k_ra = 0.32  # ra = relative_angle
    k_s = 0.20
    k_ma = 0.01
    k_mv = 0.05
    k_mw = 0.01
    k_cv = 0  # 実験には使わない
    subgoals_p = [(4, 3.5)]
    obstacles_p = [(3, 2)]
    optimum_velocity = 0.03
    length_step = 10
    relative_angle_a = 0
    relative_angle_b = 180 - relative_angle_a
    n = 0

#    planner = extend_planner.ExtendPlanner(
#        num_grid_x, num_grid_y, search_range_x, search_range_y,
#        k_o, k_rv, k_rd, k_ra, k_s, k_ma, k_mv, k_mw, d_t)
#    aさんはwalking_modelに基づいて，経路計画する
    planner_a = \
        partner_and_self_anticipation_planner.PartnerSelfAnticipationPlanner(
                "a", num_grid_x, num_grid_y, search_range_x, search_range_y,
                k_o, k_rv, k_rd, k_ra, k_s, k_ma, k_mv, k_mw, d_t,
                relative_angle_a)
#   bさんはutilityに基づいて，経路計画する
    planner_b = \
        partner_and_self_anticipation_planner.PartnerSelfAnticipationPlanner(
                "b", num_grid_x, num_grid_y, search_range_x, search_range_y,
                k_o, k_rv, k_rd, k_ra, k_s, k_ma, k_mv, k_mw, d_t,
                relative_angle_b)
    human_a = Human(
        trajectory_a, trajectory_b, subgoals_p,
        obstacles_p, optimum_velocity, planner_a)
    human_b = Human(
        trajectory_b, trajectory_a, subgoals_p,
        obstacles_p, optimum_velocity, planner_b)
    logger = Logger(length_step, subgoals_p, obstacles_p)

    logger.log_leader(human_a.s)
    logger.log_follower(human_b.s)

#        print("length_step", length_step)
#        print("n", n)
    while n < length_step:

        human_a.measure(human_a.s, human_b.s)
        human_b.measure(human_b.s, human_a.s)

        human_a.decide_action()
        human_b.decide_action()

        human_a.move()
        human_b.move()

        logger.log_leader(human_a.s)
        logger.log_follower(human_b.s)

        print("step", n)
        logger.display()
        logger.print()
        print("==================================================================================")

        n += 1  # インクリメント
#    logger.display()
