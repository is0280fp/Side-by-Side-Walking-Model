# -*- coding: utf-8 -*-
"""
Created on Fri Jul 14 21:56:53 2017

@author: yume
"""

import copy
import matplotlib.pyplot as plt
import numpy as np
import partner_and_self_anticipation_planner
from agents_ver3 import Robot
from states import AgentState
from agents_ver3 import Human
from environments import EnvironmentState


class Logger(object):
    def __init__(self, length_step):
        self.l_s = []  # 現在位置を格納するリスト
        self.f_s = []  # 現在位置を格納するリスト
        self.length_step = length_step

    def log_leader(self, s):
        self.l_s.append(copy.copy(s))

    def log_follower(self, s):
        self.f_s.append(copy.copy(s))

    def display(self):
        l_p = np.array([s.p for s in self.l_s])
        f_p = np.array([s.p for s in self.f_s])
        relative_distance = np.sqrt((l_p - f_p) * 2)
        plt.plot(*l_p.T, "-o", label="Robot")
        plt.plot(*f_p.T, "*", label="Human")
#        plt.plot(self.subgoals.T, "^", label="Goal")
#        plt.plot(self.obstacles_p.T[0], self.obstacles_p.T[1], "^",
#                 label="obstacle")
        print("relative_distance", relative_distance[-1])
        plt.xticks(np.arange(-0, 2, 0.2))
        plt.yticks(np.arange(-1.0, 1, 0.2))  # 表の軸を0~20に固定
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


def make_environment(ps):
    ps = np.array(ps)
    environmental_map = []
    for p in ps[0:]:
        state = EnvironmentState(p)
        environmental_map.append(state)
    return environmental_map


if __name__ == '__main__':
    # 表描画
    trajectory_a = make_trajectory([
            [1.49423710358, -0.76229029302],
            [1.46490391443, -0.736084754334],
            [1.42321161195, -0.77479323992],
            [1.36483836463, -0.6274773265]
            ])
    trajectory_b = make_trajectory([
            [1.73578850047, -0.99751806081],
            [1.74111587829, -0.937682491898],
            [1.58094249456, -0.831414990608],
            [1.46679422611, -0.745349506114]
            ])
    subgoals = make_environment([
            [0.1, 0.3]
            ])

    d_t = 0.1
    num_grid_x = 7
    num_grid_y = 7
    search_range_x = 0.05
    search_range_y = 0.05

    k_o = 0
    k_rv = 0.01
    k_rd = 0.5
    k_ra = 0.9  # ra = relative_angle
    k_s = 0.5
    k_ma = 0.01
    k_mv = 0.05
    k_mw = 0.01
    k_pt = 0  # 新しいfactor
    length_step = 40
    relative_angle_a = 0
    relative_angle_b = 180 - relative_angle_a

    n = 0
    initial_state_a = trajectory_a[-1]
    initial_state_b = trajectory_b[-1]
#    aさんはwalking_modelに基づいて，経路計画する
#   bさんはutilityに基づいて，経路計画する
    planner_a = \
        partner_and_self_anticipation_planner.PartnerSelfAnticipationPlanner(
                "a", num_grid_x, num_grid_y, search_range_x, search_range_y,
                k_o, k_rv, k_rd, k_ra, k_s, k_ma, k_mv, k_mw, k_pt, d_t,
                relative_angle_a)
    human_a = Robot(
        subgoals, initial_state_a, planner_a, d_t, trajectory_a, trajectory_b)

    human_b = Human(subgoals, initial_state_b, d_t, trajectory_b, trajectory_a)

    logger = Logger(length_step)

    logger.log_leader(human_a.s)
    logger.log_follower(human_b.s)

#        print("length_step", length_step)
#        print("n", n)
    while n < length_step:

        human_a.measure(human_a.s, human_b.s, [3, 2])
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
