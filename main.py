# -*- coding: utf-8 -*-
"""
Created on Fri Jul 14 21:56:53 2017

@author: yume
"""

# -*- coding: utf-8 -*-
"""
Created on Wed May 24 16:09:19 2017

@author: yume
"""

import copy
import matplotlib.pyplot as plt
import numpy as np
import extend_self_anticipation_planner
from agents import AgentState
from agents import Human


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
        plt.plot(*l_p.T, "-*", label="a")
        plt.plot(*f_p.T, "-o", label="b")
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
#    trajectory_a = np.array([[1, 1],
#                             [1.03, 1.03],
#                             [1.06, 1.06],
#                             [1.09, 1.09]])
#    trajectory_a = [AgentState((1, 1)),
#                    AgentState((1.03, 1.03)),
#                    AgentState((1.06, 1.06)),
#                    AgentState((1.09, 1.09))]
    trajectory_a = make_trajectory([[0.91, 0.91],
                                    [0.94, 0.94],
                                    [0.97, 0.97],
                                    [1, 1]])
    trajectory_b = make_trajectory([[1.41, 0.41],
                                    [1.44, 0.44],
                                    [1.47, 0.47],
                                    [1.5, 0.5]])
#    trajectory_b = np.array([[1.5, 0.5],
#                             [1.53, 0.53],
#                             [1.56, 0.56],
#                             [1.59, 0.59]])
#    trajectory_b = [AgentState((1.5, 0.5)),
#                    AgentState((1.53, 0.53)),
#                    AgentState((1.56, 0.56)),
#                    AgentState((1.59, 0.59))]
    d_t = 0.03
    k_o = 0.11
    k_rv = 0.01
    k_rd = 0.25
    num_grid_x = 7
    num_grid_y = 7
    search_range_x = 0.6
    search_range_y = 0.6
    k_ra = 0.32  # ra = relative_angle
    k_s = 0.2
    k_ma = 0.01
    k_mv = 0.05
    k_mw = 0.01
    subgoals_p = [(4, 3.5)]
    length_step = 100
    n = 0
#    planner = extend_planner.ExtendPlanner(
#        num_grid_x, num_grid_y, search_range_x, search_range_y,
#        k_o, k_rv, k_rd, k_ra, k_s, k_ma, k_mv, k_mw, d_t)
    planner_a = extend_self_anticipation_planner.ExtendSelfAnticipationPlanner(
        num_grid_x, num_grid_y, search_range_x, search_range_y,
        k_o, k_rv, k_rd, k_ra, k_s, k_ma, k_mv, k_mw, d_t)
    planner_b = extend_self_anticipation_planner.ExtendSelfAnticipationPlanner(
        num_grid_x, num_grid_y, search_range_x, search_range_y,
        k_o, k_rv, k_rd, k_ra, k_s, k_ma, k_mv, k_mw, d_t)
    human_a = Human(
            trajectory_a, trajectory_b, subgoals_p, planner_a)
    human_b = Human(
            trajectory_b, trajectory_a, subgoals_p, planner_b)
    logger = Logger(length_step)

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

        n += 1  # インクリメント
#    logger.display()
