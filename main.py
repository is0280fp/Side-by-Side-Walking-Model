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

import matplotlib.pyplot as plt
import numpy as np
import self_anticipation_planner


class Agent(object):
    def __init__(self, initial_p, initial_ang, subgoal_p, planner, d_t=0.03):
        self.p = initial_p
        self.ang = initial_ang
        self.subgoals_p = subgoals_p
        self.d_t = d_t
        self.trajectory_me = []
        self.trajectory_you = []
        self.planner = planner

    def measure(self, p_me, p_you):
        self.trajectory_me.append(p_me)
        self.trajectory_you.append(p_you)

    def estimate(self):
        pass

    def decide_action(self):
        raise   NotImplementedError(
        "You have to extend Agentclass if you want to use this module")

    def move(self):
        self.p = self.p + self.v


# Leader
# plotする範囲を指定、plot数も指定
class Human(Agent):

    def _init_(self, initial_p, initial_ang, subgoal_p, planner):
        super(Human, self).__init__(initial_p, initial_ang, subgoal_p, planner)

    def decide_action(self):
        next_p = self.planner.decide_action(
                self.trajectory_me, self.trajectory_you, self.subgoals_p)
        current_p = self.trajectory_me[-1]
        self.v = next_p - current_p


class Logger(object):
    def __init__(self, length_step):
        self.l_p = []  # 現在位置を格納するリスト
        self.f_p = []  # 現在位置を格納するリスト
        self.length_step = length_step

    def log_leader(self, p):
        self.l_p.append(p)

    def log_follower(self, p):
        self.f_p.append(p)

    def display(self):
        plt.plot(self.l_p, "-*")
        plt.plot(self.f_p, "o")
        plt.xlim(0, self.length_step)  # 表の軸を0~20に固定
        plt.grid()
        plt.gcf()
        plt.show()
        print("leader.p", self.l_p[-1])
        print("follower.p", self.f_p[-1])

    def savefig(self, filename):
        plt.savefig(filename)
        self.display()
        plt.draw()


if __name__ == '__main__':
    # 表描画
    goal_x = 11
    relative_pos = 2
    l_v_max = 3
    f_v_max = 2
    human_a_initial_p = 0
    human_b_initial_p = 0
    initial_ang = 0
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
    length_step = 30
    n = 0
    planner = self_anticipation_planner.StandardPlanner(
        num_grid_x, num_grid_y, search_range_x, search_range_y,
        k_o, k_rv, k_rd, k_ra, k_s, k_ma, k_mv, k_mw, d_t)
    human_a = Human(human_a_initial_p, initial_ang, subgoals_p, planner)
    human_b = Human(human_b_initial_p, initial_ang, subgoals_p, planner)
    logger = Logger(length_step)

    logger.log_leader(human_a.p)
    logger.log_follower(human_b.p)

#        print("length_step", length_step)
#        print("n", n)
    while n < length_step:

        human_a.measure(human_b.p, human_a.p)
        human_b.measure(human_a.p, human_a.p)

        human_a.decide_action()
        human_b.decide_action()

        human_a.move()
        human_b.move()

        logger.log_leader(human_a.p)
        logger.log_follower(human_b.p)

        logger.display()

        n += 1  # インクリメント
#    logger.display()
