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
import planner


# Leader
# plotする範囲を指定、plot数も指定
class Leader(object):
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
        next_p = self.planner.decide_action(
                trajectory_me, trajectory_you, num_grid_x, num_grid_y,
                search_range_x, search_range_y, k_o, k_rv, k_rd, k_ra)
        current_p = self.trajectory_me[-1]
        self.v = next_p - current_p

    def move(self):
        self.p = self.p + self.v


# Follower
# plotする範囲を指定、plot数も指定
class Follower(object):
    def __init__(self, initial_p, initial_ang, subgoal_p, planner, d_t=0.03):
        self.p = initial_p
        self.ang = initial_ang
        self.subgoals_p = subgoals_p
        self.d_t = d_t
        self.trajectory_you = []
        self.planner = planner

    def measure(self, p_you):
        self.trajectory_you.append(p_you)

    def estimate(self):
        pass

    def decide_action(self):


    def move(self):
        self.x = self.x + self.v_x


class Logger(object):
    def __init__(self, length_step):
        self.l_x = []  # 現在位置を格納するリスト
        self.f_x = []  # 現在位置を格納するリスト
        self.length_step = length_step

    def log_leader(self, x):
        self.l_x.append(x)

    def log_follower(self, x):
        self.f_x.append(x)

    def display(self):
        plt.plot(self.l_x, "-*")
        plt.plot(self.f_x, "o")
        plt.xlim(0, self.length_step)  # 表の軸を0~20に固定
        plt.grid()
        plt.gcf()
        plt.show()
        print("leader.x", self.l_x[-1])
        print("follower.x", self.f_x[-1])

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
    l_initial_pos = 0
    f_initial_pos = 0
    num_grid_x = 7
    num_grid_y = 7
    search_range_x = 0.6
    search_range_y = 0.6
    d_t = 0.03
    k_o = 0.11
    k_rv = 0.01
    k_rd = 0.25
    k_ra = 0.32  # ra = relative_angle
    length_step = 30
    n = 0

    leader = Leader()
    follower = Follower()
    logger = Logger(length_step)

    logger.log_leader(leader.x)
    logger.log_follower(follower.x)

#        print("length_step", length_step)
#        print("n", n)
    while n < length_step:

        leader.measure(follower.x, leader.x, n)
        follower.measure(leader.x, follower.x, n)

        leader.decide_action()
        follower.decide_action()

        leader.move()
        follower.move()

        logger.log_leader(leader.x)
        logger.log_follower(follower.x)

        print("leader.v_x", leader.v_x)
        logger.display()

        n += 1  # インクリメント
#    logger.display()
