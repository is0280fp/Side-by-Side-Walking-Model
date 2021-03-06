# -*- coding: utf-8 -*-
"""
Created on Tue Jan 23 19:26:46 2018

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
import copy
from collections import defaultdict
from decide_robot_absolute_position import decide_robot_absolute_position
from decide_robot_absolute_position import avg_vector
import planner_for_seach_parameters
from agents_ver3 import Robot
from states import AgentState
from agents_ver3 import Human
from calibrate_parameters import choose_parameters
from calibrate_parameters import calculate_error


# meは人間の実測データ、 youはモデルとする

class Logger(object):
    def __init__(self, length_step):
        self.l_s = []  # 現在位置を格納するリスト
        self.f_s = []  # 現在位置を格納するリスト
        self.length_step = length_step

    def log_leader(self, s):
        self.l_s.append(copy.copy(s))
        return self.l_s

    def log_follower(self, s):
        self.f_s.append(copy.copy(s))
        return self.f_s

    def display(self):
        l_p = np.array([s.p for s in self.l_s])
        f_p = np.array([s.p for s in self.f_s])
#        relative_distance = np.sqrt((l_p - f_p) * 2)
        plt.plot(*l_p.T, "-o", label="Robot")
        plt.plot(*f_p.T, "*", label="Human")
        plt.plot(-0.2, 3.0, "^", label="Goal")
#        plt.plot(self.obstacles_p.T[0], self.obstacles_p.T[1], "^",
#                 label="obstacle")
#        print("relative_distance", relative_distance[-1])
        plt.xticks(np.arange(-0.5, 1.5, 0.5))
        plt.yticks(np.arange(-0.8, 3, 0.5))  # 表の軸を0~20に固定
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
        print("Robot_s")
        print("\n".join(
                ["{}: {}".format(i, s) for i, s in enumerate(logger.l_s)]))
#  print("\n".join([str(i) + ": " + str(s) for i, s in enumerate(logger.l_s)]))
        print()
        print("Human_s")
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
    length_step = 15
    n = 0
    lim = 5
    trajectory_a = make_trajectory([
            [0.98094249456, 0.731414990608],
            [0.88094249456, 0.631414990608],
            [0.75264596,  0.55928402]
            ])
    trajectory_b = make_trajectory([
            [1.73578850047, -0.99751806081],
            [1.58094249456, -0.831414990608],
            [1.46679422611, -0.745349506114]
            ])
    social_distance = 1.5
    subgoals = [np.array([-0.2, 3.0])]
    obstacles = []
    d_t = 0.1

    num_grid_x = 5
    num_grid_y = 5
    search_range_x = 0.2
    search_range_y = 0.2

    length_step = 40
    relative_angle_a = 0
    relative_angle_b = 180 - relative_angle_a

    n = 0
    count = 0
    errors = 0
    initial_state_a = trajectory_a[-1]
    initial_state_b = trajectory_b[-1]
    k_set = choose_parameters(0, 4, 1)
    error_lst = []
    error_paras_match = defaultdict()

    d_num_for_avg = 15
    d_lst = []

    for i, k in enumerate(k_set):
        print("paras_No.", count)

        rd_k = k[0]
        ra_k = k[1]
        sgd_k = k[2]
        mv_k = k[3]

        mav_k = 0.01
        ma_k = 0.01
        rv_k = 0.01
        od_k = 0
        planner_a = \
            planner_for_seach_parameters.PartnerSelfAnticipationPlanner(
                    rv_k,  rd_k, ra_k, mv_k, ma_k, mav_k, sgd_k, od_k,
                    "a", num_grid_x, num_grid_y,
                    search_range_x, search_range_y, d_t,
                    relative_angle_a)
        human_a = Robot(
            subgoals, initial_state_a, planner_a, d_t,
            trajectory_a, trajectory_b)
        human_b = Human(
                    subgoals, initial_state_b, d_t, trajectory_b, trajectory_a)

        logger = Logger(length_step)

        logger.log_leader(human_a.s)
        logger.log_follower(human_b.s)

        n = 0
        errors = 0
        while n < length_step:
            temp_lst = []
            human_a.measure(human_a.s, human_b.s, subgoals, obstacles)
            human_b.measure(human_b.s, human_a.s)

            human_a.decide_action()
            human_b.decide_action()

            human_a.move()
            human_b.move()

            logger.log_leader(human_a.s)
            logger.log_follower(human_b.s)

            print("step", n)
#            logger.display()
#            logger.print()

            x_you = human_b.s.p[0]
            y_you = human_b.s.p[1]
            model_x_me = human_a.s.p[0]
            model_y_me = human_a.s.p[1]
            p = np.array([x_you, y_you])
            d_lst.append(human_b.s.d)
            if len(d_lst) < d_num_for_avg:
                d_sum = np.sum(np.array(d_lst), axis=0)
            else:
                for i in range(d_num_for_avg):
                    temp_lst.append(d_lst[-1 - i])
                d_sum = np.sum(np.array(temp_lst), axis=0)
            d = avg_vector(d_sum, d_num_for_avg)
            ideal_x_me, ideal_y_me = decide_robot_absolute_position(
                    p, d, social_distance)

            ideal_p_me = np.array([ideal_x_me, ideal_y_me])
            model_p_me = np.array([model_x_me, model_y_me])
            errors += calculate_error(ideal_p_me, model_p_me)

#            print("frame", n)
#            plt.title("blue = Human, red = Model")
#            plt.plot(x_you, y_you, '*', color="r")
#            plt.plot(model_x_me, model_y_me, '.', color="g")
#            plt.plot(ideal_x_me, ideal_y_me, '.', color="b")
#            plt.xlim(-lim, lim)
#            plt.ylim(-lim, lim)
#            plt.axes().set_aspect('equal')
#            plt.grid()
#            plt.show()
#            plt.draw()
#            print("--------------------------------------------------------------")
            n += 1  # インクリメント
#        print("--------------------------------------------------------------")

        error_paras_match.setdefault(errors, k)
        error_lst.append(errors)
        count += 1

error_min = np.nanmin(np.array(error_lst))
optimum_paras = error_paras_match[error_min]
