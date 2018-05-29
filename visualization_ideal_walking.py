# -*- coding: utf-8 -*-
"""
Created on Tue Jan 23 19:26:46 2018

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
import copy
from decide_robot_absolute_position import decide_robot_absolute_position
from decide_robot_absolute_position import avg_vector
from agents_ver3 import Human
from states import AgentState


# meは人間の実測データ、 youはモデルとする
def display(l_s, f_s):
    l_p = []
    f_p = []
    for i in range(len(l_s)):
        l_p.append(l_s[i])
        f_p.append(f_s[i])
    l_p = np.array(l_p)
    f_p = np.array(f_p)
#        relative_distance = np.sqrt((l_p - f_p) * 2)
    plt.plot(*l_p.T, "-o", label="Robot")
    plt.plot(*f_p.T, "*", label="Human")
    plt.plot(-0.2, 3, "^", label="Goal")
#        plt.plot(self.obstacles_p.T[0], self.obstacles_p.T[1], "^",
#                 label="obstacle")
#        print("relative_distance", relative_distance[-1])
    plt.xticks(np.arange(-1.0, 3.0, 0.5))
    plt.yticks(np.arange(-0.8, 5.5, 0.5))  # 表の軸を0~20に固定
    plt.grid()
    plt.legend()
    plt.gca().set_aspect('equal')
    plt.show()
#        print("leader.p", self.l_p[-1])
#        print("follower.p", self.f_p[-1])


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
    length_step = 52
    n = 0
    lim = 5
    trajectory_a = make_trajectory([
            [0.98094249456, 0.731414990608],
            [0.88094249456, 0.631414990608],
            [0.75264596,  0.55928402]
            ])
    trajectory_b = make_trajectory([
            [1.73578850047, -0.99751806081],
            [1.74111587829, -0.937682491898],
            [1.58094249456, -0.831414990608],
            [1.46679422611, -0.745349506114]
            ])
#    #    KUBO
#    trajectory_a = make_trajectory([
#            [-1.88094249456, 1.931414990608],
#            [-1.68094249456, 1.831414990608],
#            [-1.45264596,  1.75928402]
#            ])
#    trajectory_b = make_trajectory([
#            [-0.60533186,  1.12512445],
#            [-0.53173167,  1.00566862],
#            [-0.64863302,  1.21137899]
#            ])
    initial_state_b = trajectory_b[-1]
    social_distance = 1.5
    d_t = 0.1
    prev_p = np.array([1.46679422611, -0.745349506114])
#    prev_p = np.array([-0.64863302,  1.21137899])
    d_num_for_avg = 15
    d_lst = []
    l_p = []
    f_p = []

    human_b = Human(
                initial_state_b, d_t, trajectory_b, trajectory_a)

    while n < length_step:
        temp_lst = []

        human_b.measure(human_b.s)
        human_b.decide_action()
        human_b.move()

        x_you = human_b.s.p[0]
        y_you = human_b.s.p[1]
        p = np.array([x_you, y_you])

        d_lst.append(human_b.s.d)
        if len(d_lst) < d_num_for_avg:
            d_sum = np.sum(np.array(d_lst), axis=0)
        else:
            for i in range(d_num_for_avg):
                temp_lst.append(d_lst[-1 - i])
            d_sum = np.sum(np.array(temp_lst), axis=0)
        d = avg_vector(d_sum, d_num_for_avg)
        x_me, y_me = decide_robot_absolute_position(p, d, social_distance)
        l_p.append(np.array([x_me, y_me]))
        f_p.append(np.array([x_you, y_you]))

        print("frame", n)
        plt.title("blue = Robot, red = Human")
        plt.plot(x_you, y_you, '*', color="r")
#        plt.quiver(x_you, y_you, d[0]*20, d[1]*20, angles='xy',scale_units='xy',scale=1)
        plt.plot(x_me, y_me, '.', color="b")
        plt.xlim(-lim, lim)
        plt.ylim(-lim, lim)
        plt.axes().set_aspect('equal')
        plt.grid()
        plt.show()
        plt.draw()
#        print("d", d)
        print("robot position", l_p[-1])
        print("")
        print("human position", f_p[-1])
        print("")
        print("relative distance", np.linalg.norm(np.array(l_p[-1]) - np.array(f_p[-1])))
        prev_p = np.array([x_you, y_you])
#        display(np.array(l_p), np.array(f_p))
        print("--------------------------------------------------------------")
        n += 1  # インクリメント
