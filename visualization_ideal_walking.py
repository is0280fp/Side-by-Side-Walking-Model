# -*- coding: utf-8 -*-
"""
Created on Tue Jan 23 19:26:46 2018

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
from decide_robot_absolute_position import decide_robot_absolute_position
from agents_ver3 import Human
from states import AgentState

# meは人間の実測データ、 youはモデルとする


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
    length_step = 20
    n = 0
    lim = 5
    trajectory_a = make_trajectory([
            [0.73578850047, 0.59751806081],
            [0.74111587829, 0.537682491898],
            [0.58094249456, 0.431414990608],
            [0.46679422611, 0.345349506114]
            ])
    trajectory_b = make_trajectory([
            [1.73578850047, -0.99751806081],
            [1.74111587829, -0.937682491898],
            [1.58094249456, -0.831414990608],
            [1.46679422611, -0.745349506114]
            ])
    initial_state_b = trajectory_b[-1]
    social_distance = 1.5
    subgoals = [np.array([-0.2, 3.0])]
    d_t = 0.1

    plt.ion()
    human_b = Human(
                subgoals, initial_state_b, d_t, trajectory_b, trajectory_a)
    while n < length_step:
        human_b.measure(human_b.s)
        human_b.decide_action()
        human_b.move()

        x_you = human_b.s.p[0]
        y_you = human_b.s.p[1]
        p = np.array([x_you, y_you])
        d = human_b.s.d
        x_me, y_me = decide_robot_absolute_position(p, d, social_distance)

        print("frame", n)
        plt.title("blue = Human, red = Model")
        plt.plot(x_you, y_you, '*', color="r")
        plt.plot(x_me, y_me, '.', color="b")
        plt.xlim(-lim, lim)
        plt.ylim(-lim, lim)
        plt.axes().set_aspect('equal')
        plt.grid()
        plt.show()
        plt.draw()
        print("-----------------------------------------------------------------------")
        n += 1  # インクリメント
