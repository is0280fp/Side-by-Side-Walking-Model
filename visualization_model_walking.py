# -*- coding: utf-8 -*-
"""
Created on Tue Jan 23 19:26:46 2018

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
from decide_robot_absolute_position import decide_robot_absolute_position
from decide_robot_absolute_position import avg_vector
from agents_ver3 import Human
from states import AgentState
# meはrobot、 youは人間とする


class Robot(object):
    def __init__(self):
        pass
    #  パラメタは以下の時
        #  k_o = 0
#        k_rd = 1.01
#        k_ra = 2.01  # ra = relative_angle
#        k_s = 3.0
#        k_mv = 0.01
#
#        k_ma = 0.1
#        k_rv = 0.1
#        k_mw = 0.1
        #  k_pt = 0  # 新しいfactor
    def load_default_trajectory(self):
             #  テスト用データの結果
#            self.ps = np.array(([
#                    [ 0.75264596,  0.55928402],
#                     [ 0.75768276,  0.62048638],
#                     [ 0.69605289,  0.74835541],
#                     [ 0.63442302,  0.87622444],
#                     [ 0.63945982,  0.9374268 ],
#                     [ 0.64449662,  0.99862917],
#                     [ 0.64953342, 1.1264982 ],
#                     [ 0.65457022,  1.25436723],
#                     [ 0.65960702,  1.31556959],
#                     [ 0.66464382,  1.37677195],
#                     [ 0.66968061,  1.43797431],
#                     [ 0.60805075,  1.56584334],
#                     [ 0.54642088,  1.69371237],
#                     [ 0.55145768,  1.75491474],
#                    [ 0.55649448, 1.8161171 ],
#                     [ 0.49486461,  1.94398613],
#                     [ 0.43323474,  2.07185516],
#                     [ 0.37160487,  2.19972419],
#                     [ 0.44330834,  2.32759322],
#                     [ 0.38167847,  2.38879558],
#                     [ 0.45338194,  2.44999794],
#                     [ 0.45841873,  2.44453364],
#                     [ 0.46345553,  2.43906933],
#                     [ 0.535159,    2.56693836],
#                     [ 0.60686246,  2.49480739],
#                     [ 0.61189926,  2.55600975],
#                     [ 0.61693606,  2.61721212],
#                     [ 0.62197286,  2.67841448],
#                     [ 0.62700966,  2.73961684],
#                     [ 0.63204646,  2.86748587],
#                     [ 0.63708326,  2.92868824],
#                     [ 0.64212006,  2.9898906 ],
#                     [ 0.64715685,  2.91775963],
#                     [ 0.65219365,  2.97896199],
#                    [ 0.72389712,  3.04016435],
#                     [ 0.79560058,  3.10136672],
#                     [ 0.86730405,  3.16256908],
#                     [ 0.80567418,  3.15710477],
#                     [ 0.74404431,  3.0849738 ],
#                     [ 0.88241445,  3.0795095 ],
#                     [ 0.95411791,  3.14071186],
#                     [ 0.95915471,  3.13524756],
#                     [ 0.96419151,  3.12978325],
#                     [ 0.96922831,  3.12431895],
#                     [ 0.90759844,  3.11885465],
#                     [ 0.9793019,   3.18005701],
#                     [ 1.05100537,  3.24125937],
#                     [ 1.12270884,  3.3691284 ],
#                     [ 1.26107897,  3.3636641 ],
#                     [ 1.33278243,  3.42486646],
#                     [ 1.4044859,   3.41940216],
#                     [ 1.34285603,  3.54727119],
#                     [ 1.34789283,  3.54180688],
#                    ]))
        #  テスト用データに動的ゴールを与えた結果
        self.ps = np.array(([
                 [ 0.75264596,  0.55928402],
                 [ 0.69101609,  0.55381972],
                 [ 0.56271956,  0.54835541],
                 [ 0.50108969,  0.67622444],
                 [ 0.43945982,  0.67076014],
                 [ 0.37782995,  0.66529583],
                 [ 0.31620009,  0.65983153],
                 [ 0.32123688,  0.72103389],
                 [ 0.32627368,  0.78223626],
                 [ 0.13131048,  0.84343862],
                 [ 0.13634728,  0.90464098],
                 [ 0.14138408,  0.96584334],
                 [ 0.14642088,  1.02704571],
                 [ 0.08479101,  1.0215814 ],
                 [ 0.02316114,  1.14945043],
                 [-0.03846873,  1.14398613],
                 [-0.10009859,  1.13852182],
                 [-0.02839513,  1.19972419],
                 [-0.090025,    1.19425988],
                 [-0.15165486,  1.32212891],
                 [-0.21328473,  1.31666461],
                 [-0.2749146,  1.3112003],
                 [-0.33654447,  1.43906933],
                 [-0.264841,   1.5002717],
                 [-0.19313754,  1.56147406],
                 [-0.32143407,  1.68934309],
                 [-0.38306394,  1.81721212],
                 [-0.44469381,  1.81174781],
                 [-0.50632367,  1.93961684],
                 [-0.63462021,  1.93415254],
                 [-0.62958341,  1.92868824],
                 [-0.69121328,  1.92322393],
                 [-0.75284315,  1.91775963],
                 [-0.74780635,  1.97896199],
                 [-0.80943622,  2.10683102],
                 [-0.87106608,  2.23470005],
                 [-0.79936262,  2.36256908],
                 [-0.92765915,  2.49043811],
                 [-0.98928902,  2.55164047],
                 [-1.05091889,  2.6795095 ],
                 [-1.04588209,  2.74071186],
                 [-1.10751196,  2.86858089],
                 [-1.16914182,  2.99644992],
                 [-1.09743836,  3.12431895],
                 [-1.15906823,  3.18552131]
                 ]))

    def decide_action(self):
        #  提案モデルに対する比較手法
        #   self.v = walking_model()
        #   self.count += 1
        #   self.v = ps[self.count]
        if self.ps.size != 0:
            self.v = self.ps[0]
            self.ps = self.ps[1:]


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
    initial_state_b = trajectory_b[-1]

    length_step = 44
    n = 0
    lim = 5
    d_num_for_avg = 15
    d_lst = []
    prev_p = np.array([1.46679422611, -0.745349506114])
    d_t = 0.1
    social_distance = 1.5
    human_b = Human(
                initial_state_b, d_t, trajectory_b, trajectory_a)
    robot = Robot()
    robot.load_default_trajectory()
    error = []

    while n < length_step:
        temp_lst = []

        human_b.measure(human_b.s)
        human_b.decide_action()
        robot.decide_action()
        human_b.move()
        x_me = robot.v[0]
        y_me = robot.v[1]
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
        ideal_x_me, ideal_y_me = decide_robot_absolute_position(
                p, d, social_distance)

        ideal_p = np.array([ideal_x_me, ideal_y_me])
        true_p = np.array([x_me, y_me])
        error.append(np.linalg.norm(ideal_p - true_p))

        print("frame", n)
        plt.title("orange = Human, blue = Robot_Model, green = Ideal_model")
        plt.plot(x_you, y_you, '*', color='#ff7f00')
        plt.plot(x_me, y_me, 'o', color='#377eb8')
        plt.plot(ideal_x_me, ideal_y_me, 'o', color="g")
        plt.xlim(-lim, lim)
        plt.ylim(-lim, lim)
        plt.axes().set_aspect('equal')
        plt.grid()
        plt.show()
        plt.draw()
        print("-----------------------------------------------------------------------")
        n += 1  # インクリメント
print(np.sum(np.array(error)) / length_step)
