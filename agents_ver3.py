# -*- coding: utf-8 -*-
"""
Created on Tue Aug 22 14:27:45 2017

@author: yume
"""

import numpy as np
import copy
from decide_robot_absolute_position import avg_vector
from decide_robot_absolute_position import decide_robot_absolute_position
from geometry import absolute_angle
from utility_visualization import vector_graph
#from kubo_trajectory import load_default_trajectory
#from matsushita_trajectory import load_default_trajectory
#from kishimoto_trajectory import load_default_trajectory
#from satooka_trajectory import load_default_trajectory
#from kawabuchi_trajectory import load_default_trajectory
#from nakamura_trajectory import load_default_trajectory


class AgentRobot(object):
    def __init__(self, initial_state,
                 planner, d_t=0.1, trajectory_me=None, trajectory_you=None):
        if trajectory_me is not None:
            self.trajectory_me = trajectory_me[:-1]
        else:
            self.trajectory_me = []

        if trajectory_you is not None:
            self.trajectory_you = trajectory_you[:-1]
        else:
            self.trajectory_you = []

        self.s = initial_state
        self.d_t = d_t
        self.planner = planner

        self.f_ma_me_lst = []
        self.f_ma_you_lst = []
        self.f_mv_me_lst = []
        self.f_mv_you_lst = []
        self.f_mw_me_lst = []
        self.f_mw_you_lst = []
        self.f_ra_lst = []
        self.f_rd_lst = []
        self.f_rv_lst = []
        self.f_o_me_lst = []
        self.f_o_you_lst = []
        self.f_s_me_lst = []
        self.f_s_you_lst = []
        self.ra_lst = []
        self.d_lst = []

        self.p_x_lst = []
        self.p_y_lst = []
        self.v_x_lst = []
        self.v_y_lst = []

    def get_current_subgoal(self, s):
        temp_lst = []
        d_num_for_avg = 15
        social_distance = 1.5

        #  d_lstは1ステップあたりの進行方向ベクトルの変化量を格納するリスト
        self.d_lst.append(s.d)
        if len(self.d_lst) < d_num_for_avg:
            d_sum = np.sum(np.array(self.d_lst), axis=0)
            velocity_delta_avg = avg_vector(d_sum, len(self.d_lst))
        else:
            for i in range(d_num_for_avg):
                temp_lst.append(self.d_lst[-1 - i])
            d_sum = np.sum(np.array(temp_lst), axis=0)
            velocity_delta_avg = avg_vector(d_sum, d_num_for_avg)

        ideal_p_me = decide_robot_absolute_position(
                s.p, velocity_delta_avg, social_distance)
        return np.array([ideal_p_me[0], ideal_p_me[1]])

    #  現在位置を観測
    def measure(self, s_me, s_you, obstacle):
        self.trajectory_me.append(copy.deepcopy(s_me))
        self.trajectory_you.append(copy.deepcopy(s_you))
        self.subgoal = self.get_current_subgoal(s_you)
        self.obstacle = obstacle

    def estimate(self):
        pass

    def decide_action(self):
        raise NotImplementedError(
                "You have to extend Agentclass if you want to use this module")

    #  現在位置を次回位置に移動する
    def move(self):
        s = self.s
        #  車体半径0.13mの時
        d = 0.13
        #  車輪半径0.25cmの時
        r = 0.25
        #  角速度
        omega = absolute_angle(self.s.p, self.s.p + self.v)
        #  速度
        v = np.linalg.norm(self.v)
        #  右車輪、左車輪に与える速度を計算
        v_r = (v - omega * d)/r
        v_l = (v + omega * d)/r
        #   現在の進行方向を表示する
        vector_graph(self.s.p[0], self.s.p[1],
                     np.array([0, self.v[0]*10]), np.array([1, self.v[1]*10]))
        #  進行方向の軌跡を表示する
        self.p_x_lst.append(self.s.p[0])
        self.p_y_lst.append(self.s.p[1])
        self.v_x_lst.append(self.v[0])
        self.v_y_lst.append(self.v[1])
        vector_graph(np.array(self.p_x_lst), np.array(self.p_y_lst),
                     np.array([np.zeros(np.array(self.v_x_lst).shape), np.array(self.v_x_lst)*10]),
                     np.array([np.zeros(np.array(self.v_y_lst).shape)+1, np.array(self.v_y_lst)*10]))
        #  現在状態（位置、進行方向ベクトル）の更新
        s.p = self.s.p + self.v
        self.s.d = self.v


    def __repr__(self):
        return repr(self.s)


class AgentHuman(object):
    def __init__(
                self, initial_state, d_t=0.03,
                trajectory_me=None, trajectory_you=None):
        if trajectory_me is not None:
            self.trajectory_me = trajectory_me[:-1]
        else:
            self.trajectory_me = []

        if trajectory_you is not None:
            self.trajectory_you = trajectory_you[:-1]
        else:
            self.trajectory_you = []
        self.s = initial_state
        self.d_t = d_t

    def measure(self, s_me, s_you=None):
        self.trajectory_me.append(copy.deepcopy(s_me))
        self.trajectory_you.append(copy.deepcopy(s_you))

    def estimate(self):
        pass

    def decide_action(self, count):
        raise NotImplementedError(
                "You have to extend Agentclass if you want to use this module")

    def move(self):
        self.s.p = self.v
#        dif = self.subgoal - self.s.p
#        self.s.d = dif / np.linalg.norm(dif)
        self.s.d = self.d

    def __repr__(self):
        return repr(self.s)


class AgentState(object):
    def __init__(self, p, d=None):  # Noneでdの引数省略可
        self.p = np.array(p)
        self.d = np.array(d)

    def __repr__(self):
        return "state({}, {})".format(self.p, self.d)


# Leader
# plotする範囲を指定、plot数も指定
class Robot(AgentRobot):
    def closest_obstacle(self):
        closest_obstacle = self.obstacle
        return closest_obstacle

    def decide_action(self):
        current_subgoal = self.subgoal
        closest_obstacle = self.closest_obstacle()
        next_p = self.planner.decide_action(
                self.trajectory_me, self.trajectory_you,
                current_subgoal, closest_obstacle)
        current_p = self.s.p
        self.v = next_p - current_p


# follower
# plotする範囲を指定、plot数も指定
class Human(AgentHuman):
    def __init__(
            self, initial_state, d_t=0.03,
            trajectory_me=None, trajectory_you=None, filepath=None):
        super(Human, self).__init__(
                initial_state, d_t=0.03,
                trajectory_me=None, trajectory_you=None)
        if filepath is None:
#            self.ps, self.ps_for_d = load_default_trajectory()
            self.load_default_trajectory()
        else:
            self.load_trajectory(filepath)

    def load_trajectory(self, filepath):
        self.ps = np.load(filepath)

    def load_default_trajectory(self):
        self.ps = np.array(([
            [ 1.39638343552 , -0.670772264944 ] ,
            [ 1.32672097618 , -0.613402891541 ] ,
            [ 1.29423710358 , -0.576229029302 ] ,
            [ 1.26490391443 , -0.536084754334 ] ,
            [ 1.22321161195 , -0.47479323992 ] ,
            [ 1.16483836463 , -0.407274773265 ] ,
            [ 1.14530817675 , -0.313360000718 ] ,
            [ 1.11968057765 , -0.236743351869 ] ,
            [ 1.04601448649 , -0.218638502313 ] ,
            [ 1.00023513822 , -0.155460433211 ] ,
            [ 0.952399037927 , -0.0957840967972 ] ,
            [ 0.876120483525 , -0.0725344816746 ] ,
            [ 0.789454093751 , 0.0024966573176 ] ,
            [ 0.701204782518 , 0.0535869111234 ] ,
            [ 0.60321063024 , 0.0943298391902 ] ,
            [ 0.501403297386 , 0.126244190932 ] ,
            [ 0.458674073834 , 0.150082486618 ] ,
            [ 0.374216612929 , 0.174522796145 ] ,
            [ 0.270766459447 , 0.197432033057 ] ,
            [ 0.140487449363 , 0.182466849487 ] ,
            [ 0.0839504584906 , 0.193064345303 ] ,
            [ 0.00867053063701 , 0.252826921391 ] ,
            [ -0.0558847139945 , 0.324567074343 ] ,
            [ -0.123450011454 , 0.386088749925 ] ,
            [ -0.213389161593 , 0.483747017494 ] ,
            [ -0.295008485871 , 0.568286730805 ] ,
            [ -0.334235606277 , 0.665673791635 ] ,
            [ -0.360719327173 , 0.738451312721 ] ,
            [ -0.371115465001 , 0.797026736796 ] ,
            [ -0.403014375994 , 0.836922394399 ] ,
            [ -0.434378950527 , 0.915403442066 ] ,
            [ -0.45977540014 , 1.02586037217 ] ,
            [ -0.457487655687 , 1.14414566633 ] ,
            [ -0.442752001844 , 1.2568106133 ] ,
            [ -0.463553999707 , 1.36427615506 ] ,
            [ -0.492616608844 , 1.41474701055 ] ,
            [ -0.527708575348 , 1.51294145345 ] ,
            [ -0.519195359154 , 1.63566784359 ] ,
            [ -0.486244990646 , 1.75864932594 ] ,
            [ -0.478019715864 , 1.87588649695 ] ,
            [ -0.451797856373 , 1.95211506565 ] ,
            [ -0.442224659187 , 2.02118731345 ] ,
            [ -0.425487090152 , 2.09929104389 ] ,
            [ -0.396311266705 , 2.19805512011 ] ,
            [ -0.365299981093 , 2.30854524504 ] ,
            [ -0.312803546216 , 2.41014342222 ] ,
            [ -0.299361592526 , 2.47706637036 ] ,
            [ -0.275267391074 , 2.54106418093 ] ,
            [ -0.267412572606 , 2.61736334628 ] ,
            [ -0.250503196907 , 2.69662228249 ] ,
            [ -0.234530700857 , 2.80601753206 ] ,
            [ -0.199066856343 , 2.92899298522 ]]))

        self.ps_for_d = np.array(([
            [1.46679422611, -0.745349506114],
            [ 1.39638343552 , -0.670772264944 ] ,
            [ 1.32672097618 , -0.613402891541 ] ,
            [ 1.29423710358 , -0.576229029302 ] ,
            [ 1.26490391443 , -0.536084754334 ] ,
            [ 1.22321161195 , -0.47479323992 ] ,
            [ 1.16483836463 , -0.407274773265 ] ,
            [ 1.14530817675 , -0.313360000718 ] ,
            [ 1.11968057765 , -0.236743351869 ] ,
            [ 1.04601448649 , -0.218638502313 ] ,
            [ 1.00023513822 , -0.155460433211 ] ,
            [ 0.952399037927 , -0.0957840967972 ] ,
            [ 0.876120483525 , -0.0725344816746 ] ,
            [ 0.789454093751 , 0.0024966573176 ] ,
            [ 0.701204782518 , 0.0535869111234 ] ,
            [ 0.60321063024 , 0.0943298391902 ] ,
            [ 0.501403297386 , 0.126244190932 ] ,
            [ 0.458674073834 , 0.150082486618 ] ,
            [ 0.374216612929 , 0.174522796145 ] ,
            [ 0.270766459447 , 0.197432033057 ] ,
            [ 0.140487449363 , 0.182466849487 ] ,
            [ 0.0839504584906 , 0.193064345303 ] ,
            [ 0.00867053063701 , 0.252826921391 ] ,
            [ -0.0558847139945 , 0.324567074343 ] ,
            [ -0.123450011454 , 0.386088749925 ] ,
            [ -0.213389161593 , 0.483747017494 ] ,
            [ -0.295008485871 , 0.568286730805 ] ,
            [ -0.334235606277 , 0.665673791635 ] ,
            [ -0.360719327173 , 0.738451312721 ] ,
            [ -0.371115465001 , 0.797026736796 ] ,
            [ -0.403014375994 , 0.836922394399 ] ,
            [ -0.434378950527 , 0.915403442066 ] ,
            [ -0.45977540014 , 1.02586037217 ] ,
            [ -0.457487655687 , 1.14414566633 ] ,
            [ -0.442752001844 , 1.2568106133 ] ,
            [ -0.463553999707 , 1.36427615506 ] ,
            [ -0.492616608844 , 1.41474701055 ] ,
            [ -0.527708575348 , 1.51294145345 ] ,
            [ -0.519195359154 , 1.63566784359 ] ,
            [ -0.486244990646 , 1.75864932594 ] ,
            [ -0.478019715864 , 1.87588649695 ] ,
            [ -0.451797856373 , 1.95211506565 ] ,
            [ -0.442224659187 , 2.02118731345 ] ,
            [ -0.425487090152 , 2.09929104389 ] ,
            [ -0.396311266705 , 2.19805512011 ] ,
            [ -0.365299981093 , 2.30854524504 ] ,
            [ -0.312803546216 , 2.41014342222 ] ,
            [ -0.299361592526 , 2.47706637036 ] ,
            [ -0.275267391074 , 2.54106418093 ] ,
            [ -0.267412572606 , 2.61736334628 ] ,
            [ -0.250503196907 , 2.69662228249 ] ,
            [ -0.234530700857 , 2.80601753206 ] ,
            [ -0.199066856343 , 2.92899298522 ]]))

    def decide_action(self):
        #  提案モデルに対する比較手法
        #   self.v = walking_model()
        #   self.count += 1
        #   self.v = ps[self.count]
        if self.ps.size != 0:
            self.v = self.ps[0]
            self.d = self.v - self.ps_for_d[0]
            self.ps = self.ps[1:]
            self.ps_for_d = self.ps_for_d[1:]


if __name__ == '__main__':
    initial_state_b = np.array([1.46679422611, -0.745349506114])
    human_a = Human(initial_state_b)
    assert(human_a.ps.ndim == 2)
    assert(human_a.ps.shape[1] == 2)
    human_a = Human(initial_state_b, "2017.09.27-14.57.16_kubo.npy")
    assert(human_a.ps.ndim == 2)
    assert(human_a.ps.shape[1] == 2)
