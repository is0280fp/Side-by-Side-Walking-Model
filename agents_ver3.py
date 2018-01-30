# -*- coding: utf-8 -*-
"""
Created on Tue Aug 22 14:27:45 2017

@author: yume
"""

import numpy as np
from segmentation_data import segmentation


class AgentRobot(object):
    def __init__(self, subgoals, initial_state,
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
        self.subgoals = subgoals
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

    def get_current_subgoal(self):
        return self.subgoals[0]

    def measure(self, s_me, s_you, subgoal, obstacle):
        self.trajectory_me.append(s_me)
        self.trajectory_you.append(s_you)
        self.subgoal = subgoal
        self.obstacle = obstacle

    def estimate(self):
        pass

    def decide_action(self):
        raise NotImplementedError(
                "You have to extend Agentclass if you want to use this module")

    def move(self):
        self.s.p = self.s.p + self.v
        dif = self.get_current_subgoal() - self.s.p
        self.s.d = dif / np.linalg.norm(dif)

    def __repr__(self):
        return repr(self.s)


class AgentHuman(object):
    def __init__(
                self, subgoals, initial_state, d_t=0.03,
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
        self.subgoal = subgoals[-1]
        self.d_t = d_t

    def measure(self, s_me, s_you=None):
        self.trajectory_me.append(s_me)
        self.trajectory_you.append(s_you)

    def estimate(self):
        pass

    def decide_action(self, count):
        raise NotImplementedError(
                "You have to extend Agentclass if you want to use this module")

    def move(self):
        self.s.p = self.v
        dif = self.subgoal - self.s.p
        self.s.d = dif / np.linalg.norm(dif)

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
        current_subgoal = self.get_current_subgoal()
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
            self, subgoals, initial_state, d_t=0.03,
            trajectory_me=None, trajectory_you=None, filepath=None):
        super(Human, self).__init__(
            subgoals, initial_state, d_t=0.03,
            trajectory_me=None, trajectory_you=None)
        if filepath is None:
            self.load_default_trajectory()
        else:
            self.load_trajectory(filepath)

    def load_trajectory(self, filepath):
        #        self.ps = np.load(filepath)
        threshold = 250
        human_center_p = []

        sequence_scans = np.load(filepath)
        background_points = sequence_scans[0].transpose()
        for i, current_scan in enumerate(sequence_scans):
            current_scan = current_scan.transpose()
            current_fg, current_bg = \
                segmentation(background_points, current_scan, threshold)
            current_fg = current_fg / 1000
            current_bg = current_bg / 1000
            if len(current_fg) == 0:
                pass
            else:
                human_center_x = np.sum(current_fg[:, 0])/len(current_fg)
                human_center_y = np.sum(current_fg[:, 1])/len(current_fg)
                human_center_p.append(
                        np.array([human_center_x, human_center_y]))
        self.ps = np.array(human_center_p)

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

    def decide_action(self):
        #  提案モデルに対する比較手法
        #   self.v = walking_model()
        #   self.count += 1
        #   self.v = ps[self.count]
        if self.ps.size != 0:
            self.v = self.ps[0]
            self.ps = self.ps[1:]


if __name__ == '__main__':

    subgoals = np.array([
            [-0.2, 3.0]
            ])
    initial_state_b = np.array([1.46679422611, -0.745349506114])
    human_a = Human(subgoals, initial_state_b)
    assert(human_a.ps.ndim == 2)
    assert(human_a.ps.shape[1] == 2)
    human_a = Human(subgoals, initial_state_b, "2017.09.27-14.57.16_kubo.npy")
    assert(human_a.ps.ndim == 2)
    assert(human_a.ps.shape[1] == 2)
