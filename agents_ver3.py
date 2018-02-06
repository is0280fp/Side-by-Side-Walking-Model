# -*- coding: utf-8 -*-
"""
Created on Tue Aug 22 14:27:45 2017

@author: yume
"""

import numpy as np
import copy
from kubo_trajectory import load_default_trajectory
#from matsushita_trajectory import load_default_trajectory
#from kishimoto_trajectory import load_default_trajectory
#from satooka_trajectory import load_default_trajectory
#from kawabuchi_trajectory import load_default_trajectory
#from nakamura_trajectory import load_default_trajectory


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

    #  現在位置を観測
    def measure(self, s_me, s_you, subgoal, obstacle):
        self.trajectory_me.append(copy.deepcopy(s_me))
        self.trajectory_you.append(copy.deepcopy(s_you))
        self.subgoal = subgoal
        self.obstacle = obstacle

    def estimate(self):
        pass

    def decide_action(self):
        raise NotImplementedError(
                "You have to extend Agentclass if you want to use this module")

    #  現在位置を次回位置に移動する
    def move(self):
        s = self.s
        s.p = self.s.p + self.v
        self.s.d = self.v

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
        self.ps = np.load(filepath)

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
