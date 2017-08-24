# -*- coding: utf-8 -*-
"""
Created on Tue Aug 22 14:27:45 2017

@author: yume
"""

import numpy as np


class Agent(object):
    def __init__(self,
                 trajectory_me, trajectory_you, subgoals_p, optimum_velocity,
                 planner, d_t=0.03):
        self.trajectory_me = trajectory_me[:-1]
        self.trajectory_you = trajectory_you[:-1]
#        self.p = trajectory_me[-1]
        self.s = trajectory_me[-1]
        self.subgoals_p = subgoals_p
        self.optimum_velocity = optimum_velocity
        self.d_t = d_t
        self.planner = planner

    def measure(self, s_me, s_you):
        self.trajectory_me.append(s_me)
        self.trajectory_you.append(s_you)

    def estimate(self):
        pass

    def decide_action(self):
        raise NotImplementedError(
                "You have to extend Agentclass if you want to use this module")

    def move(self):
        self.s.p = self.s.p + self.v
        dif = self.subgoals_p[0] - self.s.p
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
class Human(Agent):

    def decide_action(self):
        subgoal_p = self.subgoals_p[0]
        optimum_velocity = self.optimum_velocity
        next_p = self.planner.decide_action(
                self.trajectory_me, self.trajectory_you,
                subgoal_p, optimum_velocity)
        current_p = self.s.p
        self.v = next_p - current_p
