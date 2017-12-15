# -*- coding: utf-8 -*-
"""
Created on Tue Nov  7 16:58:33 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


class AgentState(object):
    def __init__(self, p, d=None):  # Noneでdの引数省略可
        self.p = np.array(p)
        self.d = np.array(d)

    def __repr__(self):
        return "state({}, {})".format(self.p, self.d)


class States(object):
    def __init__(self, trajectory_me, trajectory_you):  # Noneでdの引数省略可
        self.prev_s_me = trajectory_me[-2]
        self.prev_s_you = trajectory_you[-2]
        self.s_me = trajectory_me[-1]
        self.s_you = trajectory_you[-1]
        self.next_s_me = linear_extrapolation(trajectory_me)
        self.next_s_you = linear_extrapolation(trajectory_you)


def linear_extrapolation(trajectory):
        """位置の履歴から次の時刻の位置を等速直線運動で予測する

        Args　:
            trajectory:ｎｕｍ＿grid＿x
                位置の記録
        Returns:
            next_p(ndarray):
                .. math:: \hat{p}_{t+1}
            d(ndarray):
                p_t - p.{t-1}
        """
        temp = trajectory[-1].p - trajectory[-2].p
        next_p = trajectory[-1].p + temp
        temp = trajectory[-1].d - trajectory[-2].d
        next_d = trajectory[-1].d + temp
        next_d = next_d / np.linalg.norm(next_d)
        return AgentState(next_p, next_d)
