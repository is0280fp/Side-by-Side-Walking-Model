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
    def __init__(self, prev_s, s, next_s=None):  # Noneでdの引数省略可
        self.prev_s = prev_s
        self.s = s
        self.next_s = next_s
