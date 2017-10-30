# -*- coding: utf-8 -*-
"""
Created on Fri Oct 13 15:27:13 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


class EnvironmentState(object):
    def __init__(self, p):  # Noneでdの引数省略可
        self.p = np.array(p)
