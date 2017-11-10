# -*- coding: utf-8 -*-
"""
Created on Tue Oct 31 23:36:52 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


class BaseFactor(object):
    def __init__(self, a, b, c, d_t):
        self.a = a
        self.b = b
        self.c = c
        self.d_t = d_t

    def factor(self, state, environment):
        raise NotImplementedError("Do not use this class directoly")

    def score(self, state, environment):
        x = self.factor(state, environment)
        return self.f(x)

    def f(self, x):
        """
        eq. (9)
        unit test required
        """
        f_x = 1 / (1+(abs((x-self.c) / self.a)**(2*self.b))) - 1
        return f_x
