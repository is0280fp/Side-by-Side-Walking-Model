# -*- coding: utf-8 -*-
"""
Created on Tue Oct 31 23:36:52 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


class BaseFactor(object):
    def __init__(self, scraper, a, b, c, d_t):
        self.a = a
        self.b = b
        self.c = c
        self.d_t = d_t
        self.scraper = scraper
        assert self.d_t > 0

    def factor(self, states_me, states_you=None, subgoal=None, obstacle=None):
        raise NotImplementedError("Do not use this class directoly")

    def score(self, states_me, states_you=None, subgoal=None, obstacle=None):
        x = self.factor(states_me, states_you, subgoal, obstacle)
        return self.f(x)

    def f(self, x):
        """
        eq. (9)
        unit test required
        """
        assert not np.any(np.isnan(x))
        f_x = - np.abs((((x - self.c) / self.a)**2)**self.b)
        assert not np.any(np.isnan(f_x)), "{}, {}, {}, {}, {}, {}, {}".format(f_x, x, self.a, self.b, self.c, self, type(x))
        return f_x
