# -*- coding: utf-8 -*-
"""
Created on Fri May 18 12:54:55 2018

@author: yume
"""

class BaseFactor(object):
    def __init__(self, scraper, a, b, c, d_t):
        self.a = a
        self.b = b
        self.c = c
        self.d_t = d_t
        self.scraper = scraper
        assert self.d_t > 0

    def factor(self, states_me, states_you=None, turn_rad):
        raise NotImplementedError("Do not use this class directoly")

    def score(self, states_me, states_you=None, turn_rad):
        x = self.factor(states_me, states_you, turn_rad)
        return self.f(x)

    def f(self, x):
        """
        eq. (9)
        unit test required
        """
#        assert not np.any(np.isnan(x))
        if x is True:
            f_x = 0
        if x is False:
            f_x = 0
#        assert not np.any(np.isnan(f_x)), "{}, {}, {}, {}, {}, {}, {}".format(
#                f_x, x, self.a, self.b, self.c, self, type(x))
        return f_x
