# -*- coding: utf-8 -*-
"""
Created on Fri May 18 12:54:55 2018

@author: yume
"""

class BaseFactorNonHolonomic(object):
    def __init__(self, scraper, a, b, c, d_t):
        self.a = a
        self.b = b
        self.c = c
        self.d_t = d_t
        self.scraper = scraper
        assert self.d_t > 0

    def factor_non_holonomic(self, states_me, turn_rad):
        raise NotImplementedError("Do not use this class directoly")

    def score_non_holonomic(self, states_me, turn_rad):
        x = self.factor_non_holonomic(states_me, turn_rad)
        return self.f_non_holonomic(x)

    def f_non_holonomic(self, x):
        """
        eq. (9)
        unit test required
        """
#        assert not np.any(np.isnan(x))
        if (x == True).all():
            return 0
        else:
            return 0
#        assert not np.any(np.isnan(f_x)), "{}, {}, {}, {}, {}, {}, {}".format(
#                f_x, x, self.a, self.b, self.c, self, type(x))
