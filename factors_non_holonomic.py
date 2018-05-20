# -*- coding: utf-8 -*-
"""
Created on Thu May 17 19:01:01 2018

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
from factor_non_holonomic import BaseFactorNonHolonomic
from line_math import line
from line_math import normal
from line_math import p_on_line
from line_math import other_p
from line_math import dis_compare_r


class LimitedArea(BaseFactorNonHolonomic):
    def factor_non_holonomic(self, states_me, turn_rad):
        prev_p_me = states_me.prev_s.p
        p_me = states_me.s.p
        next_p_me = states_me.next_s.p
        return self.limited_area(prev_p_me, p_me, next_p_me, turn_rad)

    def limited_area(self, prev_p_me, p_me, next_p_me, turn_rad):
        """
        unit test required
        """
        a, b, c = line(prev_p_me, p_me)
        e, d = normal(a, b, p_me)
        one_p = p_on_line(e, d)
        center_p = other_p(p_me, one_p, turn_rad)
#        flag = np.array([dis_compare_r(next_p_me, center_p[0], turn_rad),
#                         dis_compare_r(next_p_me, center_p[1], turn_rad)])
        flag = np.array([True, True])
        return flag
