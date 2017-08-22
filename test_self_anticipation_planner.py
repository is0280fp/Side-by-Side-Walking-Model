# -*- coding: utf-8 -*-
"""
Created on Tue Aug  8 15:10:34 2017

@author: yume
"""

import numpy as np
import unittest
import self_anticipation_planner
from agents import AgentState


class TestSelfAnticipationPlanner(unittest.TestCase):
    def setUp(self):
        self.planner = self_anticipation_planner.SelfAnticipationPlanner()

    def check_relative_angle(self, p_me, p_you, d_you, r_a):
        s_me = AgentState(p_me)
        s_you = AgentState(p_you, d_you)
        r_a_actual = self.planner.relative_angle(s_me, s_you)
        np.testing.assert_allclose(r_a, r_a_actual)

#    def test_relative_angle(self):
#        self.check_relative_angle([2, 2], [1, 2], [0, 1], np.pi / 2)
#        self.check_relative_angle([1, 2], [2, 2], [0, 1], np.pi / 2)
#        self.check_relative_angle([1, 4], [1, 2], [0, 2], 0)
#        self.check_relative_angle([1, 2], [1, 4], [0, 1], np.pi)
#        self.check_relative_angle([2, 2], [1, 1], [0, 1], np.pi / 4)
#        self.check_relative_angle([1, 1], [2, 2], [0, 1], np.pi * 3 / 4)
#        self.check_relative_angle([2, 2], [1, 1], [1, 1], 0)
#        self.check_relative_angle([1, 1], [2, 2], [2, 2], np.pi)
#        self.check_relative_angle(
#                [6, 6], [3, 3], [np.sqrt(3), 3], np.pi * 1 / 12)
#        self.check_relative_angle(
#                [3, 3], [6, 6], [np.sqrt(3), 3], np.pi * 11 / 12)
#        self.check_relative_angle([1, 1], [2, 2], [1, 0], np.pi * 3 / 4)
#        self.check_relative_angle([1, 1], [2, 2], [-1, 0], np.pi / 4)
#        self.check_relative_angle([1, 1], [2, 2], [-1, -1], 0)
#        self.check_relative_angle([2, 2], [1, 1], [-0.5, -0.5], np.pi)
#        self.check_relative_angle([3, 1], [2, 2], [-2, 2], np.pi)
#        self.check_relative_angle(
#                [2 + np.sqrt(3), 1], [2, 2], [-2, 2], np.pi * 11 / 12)

    def check_revision_theta(self, theta, expected):
        theta = np.deg2rad(theta)
        expected = np.deg2rad(expected)
        actual = self.planner.revision_theta(theta)
        np.testing.assert_allclose(expected, actual)

    def test_revision_theta(self):
        self.check_revision_theta(90, 90)
        self.check_revision_theta(360, 0)
        self.check_revision_theta(400, 40)
        self.check_revision_theta(270, -90)

if __name__ == '__main__':
    unittest.main()
