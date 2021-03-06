# -*- coding: utf-8 -*-
"""
Created on Tue Oct 31 23:46:32 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
from factor import BaseFactor
from geometry import absolute_angle
from geometry import revision_theta
from geometry import motion_velocity
from geometry import motion_acceleration
from geometry import motion_angular_velocity


class RelativeVelocity(BaseFactor):
    def factor(self, states_me, states_you, subgoal=None, obstacle=None):
        d_me = states_me.next_s.d
        d_you = states_you.next_s.d
        return self.relative_velocity(d_me, d_you, self.d_t)

    def relative_velocity(self, d_me, d_you, d_t):
        """
        unit test required
        """
        return np.linalg.norm((d_me - d_you) / d_t)


class RelativeDistance(BaseFactor):
    def factor(self, states_me, states_you, subgoal=None, obstacle=None):
        p_me = states_me.next_s.p
        p_you = states_you.next_s.p
        return self.relative_distance(p_me, p_you)

    def relative_distance(self, p_me, p_you):
        return np.linalg.norm(p_you - p_me)


class RelativeAngle(BaseFactor):
    def factor(self, states_me, states_you, subgoal=None, obstacle=None):
        p_me = states_me.next_s.p
        p_you = states_you.next_s.p
        d_you = states_you.next_s.d
        return self.relative_angle(p_me, p_you, d_you)

    def relative_angle(self, p_me, p_you, d_you):
        """
        unit test required
        """
#        データ可視化のためのローカル変数
        self.pass_p_me = p_me
        self.pass_p_you = p_you
        self.pass_d_you = d_you
        self.v_yoko = p_me - p_you

        # youの進行方向の絶対角度
        theta_mae = np.arctan2(d_you[1], d_you[0])
        theta_yoko = absolute_angle(p_you, p_me)
        theta = theta_yoko - theta_mae
        r_a = revision_theta(theta)

#        データ可視化のためのローカル変数
        self.pass_theta_mae = theta_mae
        self.pass_theta_yoko = theta_yoko
        self.pass_theta = theta
        self.pass_r_a = r_a
        return np.abs(r_a)

    def pass_values(self):
        self.scraper.add_ra_values(self.pass_p_me, self.pass_p_you,
                                   self.pass_d_you, self.v_yoko,
                                   self.pass_theta_mae, self.pass_theta_yoko,
                                   self.pass_theta, self.pass_r_a)


class Velocity(BaseFactor):
    def factor(self, states_me, states_you=None, subgoal=None, obstacle=None):
        p = states_me.s.p
        next_p = states_me.next_s.p
        return self.velocity(p, next_p, self.d_t)

    def velocity(self, p, next_p, d_t):
        return motion_velocity(p, next_p, d_t)


class Acceleration(BaseFactor):
    def factor(self, states_me, states_you=None, subgoal=None, obstacle=None):
        prev_p = states_me.prev_s.p
        p = states_me.s.p
        next_p = states_me.next_s.p
        return self.acceleration(prev_p, p, next_p, self.d_t)

    def acceleration(self, prev_p, p, next_p, d_t):
        motion_v = motion_velocity(prev_p, p, d_t)
        next_motion_v = motion_velocity(p, next_p, d_t)
        return motion_acceleration(motion_v, next_motion_v)


class AngularVelocity(BaseFactor):
    def factor(self, states_me, states_you=None, subgoal=None, obstacle=None):
        prev_p = states_me.prev_s.p
        p = states_me.s.p
        next_p = states_me.next_s.p
        return self.angularvelocity(prev_p, p, next_p, self.d_t)

    def angularvelocity(self, prev_p, p, next_p, d_t):
        # 現在と予測による角速度
        theta = motion_angular_velocity(prev_p, p, next_p, d_t)
        m_w = revision_theta(theta)
        return np.abs(m_w)


class MovingTowardSubgoals(BaseFactor):
    def factor(self, states_me, states_you=None, subgoal=None, obstacle=None):
        p = states_me.s.p
        next_p = states_me.next_s.p
        subgoal_p = subgoal
        return self.moving_toward_subgoals(p, next_p, subgoal_p)

    def moving_toward_subgoals(self, p, next_p, subgoal_p):
        e_o = absolute_angle(p, subgoal_p) - absolute_angle(p, next_p)
        return e_o


class DistanceToObstacle(BaseFactor):
    def __init__(self, scraper, a, b):
        self.a = a
        self.b = b
        self.scraper = scraper

    def factor(self, states_me, states_you=None, subgoal=None, obstacle=None):
        p = states_me.next_s.p
        obstacle_p = obstacle
        if not obstacle_p:
            return np.inf
        return self.distance_to_obstacle(p, obstacle_p)

    def distance_to_obstacle(self, p, obstacle_p):
        return np.sqrt(np.sum((obstacle_p - p) ** 2))

    def f(self, x):
        """
        eq. (9)
        unit test required
        """
        f_x = - np.abs((self.a / x) ** (2*self.b))
        return f_x
