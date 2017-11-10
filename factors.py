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
    def factor(self, states, environment):
        d_me = states.s_me.d_me
        d_you = states.s_you.d_you
        return self.relative_velocity(d_me, d_you, self.d_t)

    def relative_velocity(self, d_me, d_you, d_t):
        """
        unit test required
        """
        return np.linalg.norm((d_me - d_you) / d_t)


class RelativeDistance(BaseFactor):
    def factor(self, states, environment):
        p_me = states.s_me.p_me
        p_you = states.s_you.p_you
        return self.relative_distance(p_me, p_you)

    def relative_distance(self, p_me, p_you):
        return np.linalg.norm(p_you - p_me)


class RelativeAngle(BaseFactor):
    def factor(self, states, environment):
        p_me = states.s_me.p_me
        p_you = states.s_you.p_you
        d_you = states.s_you.d_you
        return self.relative_angle(p_me, p_you, d_you)

    def relative_angle(self, p_me, p_you, d_you):
        """
        unit test required
        """
        # youの進行方向の絶対角度
        theta_mae = np.arctan2(d_you[1], d_you[0])
        theta_yoko = absolute_angle(p_you, p_me)
        theta = theta_yoko - theta_mae
        r_a = revision_theta(theta)
        return np.abs(r_a)


class Velocity(BaseFactor):
    def factor(self, states, environment):
        p = states.s_me.p_me
        next_p = states.next_s_me.next_p_me
        return self.velocity(p, next_p, self.d_t)

    def velocity(self, p, next_p, d_t):
        return motion_velocity(p, next_p, d_t)


class Acceleration(BaseFactor):
    def factor(self, states, environment):
        prev_p = states.prev_s_me.prev_p_me
        p = states.s_me.p_me
        next_p = states.next_s_me.next_p_me
        return self.acceleration(prev_p, p, next_p, self.d_t)

    def acceleration(self, prev_p, p, next_p, d_t):
        prev_m_v = motion_velocity(prev_p, p, d_t)
        motion_v = motion_velocity(p, next_p, d_t)
        return motion_acceleration(prev_m_v, motion_v)


class AngularVelocity(BaseFactor):
    def factor(self, states, environment):
        prev_p = states.prev_s_me.prev_p_me
        p = states.s_me.p_me
        next_p = states.next_s_me.next_p_me
        return self.angularvelocity(prev_p, p, next_p, self.d_t)

    def angularvelocity(self, prev_p, p, next_p, d_t):
        # 現在と予測による角速度
        return motion_angular_velocity(prev_p, p, next_p, d_t)


class MovingTowardSubgoals(BaseFactor):
    def factor(self, states, environment):
        p = states.s_me.p_me
        next_p = states.next_s_me.next_p_me
        subgoal_p = environment.current_subgoal
        return self.moving_toward_subgoals(p, next_p, subgoal_p)

    def moving_toward_subgoals(self, p, next_p, subgoal_p):
        e_o = absolute_angle(p, subgoal_p) - absolute_angle(p, next_p)
        return e_o


class DistanceToObstacle(BaseFactor):
    def factor(self, states, environment):
        p = states.s_me.p_me
        obstacle_p = environment.closest_obstacle
        return self.distance_to_obstacle(p, obstacle_p)

    def distance_to_obstacle(self, p, obstacle_p):
        return np.sqrt(np.sum((obstacle_p - p) ** 2))
