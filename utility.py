import numpy as np


class DistanceToObstacle(object):
    def __init__(self, a, b):
        self.a = a
        self.b = b

    def distance_to_obstacle(self, p, obstacle):
        distance = np.sqrt(np.sum((obstacle - p) ** 2))
        return distance

    def calculation_environmental_factors(self, p, obstacle):
        e_o = self.distance_to_obstacle(p, obstacle)
        return e_o

    def f_obstacle(self, x):
        f_o = - np.abs((self.a / x) ** (2*self.b))
        return f_o

    def calculation_f_o_utility(self, s, obstacle):
        self.s = s
        p = self.s.p
        e_o = self.calculation_environmental_factors(p, obstacle)
        f_o = self.f_obstacle(e_o)
        return f_o


class MovingTowardSubgoals(object):
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def angb(self, p1, p2):
        # p2からみたp1の相対位置ベクトルの絶対角度
        d = np.arctan2(p1[1] - p2[1], p1[0] - p2[0])
        return d

    def calculation_environmental_factors(self, p, next_p, subgoal_p):
        e_s = self.angb(subgoal_p, p) - self.angb(next_p, p)
        return e_s

    def f(self, x):
        """
        eq. (9)
        """
        f_x = 1 / (1+(abs((x-self.c) / self.a)**(2*self.b))) - 1
        return f_x

    def calculation_f_s_utility(self, s, next_s, subgoal):
        self.s = s
        self.next_s = next_s
        self.subgoal = subgoal
        p = self.s.p
        next_p = self.next_s.p
        subgoal_p = self.subgoal.p
        e_s = self.calculation_environmental_factors(p, next_p, subgoal_p)
        f_s = self.f(e_s)
        return f_s


class RelativeVelocity(object):
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def calculation_relative_factors(self, d_me, d_you, d_t):
        # (relative factor)
        # relative_angle
        r_v = np.linalg.norm((d_me - d_you) / d_t)  # relative_v
        return r_v

    def f(self, x):
        """
        eq. (9)
        """
        f_x = 1 / (1+(abs((x-self.c) / self.a)**(2*self.b))) - 1
        return f_x

    def calculation_f_rv_utility(self, s_me, s_you, d_t):
        self.s_me = s_me
        self.s_you = s_you
        d_me = self.s_me.d
        d_you = self.s_you.d
        r_v = self.calculation_relative_factors(d_me, d_you, d_t)
        f_rv = self.f(r_v)
        return f_rv


class RelativeDistance(object):
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def calculation_relative_factors(self, p_me, p_you):
        # (relative factor)
        r_d = np.linalg.norm(p_you - p_me)  # socialrelativedistance
        return r_d

    def f(self, x):
        """
        eq. (9)
        """
        f_x = 1 / (1+(abs((x-self.c) / self.a)**(2*self.b))) - 1
        return f_x

    def calculation_f_rd_utility(self, s_me, s_you):
        self.s_me = s_me
        self.s_you = s_you
        p_you = self.s_you.p
        p_me = self.s_me.p
        r_d = self.calculation_relative_factors(p_me, p_you)
        f_rd = self.f(r_d)
        return f_rd


class RelativeAngle(object):
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def angb(self, p1, p2):
        # p2からみたp1の相対位置ベクトルの絶対角度
        d = np.arctan2(p1[1] - p2[1], p1[0] - p2[0])
        return d

    def revision_theta(self, theta):
        theta += np.pi
        theta %= 2 * np.pi
        if theta < 0:
            theta += np.pi
        else:
            theta -= np.pi
        r_a = theta
        return r_a

    def relative_angle(self, p_me, p_you, d_you):
        # youの進行方向の絶対角度
        theta_mae = np.arctan2(d_you[1], d_you[0])
        theta_yoko = self.angb(p_me, p_you)
        theta = theta_yoko - theta_mae
        r_a = self.revision_theta(theta)
        return np.abs(r_a)

    def calculation_relative_factors(self, p_me, p_you, d_you):
        r_a = self.relative_angle(p_me, p_you, d_you)
        return r_a

    def f(self, x):
        """
        eq. (9)
        """
        f_x = 1 / (1+(abs((x-self.c) / self.a)**(2*self.b))) - 1
        return f_x

    def calculation_f_ra_utility(self, s_me, s_you):
        self.s_me = s_me
        self.s_you = s_you
        p_me = self.s_me.p
        p_you = self.s_you.p
        d_you = self.s_you.d
        r_a = self.calculation_relative_factors(p_me, p_you, d_you)
        f_ra = self.f(r_a)
        return f_ra, r_a


class Acceleration(object):
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def m_v(self, prev_p, p, next_p, d_t):
        """eq. (1)
        """
        prev_m_v = np.sqrt(np.sum((p - prev_p) ** 2)) / d_t
        m_v = np.sqrt(np.sum((next_p - p) ** 2)) / d_t
        return prev_m_v, m_v

    def m_a(self, prev_m_v, motion_v):
        m_a = motion_v - prev_m_v
        return m_a

    def calculation_motion_factors(self, prev_p, p, next_p, d_t):
        prev_m_v, motion_v = self.m_v(prev_p, p, next_p, d_t)
        motion_a = self.m_a(prev_m_v, motion_v)
        return motion_a

    def f(self, x):
        """
        eq. (9)
        """
        f_x = 1 / (1+(abs((x-self.c) / self.a)**(2*self.b))) - 1
        return f_x

    def calculation_f_ma_utility(self, prev_s, s, next_s, d_t):
        self.prev_s = prev_s
        self.s = s
        self.next_s = next_s
        prev_p = self.prev_s.p
        p = self.s.p
        next_p = self.next_s.p
        m_a = self.calculation_motion_factors(prev_p, p, next_p, d_t)
        f_ma = self.f(m_a)
        return f_ma


class Velocity(object):
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def m_v(self, p, next_p, d_t):
        """eq. (1)
        """
        m_v = np.sqrt(np.sum((next_p - p) ** 2)) / d_t
        return m_v

    def calculation_motion_factors(self, p, next_p, d_t):
        motion_v = self.m_v(p, next_p, d_t)
        return motion_v

    def f(self, x):
        """
        eq. (9)
        """
        f_x = 1 / (1+(abs((x-self.c) / self.a)**(2*self.b))) - 1
        return f_x

    def calculation_f_mv_utility(self, s, next_s, d_t):
        self.s = s
        self.next_s = next_s
        p = self.s.p
        next_p = self.next_s.p
        m_v = self.calculation_motion_factors(p, next_p, d_t)
        f_mv = self.f(m_v)
        return f_mv


class AngularVelocity(object):
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def angb(self, p1, p2):
        # p2からみたp1の相対位置ベクトルの絶対角度
        d = np.arctan2(p1[1] - p2[1], p1[0] - p2[0])
        return d

    def m_w(self, prev_p, p, next_p, d_t):
        """eq. (2)
        """
        ang = self.angb(p, prev_p)       # 時刻tのd_t(direction=向き)
        next_ang = self.angb(next_p, p)  # 時刻tのd_t+1(direction=向き)
        return (next_ang - ang) / d_t

    def calculation_motion_factors(self, prev_p, p, next_p, d_t):
        motion_w = self.m_w(prev_p, p, next_p, d_t)  # 現在と予測による角速度
        return motion_w

    def f(self, x):
        """
        eq. (9)
        """
        f_x = 1 / (1+(abs((x-self.c) / self.a)**(2*self.b))) - 1
        return f_x

    def calculation_f_mw_utility(self, prev_s, s, next_s, d_t):
        self.prev_s = prev_s
        self.s = s
        self.next_s = next_s
        p = self.s.p
        prev_p = self.prev_s.p
        next_p = self.next_s.p
        m_w = self.calculation_motion_factors(prev_p, p, next_p, d_t)
        f_mw = self.f(m_w)
        return f_mw
