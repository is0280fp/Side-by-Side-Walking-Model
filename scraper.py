from collections import defaultdict
import numpy as np


class UtilityScraper(object):
    def __init__(self, num_grid_x, num_grid_y):
        self.num_grid_x = num_grid_x
        self.num_grid_y = num_grid_y
        self.utilities = defaultdict(list)
        self.factors = defaultdict(list)
        self.ra_values = defaultdict(list)

    def add(self, f_ma_me, f_ma_you, f_mv_me, f_mv_you,
            f_mw_me, f_mw_you, f_ra, f_rd, f_rv,
            f_o_me, f_o_you, f_s_me, f_s_you,
            o_me_dis, o_you_dis, s_me_theta, s_you_theta, rv_vec,
            rd_dis, ra_theta, mv_me_vec, mv_you_vec, ma_me_gal,
            ma_you_gal, mw_me_rad, mw_you_rad):
        self.utilities["f_rv"].append(f_rv)
        self.utilities["f_rd"].append(f_rd)
        self.utilities["f_ra"].append(f_ra)
        self.utilities["f_mv_me"].append(f_mv_me)
        self.utilities["f_mv_you"].append(f_mv_you)
        self.utilities["f_ma_me"].append(f_ma_me)
        self.utilities["f_ma_you"].append(f_ma_you)
        self.utilities["f_mw_me"].append(f_mw_me)
        self.utilities["f_mw_you"].append(f_mw_you)
        self.utilities["f_o_me"].append(f_o_me)
        self.utilities["f_o_you"].append(f_o_you)
        self.utilities["f_s_me"].append(f_s_me)
        self.utilities["f_s_you"].append(f_s_you)

        self.factors["rv_vec"].append(rv_vec)
        self.factors["rd_dis"].append(rd_dis)
        self.factors["ra_theta"].append(ra_theta)
        self.factors["mv_me_vec"].append(mv_me_vec)
        self.factors["mv_you_vec"].append(mv_you_vec)
        self.factors["ma_me_gal"].append(ma_me_gal)
        self.factors["ma_you_gal"].append(ma_you_gal)
        self.factors["mw_me_rad"].append(mw_me_rad)
        self.factors["mw_you_rad"].append(mw_you_rad)
        self.factors["o_me_dis"].append(o_me_dis)
        self.factors["o_you_dis"].append(o_you_dis)
        self.factors["s_me_theta"].append(s_me_theta)
        self.factors["s_you_theta"].append(s_you_theta)

    def add_ra_values(self, p_me, p_you, d_you, v_yoko):
        self.ra_values["p_me"].append(p_me)
        self.ra_values["p_you"].append(p_you)
        self.ra_values["v_mae"].append(d_you)
        self.ra_values["v_yoko"].append(v_yoko)
#        self.ra_values["theta_mae"].append(theta_mae)
#        self.ra_values["theta_yoko"].append(theta_yoko)
#        self.ra_values["theta"].append(theta)
#        self.ra_values["r_a"].append(r_a)

    def get_utility_maps(self):
        maps = {}
        for name, lst in self.utilities.items():
            assert not np.any(np.isnan(lst)), "{}, {}".format(name, lst)
            lst = np.array(lst)
            num_step = len(lst) // (self.num_grid_y * self.num_grid_x *
                    self.num_grid_y * self.num_grid_x)
            maps[name] = lst.reshape(
                    num_step, self.num_grid_y, self.num_grid_x,
                    self.num_grid_y, self.num_grid_x)
        return maps

    def get_factors_maps(self):
        theta_maps = {}
        for name, lst in self.factors.items():
            assert not np.any(np.isnan(lst)), "{}, {}".format(name, lst)
            lst = np.array(lst)
            num_step = len(lst) // (self.num_grid_y * self.num_grid_x *
                    self.num_grid_y * self.num_grid_x)
            theta_maps[name] = lst.reshape(
                    num_step, self.num_grid_y, self.num_grid_x,
                    self.num_grid_y, self.num_grid_x)
        return theta_maps

    def get_values_maps(self):
        values_maps = {}
        for name, lst in self.ra_values.items():
            assert not np.any(np.isnan(lst)), "{}, {}".format(name, lst)
            lst = np.array(lst)
            num_step = len(lst) // (self.num_grid_y * self.num_grid_x *
                    self.num_grid_y * self.num_grid_x)
            values_maps[name] = lst.reshape(
                    num_step, self.num_grid_y, self.num_grid_x,
                    self.num_grid_y, self.num_grid_x, len(lst[0]))
        return values_maps
