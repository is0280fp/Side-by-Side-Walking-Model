# -*- coding: utf-8 -*-
"""
Created on Mon Oct  9 18:09:46 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


def utility_changing_graph(f, name, y_label, x_label):
    plt.plot(f, "r.-")
#    plt.xticks(np.arange(0, step, 1))   # stepとはステップ数の合計
#    plt.yticks(np.arange(0, 1, 0.2))
    plt.grid()
    plt.title(name)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.show()


def vector_graph(x, y, u, v):
    plt.quiver(x, y, u, v, angles='xy', scale_units='xy', scale=1)
#    x, yはベクトルの始点, u, vはベクトルの成分
    plt.grid()
    plt.draw()
    plt.show()


def utility_color_map(utility, num_grid_x, num_grid_y, name):
    utility_map = utility.reshape(
            num_grid_y, num_grid_x, num_grid_y, num_grid_x).transpose(
                0, 2, 1, 3).reshape(num_grid_y**2, num_grid_x**2)
    print("utlity_map", utility_map.std())
    plt.matshow(utility_map)
    plt.gca().invert_yaxis()
    plt.title(name)
    plt.colorbar()
    plt.show()


def each_utility_visualization():
    pass


if __name__ == '__main__':
    f_o = [0.5, 1]
    f_rv = [1, 1.5]
    f_rd = [1.5, 2.0]
    f_ra = [2.0, 2.5]
    f_s = [2.5, 3.0]
    f_ma = [3.0, 3.5]
    f_mv = [3.5, 4.0]
    f_mw = [4.0, 4.5]
    f = np.array([[1],
                 [1.03],
                 [1.06],
                 [1.09]])
    utility_changing_graph(f, "utility")
