# -*- coding: utf-8 -*-
"""
Created on Tue Feb  6 19:33:27 2018

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


def load_default_trajectory():
    ps = np.array(([
       [-0.71415886,  1.36807146],
       [-0.65942028,  1.31560818],
       [-0.57682281,  1.19824551],
       [-0.54499915,  1.19756838],
       [-0.48678823,  1.15174791],
       [-0.39470556,  1.07161914],
       [-0.28674321,  1.01082418],
       [-0.21545214,  0.96727308],
       [-0.14760272,  0.87146496],
       [-0.06898178,  0.81733056],
       [ 0.02074634,  0.70566916],
       [ 0.11299937,  0.59961756],
       [ 0.12287757,  0.45621744],
       [ 0.14029353,  0.33132491],
       [ 0.13169849,  0.24251965],
       [ 0.14806012,  0.14390148],
       [ 0.2153268 ,  0.0616198 ],
       [ 0.30689797, -0.00254006],
       [ 0.38863397, -0.06693987],
       [ 0.45917671, -0.12264059],
       [ 0.59018274, -0.17908189],
       [ 0.71385341, -0.24958655],
       [ 0.81032887, -0.31678456],
       [ 0.87588951, -0.35268465],
       [ 0.99197345, -0.38970301],
       [ 1.08601079, -0.4110579 ],
       [ 1.17439486, -0.41878866],
       [ 1.26354752, -0.42823732],
       [ 1.41635381, -0.44044825],
       [ 1.46756639, -0.42105154],
       [ 1.69479717, -0.44748277],
       [ 1.68158531, -0.36343995],
       [ 1.76502564, -0.31800656],
       [ 1.99205926, -0.33483458],
       [ 1.87275922, -0.2669475 ],
       [ 2.1319081 , -0.25794243],
       [ 2.12192594, -0.2034756 ],
       [ 2.11364122, -0.14326507],
       [ 2.44007374, -0.12841026],
       [ 2.5096367 , -0.10852375],
       [ 2.38058937, -0.08734194],
       [ 2.14511135, -0.05778038],
       [ 2.11534984, -0.03074588]
             ]))

    ps_for_d = np.array([[[-0.05743556,  0.12765584],
        [ 0.05473858, -0.05246328],
        [ 0.08259747, -0.11736267],
        [ 0.03182366, -0.00067713],
        [ 0.05821092, -0.04582047],
        [ 0.09208267, -0.08012877],
        [ 0.10796235, -0.06079496],
        [ 0.07129107, -0.0435511 ],
        [ 0.06784942, -0.09580812],
        [ 0.07862094, -0.0541344 ],
        [ 0.08972812, -0.1116614 ],
        [ 0.09225303, -0.1060516 ],
        [ 0.0098782 , -0.14340012],
        [ 0.01741596, -0.12489253],
        [-0.00859504, -0.08880526],
        [ 0.01636163, -0.09861817],
        [ 0.06726668, -0.08228168],
        [ 0.09157117, -0.06415986],
        [ 0.081736  , -0.06439981],
        [ 0.07054274, -0.05570072],
        [ 0.13100603, -0.0564413 ],
        [ 0.12367067, -0.07050466],
        [ 0.09647546, -0.06719801],
        [ 0.06556064, -0.03590009],
        [ 0.11608394, -0.03701836],
        [ 0.09403734, -0.02135489],
        [ 0.08838407, -0.00773076],
        [ 0.08915266, -0.00944866],
        [ 0.15280629, -0.01221093],
        [ 0.05121258,  0.01939671],
        [ 0.22723078, -0.02643123],
        [-0.01321186,  0.08404282],
        [ 0.08344033,  0.04543339],
        [ 0.22703362, -0.01682802],
        [-0.11930004,  0.06788708],
        [ 0.25914888,  0.00900507],
        [-0.00998216,  0.05446683],
        [-0.00828472,  0.06021053],
        [ 0.32643252,  0.01485481],
        [ 0.06956296,  0.01988651],
        [-0.12904733,  0.02118181],
        [-0.23547802,  0.02956156],
        [-0.02976151,  0.0270345 ]]])
    return (ps, ps_for_d)


if __name__ == '__main__':
    ps, ds_for_d = load_default_trajectory()
    for p in ps:
        plt.plot(p[0], p[1], "*", color='#ff7f00')
