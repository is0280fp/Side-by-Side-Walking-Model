# -*- coding: utf-8 -*-
"""
Created on Tue Feb  6 19:33:27 2018

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt


def load_default_trajectory():
    ps = np.array(([
       [-0.71504237,  1.33176684],
       [-0.78603195,  1.54451363],
       [-0.70289951,  1.46176321],
       [-0.63509856,  1.43071117],
       [-0.54239623,  1.3914304 ],
       [-0.4185612 ,  1.20082329],
       [-0.36874153,  1.21616726],
       [-0.31788558,  1.20425631],
       [-0.25074353,  1.12976745],
       [-0.20865555,  1.08604685],
       [-0.11988983,  1.01490709],
       [-0.06250292,  0.95092731],
       [ 0.01618663,  0.85350577],
       [ 0.0628278 ,  0.73883679],
       [ 0.0947361 ,  0.62271765],
       [ 0.15065905,  0.55226916],
       [ 0.17746561,  0.44407741],
       [ 0.17853439,  0.37345421],
       [ 0.16172696,  0.27745659],
       [ 0.1464408 ,  0.17701241],
       [ 0.13554955,  0.06062307],
       [ 0.18095232,  0.04642461],
       [ 0.24628457, -0.04325619],
       [ 0.28102871, -0.1100777 ],
       [ 0.31594001, -0.15280309],
       [ 0.34736515, -0.16454686],
       [ 0.40609506, -0.2195033 ],
       [ 0.48512315, -0.28079309],
       [ 0.5587699 , -0.31377022],
       [ 0.61295656, -0.33861096],
       [ 0.6822207 , -0.36393129],
       [ 0.74366915, -0.38727752],
       [ 0.85550713, -0.4213548 ],
       [ 0.97163821, -0.4467928 ],
       [ 0.95916461, -0.40775297],
       [ 1.10534894, -0.43547312],
       [ 1.20123508, -0.44817604],
       [ 1.25241983, -0.43638611],
       [ 1.35164554, -0.42040295],
       [ 1.54373886, -0.42310783],
       [ 1.58934171, -0.43108063],
       [ 1.57249958, -0.40048176],
       [ 1.65849966, -0.40137003],
       [ 1.80999336, -0.41492906],
       [ 1.91711779, -0.40820116],
       [ 2.25809646, -0.43774319],
       [ 2.35934682, -0.40432398],
       [ 2.24570412, -0.31895612],
       [ 2.04662948, -0.25803637],
       [ 2.19484354, -0.22902366],
       [ 2.50231044, -0.20001084],
       [ 2.55271945, -0.15675999],
       [ 2.30173429, -0.10536596],
       [ 2.69879535, -0.08254421]
             ]))

    ps_for_d = np.array([[[-0.01730455,  0.08847307],
        [-0.07098958,  0.21274679],
        [ 0.08313244, -0.08275042],
        [ 0.06780095, -0.03105204],
        [ 0.09270233, -0.03928077],
        [ 0.12383503, -0.19060711],
        [ 0.04981967,  0.01534397],
        [ 0.05085595, -0.01191095],
        [ 0.06714205, -0.07448886],
        [ 0.04208798, -0.0437206 ],
        [ 0.08876572, -0.07113976],
        [ 0.05738691, -0.06397978],
        [ 0.07868955, -0.09742154],
        [ 0.04664117, -0.11466898],
        [ 0.0319083 , -0.11611914],
        [ 0.05592295, -0.07044849],
        [ 0.02680656, -0.10819175],
        [ 0.00106878, -0.0706232 ],
        [-0.01680743, -0.09599762],
        [-0.01528616, -0.10044418],
        [-0.01089125, -0.11638934],
        [ 0.04540277, -0.01419846],
        [ 0.06533225, -0.0896808 ],
        [ 0.03474414, -0.06682151],
        [ 0.0349113 , -0.04272539],
        [ 0.03142514, -0.01174377],
        [ 0.05872991, -0.05495644],
        [ 0.07902809, -0.06128979],
        [ 0.07364675, -0.03297713],
        [ 0.05418666, -0.02484074],
        [ 0.06926414, -0.02532033],
        [ 0.06144845, -0.02334623],
        [ 0.11183798, -0.03407728],
        [ 0.11613108, -0.025438  ],
        [-0.0124736 ,  0.03903983],
        [ 0.14618433, -0.02772015],
        [ 0.09588614, -0.01270292],
        [ 0.05118475,  0.01178993],
        [ 0.09922571,  0.01598316],
        [ 0.19209332, -0.00270488],
        [ 0.04560285, -0.0079728 ],
        [-0.01684213,  0.03059887],
        [ 0.08600008, -0.00088827],
        [ 0.1514937 , -0.01355903],
        [ 0.10712443,  0.0067279 ],
        [ 0.34097867, -0.02954203],
        [ 0.10125036,  0.03341921],
        [-0.1136427 ,  0.08536786],
        [-0.19907464,  0.06091975],
        [ 0.14821406,  0.02901271],
        [ 0.3074669 ,  0.02901282],
        [ 0.05040901,  0.04325085],
        [-0.25098516,  0.05139403],
        [ 0.39706106,  0.02282175]]])
    return (ps, ps_for_d)


if __name__ == '__main__':
    ps, ds_for_d = load_default_trajectory()
    for p in ps:
        plt.plot(p[0], p[1], "*", color='#ff7f00')
