# -*- coding: utf-8 -*-
"""
Created on Wed Jan 24 20:44:29 2018

@author: yume
"""

import numpy as np
import itertools
from collections import defaultdict


def choose_parameters(start, end, interval):
    #    aは分母で０は当てはまらないため
    if start == 0:
        start_a = start + 0.01
    k1 = np.arange(start_a, end, interval)
    k2 = np.arange(start_a, end, interval)
    k3 = np.arange(start_a, end, interval)
    k4 = np.arange(start_a, end, interval)
#    k5 = np.arange(start_a, end, interval)
#    k6 = np.arange(start_a, end, interval)
#    k7 = np.arange(start_a, end, interval)
#    k_set = itertools.product(k1, k2, k3, k4, k5, k6, k7)
    k_set = itertools.product(k1, k2, k3, k4)
    return k_set


def calculate_error(ideal_walking_p, model_walking_p):
    error = np.linalg.norm(ideal_walking_p - model_walking_p)
    return error


if __name__ == '__main__':
    ideal_walking_p = np.random.rand(1000, 1000)
    model_walking_p = np.random.rand(1000, 1000)
    a_b_set = choose_parameters(0, 5, 1)
    error_lst = []

    error_paras_match = defaultdict()

    for ideal_p, model_p, ab in zip(ideal_walking_p, model_walking_p, a_b_set):
        error = calculate_error(ideal_p, model_p)
        error_paras_match.update({error: ab})
        error_lst.append(error)

    error_min = np.min(np.array(error_lst))
    optimum_paras = error_paras_match[error_min]
