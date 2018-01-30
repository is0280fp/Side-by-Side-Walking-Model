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
    a1 = np.arange(start_a, end, interval)
    b1 = np.arange(start, end, interval)
    a2 = np.arange(start_a, end, interval)
    b2 = np.arange(start, end, interval)
    a3 = np.arange(start_a, end, interval)
    b3 = np.arange(start, end, interval)
    a4 = np.arange(start_a, end, interval)
    b4 = np.arange(start, end, interval)
    a5 = np.arange(start_a, end, interval)
    b5 = np.arange(start, end, interval)
    a6 = np.arange(start_a, end, interval)
    b6 = np.arange(start, end, interval)
    a_b_set = list(itertools.product(
            a1, b1, a2, b2, a3, b3, a4, b4, a5, b5, a6, b6))
    return a_b_set


def calculate_error(ideal_walking_p, model_walking_p):
    error = np.linalg.norm(ideal_walking_p - model_walking_p)
    return error


if __name__ == '__main__':
    ideal_walking_p = np.random.rand(1000, 1000)
    model_walking_p = np.random.rand(1000, 1000)
    a_b_set = choose_parameters(0, 2, 1)
    error_lst = []

    error_paras_match = defaultdict()

    for ideal_p, model_p, ab in zip(ideal_walking_p, model_walking_p, a_b_set):
        error = calculate_error(ideal_p, model_p)
        error_paras_match.update({error: ab})
        error_lst.append(error)

    error_min = np.min(np.array(error_lst))
    optimum_paras = error_paras_match[error_min]
