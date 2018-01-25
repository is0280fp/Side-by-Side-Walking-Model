# -*- coding: utf-8 -*-
"""
Created on Wed Jan 24 20:44:29 2018

@author: yume
"""

import numpy as np
import itertools
from collections import defaultdict


def choose_parameters(start, end, interval):
    a = np.arange(start, end, interval)
    b = np.arange(start, end, interval)
    a_b_set = list(itertools.product(a, b))
    return a_b_set


def calculate_error(ideal_walking_p, model_walking_p):
    error = np.linalg.norm(ideal_walking_p - model_walking_p)
    return error


if __name__ == '__main__':
    ideal_walking_p = np.random.rand(1000, 1000)
    model_walking_p = np.random.rand(1000, 1000)
    a_b_set = choose_parameters(0, 5, 0.01)
    error_lst = []

    error_paras_match = defaultdict()

    for ideal_p, model_p, ab in zip(ideal_walking_p, model_walking_p, a_b_set):
        error = calculate_error(ideal_p, model_p)
        error_paras_match.update({error: ab})
        error_lst.append(error)

    error_min = np.min(np.array(error_lst))
    optimum_paras = error_paras_match[error_min]
