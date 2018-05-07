# -*- coding: utf-8 -*-
"""
Created on Wed Nov 15 18:50:23 2017

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt
from itertools import product


def segmentation(background_points, current_scan, threshold):
    #    スキャンデータは(x, y)の二次元array, 単位はmm
    #    background_points: 人がいない場面をLRFでとった１スキャンの点群
    #    current_scan : １スキャンのLRFでとった点群
    #    nearest_distance : current_scanの点群ごとに全background_pointsとの
    #        二点間の距離の差を計算し，current_scanとbackground_pointsの組み合わせのうち距離が
    #        最小の組み合わせの距離をnearest_distanceとする
    #    current_fg : threshold(閾値)を超えたnearest_distanceをもつcurrent_scanの点のみ
    #        current_fgに格納、超えなかった場合(0, 0)を格納(この時何もしないとplotができない)
    assert(background_points.ndim == 2)
    assert(background_points.shape[1] == 2)
    assert(current_scan.ndim == 2)
    assert(current_scan.shape[1] == 2)
    assert(threshold >= 0)
    nearest_distance_scholar = []
    current_fg = []
    current_bg = []

    for point in current_scan:
        nearest_distance = \
        ((point[None, None, :] - background_points) ** 2).sum(2).min(1) ** 0.5
        nearest_distance_scholar.append(np.linalg.norm(nearest_distance))

    for i, raw_point in enumerate(current_scan):
        if nearest_distance_scholar[i] >= threshold:
            current_fg.append(raw_point)
        else:
            current_bg.append(raw_point)
    return np.array(current_fg), np.array(current_bg)


if __name__ == '__main__':
    threshold = 250
    record_current_fg = []
    record_current_bg = []
    human_center_x = []
    human_center_y = []

    sequence_scans = np.load(
            "C:/Users/yume/Desktop/URG-40LX/2017.09.27-14.57.16_kubo.npy")
    background_points = sequence_scans[0].transpose()
    for i, current_scan in enumerate(sequence_scans):
        current_scan = current_scan.transpose()
        current_fg, current_bg = \
            segmentation(background_points, current_scan, threshold)
        if len(current_fg) == 0:
            pass
        else:
            current_fg = current_fg / 1000
            human_center_x.append(np.sum(current_fg[:, 0])/len(current_fg))
            human_center_y.append(np.sum(current_fg[:, 1])/len(current_fg))
        current_bg = current_bg / 1000
        record_current_fg.append(current_fg)
        record_current_bg.append(current_bg)
        print("frame", i)

#    for i in range(len(human_center_x)):
#        plt.figure()
#        plt.xlim(-3, 3)
#        plt.ylim(-3, 3)
#        plt.grid()
#        plt.plot(human_center_x[i], human_center_y[i], ".")
#        plt.show()

#        plt.xlim(-4, 4)
#        plt.ylim(-4, 4)
#        plt.grid()
#        plt.plot(human_center_x, human_center_y, ".")
#        plt.show()

    for i in range(len(record_current_fg)):
        for j in range(len(record_current_fg[i])):
            plt.scatter(record_current_fg[i][j][0],
                        record_current_fg[i][j][1], c="orange")
            plt.xlabel(u"y coordinate[m]")
            plt.ylabel(u"x coordinate[m]")
            plt.xlim([-2, 5])
            plt.ylim([-2, 3])
            plt.grid(True)
