# -*- coding: utf-8 -*-
"""
Created on Tue Jan 23 19:26:46 2018

@author: yume
"""

import numpy as np
import matplotlib.pyplot as plt

# meは人間の実測データ、 youはモデルとする


class Human(object):
    def __init__(self):
        pass

    def load_default_trajectory(self):
#            self.ps = np.array(([
#                    [ 1.46679423, -0.74534951], [-0.11414827,  0.08606548],
#                    [ 1.39638344, -0.67077226], [-0.3988092,   0.91703392],
#                    [ 1.32672098, -0.61340289], [-0.38920179,  0.92115252],
#                    [ 1.2942371, -0.57622903], [-0.38552571,  0.92269709],
#                    [ 1.26490391, -0.53608475], [-0.38273019,  0.92386017],
#                    [ 1.22321161, -0.47479324], [-0.37902185,  0.92538772],
#                    [ 1.16483836, -0.40727477], [-0.37184356,  0.92829541],
#                    [ 1.14530818, -0.31336   ], [-0.37619838,  0.92653914],
#                    [ 1.11968058, -0.23674335], [-0.37754404,  0.92599163],
#                    [ 1.04601449, -0.2186385 ], [-0.36101676,  0.93255933],
#                    [ 1.00023514, -0.15546043], [-0.35551803,  0.93466942],
#                    [ 0.95239904, -0.0957841 ], [-0.34886118,  0.93717441],
#                    [ 0.87612048, -0.07253448], [-0.33055111,  0.94378809],
#                    [ 0.78945409,  0.00249666], [-0.31345687,  0.94960244],
#                    [ 0.70120478,  0.05358691], [-0.29248921,  0.95626882],
#                    [ 0.60321063,  0.09432984], [-0.26643649,  0.96385248],
#                    [ 0.5014033,  0.12624419], [-0.23711164,  0.97148241],
#                    [ 0.45867407,  0.15008249], [-0.22518434,  0.97431618],
#                    [ 0.37421661,  0.1745228 ], [-0.19915704,  0.97996759],
#                    [ 0.27076646,  0.19743203], [-0.16565599,  0.9861836 ],
#                    [ 0.14048745,  0.18246685], [-0.11997309,  0.99277714],
#                    [ 0.08395046,  0.19306435], [-0.10064663,  0.99492224],
#                    [ 0.00867053,  0.25282692], [-0.07574009,  0.99712759],
#                    [-0.05588471,  0.32456707], [-0.05378818,  0.99855237],
#                    [-0.12345001,  0.38608875], [-0.02927306,  0.99957145],
#                    [-0.21338916,  0.48374702], [ 0.005321,    0.99998584],
#                    [-0.29500849,  0.56828673], [ 0.03904081,  0.99923762],
#                    [-0.33423561,  0.66567379], [ 0.05741023,  0.99835067],
#                    [-0.36071933,  0.73845131], [ 0.07088727,  0.99748433],
#                    [-0.37111547,  0.79702674], [ 0.07744152,  0.9969969 ],
#                    [-0.40301438,  0.83692239], [ 0.09344376,  0.99562456],
#                    [-0.43437895,  0.91540344], [ 0.11172973,  0.99373863],
#                    [-0.4597754,   1.02586037], [ 0.13046448,  0.99145298],
#                    [-0.45748766,  1.14414567], [ 0.13742705,  0.99051189],
#                    [-0.442752,    1.25681061], [ 0.13792643,  0.99044248],
#                    [-0.463554,    1.36427616], [ 0.15907218,  0.98726696],
#                    [-0.49261661,  1.41474701], [ 0.1815202,   0.98338722],
#                    [-0.52770858,  1.51294145], [ 0.21520985,  0.97656783],
#                    [-0.51919536,  1.63566784], [ 0.22780569,  0.97370661],
#                    [-0.48624499,  1.75864933], [ 0.22469513,  0.97442911],
#                    [-0.47801972,  1.8758865 ], [ 0.2400895,   0.97075076],
#                    [-0.45179786,  1.95211507], [ 0.23364095,  0.97232294],
#                    [-0.44222466,  2.02118731], [ 0.24022151,  0.9707181 ],
#                    [-0.42548709,  2.09929104], [ 0.24284969,  0.97006393],
#                    [-0.39631127,  2.19805512], [ 0.23777343,  0.97132064],
#                    [-0.36529998,  2.30854525], [ 0.23250949,  0.97259413],
#                    [-0.31280355,  2.41014342], [ 0.187835, 0.9822006],
#                    [-0.29936159,  2.47706637], [ 0.18666827,  0.982423],
#                    [-0.27526739,  2.54106418], [ 0.16184206,  0.98681667],
#                    [-0.26741257,  2.61736335], [ 0.17350691,  0.98483265],
#                    [-0.2345307,   2.80601753], [ 0.17525437,  0.98452319],
#                    [-0.19906686,  2.92899299], [-0.01314044,  0.99991366]
#            ]))
        self.ps = np.array(([
                    [ 1.46679423, -0.74534951],
                    [ 1.39638344, -0.67077226],
                    [ 1.32672098, -0.61340289],
                    [ 1.2942371, -0.57622903],
                    [ 1.26490391, -0.53608475],
                    [ 1.22321161, -0.47479324],
                    [ 1.16483836, -0.40727477],
                    [ 1.14530818, -0.31336   ],
                    [ 1.11968058, -0.23674335],
                    [ 1.04601449, -0.2186385 ],
                    [ 1.00023514, -0.15546043],
                    [ 0.95239904, -0.0957841 ],
                    [ 0.87612048, -0.07253448],
                    [ 0.78945409,  0.00249666],
                    [ 0.70120478,  0.05358691],
                    [ 0.60321063,  0.09432984],
                    [ 0.5014033,  0.12624419],
                    [ 0.45867407,  0.15008249],
                    [ 0.37421661,  0.1745228],
                    [ 0.27076646,  0.19743203],
                    [ 0.14048745,  0.18246685],
                    [ 0.08395046,  0.19306435],
                    [ 0.00867053,  0.25282692],
                    [-0.05588471,  0.32456707],
                    [-0.12345001,  0.38608875],
                    [-0.21338916,  0.48374702],
                    [-0.29500849,  0.56828673],
                    [-0.33423561,  0.66567379],
                    [-0.36071933,  0.73845131],
                    [-0.37111547,  0.79702674],
                    [-0.40301438,  0.83692239],
                    [-0.43437895,  0.91540344],
                    [-0.4597754,   1.02586037],
                    [-0.45748766,  1.14414567],
                    [-0.442752,    1.25681061],
                    [-0.463554,    1.36427616],
                    [-0.49261661,  1.41474701],
                    [-0.52770858,  1.51294145],
                    [-0.51919536,  1.63566784],
                    [-0.48624499,  1.75864933],
                    [-0.47801972,  1.8758865],
                    [-0.45179786,  1.95211507],
                    [-0.44222466,  2.02118731],
                    [-0.42548709,  2.09929104],
                    [-0.39631127,  2.19805512],
                    [-0.36529998,  2.30854525],
                    [-0.31280355,  2.41014342],
                    [-0.29936159,  2.47706637],
                    [-0.27526739,  2.54106418],
                    [-0.26741257,  2.61736335],
                    [-0.2345307,   2.80601753],
                    [-0.19906686,  2.92899299]
            ]))

    def decide_action(self):
        #  提案モデルに対する比較手法
        #   self.v = walking_model()
        #   self.count += 1
        #   self.v = ps[self.count]
        if self.ps.size != 0:
            self.v = self.ps[0]
            self.ps = self.ps[1:]


class Robot(object):
    def __init__(self):
        pass

    def load_default_trajectory(self):
#            self.ps = np.array(([
#                    [0.75264596,  0.55928402], [-0.12829653, -0.07213097],
#                    [ 0.73546054,  0.59826416], [-0.3629356,   0.93181423],
#                    [ 0.71323831,  0.62048638], [-0.35830934,  0.93360292],
#                    [ 0.69101609,  0.6427086 ], [-0.35356856,  0.93540861],
#                    [ 0.66879387,  0.66493083], [-0.34870936,  0.93723091],
#                    [ 0.64657165,  0.68715305], [-0.34372768,  0.93906937],
#                    [ 0.62434943,  0.70937527], [-0.33861929,  0.94092347],
#                    [ 0.6021272,   0.73159749], [-0.33337981,  0.94279261],
#                    [ 0.57990498,  0.79826416], [-0.33389414,  0.94261058],
#                    [ 0.55768276,  0.86493083], [-0.33444025,  0.94241695],
#                    [ 0.53546054,  0.88715305], [-0.32874282,  0.94441948],
#                    [ 0.51323831,  0.95381972], [-0.32914772,  0.94427844],
#                    [ 0.49101609,  1.02048638], [-0.32957969,  0.94412776],
#                    [ 0.46879387,  1.08715305], [-0.33004154,  0.94396641],
#                    [ 0.44657165, 1.15381972], [-0.33053648, 0.94379322],
#                    [ 0.37990498,  1.22048638], [-0.30984133,  0.95078828],
#                    [ 0.35768276,  1.28715305], [-0.3095919,   0.95086953],
#                    [ 0.33546054,  1.35381972], [-0.3093222,  0.9509573],
#                    [ 0.31323831,  1.42048638], [-0.30902965,  0.95105241],
#                    [ 0.29101609,  1.4427086 ], [-0.30070803,  0.95371625],
#                    [ 0.17990498,  1.50937527], [-0.24696819,  0.96902359],
#                    [ 0.2021272,   1.57604194], [-0.27177192,  0.96236169],
#                    [ 0.17990498,  1.59826416], [-0.26158752,  0.96517976],
#                    [ 0.15768276,  1.62048638], [-0.25098259,  0.9679916 ],
#                    [ 0.13546054,  1.6427086 ], [-0.23993477,  0.97078901],
#                    [ 0.06879387,  1.70937527], [-0.20389153,  0.97899349],
#                    [ 0.0021272,   1.77604194], [-0.16293542,  0.98663674],
#                    [-0.10898391,  1.88715305], [-0.08151454,  0.99667215],
#                    [-0.13120613,  1.95381972], [-0.06561548,  0.99784498],
#                    [-0.15342835,  2.02048638], [-0.04749204,  0.99887162],
#                    [-0.17565057,  2.08715305], [-0.02666468,  0.99964443],
#                    [-0.1978728,   2.15381972], [-0.00251388,  0.99999684],
#                    [-0.22009502,  2.22048638], [ 0.02577036,  0.99966789],
#                    [-0.24231724,  2.28715305], [ 0.05925939,  0.99824262],
#                    [-0.26453946,  2.35381972], [ 0.09938393,  0.99504916],
#                    [-0.28676169,  2.42048638], [ 0.14806446,  0.98897771],
#                    [-0.26453946,  2.48715305], [ 0.12486064,  0.99217429],
#                    [-0.28676169,  2.55381972], [ 0.190879,    0.98161357],
#                    [-0.26453946,  2.62048638], [ 0.16765141,  0.98584634],
#                    [-0.1978728,   2.73159749], [-0.00792517,  0.9999686 ],
#                    [-0.17565057,  2.8427086 ], [-0.15298234,  0.98822892],
#                    [-0.15342835,  2.95381972], [-0.71008407,  0.7041169 ],
#                    [-0.30898391,  3.06493083], [ 0.85908683, -0.51182986],
#                    [-0.10898391,  2.95381972], [-0.89177703,  0.45247511],
#                    [-0.26453946,  3.02048638], [ 0.95313422, -0.30254779],
#                    [-0.15342835,  2.99826416], [-0.99930611,  0.03724659],
#                    [-0.26453946,  3.02048638], [ 0.95313422, -0.30254779],
#                    [-0.10898391,  2.99826416], [-0.99981818,  0.01906832],
#                    [-0.17565057,  3.02048638], [-0.76519597, -0.64379743],
#                    [-0.1978728,   2.99826416], [-0.77477769,  0.63223377],
#                    [-0.3978728,   3.19826416], [ 0.70640785, -0.70780502],
#                    [-0.37565057,  3.17604194], [ 0.70631947, -0.70789321]
#                    ]))
        self.ps = np.array(([
                    [0.75264596,  0.55928402],
                    [ 0.73546054,  0.59826416],
                    [ 0.71323831,  0.62048638],
                    [ 0.69101609,  0.6427086 ],
                    [ 0.66879387,  0.66493083],
                    [ 0.64657165,  0.68715305],
                    [ 0.62434943,  0.70937527],
                    [ 0.6021272,   0.73159749],
                    [ 0.57990498,  0.79826416],
                    [ 0.55768276,  0.86493083],
                    [ 0.53546054,  0.88715305],
                    [ 0.51323831,  0.95381972],
                    [ 0.49101609,  1.02048638],
                    [ 0.46879387,  1.08715305],
                    [ 0.44657165, 1.15381972],
                    [ 0.37990498,  1.22048638],
                    [ 0.35768276,  1.28715305],
                    [ 0.33546054,  1.35381972],
                    [ 0.31323831,  1.42048638],
                    [ 0.29101609,  1.4427086 ],
                    [ 0.17990498,  1.50937527],
                    [ 0.2021272,   1.57604194],
                    [ 0.17990498,  1.59826416],
                    [ 0.15768276,  1.62048638],
                    [ 0.13546054,  1.6427086 ],
                    [ 0.06879387,  1.70937527],
                    [ 0.0021272,   1.77604194],
                    [-0.10898391,  1.88715305],
                    [-0.13120613,  1.95381972],
                    [-0.15342835,  2.02048638],
                    [-0.17565057,  2.08715305],
                    [-0.1978728,   2.15381972],
                    [-0.22009502,  2.22048638],
                    [-0.24231724,  2.28715305],
                    [-0.26453946,  2.35381972],
                    [-0.28676169,  2.42048638],
                    [-0.26453946,  2.48715305],
                    [-0.28676169,  2.55381972],
                    [-0.26453946,  2.62048638],
                    [-0.1978728,   2.73159749],
                    [-0.17565057,  2.8427086],
                    [-0.15342835,  2.95381972],
                    [-0.30898391,  3.06493083],
                    [-0.10898391,  2.95381972],
                    [-0.26453946,  3.02048638],
                    [-0.15342835,  2.99826416],
                    [-0.26453946,  3.02048638],
                    [-0.10898391,  2.99826416],
                    [-0.17565057,  3.02048638],
                    [-0.1978728,   2.99826416],
                    [-0.3978728,   3.19826416],
                    [-0.37565057,  3.17604194]
                    ]))

    def decide_action(self):
        #  提案モデルに対する比較手法
        #   self.v = walking_model()
        #   self.count += 1
        #   self.v = ps[self.count]
        if self.ps.size != 0:
            self.v = self.ps[0]
            self.ps = self.ps[1:]


if __name__ == '__main__':
    length_step = 48
    n = 0
    lim = 5

    human = Human()
    human.load_default_trajectory()
    robot = Robot()
    robot.load_default_trajectory()
    while n < length_step:
        human.decide_action()
        robot.decide_action()
        x_me = human.v[0]
        y_me = human.v[1]
        x_you = robot.v[0]
        y_you = robot.v[1]

        print("frame", n)
        plt.title("blue = Human, red = Model")
        plt.plot(x_you, y_you, '*', color="r")
        plt.plot(x_me, y_me, '.', color="b")
        plt.xlim(-lim, lim)
        plt.ylim(-lim, lim)
        plt.axes().set_aspect('equal')
        plt.grid()
        plt.show()
        plt.draw()
        print("-----------------------------------------------------------------------")
        n += 1  # インクリメント
