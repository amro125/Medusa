from visual_kinematics.RobotSerial import *
import numpy as np
import math
from math import pi
import time
from xarm.wrapper import XArmAPI

arm0 = XArmAPI('192.168.1.206')
arm1 = XArmAPI('192.168.1.208')


def smallestjointd(point, set):
    distance = []
    for j in set:
        distance.append(math.dist(point, set))
    d = min(distance)
    return d


def jointcoord(theta, jsols):
    f = []
    angle = []
    for i in range(7):
        angle.append(theta[i] * pi / 180)
        test = np.array(angle)
        # print(test)
        if i > 0:
            f.append(jsols[i - 1].forward(test).t_3_1.reshape([3, ]))
    return f


def main():
    # offset is 34,-19,-10

    np.set_printoptions(precision=3, suppress=True)
    joints = []
    jsols = []
    # format is d, a, alpha,theta
    dh_params = [[0.267, 0., -0.5 * pi, 0.],
                 [0., 0., 0.5 * pi, 0.],
                 [0.293, 0.0525, 0.5 * pi, 0.],
                 [0., 0.0775, 0.5 * pi, 0.],
                 [0.3425, 0., 0.5 * pi, 0.],
                 [0., 0.076, -0.5 * pi, 0.],
                 [0.097, 0., 0., 0.]]
    for i in range(7):
        joints.append(dh_params[i])
        test = np.array(joints)
        # print(test)
        # input()
        if i > 0:
            jsols.append(RobotSerial(test))
            # print(jsols)

    # j1 = RobotSerial()
    theta = [0., 0., -0.25 * pi, 0., 0., 0.]
    start = time.time()

    end = time.time()
    #

    offset = np.array()
    while True:
        input()
        theta = arm0.angles
        check = np.array(arm1.position[0:3])
        print(check)
        f = jointcoord(theta, jsols)

        print(f)

    print(start - end)


main()
