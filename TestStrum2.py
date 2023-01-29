import os
import sys
import time
import numpy as np
import math
from xarm.wrapper import XArmAPI


def strumbot(traj):
    j_angles = pos
    for i in range(len(traj)):
        # run command
        start_time = time.time()
        j_angles[4] = traj[i]
        arms[0].set_servo_angle_j(angles=j_angles, is_radian=False)
        tts = time.time() - start_time
        sleep = 0.002 - tts
        print(j_angles)
        if tts > 0.002:
            print(tts)
            sleep = 0
        time.sleep(sleep)


def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        angle = pos.copy()
        angle[4] = 0
        a.set_servo_angle(angle=angle, wait=False, speed=10, acceleration=0.25, is_radian=False)


def fifth_poly(q_i, q_f, t):
    # time/0.005
    traj_t = np.arange(0, t, 0.002)
    dq_i = 0
    dq_f = 0
    ddq_i = 0
    ddq_f = 0
    a0 = q_i
    a1 = dq_i
    a2 = 0.5 * ddq_i
    a3 = 1 / (2 * t ** 3) * (20 * (q_f - q_i) - (8 * dq_f + 12 * dq_i) * t - (3 * ddq_f - ddq_i) * t ** 2)
    a4 = 1 / (2 * t ** 4) * (30 * (q_i - q_f) + (14 * dq_f + 16 * dq_i) * t + (3 * ddq_f - 2 * ddq_i) * t ** 2)
    a5 = 1 / (2 * t ** 5) * (12 * (q_f - q_i) - (6 * dq_f + 6 * dq_i) * t - (ddq_f - ddq_i) * t ** 2)
    traj_pos = a0 + a1 * traj_t + a2 * traj_t ** 2 + a3 * traj_t ** 3 + a4 * traj_t ** 4 + a5 * traj_t ** 5
    return traj_pos


def poseToPose(poseI, poseF, t):
    traj = []
    for p in range(len(poseI)):
        traj.append(fifth_poly(poseI[p], poseF[p], t))
        # print(p)
    return traj

def gotoPose(numarm, traj):
    track_time = time.time()
    initial_time = time.time()
    for ang in range(len(traj[0])):
        angles = [traj[0][ang], traj[1][ang], traj[2][ang], traj[3][ang], traj[4][ang], traj[5][ang], traj[6][ang]]
        start_time = time.time()
        arms[numarm].set_servo_angle_j(angles=angles, is_radian=False)
        # print(angles)
        while track_time < initial_time + 0.004:
            track_time = time.time()
            time.sleep(0.001)
        initial_time += 0.004

if __name__ == '__main__':
    # global arm1
    arm1 = XArmAPI('192.168.1.203')
    global arms
    arms = [arm1]
    strumD = 30
    speed = 0.25
    global pos
    IP0 = [-.25, 87.4, -2, 126.5, -strumD / 2, 51.7, -45]
    IP1 = [2.67 , 86.1, 0, 127.1, -strumD / 2, 50.1, -45]
    IP2 = [1.3, 81.68, 0.0, 120, -strumD / 2, 54.2, -45]
    IP3 = [-1.4, 81, 0, 117.7, -strumD / 2, 50.5, -45]
    
    pos = [-29.5, 85.6, 32.5, 109.2, -strumD / 2, 49.3, -45]
    midpos = [1]
    pos2 = [39.5, 84.5, -40, 96.6,-strumD / 2, 49.3, -31.2]


    # totalArms = len(arms)

    # test = np.array([1, 2, 3])
    # print(len(np.where(test == 4)[0]))
    setup()
    # print(pos)
    input("test strumming")
    arm1.set_mode(0)
    arm1.set_state(0)
    arm1.set_servo_angle(angle=pos, wait=True, speed=10, acceleration=0.25,
                      is_radian=False)
    input("begin strum")
    arm1.set_mode(1)
    arm1.set_state(0)
    i = 0
    uptraj = fifth_poly(-strumD/2, strumD/2, speed)
    downtraj = fifth_poly(strumD/2, -strumD/2, speed)
    both = [uptraj, downtraj]
    trajA = poseToPose(pos,pos2,0.5)

    while True:
        # input()
        print("got!")
        direction = i % 2
        strumbot(both[direction])
        i += 1

        input()
        #
        # direction = i % 2
        # strumbot(both[direction])
        # i += 1
        # time.sleep(.04)
        #
        # direction = i % 2
        # strumbot(both[direction])
        # i += 1
        # time.sleep(.04)
        #
        # direction = i % 2
        # strumbot(both[direction])
        # i += 1
        # time.sleep(.04)
        #
        # direction = i % 2
        # strumbot(both[direction])
        # i += 1
        # time.sleep(.04)
        #
        # direction = i % 2
        # strumbot(both[direction])
        # i += 1
        # time.sleep(.04)