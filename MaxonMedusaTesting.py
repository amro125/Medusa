import os
import sys
import time
import numpy as np
import math
from xarm.wrapper import XArmAPI
from GuitarBotUDP import GuitarBotUDP


def strumbot(traj2, traj4):
    j_angles = pos
    for i in range(len(traj2)):
        # run command
        start_time = time.time()
        j_angles[3] = traj4[i]
        j_angles[1] = traj2[i]
        arms[0].set_servo_angle_j(angles=j_angles, is_radian=False)
        tts = time.time() - start_time
        sleep = 0.002 - tts
        # print(j_angles)
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
        angle[4] = 0.0
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


if __name__ == '__main__':
    # global arm1
    UDP_IP = "192.168.1.50"
    UDP_PORT = 1001
    guitarbot_udp = GuitarBotUDP(UDP_IP, UDP_PORT)
    arm1 = XArmAPI('192.168.1.237')
    global arms
    arms = [arm1]
    strumD = 0
    speed = 0.25
    global pos
    pos1 =[-4.9, 65, 3.5, 100.3, -strumD/2, 43.4, 92.3]
    pos2 = [-4.9, 45, 3.5, 70.7, -strumD/2, 43.4, 92.3]
    pos3 = [-4.9, 35, 3.5, 57.1, -strumD / 2, 43.4, 92.3]
    pos4 = [-4.9, 25, 3.5, 44.75, -strumD / 2, 43.4, 92.3]
    pos = pos1
    positions = [pos1, pos2, pos3, pos4]
    # totalArms = len(arms)
    # test = np.array([1, 2, 3])
    # print(len(np.where(test == 4)[0]))
    # x, y = input("Enter two values: ").split()
    # input("nice")
    setup()
    print(pos)
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
    initial = pos
    while True:
        action, control = input("what do? Motor control (M)?").split()
        # if user == m:
        action = int(action)
        control = int(control)
        if action == 1:
            j2 = fifth_poly(initial[1], positions[control][1], 1)
            j4 = fifth_poly(initial[3], positions[control][3], 1)
            strumbot(j2, j4)
            initial = positions[control]
        elif action == 2:
            if control == 1:
                guitarbot_udp.send_msg_picker(ipickercommand=2, bstartpicker=1, pgain=5000, dgain=50, ipickerpos=10,
                                              ipickervel=7, ipickeracc=100)
            # input()
            elif control == 0:
                guitarbot_udp.send_msg_picker(ipickercommand=2, bstartpicker=0, pgain=5000, dgain=50, ipickerpos=10,
                                              ipickervel=7, ipickeracc=100)





        # print("got!")
        # direction = i % 2
        # strumbot(both[direction])
        # i += 1