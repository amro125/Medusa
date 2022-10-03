import os
import sys
import time
import numpy as np
import math
from queue import Queue
from threading import Thread
from xarm.wrapper import XArmAPI
import random
from GuitarBotUDP import GuitarBotUDP


def fifth_poly(q_i, q_f, t):
    # time/0.005
    traj_t = np.arange(0, t, 0.004)
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


def sineq(ipos, j2, j4, j6, speed, t):
    sinjs = [0, 0, 0]
    inputs = [j2, j4, j6]
    phases = [0.75, 0.5, 0]
    for x in range(len(sinjs)):
        sinjs[x] = inputs[x]*math.sin((math.pi/speed)*(t-phases[x]))
    output = ipos.copy()
    output[1] = ipos[1]+sinjs[0]
    output[3] = ipos[3]+sinjs[1]
    output[5] = ipos[5]+sinjs[2]
    return output


def setup(a):
    # for a in arms:
    a.set_simulation_robot(on_off=False)
    a.motion_enable(enable=True)
    a.clean_warn()
    a.clean_error()
    a.set_mode(0)
    a.set_state(0)


def snakeMove(inq, num):
    snakeSpeeds = random.randint(2, 5)
    # snakeSpeeds = 2
    direction = random.randint(0, 36)
    j1speed = 10*random.randint(5, 12)
    j1amp = random.randint(5, 90)
    j2amp = random.randint(2, 5)
    # j2amp = 5
    j4sign = random.randint(0,3)
    j4amp = ((-1)**j4sign)*random.randint(10, 30) # 130 mid max range 30
    # j4amp = -30
    j6amp = random.randint(20, 30)
    curIP = IP[num].copy()
    initial = sineq(curIP, j2amp, j4amp, j6amp, snakeSpeeds, 0)
    print("ready")
    setup(arms[num])
    arms[num].set_servo_angle(angle=initial, wait=False, speed=10, acceleration=0.25, is_radian=False)

    inq.get()
    arms[num].set_mode(1)
    arms[num].set_state(0)
    t = 0
    track_time = time.time()
    initial_time = time.time()
    # inq.get()
    while inq.empty() == True:
        j1 = j1amp * math.sin((math.pi / j1speed) * (t))
        newpos = sineq(curIP, j2amp, j4amp, j6amp, snakeSpeeds, t)
        newpos[0] = j1
        # print(newpos)
        arms[num].set_servo_angle_j(angles=newpos, is_radian=False)

        while track_time < initial_time + 0.004:
            track_time = time.time()
            time.sleep(0.0001)
        initial_time += 0.004
        t += 0.004





if __name__ == '__main__':
    # global arm1
    UDP_IP = "192.168.1.50"
    UDP_PORT = 1001
    arm1 = XArmAPI('192.168.1.208')     # ALWAYS GIVER       Actually Robot 1    208
    arm2 = XArmAPI('192.168.1.226')     # ALWAYS RECEIVER    Actually Robot 4
    arm3 = XArmAPI('192.168.1.244')
    arm4 = XArmAPI('192.168.1.203')
    arm5 = XArmAPI('192.168.1.237')

    arms = [arm1, arm2, arm3, arm4, arm5]
    # arms = [arm1]
    global stop_threads

    global IP
    IP0 = [0, -60, 0, 120, 0, 0, 0]
    IP1 = [0, -60, 0, 120, 0, 0, 0]
    IP2 = [0.0, -50, 0, 100, 0, 0, 0]
    IP3 = [0.0, -50, 0, 100, 0, 0, 0]
    IP4 = [0.0, -50, 0, 100, 0, 0, 0]                # [-1.6, 81.8, 0, 120, -strumD/2, 50.13, -45]


    IP = [IP0, IP1, IP2, IP3, IP4]
    # arms = [arm1]
    # totalArms = len(arms)
    # setup()
    input("lets go")
    traj_t = np.arange(0, 3, 0.004)
    # for x in traj_t:
    #     y = math.sin((2*math.pi/3)*x)
    #     print(y)
    #     time.sleep(0.004)
    q0 = Queue()
    q1 = Queue()
    q2 = Queue()
    q3 = Queue()
    q4 = Queue()
    fq = Queue()
    qList = [q0, q1, q2, q3, q4]

    xArm0 = Thread(target=snakeMove, args=(q0, 0,))
    xArm1 = Thread(target=snakeMove, args=(q1, 1,))
    xArm2 = Thread(target=snakeMove, args=(q2, 2,))
    xArm3 = Thread(target=snakeMove, args=(q3, 3,))
    xArm4 = Thread(target=snakeMove, args=(q4, 4,))

    xArm0.start()
    xArm1.start()
    xArm2.start()
    xArm3.start()
    xArm4.start()
    input("start")
    for q in qList:
        q.put(1)

    while True:
        try:
            fq.get()
        except KeyboardInterrupt:
            for q in qList:
                q.put(1)
            stop_threads = True
            # t1.join()
            xArm0.join()
            sys.exit()
            print("bye")

    # xArm1.start()
    # xArm2.start()
    # xArm3.start()
    # xArm4.start()
    # input("letsgo again")
    # for a in arms:
    #     a.set_mode(1)
    #     a.set_state(0)



    # totalArms = len(arms)
    # test = np.array([1, 2, 3])
    # print(len(np.where(test == 4)[0]))
    # x, y = input("Enter two values: ").split()
    # input("nice")


    # while True:
