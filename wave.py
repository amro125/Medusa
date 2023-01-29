import os
import sys
import time
import numpy as np
import math

from rtpmidi import RtpMidi
from queue import Queue
from threading import Thread
from xarm.wrapper import XArmAPI

def setup():
    for a in range(len(arms)):
        arms[a].set_simulation_robot(on_off=False)
        arms[a].motion_enable(enable=True)
        arms[a].clean_warn()
        arms[a].clean_error()
        arms[a].set_mode(0)
        arms[a].set_state(0)
        curIP = IP[a]
        arms[a].set_servo_angle(angle=curIP, wait=False, speed=10, acceleration=0.25, is_radian=False)

def wave(inq, numarm):
    cycles = 3
    speed = 4
    timearray = np.arange(0, 2*speed, 0.004)
    wave = []
    for t in timearray:
        wave.append(-math.cos((math.pi / speed) * (t)))
    pos = IP[numarm]
    j_angles = pos.copy()
    sign = -1
    amp = 30
    inq.get()
    while True:
        for r in range(3):
            for c in range(cycles):
                track_time = time.time()
                initial_time = time.time()
                for w in wave:
                    pos = IP[numarm]
                    j_angles[0] = pos[0] - 0.5 * amp * w - 0.5 * amp
                    j_angles[4] = pos[4] + 3 * amp * w + 3 * amp
                    j_angles[1] = pos[1] + 0.5*amp*w + 0.5*amp
                    j_angles[3] = pos[3] + amp*w + amp
                    j_angles[5] = pos[5] + 0.5*amp*w + 0.5*amp
                    arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)
                    # if numarm == 0:
                    #     print(j_angles)
                    # print(j_angles)
                    while track_time < initial_time + 0.004:
                        track_time = time.time()
                        time.sleep(0.0001)
                    initial_time += 0.004
            speed += sign*1
            wave = []
            timearray = np.arange(0, 2 * speed, 0.004)
            print("new speed",speed)
            for t in timearray:
                wave.append(-math.cos((math.pi / speed) * (t)))
        sign = -1*sign

if __name__ == '__main__':
    ROBOT = "xArms"
    PORT = 5004
    global IP
    global arms
    global strumD
    global speed
    global notes

    strumD = 30
    speed = 0.25
    IP0 = [162, -60, 0, 30, 0, 40, 0]
    IP1 = [-162, -60, 0, 30, 0, 40, 0]
    IP2 = [90, -60, 0, 40, 0, 40, 0]
    IP3 = [0, -30, 0, 60, 0, 30, 0]
    IP4 = [-90, -80, 0, 30, 0, 30, 0]         # [-3.9, 65, 3.5, 100.3, -strumD/2, 42.7, 101.1]
    DRUM1 = [165, -45, 0, 45, 0, 24, 0] #Snare
    DRUM2 = [-165, -70, 0, 30, 0, 30, 0]  #Bodharn
    notes = np.array([64, 60, 69, 55, 62])


    IP = [IP0, IP1, IP2, IP3, IP4, DRUM1, DRUM2]
    up = [[], [], [], [], [], [30, 60, 30], []]
    arm0 = XArmAPI('192.168.1.208')
    arm1 = XArmAPI('192.168.1.226')
    arm2 = XArmAPI('192.168.1.244')
    arm3 = XArmAPI('192.168.1.203')
    arm4 = XArmAPI('192.168.1.237')
    drumarm1 = XArmAPI('192.168.1.236')
    drumarm2 = XArmAPI('192.168.1.204')
    arms = [arm0, arm1, arm2, arm3, arm4, drumarm1, drumarm2]

    setup()
    input("lets go")
    input("letsgo again")
    for a in arms:
        a.set_mode(1)
        a.set_state(0)


    q0 = Queue()
    q1 = Queue()
    q2 = Queue()
    q3 = Queue()
    q4 = Queue()
    dq1 = Queue()
    dq2 = Queue()
    qList = [q0, q1, q4, dq2, dq1, q2]

    xArm0 = Thread(target=wave, args=(q0, 0,))
    xArm1 = Thread(target=wave, args=(q1, 1,))
    xArm2 = Thread(target=wave, args=(q2, 2,))
    # xArm3 = Thread(target=middle, args=(q3, 3,))
    xArm4 = Thread(target=wave, args=(q4, 4,))
    drumArm1 = Thread(target=wave, args=(dq1, 5,))
    drumArm2 = Thread(target=wave, args=(dq2, 6,))

    xArm0.start()
    xArm1.start()
    xArm2.start()
    # xArm3.start()
    xArm4.start()
    drumArm1.start()
    drumArm2.start()
    # tension = fifth_poly(0, -10, 0.5)
    # print(tension)
    input("TEST")
    time.sleep(3)
    for q in qList:
        q.put(1)
        time.sleep(1)
    while True:
        input("hi")