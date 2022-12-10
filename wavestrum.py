import os
import sys
import time
import numpy as np
import math
import positions

# from rtpmidi import RtpMidi
from queue import Queue
from threading import Thread
from xarm.wrapper import XArmAPI
import positions


# def sinmaker(numarm):

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
    cycles = 5
    speed = 4
    strikes = speed/4
    tup = np.arange(0, 2*speed, 0.004)
    # tstrike = np.arange(0, 2 * strikes, 0.004)
    wave = []
    strike = []
    for t in tup:
        wave.append(math.cos((math.pi / speed) * (t)))
        strike.append(-math.cos((math.pi / strikes) * (t)))
    pos = IP[numarm]
    j_angles = pos.copy()
    sign = -1
    amp = 40
    amp2 = 5
    inq.get()
    while True:
        for c in range(cycles):
            track_time = time.time()
            initial_time = time.time()
            for i in range(len(wave)):
                pos = IP[numarm]
                j_angles[1] = pos[1] + 0.5*amp*wave[i] - 0.5*amp
                j_angles[0] = pos[0] + 0.5*amp2*strike[i] + 0.5*amp2
                j_angles[3] = pos[3] + 1.556*(0.5*amp*wave[i] - 0.5*amp)
                arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)
                # if numarm == 0:
                #     print(j_angles)
                # print(j_angles)
                while track_time < initial_time + 0.004:
                    track_time = time.time()
                    time.sleep(0.0001)
                initial_time += 0.004

def testfunction():
    print(positions.testglobal)

if __name__ == '__main__':
    ROBOT = "xArms"
    PORT = 5004
    global IP
    global arms
    global strumD
    global speed
    global notes
    global testglobal
    testglobal = 2
    strumD = 0
    speed = 0.25
    amp2 = 5
    IP0 = [-1-amp2/2, 87.1, -2, 126.5, -strumD / 2, 51.7, -45]
    IP1 = [2.1-amp2/2, 86.3, 0, 127.1, -strumD / 2, 50.1, -45]
    IP2 = [1.5-amp2/2, 81.6, 0.0, 120, -strumD / 2, 54.2, -45]
    IP3 = [2.5-amp2/2, 81, 0, 117.7, -strumD / 2, 50.5, -45]
    IP4 = pos = [-1.6, 81.8, 0, 120, -strumD / 2, 50.65, -45]  # [-3.9, 65, 3.5, 100.3, -strumD/2, 42.7, 101.1]
    DRUM1IP = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0]  # Snare

    DRUM2 = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0]  # Bodharn
    notes = np.array([64, 60, 69, 55, 62])


    IP = [IP0, IP1, IP2, IP3, IP4, DRUM1IP, DRUM2]
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
    qList = [q0, q1, q4, q3, q2]

    xArm0 = Thread(target=wave, args=(q0, 0,))
    xArm1 = Thread(target=wave, args=(q1, 1,))
    xArm2 = Thread(target=wave, args=(q2, 2,))
    xArm3 = Thread(target=wave, args=(q3, 3,))
    xArm4 = Thread(target=wave, args=(q4, 4,))
    # drumArm1 = Thread(target=wave, args=(dq1, 5,))
    # drumArm2 = Thread(target=wave, args=(dq2, 6,))

    xArm0.start()
    xArm1.start()
    xArm2.start()
    xArm3.start()
    xArm4.start()
    # drumArm1.start()
    # drumArm2.start()
    # tension = fifth_poly(0, -10, 0.5)
    # print(tension)
    input("TEST")
    time.sleep(3)
    q0.put(1)
    q1.put(1)
    time.sleep(4)
    q2.put(1)
    q3.put(1)
    q4.put(1)

    # for q in qList:
    #     q.put(1)
    #     time.sleep(1)
    while True:
        input("hi")