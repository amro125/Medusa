import os
import sys
import time
import numpy as np
import math
from rtpmidi import RtpMidi
from pymidi import server
from queue import Queue
from threading import Thread
from xarm.wrapper import XArmAPI
import random

from GuitarBotUDP import GuitarBotUDP


class MyHandler(server.Handler):

    def on_peer_connected(self, peer):
        # Handler for peer connected
        print('Peer connected: {}'.format(peer))

    def on_peer_disconnected(self, peer):
        # Handler for peer disconnected
        print('Peer disconnected: {}'.format(peer))

    def on_midi_commands(self, peer, command_list):
        # Handler for midi msgs
        for command in command_list:
            chn = command.channel
            if chn == 1:  # this means its channel 2!!!!!
                if command.command == 'note_on':
                    print("YEYE START")


            if chn == 13:  # this means its channel 14!!!!!

                if command.command == 'note_on':
                    print(chn)
                    key = command.params.key.__int__()
                    velocity = command.params.velocity

                    rob = np.where(notes == key)[0]
                    if len(rob) > 0:
                        print(int(rob))
                        qList[int(rob)].put(1)



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



def poseToPose(poseI, poseF, t):
    traj = []
    for p in range(len(poseI)):
        traj.append(fifth_poly(poseI[p], poseF[p], t))
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


def strumbot(numarm, traj):
    pos = strump[numarm]
    j_angles = pos
    track_time = time.time()
    initial_time = time.time()
    for i in range(len(traj)):
        # run command
        start_time = time.time()
        j_angles[4] = traj[i]
        arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)
        # print(j_angles)
        while track_time < initial_time + 0.004:
            track_time = time.time()
            time.sleep(0.001)
        initial_time += 0.004

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
    uptraj = fifth_poly(-strumD / 2, strumD / 2, speed)
    downtraj = fifth_poly(strumD / 2, -strumD / 2, speed)
    both = [uptraj, downtraj]
    snakeSpeeds = 0.5*random.randint(3, 16)

    # snakeSpeeds = 2
    direction = random.randint(0, 36)
    j1speed = 10*random.randint(4, 12)
    if num == 3:
        snakeSpeeds = 8
        j1speed = 140
    if num < 2:
        j1amp = random.randint(40, 140)
    elif num > 4:
        j1amp = random.randint(10, 30)
    else:
        j1amp = random.randint(10, 90)
    j2amp = random.randint(2, 5)
    # j2amp = 5
    j4sign = random.randint(0,3)
    j4amp = ((-1)**j4sign)*random.randint(10, 30) # 130 mid max range 30
    # j4amp = -30
    j6amp = random.randint(20, 30)
    curIP = IP[num].copy()
    initial = sineq(curIP, j2amp, j4amp, j6amp, snakeSpeeds, 0)
    uptraj = fifth_poly(-strumD / 2, strumD / 2, speed)
    downtraj = fifth_poly(strumD / 2, -strumD / 2, speed)
    both = [uptraj, downtraj]
    # tension = fifth_poly(0, -20, 0.5)
    # release = fifth_poly(-20, 0, 0.75)
    print("ready")


    setup(arms[num])


    startT = 0
    # start the main loop here
    while True:
        arms[num].set_mode(0)
        arms[num].set_state(0)
        arms[num].set_servo_angle(angle=initial, wait=True, speed=20, acceleration=0.25, is_radian=False)
        time.sleep(0.05)
        print("moved to start")
        if startT == 0:
            inq.get()
            inq.queue.clear()
            startT = 1
        arms[num].set_mode(1)
        arms[num].set_state(0)
        t = 0
        track_time = time.time()
        initial_time = time.time()
        # inq.get()
        stall = 0


        while inq.empty() == True:
            j1 = j1amp * math.sin((math.pi / j1speed) * (t)) + IP[num][0]
            newpos = sineq(curIP, j2amp, j4amp, j6amp, snakeSpeeds, t)
            newpos[0] = j1
            # if num == 6:
            #     print(newpos)
            arms[num].set_servo_angle_j(angles=newpos, is_radian=False)
            # print(newpos)

            while track_time < initial_time + 0.004:
                track_time = time.time()
                time.sleep(0.0001)
            initial_time += 0.004
            t += 0.004
        prepstring = strump[num]
        initialangles = arms[num].angles
        posetraj = poseToPose(initialangles, prepstring, 7)
        print(newpos)
        gotoPose(num, posetraj)
        stallref = time.time()
        dir = 0
        while stall < 7:
            if inq.empty() == True:
                stall = time.time() - stallref
                time.sleep(0.004)
            else:
                inq.get()
                direction = dir % 2
                check = arms[num].angles
                if check[4] > 0:
                    direction = 1
                else:
                    direction = 0
                time.sleep(delayarray[direction, num])  # time delay before playing
                print(num)
                # print(delayarray[0, num])
                strumbot(num, both[direction])
                dir += 1
                stallref = time.time()



    # arms[num].set_mode(0)
    # arms[num].set_state(0)
    # arms[num].set_servo_angle(angle=initial, wait=True, speed=10, acceleration=0.25, is_radian=False)






if __name__ == '__main__':
    # global arm1
    ROBOT = "xArms"
    PORT = 5004
    UDP_IP = "192.168.1.50"
    UDP_PORT = 1001
    arm1 = XArmAPI('192.168.1.208')     # ALWAYS GIVER       Actually Robot 1    208
    arm2 = XArmAPI('192.168.1.226')     # ALWAYS RECEIVER    Actually Robot 4
    arm3 = XArmAPI('192.168.1.244')
    arm4 = XArmAPI('192.168.1.203')
    arm5 = XArmAPI('192.168.1.237')
    drumarm1 = XArmAPI('192.168.1.236')
    drumarm2 = XArmAPI('192.168.1.204')
    arms = [arm1, arm2, arm3, arm4, arm5, drumarm1, drumarm2]
    # arms = [arm1]
    global stop_threads
    global strumD
    global speed
    strumD = 30
    speed = 0.25
    global strump
    global notes
    strum0 = [-1, 87.1, -2, 126.5, -strumD/2, 51.7, -45]
    strum1 = [2.1, 86.3, 0, 127.1, -strumD/2, 50.1, -45]
    strum2 = [1.5, 81.6, 0.0, 120, -strumD/2, 54.2, -45]
    strum3 = [2.5, 81, 0, 117.7, -strumD/2, 50.5, -45]
    strum4 = [-1.6, 81.8, 0, 120, -strumD/2, 50.65, -45]         # [-3.9, 65, 3.5, 100.3, -strumD/2, 42.7, 101.1]
    strump = [strum0, strum1, strum2, strum3, strum4]
    notes = np.array([64, 60, 69, 55, 62])


    global IP
    IP0 = [0, -60, 0, 120, 0, 0, 0]
    IP1 = [0, -60, 0, 120, 0, 0, 0]
    IP2 = [0.0, -50, 0, 100, 0, 0, 0]
    IP3 = [0.0, -50, 0, 100, 0, 0, 0]
    IP4 = [0.0, -50, 0, 100, 0, 0, 0]                # [-1.6, 81.8, 0, 120, -strumD/2, 50.13, -45]
    IP5 = [120, -10, 0, 95, 0, 0, 0]
    IP6 = [-120, -20, 0, 95, 0, 10, 0]


    IP = [IP0, IP1, IP2, IP3, IP4, IP5, IP6]

    global delayarray
    delayarray = np.array([[0.15, 0.15, 0.15, 0.15, 0.15, 0.0, 0.0], [0.1, 0.15, 0.1, 0.15, 0.125, 0.0, 0.0]])
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
    q5 = Queue()
    q6 = Queue()
    fq = Queue()
    qList = [q0, q1, q2, q3, q4, q5, q6]

    xArm0 = Thread(target=snakeMove, args=(q0, 0,))
    xArm1 = Thread(target=snakeMove, args=(q1, 1,))
    xArm2 = Thread(target=snakeMove, args=(q2, 2,))
    xArm3 = Thread(target=snakeMove, args=(q3, 3,))
    xArm4 = Thread(target=snakeMove, args=(q4, 4,))
    xArm5 = Thread(target=snakeMove, args=(q5, 5,))
    xArm6 = Thread(target=snakeMove, args=(q6, 6,))

    xArm0.start()
    xArm1.start()
    xArm2.start()
    xArm3.start()
    xArm4.start()
    xArm5.start()
    xArm6.start()
    input("start")
    for q in qList:
        q.put(1)
    # time.sleep(2)
    input("trigger arm")
    # time.sleep(5)
    print("RTP Time")
    rtp_midi = RtpMidi(ROBOT, MyHandler(), PORT)
    print("test")
    rtp_midi.run()



        # try:
        #     fq.get()
        # except KeyboardInterrupt:
        #     for q in qList:
        #         q.put(1)
        #     stop_threads = True
        #     # t1.join()
        #     xArm0.join()
        #     sys.exit()
        #     print("bye")

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
