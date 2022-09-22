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
            if chn == 12:  # this means its channel 13!!!!!
                if command.command == 'note_on':
                    # print(chn)
                    key = command.params.key.__int__()
                    velocity = command.params.velocity
                    for q in qList:
                        q.put(2)
                    # print('key {} with velocity {}'.format(key, velocity))
                    # q.put(velocity)


                    #playDance(dances[velocity])


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

        # arms[a].set_servo_angle(angle=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait=False, speed=10, acceleration=0.25, is_radian=False)

def setup2():
    for a in range(len(arms)):
        curIP = IP[a]
        arms[a].set_servo_angle(angle=curIP, wait=False, speed=10, acceleration=0.25, is_radian=False)

        # print(curIP)

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


def strumbot(numarm, traj):
    pos = IP[numarm]
    j_angles = pos
    track_time = time.time()
    initial_time = time.time()
    for i in range(len(traj)):
        # run command
        start_time = time.time()
        j_angles[4] = traj[i]
        arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)
        while track_time < initial_time + 0.004:
            track_time = time.time()
            time.sleep(0.0001)
        initial_time += 0.004


def prepGesture(numarm, traj):
    pos = IP[numarm]
    j_angles = pos.copy()
    track_time = time.time()
    initial_time = time.time()
    for i in range(len(traj)):
        # run command
        j_angles[1] = pos[1] + traj[i]
        j_angles[3] = pos[3] + traj[i]
        arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)
        # print(j_angles)
        while track_time < initial_time + 0.004:
            track_time = time.time()
            time.sleep(0.0001)
        initial_time += 0.004



def strummer(inq,num):
    i = 0
    uptraj = fifth_poly(-strumD/2, strumD/2, speed)
    downtraj = fifth_poly(strumD/2, -strumD/2, speed)
    both = [uptraj, downtraj]
    tension = fifth_poly(0, -20, 0.5)
    release = fifth_poly(-20, 0, 0.75)
    while True:
        play = inq.get()
        print("got!")
        if play == 1:
            direction = i % 2
            strumbot(num, both[direction])
            i += 1
        elif play == 2:
            prepGesture(num, tension)
            time.sleep(0.25)
            prepGesture(num, release)


# Press the green button in the gutter to run the script.
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
    IP0 = [-1, 87.1, -2, 126.5, -strumD/2, 51.7, -45]
    IP1 = [2.1, 86.3, 0, 127.1, -strumD/2, 50.1, -45]
    IP2 = [1.5, 81.6, 0.0, 120, -strumD/2, 54.2, -45]
    IP3 = [2.5, 81, 0, 117.7, -strumD/2, 50.5, -45]
    IP4 = [-3.9, 65, 3.5, 100.3, -strumD/2, 42.7, 101.1]                  # [-1.6, 81.8, 0, 120, -strumD/2, 50.13, -45]
    DRUM1 = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0] #DRUMMMING
    notes = np.array([64, 60, 69, 55, 62])


    IP = [IP0, IP1, IP2, IP3, IP4]
    arm0 = XArmAPI('192.168.1.208')
    arm1 = XArmAPI('192.168.1.226')
    arm2 = XArmAPI('192.168.1.244')
    arm3 = XArmAPI('192.168.1.203')
    arm4 = XArmAPI('192.168.1.237')
    arms = [arm0, arm1, arm2, arm3, arm4]
    # arms = [arm1]
    totalArms = len(arms)
    setup()
    input("lets go")
    setup2()
    input("letsgo again")
    for a in arms:
        a.set_mode(1)
        a.set_state(0)

    q0 = Queue()
    q1 = Queue()
    q2 = Queue()
    q3 = Queue()
    q4 = Queue()
    qList = [q0, q1, q2, q3, q4]

    xArm0 = Thread(target=strummer, args=(q0, 0,))
    xArm1 = Thread(target=strummer, args=(q1, 1,))
    xArm2 = Thread(target=strummer, args=(q2, 2,))
    xArm3 = Thread(target=strummer, args=(q3, 3,))
    xArm4 = Thread(target=strummer, args=(q4, 4,))

    xArm0.start()
    xArm1.start()
    xArm2.start()
    xArm3.start()
    xArm4.start()
    # tension = fifth_poly(0, -10, 0.5)
    # print(tension)
    input("TEST")
    # time.sleep(5)
    # q1.put(2)
    # input()

    rtp_midi = RtpMidi(ROBOT, MyHandler(), PORT)
    print("test")
    rtp_midi.run()

    # while True:
    #     strumnum = input("which robot")
    #
    #     qList[0].put(0)



# See PyCharm help at https://www.jetbrains.com/help/pycharm/
