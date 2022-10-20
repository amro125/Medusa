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

            if chn == 2:  # this means its channel 3 !!!!!
                if command.command == 'note_on':
                    #print("DRUMMO")
                    key = command.params.key.__int__()
                    velocity = command.params.velocity
                    notetype = np.where(notes == key)[0]
                    if len(notetype) > 0:
                        dq1.put(notetype)



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

def spline_poly(q_i, q_f, ta, tt, ts):
    #spline poly for drumming, where ta is time of acceleration (5th order poly going 1/6th the total dist)
    #and tt is the time for the striker to hit, turn around, and go back to original position (another 5th order poly)

    #initial accel (using first half of a 5th order poly)
    #ta is double the time till max acceleration (time doing 5th order poly)
    traj_ta = np.arange(0, ta, 0.004)
    dq_i = 0
    dq_f = 0
    ddq_i = 0
    ddq_f = 0
    a0 = q_i
    a1 = dq_i
    a2 = 0.5 * ddq_i
    a3 = 1 / (2 * ta ** 3) * (20 * (q_f - q_i) / 6 - (8 * dq_f + 12 * dq_i) * ta - (3 * ddq_f - ddq_i) * ta ** 2)
    a4 = 1 / (2 * ta ** 4) * (30 * (q_i - q_f) / 6 + (14 * dq_f + 16 * dq_i) * ta + (3 * ddq_f - 2 * ddq_i) * ta ** 2)
    a5 = 1 / (2 * ta ** 5) * (12 * (q_f - q_i) / 6 - (6 * dq_f + 6 * dq_i) * ta - (ddq_f - ddq_i) * ta ** 2)
    fifth_pos = a0 + a1 * traj_ta + a2 * traj_ta ** 2 + a3 * traj_ta ** 3 + a4 * traj_ta ** 4 + a5 * traj_ta ** 5
    fifth_vel = a1 + 2 * a2 * traj_ta + 3 * a3 * traj_ta ** 2 + 4 * a4 * traj_ta ** 3 + 5 * a5 * traj_ta ** 4

    # print("fifth pos")
    # print(fifth_pos)
    # halfway point of acceleration array (hp)
    hp = math.floor(len(fifth_pos) / 2)
    delta1 = abs(fifth_pos[0] - fifth_pos[hp])
    # speed halfway (max speed)
    hv = fifth_vel[hp]

    # 5th order turnaround
    # tt is time for turning around
    traj_tt = np.arange(0, tt, 0.004)
    dq_i = hv
    dq_f = -hv
    ddq_i = 0
    ddq_f = 0
    # nq_i = pc[len(pc)-1] # new initial pos is the end of constant velocity part
    a0 = 0
    a1 = dq_i
    a2 = 0.5 * ddq_i
    a3 = 1 / (2 * ta ** 3) * (20 * (0) - (8 * dq_f + 12 * dq_i) * ta - (3 * ddq_f - ddq_i) * ta ** 2)
    a4 = 1 / (2 * ta ** 4) * (30 * (0) + (14 * dq_f + 16 * dq_i) * ta + (3 * ddq_f - 2 * ddq_i) * ta ** 2)
    a5 = 1 / (2 * ta ** 5) * (12 * (0) - (6 * dq_f + 6 * dq_i) * ta - (ddq_f - ddq_i) * ta ** 2)
    tfifth_pos = a0 + a1 * traj_ta + a2 * traj_ta ** 2 + a3 * traj_ta ** 3 + a4 * traj_ta ** 4 + a5 * traj_ta ** 5

    thp = math.floor(len(tfifth_pos) / 2)  # halfway point of turnaround traj
    delta2 = abs(tfifth_pos[0] - tfifth_pos[thp])

    # constant speed
    # tc is time at constant speed
    delta3 = abs(q_i - q_f) - delta1 - delta2
    if (delta3 < 0):
        print("accel time and turnaround time too big")

    tc = delta3 / abs(hv)

    traj_tc = np.arange(0, tc, 0.004)
    pc = fifth_pos[hp] + traj_tc * hv

    # ts is stall time (time waiting at bottom)
    traj_ts = np.arange(0, ts, 0.004)
    traj_ts = np.ones(len(traj_ts) - 1) * (pc[len(pc) - 1] + tfifth_pos[thp])

    # print("pc")
    # print(pc)

    # print("tfifth_pos")
    # print(pc[len(pc)-1] + tfifth_pos)

    half_traj = np.concatenate((fifth_pos[0:hp], pc, pc[len(pc) - 1] + tfifth_pos[0:thp], traj_ts))
    full_traj = np.append(half_traj, np.flip(half_traj))

    return full_traj

def drumbot(trajz, trajp, arm):

    #j_angles = pos
    track_time = time.time()
    initial_time = time.time()
    for i in range(len(trajz)):
        # run command
        #start_time = time.time()
        #j_angles[4] = traj[i]
        #arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)
        mvpose = [492,0,trajz[i],180,trajp[i],0]
        #print(mvpose[2])
        arms[arm].set_servo_cartesian(mvpose, speed=100, mvacc=2000)
        while track_time < initial_time + 0.004:
            track_time = time.time()
            time.sleep(0.0001)
        initial_time += 0.004


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


def drummer(inq,num):
    i = 1
    #uptraj = fifth_poly(-strumD/2, strumD/2, speed)
    #downtraj = fifth_poly(strumD/2, -strumD/2, speed)
    #both = [uptraj, downtraj]
    #tension = fifth_poly(0, -20, 0.5)
    #release = fifth_poly(-20, 0, 0.75)

    #downtrajz= fifth_poly(325, 20, .3)
    #uptrajz= fifth_poly(20, 325, .3)
    #downtrajp= fifth_poly(-89, -36, .3)
    #uptrajp = fifth_poly(-36, -89, .3)
    #trajz = np.append(downtrajz, uptrajz)
    #trajp = np.append(downtrajp, uptrajp)

    #single hit
    trajz = spline_poly(325, 18, .08, .04, 0.01)
    trajp = spline_poly(-89, -30, .08, .04, 0.01)

    #double hit
    trajz2 = spline_poly(325, 18, .08, .04, 0.1)
    trajp2 = spline_poly(-89, -30, .08, .04, 0.1)

    #triple hit
    trajz3 = spline_poly(325, 18, .08, .04, 0.15)
    trajp3 = spline_poly(-89, -30, .08, .04, 0.15)


    while True:
        play = inq.get()
        print("got!")
        if play == 0:
            drumbot(trajz, trajp, num)
        elif play == 1:
            drumbot(trajz2, trajp2, num)
        elif play == 2:
            drumbot(trajz3, trajp3, num)
        #if i == 1:
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
            time.sleep(.1)
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
    IP4 = pos = [-1.6, 81.8, 0, 120, -strumD/2, 50.65, -45]         # [-3.9, 65, 3.5, 100.3, -strumD/2, 42.7, 101.1]
    DRUM1 = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0] #Snare
    DRUM2 = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0]  #Bodharn
    notes = np.array([64, 60, 69, 55, 62])


    IP = [IP0, IP1, IP2, IP3, IP4, DRUM1, DRUM2]
    arm0 = XArmAPI('192.168.1.208')
    arm1 = XArmAPI('192.168.1.226')
    arm2 = XArmAPI('192.168.1.244')
    arm3 = XArmAPI('192.168.1.203')
    arm4 = XArmAPI('192.168.1.237')
    drumarm1 = XArmAPI('192.168.1.236')
    drumarm2 = XArmAPI('192.168.1.204')
    arms = [arm0, arm1, arm2, arm3, arm4, drumarm1, drumarm2]
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
    dq1 = Queue()
    dq2 = Queue()
    qList = [q0, q1, q2, q3, q4, dq1, dq2]

    xArm0 = Thread(target=strummer, args=(q0, 0,))
    xArm1 = Thread(target=strummer, args=(q1, 1,))
    xArm2 = Thread(target=strummer, args=(q2, 2,))
    xArm3 = Thread(target=strummer, args=(q3, 3,))
    xArm4 = Thread(target=strummer, args=(q4, 4,))
    drumArm1 = Thread(target=drummer, args=(dq1, 5,))
    drumArm2 = Thread(target=drummer, args=(dq2, 6,))

    xArm0.start()
    xArm1.start()
    xArm2.start()
    xArm3.start()
    xArm4.start()
    drumArm1.start()
    drumArm2.start()
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
