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

import positions


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
                    print("DRUMMO")
                    key = command.params.key.__int__()
                    velocity = command.params.velocity
                    notetype = np.where(notes == key)[0]
                    if len(notetype) > 0:
                        dq1.put(notetype)
                        #how do we output velocity too?
                        #a second dq1 (dq11), or can I .put(notetype, velocity)

            if chn == 3:  # this means its channel 4 !!!!!
                if command.command == 'note_on':
                    print("DRUMMO2")
                    dq2.put(1)


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


def robomove(numarm, trajectory):
    track_time = time.time()
    initial_time = time.time()
    for j_angles in trajectory:
        # run command
        start_time = time.time()
        arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)
        while track_time < initial_time + 0.004:
            track_time = time.time()
            time.sleep(0.0001)
        initial_time += 0.004



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

def spline_poly(q_i, q_f, q_in, ta, tt, ttopstop, tbotstop):

    #qi is initial pos, qf is final pos (strike), qin is new initial (return pos)

    #initial accel (using first half of a 5th order poly)
    #ta is double the time till max acceleration (time doing 5th order poly)

    ########### this code calculates the trajectory for the first half (the way down) #############

    traj_ta = np.arange(0, ta, 0.004)
    dq_i = 0
    dq_f = 0
    ddq_i = 0
    ddq_f = 0
    a0 = q_i
    a1 = dq_i
    a2 = 0.5 * ddq_i
    a3 = 1 / (2 * ta ** 3) * (20 * (q_f - q_i)/2 - (8 * dq_f + 12 * dq_i) * ta - (3 * ddq_f - ddq_i) * ta ** 2)
    a4 = 1 / (2 * ta ** 4) * (30 * (q_i - q_f)/2 + (14 * dq_f + 16 * dq_i) * ta + (3 * ddq_f - 2 * ddq_i) * ta ** 2)
    a5 = 1 / (2 * ta ** 5) * (12 * (q_f - q_i)/2 - (6 * dq_f + 6 * dq_i) * ta - (ddq_f - ddq_i) * ta ** 2)
    fifth_pos = a0 + a1 * traj_ta + a2 * traj_ta ** 2 + a3 * traj_ta ** 3 + a4 * traj_ta ** 4 + a5 * traj_ta ** 5
    fifth_vel = a1 + 2 * a2 * traj_ta + 3 * a3 * traj_ta ** 2 + 4 * a4 * traj_ta ** 3 + 5 * a5 * traj_ta ** 4

    #halfway point of acceleration array (hp)
    hp = math.floor(len(fifth_pos) / 2)
    delta1 = abs(fifth_pos[0] - fifth_pos[hp])
    #speed halfway (max speed)
    hv = fifth_vel[hp]


    #5th order turnaround
    #tt is time for turning around
    traj_tt = np.arange(0, tt, 0.004)
    dq_i = hv
    dq_f = -hv
    ddq_i = 0
    ddq_f = 0
    #nq_i = pc[len(pc)-1] # new initial pos is the end of constant velocity part
    a0 = 0
    a1 = dq_i
    a2 = 0.5 * ddq_i
    a3 = 1 / (2 * ta ** 3) * (20 * (0) - (8 * dq_f + 12 * dq_i) * ta - (3 * ddq_f - ddq_i) * ta ** 2)
    a4 = 1 / (2 * ta ** 4) * (30 * (0) + (14 * dq_f + 16 * dq_i) * ta + (3 * ddq_f - 2 * ddq_i) * ta ** 2)
    a5 = 1 / (2 * ta ** 5) * (12 * (0) - (6 * dq_f + 6 * dq_i) * ta - (ddq_f - ddq_i) * ta ** 2)
    tfifth_pos = a0 + a1 * traj_ta + a2 * traj_ta ** 2 + a3 * traj_ta ** 3 + a4 * traj_ta ** 4 + a5 * traj_ta ** 5

    thp = math.floor(len(tfifth_pos) / 2) #halfway point of turnaround traj
    delta2 = abs(tfifth_pos[0] - tfifth_pos[thp])

    #constant speed
    #tc is time at constant speed
    delta3 = abs(q_i - q_f) - delta1 - delta2
    if(delta3 < 0):
        print("accel time and turnaround time too big")

    tc = delta3 / abs(hv)

    traj_tc = np.arange(0, tc, 0.004)
    pc = fifth_pos[hp] + traj_tc*hv

    # stall time at top / bottom
    traj_top = np.ones(int(ttopstop / 0.004)) * q_i  # time stopped at top of trajectory, before strike
    traj_bot = np.ones(int(tbotstop / 0.004)) * q_f  # time stopped at bottom of trajectory, after strike (half of the total time)

    ########### this code calculates the trajectory for the second half (the way up) #############

    traj_ta = np.arange(0, ta, 0.004)
    dq_i = 0
    dq_f = 0
    ddq_i = 0
    ddq_f = 0
    a0 = q_in
    a1 = dq_i
    a2 = 0.5 * ddq_i
    a3 = 1 / (2 * ta ** 3) * (20 * (q_f - q_in) / 2 - (8 * dq_f + 12 * dq_i) * ta - (3 * ddq_f - ddq_i) * ta ** 2)
    a4 = 1 / (2 * ta ** 4) * (30 * (q_in - q_f) / 2 + (14 * dq_f + 16 * dq_i) * ta + (3 * ddq_f - 2 * ddq_i) * ta ** 2)
    a5 = 1 / (2 * ta ** 5) * (12 * (q_f - q_in) / 2 - (6 * dq_f + 6 * dq_i) * ta - (ddq_f - ddq_i) * ta ** 2)
    fifth_pos2 = a0 + a1 * traj_ta + a2 * traj_ta ** 2 + a3 * traj_ta ** 3 + a4 * traj_ta ** 4 + a5 * traj_ta ** 5
    fifth_vel2 = a1 + 2 * a2 * traj_ta + 3 * a3 * traj_ta ** 2 + 4 * a4 * traj_ta ** 3 + 5 * a5 * traj_ta ** 4

    # halfway point of acceleration array (hp)
    hp2 = math.floor(len(fifth_pos2) / 2)
    delta1 = abs(fifth_pos2[0] - fifth_pos2[hp2])
    # speed halfway (max speed)
    hv = fifth_vel2[hp2]

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
    tfifth_pos2 = a0 + a1 * traj_ta + a2 * traj_ta ** 2 + a3 * traj_ta ** 3 + a4 * traj_ta ** 4 + a5 * traj_ta ** 5

    thp2 = math.floor(len(tfifth_pos2) / 2)  # halfway point of turnaround traj
    delta2 = abs(tfifth_pos2[0] - tfifth_pos2[thp2])

    # constant speed
    # tc is time at constant speed
    delta3 = abs(q_in - q_f) - delta1 - delta2
    if (delta3 < 0):
        print("accel time and turnaround time too big")

    tc = delta3 / abs(hv)

    traj_tc = np.arange(0, tc, 0.004)
    pc2 = fifth_pos2[hp2] + traj_tc * hv

    #stall time at top / bottom
    traj_top2 = np.ones(int(ttopstop / 0.004)) * q_in  # time stopped at top of trajectory, before strike
    traj_bot2 = np.ones(int(tbotstop / 0.004)) * q_f  # time stopped at bottom of trajectory, after strike (half of the total time)

    half_traj1 = np.concatenate((traj_top,fifth_pos[0:hp], pc, pc[len(pc)-1] + tfifth_pos[0:thp], traj_bot))
    half_traj2 = np.flip(np.concatenate((traj_top2, fifth_pos2[0:hp2], pc2, pc2[len(pc2) - 1] + tfifth_pos2[0:thp2], traj_bot2)))

    full_traj = np.append(half_traj1,half_traj2)

    return full_traj


def drumbot(traj1, traj2, traj3, traj4, traj5, traj6, traj7, arm):

    #j_angles = pos
    track_time = time.time()
    initial_time = time.time()

    # print(len(traj1))
    # print(len(traj2))
    # print(len(traj3))
    # print(len(traj4))
    # print(len(traj5))
    # print(len(traj6))
    # print(len(traj7))

    for i in range(min(len(traj2),len(traj4),len(traj6))):
    #for i in range(len(traj2)):
        # run command
        #start_time = time.time()
        #j_angles[4] = traj[i]
        #arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)
        jointangles = [traj1[i],traj2[i],traj3[i],traj4[i],traj5[i],traj6[i],traj7[i]]
        print(traj2[i])
        arms[arm].set_servo_angle_j(angles=jointangles, is_radian=False)
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


def drummer(inq, num):
    # test what num is, then set trajectory depending on num

    i = 1

    # when num is 5 (snare)

        #get velocity of C1 as 1,2,3,4 to determine the next IP

        #if velocity of C1 is 1 (middle snare):
            IPN = IP1
        # if velocity of C1 is 2 (pure rim):
            IPN = IP2
        # if velocity of C1 is 3 (pure wood):
            IPN = IP3
        # if velocity of C1 is 4 (rimshot):
            IPN = IP4
        # if else, default to 1
            IPN = IP1


        # if note is 1 (normal strike)
        traj1 = spline_poly(IP1[0], FP1[0], IPN[0], .2, .08, 0, .32)
        traj2 = spline_poly(IP1[1], FP1[1], IPN[1], .4, .08, 0, 0)
        traj3 = spline_poly(IP1[2], FP1[2], IPN[2], .2, .08, 0, .32)
        traj4 = spline_poly(IP1[3], FP1[3], IPN[3], .32, .08, 0, 0.18)
        traj5 = spline_poly(IP1[4], FP1[4], IPN[4], .2, .08, 0, 0.32)
        traj6 = spline_poly(IP1[5], FP1[5], IPN[5], .2, .08, 0, 0.32)
        traj7 = spline_poly(IP1[6], FP1[6], IPN[6], .2, .08, 0, 0.32)

        #if note is 2 (double strike)
        #added .1 to all stopbots
        traj1 = spline_poly(IP1[0], FP1[0], IPN[0], .2, .08, 0, .42)
        traj2 = spline_poly(IP1[1], FP1[1], IPN[1], .4, .08, 0, .1)
        traj3 = spline_poly(IP1[2], FP1[2], IPN[2], .2, .08, 0, .42)
        traj4 = spline_poly(IP1[3], FP1[3], IPN[3], .32, .08, 0, 0.28)
        traj5 = spline_poly(IP1[4], FP1[4], IPN[4], .2, .08, 0, 0.42)
        traj6 = spline_poly(IP1[5], FP1[5], IPN[5], .2, .08, 0, 0.42)
        traj7 = spline_poly(IP1[6], FP1[6], IPN[6], .2, .08, 0, 0.42)

        #if note is 3 (triple strike)
        #added .2 to all stopbots
        traj1 = spline_poly(IP1[0], FP1[0], IPN[0], .2, .08, 0, .52)
        traj2 = spline_poly(IP1[1], FP1[1], IPN[1], .4, .08, 0, .2)
        traj3 = spline_poly(IP1[2], FP1[2], IPN[2], .2, .08, 0, .52)
        traj4 = spline_poly(IP1[3], FP1[3], IPN[3], .32, .08, 0, 0.48)
        traj5 = spline_poly(IP1[4], FP1[4], IPN[4], .2, .08, 0, 0.52)
        traj6 = spline_poly(IP1[5], FP1[5], IPN[5], .2, .08, 0, 0.52)
        traj7 = spline_poly(IP1[6], FP1[6], IPN[6], .2, .08, 0, 0.52)

        #if note is 4 (pure rim)
        traj1 = spline_poly(IP2[0], FP2[0], IPN[0], .2, .08, 0, .32)
        traj2 = spline_poly(IP2[1], FP2[1], IPN[1], .4, .08, 0, 0)
        traj3 = spline_poly(IP2[2], FP2[2], IPN[2], .2, .08, 0, .32)
        traj4 = spline_poly(IP2[3], FP2[3], IPN[3], .32, .08, 0, 0.18)
        traj5 = spline_poly(IP2[4], FP2[4], IPN[4], .2, .08, 0, 0.32)
        traj6 = spline_poly(IP2[5], FP2[5], IPN[5], .2, .08, 0, 0.32)
        traj7 = spline_poly(IP2[6], FP2[6], IPN[6], .2, .08, 0, 0.32)

        #if note is 5 (pure wood)
        traj1 = spline_poly(IP3[0], FP3[0], IPN[0], .2, .08, 0, .32)
        traj2 = spline_poly(IP3[1], FP3[1], IPN[1], .4, .08, 0, 0)
        traj3 = spline_poly(IP3[2], FP3[2], IPN[2], .2, .08, 0, .32)
        traj4 = spline_poly(IP3[3], FP3[3], IPN[3], .32, .08, 0, 0.18)
        traj5 = spline_poly(IP3[4], FP3[4], IPN[4], .2, .08, 0, 0.32)
        traj6 = spline_poly(IP3[5], FP3[5], IPN[5], .2, .08, 0, 0.32)
        traj7 = spline_poly(IP3[6], FP3[6], IPN[6], .2, .08, 0, 0.32)

        #if note is 6 (rimshot rim + skin)
        traj1 = spline_poly(IP4[0], FP4[0], IPN[0], .2, .08, 0, .32)
        traj2 = spline_poly(IP4[1], FP4[1], IPN[1], .4, .08, 0, 0)
        traj3 = spline_poly(IP4[2], FP4[2], IPN[2], .2, .08, 0, .32)
        traj4 = spline_poly(IP4[3], FP4[3], IPN[3], .32, .08, 0, 0.18)
        traj5 = spline_poly(IP4[4], FP4[4], IPN[4], .2, .08, 0, 0.32)
        traj6 = spline_poly(IP4[5], FP4[5], IPN[5], .2, .08, 0, 0.32)
        traj7 = spline_poly(IP4[6], FP4[6], IPN[6], .2, .08, 0, 0.32)

    # when num is 6 (bodhron)
        # if note is 1 (normal strike)
        traj1 = spline_poly(IP1[0], FP1[0], IPN[0], .2, .08, 0, .32)
        traj2 = spline_poly(IP1[1], FP1[1], IPN[1], .4, .08, 0, 0)
        traj3 = spline_poly(IP1[2], FP1[2], IPN[2], .2, .08, 0, .32)
        traj4 = spline_poly(IP1[3], FP1[3], IPN[3], .32, .08, 0, 0.18)
        traj5 = spline_poly(IP1[4], FP1[4], IPN[4], .2, .08, 0, 0.32)
        traj6 = spline_poly(IP1[5], FP1[5], IPN[5], .2, .08, 0, 0.32)
        traj7 = spline_poly(IP1[6], FP1[6], IPN[6], .2, .08, 0, 0.32)

while True:
    play = inq.get()
    print("got!")

    #send trajectories to drumbot to perform
    drumbot(traj1, traj2, traj3, traj4, traj5, traj6, traj7, num)

def strummer(inq,num):
    i = 0
    uptraj = fifth_poly(-strumD/2, strumD/2, speed)
    downtraj = fifth_poly(strumD/2, -strumD/2, speed)
    both = [uptraj, downtraj]
    tension = fifth_poly(0, -20, 0.5)
    release = fifth_poly(-20, 0, 0.75)
    while True:
        play = inq.get() #WHERE I AM GETTING A PLAY A NOT COMMAND

        print("got!")
        if play == 1:
            direction = i % 2
            time.sleep(delayarray[direction, num]) #time delay before playing
            print(num)
            print(delayarray[0, num])
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
    #global positions
    global IPN
    global IP1
    global FP1
    global IP2
    global FP2
    global IP3
    global FP3
    global IP4
    global FP4

    strumD = 30
    speed = 0.25
    #SIP are strings initial positions
    
    SIP0 = [-1, 87.1, -2, 126.5, -strumD/2, 51.7, -45]
    SIP1 = [2.1, 86.3, 0, 127.1, -strumD/2, 50.1, -45]
    SIP2 = [1.5, 81.6, 0.0, 120, -strumD/2, 54.2, -45]
    SIP3 = [2.5, 81, 0, 117.7, -strumD/2, 50.5, -45]
    SIP4 = [-1.6, 81.8, 0, 120, -strumD/2, 50.65, -45]         # [-3.9, 65, 3.5, 100.3, -strumD/2, 42.7, 101.1]
    DRUM1 = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0] #DRUMMMING
    DRUM2 = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0] #DRUMMMING
    notes = np.array([64, 60, 69, 55, 62])
    #drumnote nparray
    
    #IPS for different drum strikes on snare

    #IP1 is middle snare
    IP1 = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    FP1 = [0.1, 30, 0.1, 60, 0.1, -12, 0.1]
    #IP2 is pure rim
    IP2 = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    FP2 = [0.1, 65, 0.1, 88.1, 0.1, -8, 0.1]
    #IP3 is pure wood
    IP3 = [0, 41.6, -16.1, 101.7, 0, 6, 0]
    FP3 = [0.1, 60, -16.0, 81.3, 0.1, 8.2, 0.1]
    # IP4 is rimshot (rim and skin)
    IP4 = [30.0, 67.2, 21.8, 109.1, 94.7, -94.9, -31.4]
    FP4 = [30.1, 75.8, 21.9, 90.4, 94.8, -80.1, -31.5]



    IP = [SIP1, SIP2, SIP3, SIP4, DRUM1, DRUM2]
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
    delayarray = np.array([[0.15, 0.15, 0.15, 0.15, 0.15, 0.0, 0.0], [0.1, 0.15, 0.1, 0.15, 0.125, 0.0, 0.0]])

    #nums left to right

    xArm0 = Thread(target=strummer, args=(q0, 0,)) #num 2
    xArm1 = Thread(target=strummer, args=(q1, 1,)) #num 4
    xArm2 = Thread(target=strummer, args=(q2, 2,)) #num 1
    xArm3 = Thread(target=strummer, args=(q3, 3,)) #num 3
    xArm4 = Thread(target=strummer, args=(q4, 4,)) #num 5
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

    #w/out rtp midi
    # while True:
    #     dq1.put(1)
    #     time.sleep(1.0)


