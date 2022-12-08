import os
import sys
import time
import numpy as np
import math
#import matplotlib.pyplot as plt

#from rtpmidi import RtpMidi
#from pymidi import server
from queue import Queue
from threading import Thread
from xarm.wrapper import XArmAPI
from numpy import diff


def setup():
    for a in range(len(arms)):
        arms[a].set_simulation_robot(on_off=False)
        arms[a].motion_enable(enable=True)
        arms[a].clean_warn()
        arms[a].clean_error()
        arms[a].set_mode(0)
        arms[a].set_state(0)
        # curIP = IP[a]
        # arms[a].set_servo_angle(angle=curIP, wait=False, speed=10, acceleration=0.25, is_radian=False)

        arms[a].set_servo_angle(angle=[30.0, 67.2, 21.8, 109.1, 94.7, -94.9, -31.4], wait=False, speed=10, acceleration=0.25, is_radian=False)

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


def fifth_poly(q_i, q_f, t, ttopstop, tbotstop):
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

    traj_top = np.ones(int(ttopstop / 0.004)) * q_i #time stopped at top of trajectory, before strike
    traj_bot = np.ones(int(tbotstop / 0.004)) * q_f #time stopped at bottom of trajectory, after strike

    half_traj = np.concatenate((traj_top, traj_pos, traj_bot))
    full_traj = np.append(half_traj, np.flip(half_traj))

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



def drummer(inq,num):

    i = 0

    # trajz = spline_poly(325, 35, .08, .08, 0.01)
    # trajp = spline_poly(-89, -28, .08, .08, 0.01)
    #
    # trajz2 = spline_poly(325, 35, .08, .08, .1)
    # trajp2 = spline_poly(-89, -28, .08, .08, .1)
    #
    # trajz3 = spline_poly(325, 35, .08, .08, .15)
    # trajp3 = spline_poly(-89, -28, .08, .08, .15)

    # traj2 = fifth_poly(IP[1], FP[1]e, 1.25, .25, .254)
    # traj4 = fifth_poly(IP[3], FP[3], 1.25, .45, .054)
    # traj6 = fifth_poly(IP[5], FP[5], .75, 1, .004)



    # traj2 = spline_poly(IP[1], FP[1]e, .7, .08, .25, .254)
    # traj4 = spline_poly(IP[3], FP[3], .7, .08, .45, .054)
    # traj6 = spline_poly(IP[5], FP[5], .7, .08,  1, .004)
    #
    # plt.plot(np.arange(0, len(traj2), 1), traj2)
    # plt.show()


    while True:

        # i+=1
        play = inq.get()
        #print(i)
        print("got!")

        drumbot(traj1, traj2, traj3, traj4, traj5, traj6, traj7, num)

        #end of run indef

        # if i%3 == 1:
        #     #direction = i % 2
        #     #strumbot(downtrajz, downtrajp)
        #     drumbot(trajz, trajp, num)
        #
        # elif i%3 == 2:
        #     drumbot(trajz2, trajp2, num)
        #
        #     #strumbot(uptrajz, uptrajp)
        #     #i = 1
        #     #prepGesture(num, tension)
        #     #time.sleep(0.25)
        #     #prepGesture(num, release)
        # elif i%3 == 0:
        #     drumbot(trajz3, trajp3, num)



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    ROBOT = "xArms"
    PORT = 5004
    global IP
    global FP
    global IPN
    #global arms
    global strumD
    global speed
    global notes

    strumD = 30
    speed = 0.25

    #IP1 is rimshot (rim and skin)
    IP1 = [30.0, 67.2, 21.8, 109.1, 94.7, -94.9, -31.4]
    FP1 = [30.1, 75.8, 21.9, 90.4, 94.8, -80.1, -31.5]
    #IPN = [30.1, 67.2, 21.9, 109.1, 94.8, -94.9, -31.5]

    #IP2 is pure rim
    IP2 = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    FP2 = [0.1, 65, 0.1, 88.1, 0.1, -8, 0.1]

    # #IP3 is pure rim
    # IP2 = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    # FP2 = [0.1, 60, 0.1, 88.1, 0.1, 10.2, 0.1]

    #IP3 is pure wood
    IP3 = [0, 41.6, -16.1, 101.7, 0, 6, 0]
    FP3 = [0.1, 60, -16.0, 81.3, 0.1, 8.2, 0.1]

    #IP4 is middle snare
    IP4 = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    FP4 = [0.1, 30, 0.1, 60, 0.1, -12, 0.1]
    #direction = 0 #0 is decreasing range of hit
    #notes = np.array([64, 60, 69, 55, 62])


    arm0 = XArmAPI('192.168.1.236')

    arms = [arm0]
    # arms = [arm1]
    totalArms = len(arms)
    setup()
    input("lets go")

    for a in arms:
        a.set_mode(1)
        a.set_state(0)

    q0 = Queue()
    qList = [q0]

    xArm0 = Thread(target=drummer, args=(q0, 0,))
    xArm0.start()

    #arms[1].get_servo_angle()
    #plot position
    #plt.plot(np.arange(0, len(traj2), 1), traj2, 'r', np.arange(0, len(traj4), 1), traj4, 'b',np.arange(0, len(traj6), 1), traj6, 'g')

    #plot predicted velocity
    # dx = 1
    # dy2 = diff(diff(traj2))/ dx
    # plt.plot(np.arange(0, len(dy2), 1), dy2, 'r')

    #plt.show()

    #input("start RTP MIDI")
    #rtp_midi = RtpMidi(ROBOT, MyHandler(), PORT)
    #print("test")
    #rtp_midi.run()
    #print("test2")

    count = 0

    while True:

        if(count == 0):
            traj1 = spline_poly(IP1[0], FP1[0], IP2[0], .2, .08, 0, .32)
            traj2 = spline_poly(IP1[1], FP1[1], IP2[1], .4, .08, 0, 0)
            traj3 = spline_poly(IP1[2], FP1[2], IP2[2], .2, .08, 0, .32)
            traj4 = spline_poly(IP1[3], FP1[3], IP2[3], .32, .08, 0, 0.18)
            traj5 = spline_poly(IP1[4], FP1[4], IP2[4], .2, .08, 0, 0.32)
            traj6 = spline_poly(IP1[5], FP1[5], IP2[5], .2, .08, 0, 0.32)
            traj7 = spline_poly(IP1[6], FP1[6], IP2[6], .2, .08, 0, 0.32)
        if(count == 1):
            traj1 = spline_poly(IP2[0], FP2[0], IP3[0], .2, .08, 0, .32)
            traj2 = spline_poly(IP2[1], FP2[1], IP3[1], .4, .08, 0, 0)
            traj3 = spline_poly(IP2[2], FP2[2], IP3[2], .2, .08, 0, .32)
            traj4 = spline_poly(IP2[3], FP2[3], IP3[3], .32, .08, 0, 0.18)
            traj5 = spline_poly(IP2[4], FP2[4], IP3[4], .2, .08, 0, 0.32)
            traj6 = spline_poly(IP2[5], FP2[5], IP3[5], .2, .08, 0, 0.32)
            traj7 = spline_poly(IP2[6], FP2[6], IP3[6], .2, .08, 0, 0.32)
        if(count == 2):
            traj1 = spline_poly(IP3[0], FP3[0], IP4[0], .2, .08, 0, .32)
            traj2 = spline_poly(IP3[1], FP3[1], IP4[1], .4, .08, 0, 0)
            traj3 = spline_poly(IP3[2], FP3[2], IP4[2], .2, .08, 0, .32)
            traj4 = spline_poly(IP3[3], FP3[3], IP4[3], .32, .08, 0, 0.18)
            traj5 = spline_poly(IP3[4], FP3[4], IP4[4], .2, .08, 0, 0.32)
            traj6 = spline_poly(IP3[5], FP3[5], IP4[5], .2, .08, 0, 0.32)
            traj7 = spline_poly(IP3[6], FP3[6], IP4[6], .2, .08, 0, 0.32)
        if(count == 3):
            traj1 = spline_poly(IP4[0], FP4[0], IP1[0], .2, .08, 0, .32)
            traj2 = spline_poly(IP4[1], FP4[1], IP1[1], .4, .08, 0, 0)
            traj3 = spline_poly(IP4[2], FP4[2], IP1[2], .2, .08, 0, .32)
            traj4 = spline_poly(IP4[3], FP4[3], IP1[3], .32, .08, 0, 0.18)
            traj5 = spline_poly(IP4[4], FP4[4], IP1[4], .2, .08, 0, 0.32)
            traj6 = spline_poly(IP4[5], FP4[5], IP1[5], .2, .08, 0, 0.32)
            traj7 = spline_poly(IP4[6], FP4[6], IP1[6], .2, .08, 0, 0.32)
        #plt.plot(np.arange(0, len(traj2)*0.004, 0.004), traj2, 'r', np.arange(0, len(traj4)*0.004, 0.004), traj4, 'b', np.arange(0, len(traj6)*0.004, 0.004), traj6, 'g')
        #plt.show()

        q0.put(1)
        time.sleep(2.5)

        count = count + 1
        if(count == 4):
            count = 0


        #xArm0 = Thread(target=drummer, args=(q0, 0,))
        #xArm0.start()

