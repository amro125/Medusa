import os
import sys
import time
import numpy as np
import math
import matplotlib.pyplot as plt

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

        arms[a].set_servo_angle(angle=[0, 23.1, 0, 51.4, 0, -60.8, 0], wait=False, speed=10, acceleration=0.25, is_radian=False)

def spline_poly(q_i, q_f, q_in, ta, tt, ttopstop, tbotstop, pba, pbt, pbtopstop):
    # qi is initial pos, qf is final pos (strike), qin is new initial (return pos)

    # initial accel (using first half of a 5th order poly)
    # ta is double the time till max acceleration (time doing 5th order poly)

    # pba is pullback amount, pbt is pullback time (.5 is good default)

    ########### this code calculates the trajectory for the first half (the way down) #############

    traj_ta = np.arange(0, ta, 0.004)
    dq_i = 0
    dq_f = 0
    ddq_i = 0
    ddq_f = 0
    a0 = (q_i - pba)
    a1 = dq_i
    a2 = 0.5 * ddq_i
    a3 = 1 / (2 * ta ** 3) * (
                20 * (q_f - (q_i - pba)) / 2 - (8 * dq_f + 12 * dq_i) * ta - (3 * ddq_f - ddq_i) * ta ** 2)
    a4 = 1 / (2 * ta ** 4) * (
                30 * ((q_i - pba) - q_f) / 2 + (14 * dq_f + 16 * dq_i) * ta + (3 * ddq_f - 2 * ddq_i) * ta ** 2)
    a5 = 1 / (2 * ta ** 5) * (12 * (q_f - (q_i - pba)) / 2 - (6 * dq_f + 6 * dq_i) * ta - (ddq_f - ddq_i) * ta ** 2)
    fifth_pos = a0 + a1 * traj_ta + a2 * traj_ta ** 2 + a3 * traj_ta ** 3 + a4 * traj_ta ** 4 + a5 * traj_ta ** 5
    fifth_vel = a1 + 2 * a2 * traj_ta + 3 * a3 * traj_ta ** 2 + 4 * a4 * traj_ta ** 3 + 5 * a5 * traj_ta ** 4

    # halfway point of acceleration array (hp)
    hp = math.floor(len(fifth_pos) / 2)
    delta1 = abs(fifth_pos[0] - fifth_pos[hp])
    # speed halfway (max speed)
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
    delta3 = abs((q_i - pba) - q_f) - delta1 - delta2
    if (delta3 < 0):
        print("accel time and turnaround time too big")

    tc = delta3 / abs(hv)

    traj_tc = np.arange(0, tc, 0.004)
    pc = fifth_pos[hp] + traj_tc * hv

    # stall time at top / bottom
    # traj_top = np.ones(int(ttopstop / 0.004)) * q_i  # time stopped at top of trajectory, before strike
    # traj_top can be used for the pullback

    traj_top = fifth_poly(q_i, q_i - pba, pbt, pbtopstop, 0)
    thp_pb = math.floor(len(traj_top) / 2)  # halfway point of turnaround traj
    traj_top = traj_top[0:thp_pb]

    traj_bot = np.ones(
        int(tbotstop / 0.004)) * q_f  # time stopped at bottom of trajectory, after strike (half of the total time)

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


def drumbot(traj2, traj4, traj6, arm):
# def drumbot(traj1, traj2, traj3, traj4, traj5, traj6, traj7, arm):

    #j_angles = pos
    track_time = time.time()
    initial_time = time.time()
    for i in range(min(len(traj2),len(traj4),len(traj6))):
        # for i in range(min(len(traj1),len(traj2),len(traj3),len(traj4),len(traj5),len(traj6),len(traj7))):
        # run command
        # start_time = time.time()
        # j_angles[4] = traj[i]
        # arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)

        jointangles = [0, traj2[i], 0, traj4[i], 0, traj6[i], 0]
        #jointangles = [traj1[i], traj2[i], traj3[i], traj4[i], traj5[i], traj6[i], traj7[i]]

        print(traj6[i])
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

    # traj2 = fifth_poly(IP[1], FP[1], 1.25, .25, .254)
    # traj4 = fifth_poly(IP[3], FP[3], 1.25, .45, .054)
    # traj6 = fifth_poly(IP[5], FP[5], .75, 1, .004)

    # traj2 = spline_poly(IP[1], FP[1], .7, .08, .25, .254)
    # traj4 = spline_poly(IP[3], FP[3], .7, .08, .45, .054)
    # traj6 = spline_poly(IP[5], FP[5], .7, .08,  1, .004)
    #
    # plt.plot(np.arange(0, len(traj2), 1), traj2)
    # plt.show()


    while True:
        # i+=1
        play = inq.get()
        # print(i)
        print("got!")

        drumbot(traj2, traj4, traj6, num)

        # more joints
        #drumbot(traj1, traj2, traj3, traj4, traj5, traj6, traj7, num)

        # end of run indef

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
    IP = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    FP = [0, 51, 0, 60, 0, -12, 0]
    # IPN = [0, 26.1, 0, 52.4, 0, -57.8, 0]
    IPN = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    direction = 0  # 0 is decreasing range of hit
    # notes = np.array([64, 60, 69, 55, 62])

    # positions for different strike types
    # IP1 is middle snare
    IP1 = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    FP1 = [0.1, 48, 0.1, 60, 0.1, -12, 0.1]
    # IP2 is pure rim
    IP2 = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    FP2 = [0.1, 65, 0.1, 88.1, 0.1, -8, 0.1]
    # IP3 is pure wood
    IP3 = [0, 41.6, -16.1, 101.7, 0, 6, 0]
    FP3 = [0.1, 62, -16.0, 81.3, 0.1, 8.2, 0.1]

    # positions for lighter strikes
    # IP6 7 and 8 are for dynamics
    IP6 = [0, 33.1, 0, 53.4, 0, -55.8, 0]
    FP6 = [0.1, 47, 0.1, 60, 0.1, -12, 0.1]
    IP7 = [0, 38.1, 0, 55.4, 0, -50.8, 0]
    FP7 = [0.1, 46, 0.1, 60, 0.1, -10, 0.1]
    IP8 = [0, 43.1, 0, 57.4, 0, -45.8, 0]
    FP8 = [0.1, 45, 0.1, 60, 0.1, -8, 0.1]

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

    x = 0

    while True:
        # dynamics (hardest to lightest)

        # traj2 = spline_poly(IP[1], FP[1]+1, IPN[1], .4, .08, 0, 0, 32, .5, 0)
        # traj4 = spline_poly(IP[3], FP[3]+1, IPN[3], .3, .08, .13, .1, 0, .5, 0)
        # traj6 = spline_poly(IP[5], FP[5]+1, IPN[5], .2, .08, .35, .1, 32, .5, 0)

        # default hit (single) (open hit) (P 1500 D 40)
        # traj2 = spline_poly(IP[1], FP[1]-2, IPN[1], .5, .08, 0, 0, 24, .5, 0)
        # traj4 = spline_poly(IP[3], FP[3]-2, IPN[3], .4, .08, .13, .1, 0, .5, 0)
        # traj6 = spline_poly(IP[5], FP[5]-2, IPN[5], .3, .08, .35, .1, 24, .5, 0)

        # traj2 = spline_poly(IP[1], FP[1]-4, IPN[1], .6, .08, 0, 0, 16, .5, 0)
        # traj4 = spline_poly(IP[3], FP[3]-4, IPN[3], .5, .08, .13, .1, 0, .5, 0)
        # traj6 = spline_poly(IP[5], FP[5]-4, IPN[5], .4, .08, .35, .1, 16, .5, 0)

        # traj2 = spline_poly(IP[1], FP[1]-6, IPN[1], .7, .08, 0, 0, 8, .5, 0)
        # traj4 = spline_poly(IP[3], FP[3]-6, IPN[3], .6, .08, .13, .1, 0, .5, 0)
        # traj6 = spline_poly(IP[5], FP[5]-6, IPN[5], .5, .08, .35, .1, 8, .5, 0)

        # ----------------------- positional hits ---------------

        # pure rim
        # traj2 = spline_poly(IP[1], FP2[1]+8, IPN[1], .5, .08, 0, 0, 24, .5, 0)
        # traj4 = spline_poly(IP[3], FP2[3]+10, IPN[3], .4, .08, .13, .1, -10, .5, 0)
        # traj6 = spline_poly(IP[5], FP2[5]+10, IPN[5], .3, .08, .35, .1, 24, .5, 0)

        # pure wood (need to add in all joint in drumbot)
        # traj1 = spline_poly(IP[0], FP3[0], IP[0], .2, .08, .4, .32, 0, .5, 0)
        # traj2 = spline_poly(IP[1], FP3[1], IP[1], .5, .08, .1, 0, 16, .5, 0)
        # traj3 = spline_poly(IP[2], FP3[2], IP[2], .2, .08, .4, .32, 8, .5, 0)
        # traj4 = spline_poly(IP[3], FP3[3], IP[3], .4, .08, .13, 0.18, -10, .5, 0)
        # traj5 = spline_poly(IP[4], FP3[4], IP[4], .2, .08, .4, 0.32, 0, .5, 0)
        # traj6 = spline_poly(IP[5], FP3[5], IP[5], .3, .08, .35, 0.32, 16, .5, .1)
        # traj7 = spline_poly(IP[6], FP3[6], IP[6], .2, .08, .4, 0.32, 0, .5, 0)

        # ------------------ single, double, triple ---------------------

        # double (P 1000 D 40)
        # traj2 = spline_poly(IP[1], FP[1] + 3, IPN[1], .5, .08, 0, 0, 24, .5, 0)
        # traj4 = spline_poly(IP[3], FP[3], IPN[3], .4, .08, 0.1, 0, 0, .5, .13)
        # traj6 = spline_poly(IP[5], FP[5] + 3, IPN[5], .3, .08, 0.4, 0, 24, .5, .35)

        # triple (P 500 D 20)
        traj2 = spline_poly(IP[1], FP[1] - 3, IPN[1], .5, .08, 0, .2, 24, .5, 0)
        traj4 = spline_poly(IP[3], FP[3], IPN[3], .4, .08, 0.5, 0, 0, .5, .20)
        traj6 = spline_poly(IP[5], FP[5], IPN[5], .3, .08, 0.3, .1, 30, .5, .50)

        # ----------------------------------------------------------------

        # cool hit!
        # traj2 = spline_poly(IP[1], FP[1], IPN[1], .5, .08, 0, .2, 24, .5, 0)
        # traj4 = spline_poly(IP[3], FP[3], IPN[3], .4, .08, 0.1, 0, 0, .5, .20)
        # traj6 = spline_poly(IP[5], FP[5], IPN[5], .3, .08, 0.1, .1, 30, .5, .50)

        # traj2 = spline_poly(IP[1], FP[1], IP[1], .25, .08, .2, 0, 0)
        # traj4 = spline_poly(IP[3], FP[3], IP[3], .25, .08, .2, 0, 0)
        # traj6 = spline_poly(IP[5], FP[5], IP[5], .25, .08, .2, 0, 0)

        # traj2 = spline_poly(IP6[1], FP6[1], IP6[1], .3, .08, .2, 0, 0)
        # traj4 = spline_poly(IP6[3], FP6[3], IP6[3], .3, .08, .2, 0, 0)
        # traj6 = spline_poly(IP6[5], FP6[5], IP6[5], .3, .08, .2, 0, 0)

        # traj2 = spline_poly(IP7[1], FP7[1], IP7[1], .38, .08, .2, 0, 0)
        # traj4 = spline_poly(IP7[3], FP7[3], IP7[3], .38, .08, .2, 0, 0)
        # traj6 = spline_poly(IP7[5], FP7[5], IP7[5], .38, .08, .2, 0, 0)

        # traj2 = spline_poly(IP8[1], FP8[1], IP8[1], .46, .08, .2, 0, 0)
        # traj4 = spline_poly(IP8[3], FP8[3], IP8[3], .46, .08, .2, 0, 0)
        # traj6 = spline_poly(IP8[5], FP8[5], IP8[5], .46, .08, .2, 0, 0)

        # traj2 = spline_poly(IP[1], FP[1], IPN[1], .4 + x, .08, 0, 0, 0)
        # traj4 = spline_poly(IP[3], FP[3], IPN[3], .32 + x, .08, .13, 0, 0)
        # traj6 = spline_poly(IP[5], FP[5], IPN[5], .2 + x, .08, .35, 0, 0)

        # pull back 1 (line 96, time is 0.5)

        # traj2 = spline_poly(IP[1], FP[1], IPN[1], .4, .08, 0, 0, 20)
        # traj4 = spline_poly(IP[3], FP[3], IPN[3], .32, .08, .13, .1, 0)
        # traj6 = spline_poly(IP[5], FP[5], IPN[5], .2 , .08, .35, .1, 30)

        # pull back 2

        # traj2 = spline_poly(IP[1], FP[1], IPN[1], .4, .08, 0, 0, 30)
        # traj4 = spline_poly(IP[3], FP[3], IPN[3], .32, .08, .13, .1, 0)
        # traj6 = spline_poly(IP[5], FP[5], IPN[5], .2 , .08, .35, .1, 50)

        # pull back 3

        # traj2 = spline_poly(IP[1], FP[1], IPN[1], .5, .08, 0, 0, 40)
        # traj4 = spline_poly(IP[3], FP[3], IPN[3], .42, .08, .13, .1, -10)
        # traj6 = spline_poly(IP[5], FP[5], IPN[5], .3, .08, .35, .1, 30)
        #

        # -------------plotting trajectories ----

        # normal
        plt.plot(np.arange(0, len(traj2) * 0.004, 0.004), traj2, 'r', np.arange(0, len(traj4) * 0.004, 0.004), traj4,
                 'b', np.arange(0, len(traj6) * 0.004, 0.004), traj6, 'g')
        # plot for 4 joints (wood hit)
        # plt.plot(np.arange(0, len(traj2) * 0.004, 0.004), traj2, 'r', np.arange(0, len(traj4) * 0.004, 0.004), traj4,
        #          'b', np.arange(0, len(traj6) * 0.004, 0.004), traj6, 'g', np.arange(0, len(traj3) * 0.004, 0.004), traj3, 'm' )
        # plt.legend(["Joint 2", "Joint 4", "Joint 6", "Joint 3"], loc='center left', bbox_to_anchor=(1.02, 0.5))
        plt.legend(["Joint 2", "Joint 4", "Joint 6"], loc='center left', bbox_to_anchor=(1.02, 0.5))
        plt.subplots_adjust(right=0.8)
        plt.title("xArm Joint Trajectories for Triple Hit")
        plt.xlabel("Time (s)")
        plt.ylabel("Joint Angle (degrees)")
        plt.minorticks_on()
        plt.show()

        #
        # q0.put(1)
        # time.sleep(3.0)

        # q0.put(1)
        # time.sleep(3.5 - x*30)
        #
        # for i in range(len(IPN)):
        #     IP[i] = IPN[i]
        #
        # if IPN[1] > (FP[1] - 9):
        #     direction = 1
        #
        # if IPN[1] < 23:
        #     direction = 0
        #
        # if direction == 0: #smaller, softer
        #     x = x +.01
        #     FP[1] = FP[1] - 0.4
        #     IPN[1] = IPN[1] + 3
        #     IPN[3] = IPN[3] + 1
        #     IPN[5] = IPN[5] + 3
        # elif direction == 1:
        #     x = x - .01
        #     FP[1] = FP[1] + 0.4
        #     IPN[1] = IPN[1] - 3
        #     IPN[3] = IPN[3] - 1
        #     IPN[5] = IPN[5] - 3

        # xArm0 = Thread(target=drummer, args=(q0, 0,))
        # xArm0.start()

        q0.put(1)
        time.sleep(3.0)
