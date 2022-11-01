import os
import sys
import time
import numpy as np
import math

#from rtpmidi import RtpMidi
#from pymidi import server
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
        # curIP = IP[a]
        # arms[a].set_servo_angle(angle=curIP, wait=False, speed=10, acceleration=0.25, is_radian=False)

        arms[a].set_servo_angle(angle=[0.0, -10.5, 0.0, 114.3, 0.0, -71.1, 0.0], wait=False, speed=10, acceleration=0.25, is_radian=False)

def spline_poly(q_i, q_f, ta, tt, ts):

    #t is total time

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
    a3 = 1 / (2 * ta ** 3) * (20 * (q_f - q_i)/6 - (8 * dq_f + 12 * dq_i) * ta - (3 * ddq_f - ddq_i) * ta ** 2)
    a4 = 1 / (2 * ta ** 4) * (30 * (q_i - q_f)/6 + (14 * dq_f + 16 * dq_i) * ta + (3 * ddq_f - 2 * ddq_i) * ta ** 2)
    a5 = 1 / (2 * ta ** 5) * (12 * (q_f - q_i)/6 - (6 * dq_f + 6 * dq_i) * ta - (ddq_f - ddq_i) * ta ** 2)
    fifth_pos = a0 + a1 * traj_ta + a2 * traj_ta ** 2 + a3 * traj_ta ** 3 + a4 * traj_ta ** 4 + a5 * traj_ta ** 5
    fifth_vel = a1 + 2 * a2 * traj_ta + 3 * a3 * traj_ta ** 2 + 4 * a4 * traj_ta ** 3 + 5 * a5 * traj_ta ** 4

    #print("fifth pos")
    #print(fifth_pos)
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

    #print("tfifth_pos")
    #print(pc[len(pc)-1] + tfifth_pos)

    #ts is stall time at bottom
    sfifth_pos = np.ones(int(ts / 0.004)) * pc[len(pc)-1] + tfifth_pos[thp]

    #print('stall')
    #print(sfifth_pos)

    half_traj = np.concatenate((fifth_pos[0:hp], pc, pc[len(pc)-1] + tfifth_pos[0:thp], sfifth_pos))
    full_traj = np.append(half_traj, np.flip(half_traj))

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
    traj_bot = np.ones(int(tbotstop / 0.004)) * q_i #time stopped at bottom of trajectory, after strike

    half_traj = np.concatenate((traj_top, traj_pos, traj_bot))
    full_traj = np.append(half_traj, np.flip(half_traj))

    return full_traj


def drumbot(traj2, traj4, traj6, arm):

    #j_angles = pos
    track_time = time.time()
    initial_time = time.time()
    for i in range(len(traj2)):
        # run command
        #start_time = time.time()
        #j_angles[4] = traj[i]
        #arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)
        jointangles = [0,traj2[i],0,traj4[i],0,traj6[i],0]
        print(traj4[i])
        #arms[arm].set_servo_angle_j(angles=jointangles, is_radian=False)
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

    traj2 = fifth_poly(IP[1], FP[1], 2, .5, 1.5)
    traj4 = fifth_poly(IP[3], FP[3], 2, 1, 1)
    traj6 = fifth_poly(IP[5], FP[5], 1.5, 2, .5)


    while True:

        # i+=1
        play = inq.get()
        print(i)
        print("got!")

        drumbot(traj2, traj4, traj6, num)


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
    # global arms
    global strumD
    global speed
    global notes

    strumD = 30
    speed = 0.25
    IP = [0, -10.5, 0, 114.3, 0, -71.1, 0]
    FP = [0, 44, 0, 33.8, 0, -53.9, 0]
    #notes = np.array([64, 60, 69, 55, 62])


    arm0 = XArmAPI('192.168.1.204')

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

    #input("start RTP MIDI")
    #rtp_midi = RtpMidi(ROBOT, MyHandler(), PORT)
    #print("test")
    #rtp_midi.run()
    #print("test2")


    while True:
        q0.put(1)
        #xArm0 = Thread(target=drummer, args=(q0, 0,))
        #xArm0.start()
        time.sleep(10)