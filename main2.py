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

            if chn == 3:  # this means its channel 4 !!!!!
                if command.command == 'note_on':
                    #print("DRUMMO")
                    key = command.params.key.__int__()
                    velocity = command.params.velocity
                    notetype = np.where(notes == key)[0]
                    if len(notetype) > 0:
                        dq2.put(notetype)

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
                    # for q in qList:
                    #     q.put(2)
                    for q in qList[0:5]:
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

        #arms[a].set_servo_angle(angle=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait=False, speed=10, acceleration=0.25, is_radian=False)

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

def drumbot(traj2, traj4, traj6, arm):

    #j_angles = pos
    track_time = time.time()
    initial_time = time.time()
    for i in range(len(traj2)):
        # run command
        jointangles = [0, traj2[i], 0, traj4[i], 0, traj6[i], 0]
        print(traj6[i])
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

#chimbot plays chimes with drummer function
def chimbot(traj1, traj2, traj4, traj6, arm):

    #j_angles = pos
    track_time = time.time()
    initial_time = time.time()
    for i in range(len(traj1)):
        # run command
        #start_time = time.time()
        #j_angles[4] = traj[i]
        #arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)
        jointangles = [traj1[i],traj2[i],0,traj4[i],0,traj6[i],0]
        #print(traj4[i])
        arms[arm].set_servo_angle_j(angles=jointangles, is_radian=False)
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

    # hits for the snare (and bodhron for now)

    #single hit
    # trajz = spline_poly(325, 18, .10, .04, 0.01)
    # trajp = spline_poly(-89, -30, .10, .04, 0.01)
    s_traj2 = spline_poly(DRUM1IP[1], DRUM1FP[1], DRUM1IP_N[1], .4, .08, 0, .25)
    s_traj4 = spline_poly(DRUM1IP[3], DRUM1FP[3], DRUM1IP_N[3], .4, .08, .2, .05)
    s_traj6 = spline_poly(DRUM1IP[5], DRUM1FP[5], DRUM1IP_N[5], .2, .08, .75, 0)

    #double hit
    # trajz2 = spline_poly(325, 18, .10, .04, 0.1)
    # trajp2 = spline_poly(-89, -30, .10, .04, 0.1)
    d_traj2 = spline_poly(DRUM1IP[1], DRUM1FP[1], DRUM1IP_N[1], .4, .08, 0, .35)
    d_traj4 = spline_poly(DRUM1IP[3], DRUM1FP[3], DRUM1IP_N[3], .4, .08, .2, .15)
    d_traj6 = spline_poly(DRUM1IP[5], DRUM1FP[5], DRUM1IP_N[5], .2, .08, .75, .1)

    #triple hit
    # trajz3 = spline_poly(325, 18, .10, .04, 0.15)
    # trajp3 = spline_poly(-89, -30, .10, .04, 0.15)
    t_traj2 = spline_poly(DRUM1IP[1], DRUM1FP[1], DRUM1IP_N[1], .4, .08, 0, .4)
    t_traj4 = spline_poly(DRUM1IP[3], DRUM1FP[3], DRUM1IP_N[3], .4, .08, .2, .2)
    t_traj6 = spline_poly(DRUM1IP[5], DRUM1FP[5], DRUM1IP_N[5], .2, .08, .75, .15)

    # bodharn drum chime
    uptraj1 = fifth_poly(0, -103.1, 1.5)
    uptraj2 = fifth_poly(23.1, -31.5, 1.5)
    uptraj4 = fifth_poly(51.4, 99.8, 1.5)
    uptraj6 = fifth_poly(-60.8, 0, 1.5)

    midtraj1 = fifth_poly(-103.1, -143.6, 0.5)
    midtraj2 = fifth_poly(-31.5, -32.8, 0.5)
    midtraj4 = fifth_poly(99.8, 99.8, 0.5)
    midtraj6 = fifth_poly(0, 33.4, 0.5)

    combtraj1 = np.append(uptraj1, midtraj1)
    combtraj2 = np.append(uptraj2, midtraj2)
    combtraj4 = np.append(uptraj4, midtraj4)
    combtraj6 = np.append(uptraj6, midtraj6)

    downtraj1 = fifth_poly(-143.6, 0, 1.5)
    downtraj2 = fifth_poly(-32.8, 23.1, 1.5)
    downtraj4 = fifth_poly(99.8, 51.4, 1.5)
    downtraj6 = fifth_poly(33.4, -60.8, 1.5)

    traj1 = np.append(combtraj1, downtraj1)
    traj2 = np.append(combtraj2, downtraj2)
    traj4 = np.append(combtraj4, downtraj4)
    traj6 = np.append(combtraj6, downtraj6)

    #snare thunder

    thunduptraj1 = fifth_poly(0, 6.9, 1.0)
    thunduptraj2 = fifth_poly(23.1, 20.7, 1.0)
    thunduptraj4 = fifth_poly(51.4, 155.5, 1.0)
    thunduptraj6 = fifth_poly(-60.8, -60.8, 1.0)

    thunddowntraj1 = fifth_poly(6.9, 0, 1.0)
    thunddowntraj2 = fifth_poly(20.7, 23.1, 1.0)
    thunddowntraj4 = fifth_poly(155.5, 51.4, 1.0)
    thunddowntraj6 = fifth_poly(-60.8, -60.8, 1.0)

    thundtraj1 = np.append(thunduptraj1, thunddowntraj1)
    thundtraj2 = np.append(thunduptraj2, thunddowntraj2)
    thundtraj4 = np.append(thunduptraj4, thunddowntraj4)
    thundtraj6 = np.append(thunduptraj6, thunddowntraj6)

    while True:
        play = inq.get()
        print("got!")
        if play == 0:
            drumbot(s_traj2, s_traj4, s_traj6, num)
        elif play == 1:
            drumbot(d_traj2, d_traj4, d_traj6, num)
        elif play == 2:
            drumbot(t_traj2, t_traj4, t_traj6, num)
        elif play == 3:
            chimbot(traj1, traj2, traj4, traj6, num)
        elif play == 4:
            chimbot(thundtraj1, thundtraj2, thundtraj4, thundtraj6, num)
def strummer(inq,num):
    i = 0
    uptraj = fifth_poly(-strumD/2, strumD/2, speed)
    downtraj = fifth_poly(strumD/2, -strumD/2, speed)
    both = [uptraj, downtraj]
    tension = fifth_poly(0, -20, 0.5)
    release = fifth_poly(-20, 0, 0.75)
    while True:
        play = inq.get()  # WHERE I AM GETTING A PLAY A NOT COMMAND

        print("got!")
        if play == 1:
            direction = i % 2
            time.sleep(delayarray[direction, num])  # time delay before playing
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

    strumD = 30
    speed = 0.25
    IP0 = [-1, 87.1, -2, 126.5, -strumD/2, 51.7, -45]
    IP1 = [2.1, 86.3, 0, 127.1, -strumD/2, 50.1, -45]
    IP2 = [1.5, 81.6, 0.0, 120, -strumD/2, 54.2, -45]
    IP3 = [2.5, 81, 0, 117.7, -strumD/2, 50.5, -45]
    IP4 = pos = [-1.6, 81.8, 0, 120, -strumD/2, 50.65, -45]         # [-3.9, 65, 3.5, 100.3, -strumD/2, 42.7, 101.1]
    DRUM1IP = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0] #Snare
    DRUM1FP = [0, 50, 0, 60, 0, -20, 0]
    DRUM1IP_N = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0]
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
    delayarray = np.array([[0.15, 0.15, 0.15, 0.15, 0.15, 0.0, 0.0], [0.1, 0.15, 0.1, 0.15, 0.125, 0.0, 0.0]])
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
