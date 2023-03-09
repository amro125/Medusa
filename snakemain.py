import os
import sys
import time
import numpy as np
import math
import random
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


            if chn > 8 and chn < 16:  # MIDI CHANNEL IN LOGIC IS 1 HIGHER THAN THIS NUMBER!!!!!
                if command.command == 'note_on':
                    print(chn)
                    key = command.params.key.__int__()
                    velocity = command.params.velocity
                    rob = np.where(notes == key)[0]
                    # print(rob)
                    if len(rob) > 0:
                        strumtype = chn - 8
                        print(int(rob))
                        qList[int(rob)].put(strumtype)
            if chn == 2:  # MIDI CHANNEL IN LOGIC IS 1 HIGHER THAN THIS NUMBER!!!!!
                if command.command == 'note_on':
                    print(chn)
                    key = command.params.key.__int__()
                    velocity = command.params.velocity
                    rob = np.where(notes == key)[0]
                    # print(rob)
                    if len(rob) > 0:
                        print(int(rob))
                        qList[int(rob)].put(10)
            if chn == 5:  # this means its channel 11!!!!
                if command.command == 'note_on':
                    print(chn)
                    key = command.params.key.__int__()
                    velocity = command.params.velocity
                    rob = np.where(notes == key)[0]
                    # print(rob)
                    if len(rob) > 0:
                        strumtype = velocity + 5
                        print("mode to ", velocity)
                        print(int(rob))
                        qList[int(rob)].put(strumtype)


            if chn == 6:  # this means its channel 12!!!!!
                if command.command == 'note_on':
                    # print(chn)
                    key = command.params.key.__int__()
                    velocity = command.params.velocity
                    for q in qList:
                        q.put(5)
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



def snakemake(num):
    snakeSpeeds = 0.5*random.randint(3, 16)

    # snakeSpeeds = 2
    direction = random.randint(0, 36)
    j1speed = random.randint(4, 12)*snakeSpeeds
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
    ipos = positions.snakeIP[num].copy()
    traj = []
    sinjs = [0, 0, 0]
    inputs = [j2amp, j4amp, j6amp]
    phases = [0.75, 0.5, 0]
    tarray = np.arange(0, j1speed, 0.004)
    # input()
    for t in tarray:
        for x in range(len(sinjs)):
            sinjs[x] = inputs[x] * math.sin(2*(math.pi / j1speed) * (t - phases[x]))
        # print(sinjs)
        output = ipos.copy()
        output[1] = ipos[1] + sinjs[0]
        output[3] = ipos[3] + sinjs[1]
        output[5] = ipos[5] + sinjs[2]
        buffer = output.copy()
        traj.append(buffer)
    #     print(buffer)
    # input("WAUHTS")
    return ipos, traj

def poseToPose(poseI, poseF, t):
    traj = []
    for p in range(len(poseI)):
        traj.append(fifth_poly(poseI[p], poseF[p], t))
        # print(p)
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
def robomove(numarm, trajectory):
    track_time = time.time()
    initial_time = time.time()
    for j_angles in trajectory:
        # run command
        start_time = time.time()
        # print(j_angles)
        arms[numarm].set_servo_angle_j(angles=j_angles, is_radian=False)
        while track_time < initial_time + 0.004:
            track_time = time.time()
            time.sleep(0.0001)
        initial_time += 0.004

def setup():
    for a in range(len(arms)):
        arms[a].set_simulation_robot(on_off=False)
        # arms[a].motion_enable(enable=True)
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
    pos = arms[numarm].angles
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
    bothnorm = [uptraj, downtraj]
    if num < 5:
        utrajfirst  = positions.utraj[num]
        spintrajfirst = positions.spintraj[num]

        utrajsecond = utrajfirst[::-1]
        bothu = [utrajfirst, utrajsecond]

        spintrajsecond = spintrajfirst[::-1]
        bothspin = [spintrajfirst, spintrajsecond]
        wave = positions.sintraj[num]
        circ = positions.circletraj[num]
    tension = fifth_poly(0, -20, 0.5)
    release = fifth_poly(0, 20, 0.75)
    strumMode = 1

    otherwave = positions.wtraj[num]
    startT = 0
    while True:
        snakestart, snaketraj = snakemake(num)
        # print(snaketraj)
        # input("wait_)")
        initialangles = arms[num].angles
        snakepos = poseToPose(initialangles, snaketraj[0], 10)
        # print(snakestart)
        gotoPose(num, snakepos)

        # USE THE OTHER CODE TO GO TO START POSITION
        # arms[num].set_mode(0)
        # arms[num].set_state(0)
        # arms[num].set_servo_angle(angle=initial, wait=True, speed=20, acceleration=0.25, is_radian=False)
        time.sleep(0.05)
        # print("moved to start")
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

        snakei = 0
        while inq.empty() == True:
            strumMode = 10
            snakej = snaketraj[snakei]
            # start_time = time.time()
            # print(snakej)
            arms[num].set_servo_angle_j(angles=snakej, is_radian=False)

            while track_time < initial_time + 0.004:
                track_time = time.time()
                time.sleep(0.0001)
            initial_time += 0.004
            t += 0.004
            snakei += 1
            if snakei == len(snaketraj):
                snakei = 0



        stallref = time.time()
        dir = 0
        while stall < 20:
            if inq.empty() == True:
                stall = time.time() - stallref
                time.sleep(0.004)
            else:
                play = inq.get()
                newmode = int(play)
                if newmode == 10:
                    newmode = strumMode
                if newmode != strumMode:
                    i = 0
                    tswitch = 1
                    if strumMode == 10:
                        tswitch = 10
                    if newmode == 5:
                        tswitch = 10
                    poseI = arms[num].angles
                    poseF = AllIP[newmode - 1][num]
                    setp = poseToPose(poseI, poseF, 8)
                    gotoPose(num, setp)
                    strumMode = newmode

                    # posetoPose(,)
                # print("got!")

                if play == 1:  # this is normal playing
                    direction = i % 2
                    time.sleep(delayarray[direction, num])  # time delay before playing
                    print(num)
                    print(delayarray[0, num])
                    strumbot(num, bothnorm[direction])
                    i += 1
                elif play == 2:  # u traj strum
                    direction = i % 2
                    robomove(num, bothu[direction])
                    i += 1
                elif play == 3:  # Wavey strum
                    robomove(num, wave)
                elif play == 4:  # circle
                    robomove(num, circ)
                elif play == 5: # spinny
                    direction = i % 2
                    robomove(num, bothspin[direction])
                    i += 1
                elif play == 6:  # the groove wave (DANCE)
                    robomove(num, otherwave)
                elif play == 10:
                    prepGesture(num, tension)
                    time.sleep(0.25)
                    prepGesture(num, release)
                stallref = time.time()
                stall = time.time() - stallref



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    ROBOT = "xArms"
    PORT = 5004
    global IP
    global arms
    global strumD
    global speed
    global notes

    global CP
    global CPpass


    strumD = 30
    speed = 0.25
    #SIP are strings initial positions
    

    #for the following IP's, make sure the FP is different in every element than the IP, even if its by .1

    
    #IPS for different drum strikes on snare
    #
    # #IP1 is middle snare
    # IP1 = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    # FP1 = [0.1, 48, 0.1, 60, 0.1, -12, 0.1]
    # #IP2 is pure rim
    # IP2 = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    # FP2 = [0.1, 65, 0.1, 88.1, 0.1, -8, 0.1]
    # #IP3 is pure wood
    # IP3 = [0, 41.6, -16.1, 101.7, 0, 6, 0]
    # FP3 = [0.1, 60, -16.0, 81.3, 0.1, 8.2, 0.1]
    # # IP4 is rimshot (rim and skin)
    # IP4 = [30.0, 67.2, 21.8, 109.1, 94.7, -94.9, -31.4]
    # FP4 = [30.1, 75.8, 21.9, 90.4, 94.8, -80.1, -31.5]
    # # IP5 for doubles and triples
    # IP5 = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    # FP5 = [0.1, 48, 0.1, 60, 0.1, -12, 0.1]
    # #IP6 7 and 8 are for dynamics
    # IP6 = [0, 33.1, 0, 53.4, 0, -55.8, 0]
    # FP6 = [0.1, 47, 0.1, 60, 0.1, -12, 0.1]
    # IP7 = [0, 38.1, 0, 55.4, 0, -50.8, 0]
    # FP7 = [0.1, 46, 0.1, 60, 0.1, -10, 0.1]
    # IP8 = [0, 43.1, 0, 57.4, 0, -45.8, 0]
    # FP8 = [0.1, 45, 0.1, 60, 0.1, -8, 0.1]
    # #current position variables
    # CP = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0]  # DRUMMMING
    # CPpass = 0; #0 is no go, 1 is good to go
    #
    #
    # #IPS for strikes on Bodhron
    #
    # BIP1 = [0, 23.1, 0, 51.4, 0, -60.8, 0]
    # BFP1 = [0.1, 53, 0.1, 60, 0.1, -12, 0.1]
    # BFP2 = [0.1, 70, 0.1, 88.1, 0.1, -8, 0.1]



    SIP0 = [-0.25, 87.38, -2, 126.5, -strumD/2, 51.73, -45]
    SIP1 = [2.62, 86.2, 0, 127.1, -strumD/2, 50.13, -45]
    SIP2 = [1.3, 81.68, 0.0, 120, -strumD/2, 54.2, -45]
    SIP3 = [-1.4, 83.8, 0, 120, -strumD/2, 50.75, -45]
    SIP4 = [-1.8, 81.8, 0, 120, -strumD/2, 50.65, -45]         # [-3.9, 65, 3.5, 100.3, -strumD/2, 42.7, 101.1]
    DRUM1 = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0] #maxon
    DRUM2 = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0] #not amxon

    #notes for strings
    notes = np.array([64, 60, 69, 55, 62, 58, 57])
    # drumnote nparray
    # drumnotes = np.array([58, 59, 60, 61, 62, 63, 64, 65, 66, 67])




    IP = [SIP0, SIP1, SIP2, SIP3, SIP4, DRUM1, DRUM2]
    global AllIP

    print(positions.IPu)
    AllIP = [IP, positions.IPu, positions.IPs, positions.IPc, positions.IPus, positions.IPw]


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
    # setup2()
    # input("letsgo again")
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
    drumArm1 = Thread(target=strummer, args=(dq1, 5,)) #snare
    drumArm2 = Thread(target=strummer, args=(dq2, 6,)) #bodhron






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
    for q in qList:
        q.put(1)
    # time.sleep(5)
    # q1.put(2)
    # input()


    #
    # while True:
    #     robot, mode = input("play mode \n").split()
    #     # time.sleep(5)
    #     qList[int(robot)].put(int(mode))
    #     time.sleep(4)

    rtp_midi = RtpMidi(ROBOT, MyHandler(), PORT)
    print("rtpMidi Started")
    rtp_midi.run()

    #w/out rtp midi
    # while True:
    #     dq1.put(1)
    #     time.sleep(1.0)


