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

# RTP MIDI STUFF
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
                    q0.put(1)


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
        # curIP = IP[a]
        # arms[a].set_servo_angle(angle=curIP, wait=False, speed=10, acceleration=0.25, is_radian=False)

        arms[a].set_servo_angle(angle=[0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0], wait=False, speed=10, acceleration=0.25, is_radian=False)

def spline_poly(q_i, q_f, ta, tt, ts, dir):


    #dir is direction (0 is down, 1 is up)
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

    #ts is stall time (time waiting at bottom)
    traj_ts = np.arange(0, ts, 0.004)
    traj_ts = np.ones(len(traj_ts)-1) * (pc[len(pc)-1] + tfifth_pos[thp])
    print(traj_ts)

    #print("pc")
    #print(pc)

    #print("tfifth_pos")
    #print(pc[len(pc)-1] + tfifth_pos)

    half_traj = np.concatenate((fifth_pos[0:hp], pc, pc[len(pc)-1] + tfifth_pos[0:thp], traj_ts))
    #full_traj = np.append(half_traj, np.flip(half_traj))
    #if dir == 0: #down
        #do nothing
    if dir == 1:  #up
        half_traj = np.flip(half_traj)

    return half_traj


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
        print(mvpose[4])
        arms[arm].set_servo_cartesian(mvpose, speed=100, mvacc=2000)
        while track_time < initial_time + 0.004:
            track_time = time.time()
            time.sleep(0.0001)
        initial_time += 0.004


def drummer(inq,num):

    i = 1

    hidowntrajz = spline_poly(325, 20, .12, .04, .01, 0)
    hidowntrajp = spline_poly(-89, -30, .12, .04, .01, 0)

    hiuptrajz = spline_poly(325, 20, .12, .04, .01, 1)
    hiuptrajp = spline_poly(-89, -30, .12, .04, .01, 1)

    lodowntrajz = spline_poly(85, 20, .12, .04, .01, 0)
    lodowntrajp = spline_poly(-89, -30, .12, .04, .01, 0)

    louptrajz = spline_poly(85, 20, .12, .04, .01, 1)
    louptrajp = spline_poly(-89, -30, .12, .04, .01, 1)

    #traj for deeper hits(right now where pitch and z go lower, tt is shorter)

    hidowntrajz2 = spline_poly(325, -20, .07, .04, .01, 0) #don't use (probably)
    hidowntrajp2 = spline_poly(-89, 30, .07, .04, .01, 0)

    hiuptrajz2 = spline_poly(325, -20, .07, .04, .01, 1)
    hiuptrajp2 = spline_poly(-89, 30, .07, .04, .01, 1)

    lodowntrajz2 = spline_poly(85, -20, .07, .04, .01, 0)
    lodowntrajp2 = spline_poly(-89, 30, .07, .04, .01, 0)

    louptrajz2 = spline_poly(85, -20, .07, .04, .01, 1)
    louptrajp2 = spline_poly(-89, 30, .07, .04, .01, 1)

    #hihi
    hihitrajz = np.append(hidowntrajz, hiuptrajz)
    hihitrajp = np.append(hidowntrajp, hiuptrajp)

    #hilo
    hilotrajz = np.append(hidowntrajz, louptrajz)
    hilotrajp = np.append(hidowntrajp, louptrajp)

    #lolo
    lolotrajz = np.append(lodowntrajz2, louptrajz2)
    lolotrajp = np.append(lodowntrajp2, louptrajp2)

    #lohi
    lohitrajz = np.append(lodowntrajz2, hiuptrajz2)
    lohitrajp = np.append(lodowntrajp2, hiuptrajp2)

    while True:

        play = inq.get()
        print("got!")
        #drumbot(trajz, trajp, num)
        #end of run indef

        if i == 1:
            drumbot(hihitrajz, hihitrajp, num)
        elif i == 2:
            drumbot(hilotrajz, hilotrajp, num)
        elif i == 3:
            drumbot(lolotrajz, lolotrajp, num)
        elif i == 4:
            drumbot(lohitrajz, lohitrajp, num)
            i = 0

        i = i + 1

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
    IP = [0, 23.1, 0, 51.4, 0, -60.8, 0]

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

    #input("start RTP MIDI")
   # rtp_midi = RtpMidi(ROBOT, MyHandler(), PORT)
    #print("test")
    #rtp_midi.run()
    #print("test2")


    while True:
        q0.put(1)
    #    xArm0 = Thread(target=drummer, args=(q0, 0,))
     #   xArm0.start()
        time.sleep(1)