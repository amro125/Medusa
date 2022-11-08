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

        arms[a].set_servo_angle(angle=[9.3, 58.2, -8.1, 95.9, 190.9, 19.7, -98.3], wait=False, speed=10, acceleration=0.25, is_radian=False)



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

def livetraj(inq, robot):
    tf = 2
    # q i
    # 0.2 * np.floor(xi / 0.2)
    # range is 200 to -200
    t0 = 0
    t = t0
    q_i = 0
    q_dot_i = 0
    q_dot_f = 0
    q_dotdot_i = 0
    q_dotdot_f = 0
    t_array = np.arange(0, tf, 0.006)
    p = 0
    v = 0
    a = 0
    dancet = 0
    while True:
        goal = inq.get()
        print("moving", robot)
        q_i = p
        q_dot_i = 0
        q_dotdot_i = 0
        q_f = goal
        i = 0
        # or dancet != 0
        while (i <= len(t_array) or dancet != 0):
            start_time = time.time()
            if inq.empty() == False:
                goal = inq.get()
                print("switch bot", robot)
                q_i = p
                q_dot_i = v
                q_dotdot_i = 0
                q_f = goal
                i = 0
                # IF YOU WANT TO ADD SPEED CHANGES THEN SWAP THE ABOVE LINES WITH THE BELOW LINES
                # # q should input an array of [*absolute* position of joint, time(in seconds) to reach there]
                # q_f = goal[0]
                # tf = goal[1]
                # t_array = np.arange(0, tf, 0.006)
                # print("switch")
            if i >= len(t_array):
                t = tf
                # if at end, append more time
                # t_array = np.append(t_array, tf+0.006)
                # print(t_array)
                dancet += 0.006
            else:
                t = t_array[i]
                dancet = t



            # amplitude returns sin wave to oscillate over



            xwave = np.sin(dancet) * 133.5
            ywave = np.sin(dancet) * 182
            zwave = np.sin(dancet) * 79.5



            mvpose = [589.1 + xwave, 30 + ywave, 40.6 + zwave, -122.1, -7.2, -89.2]


            #arms[robot].set_servo_cartesian(mvpose, speed=100, mvacc=2000)
            print(mvpose[0])
            tts = time.time() - start_time
            sleep = 0.004 - tts

            if tts > 0.004:
                sleep = 0

            # print(tts)
            time.sleep(sleep)
            i += 1
            # if t == 1:
            # print(t, p, v, a)
        print("done")
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    ROBOT = "xArms"
    PORT = 5004
    global IP
    # global arms


    IP = [9.3, 58.2, -8.1, 95.9, 190.9, 19.7, -98.3]
    IPcar = [589.1, 31.2, 40.6, -122.1, -7.2, -89.2] #ip cartesian



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

    xArm0 = Thread(target=livetraj, args=(q0, 0,))
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
        #time.sleep(10)