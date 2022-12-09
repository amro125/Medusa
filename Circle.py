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

        arms[a].set_servo_angle(angle=[-12.2, 52.8, 26.5, 44, 11.9,-77.5, 69.3], wait=False, speed=10, acceleration=0.25, is_radian=False)





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
        #print("moving", robot)
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
                #print("switch bot", robot)
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



            xwave = np.cos(3*dancet) * 48
            ywave = np.sin(3*dancet) * 50
            zwave = -np.cos(3*dancet) * 22
            #xwave = 0
            #ywave = 0
            #zwave = 0



            mvpose = [425 + xwave, 17.6 + ywave, 102 + zwave, -111.2, 0, -90.2]


            if test:
                print(arms[robot].get_servo_angle())
                print(mvpose)
                test = False


            arms[robot].set_servo_cartesian(mvpose, speed=100, mvacc=2000)
            # print(mvpose[2])
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


    IP = [40.7, 63.9, -40, 83.9, 177.1, 24.4, -54]
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

    q0.put(1)

    # while True:
    #     q0.put(1)
    #     #xArm0 = Thread(target=drummer, args=(q0, 0,))
    #     #xArm0.start()
    #     time.sleep(22)