import os
import sys
import time
import numpy as np
import math
from xarm.wrapper import XArmAPI



def strumbot(traj):
    j_angles = pos
    for i in range(len(traj)):
        # run command
        start_time = time.time()
        j_angles[0] = traj[i]
        arms[0].set_servo_angle_j(angles=j_angles, is_radian=False)
        tts = time.time() - start_time
        sleep = 0.004 - tts
        print(j_angles)
        if tts > 0.004:
            print(tts)
            sleep = 0
        time.sleep(sleep)


def setup(strumD):
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        angle = IPstring.copy()
        # angle[0] = angle[0]- strumD/2
        a.set_servo_angle(angle=angle, wait=False, speed=10, acceleration=0.25, is_radian=False)


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


def dancepos(startp,t):
    amp5 = headS*16.6
    amp3 = 0.4*amp5
    amp1 = 0.2*amp5
    amp5extra = 30
    amptwist = 200
    headS5 = 0.045*amptwist*headS
    headS3 = 0.09*amptwist*headS

    j_angles = startp.copy()
    strike = (math.sin((math.pi / headS3) * (t)))
    two = math.sin((math.pi / headS) * (t-0.75*headS))
    three = math.sin((math.pi / headS3) * (t))
    four = math.sin((math.pi / headS) * (t-0.5*headS))
    five = math.sin((math.pi / headS5) * (t))
    six = math.sin((math.pi / (headS) * (t)))

    j_angles[0] = startp[0] - baseamp * strike
    j_angles[1] = startp[1] - amp1 * two
    j_angles[2] = startp[2] + amptwist*three
    j_angles[3] = startp[3] + amp3 * four
    j_angles[4] = startp[4] + amptwist*five
    j_angles[5] = startp[5] - amp5 * six 
    j_angles[6] = startp[6] - (amptwist*five+amptwist*three - baseamp * strike)
    return j_angles



if __name__ == '__main__':
    joint = 0
    global baseamp
    global uamp
    scaledbase = 2
    baseamp = 350
    uamp = 30
    global IPstring
    IPstring = [0.0, 3, 0.0, 125, 0.0, 11.0, -45]

    # IP0u = IPstring.copy()
    # IP0u[2] = IP0u[2]-baseamp/2
    # IP0u[3] = IP0u[3] + uamp
    # IP0u[0] = IP0u[0] - (baseamp/(scaledbase*2))
    arm1 = XArmAPI('192.168.1.242')
    global arms
    arms = [arm1]

    # pos2 = [39.5, 84.5, -40, 96.6,-strumD / 2, 49.3, -31.2]


    # totalArms = len(arms)

    # test = np.array([1, 2, 3])
    # print(len(np.where(test == 4)[0]))
    setup(2)
    
    # input("test simulation")
    # for traj in utrajectory:
    #     time.sleep(0.004)
    #     print(traj)

    # print(pos)
    # print(pos[joint]-strumD/2)
    input("test strumming")
    global zoom
    global strikes
    global headS

    # zoom =30
    strikes = 420
    headS = 2
    arm1.set_mode(0)
    arm1.set_state(0)
    starting = dancepos(IPstring, 0)
    arm1.set_servo_angle(angle=starting, wait=True, speed=10, acceleration=0.25,
                      is_radian=False)
    
    input("begin strum")
    
    
    arm1.set_mode(1)
    arm1.set_state(0)
    i = 0
    amp = uamp

    t = 0.0
    input("start")
    time.sleep(5)
    while True:
        start_time = time.time()
        # j_angles[0] = pose[0] + 0.5 * (baseamp/scaledbase) * strike + 0.5 * (baseamp/scaledbase)
        goto = dancepos(IPstring, t)
        t += 0.004
        tts = time.time() - start_time

        # print(goto)
        arm1.set_servo_angle_j(angles=goto, is_radian=False)
        while tts < 0.004:
            # print(tts)
            tts = time.time() - start_time
            time.sleep(0.0001)
        # print(tts)


        