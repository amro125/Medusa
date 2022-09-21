import os
import sys
import time
import numpy as np
import math
from queue import Queue
from threading import Thread
from xarm.wrapper import XArmAPI
import random
from GuitarBotUDP import GuitarBotUDP


def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        angle = pos.copy()
        angle[4] = 0.0
        a.set_servo_angle(angle=angle, wait=False, speed=10, acceleration=0.25, is_radian=False)
def snakeMove(a,num):


    direction = random.randint(0, 36)
    speed = random.choice()
    J2 = random.randint(2, 10)
    J4 = random.randint(10, 30) # 130 mid max range 30



if __name__ == '__main__':
    # global arm1
    UDP_IP = "192.168.1.50"
    UDP_PORT = 1001
    arm1 = XArmAPI('192.168.1.208')     # ALWAYS GIVER       Actually Robot 1
    arm2 = XArmAPI('192.168.1.226')     # ALWAYS RECEIVER    Actually Robot 4
    arm3 = XArmAPI('192.168.1.244')
    arm4 = XArmAPI('192.168.1.203')
    arm5 = XArmAPI('192.168.1.237')

    arms = [arm1, arm2, arm3, arm4, arm5]
    # arms = [arm1]
    totalArms = len(arms)
    setup()
    input("lets go")

    # input("letsgo again")
    for a in arms:
        a.set_mode(1)
        a.set_state(0)

    # totalArms = len(arms)
    # test = np.array([1, 2, 3])
    # print(len(np.where(test == 4)[0]))
    # x, y = input("Enter two values: ").split()
    # input("nice")


    while True:
        action, control = input("what do? Motor control (M)?").split()
        # if user == m:
        action = int(action)
        control = int(control)
        if action == 1:
            j2 = fifth_poly(initial[1], positions[control][1], 1)
            j4 = fifth_poly(initial[3], positions[control][3], 1)
            strumbot(j2, j4)
            initial = positions[control]
        elif action == 2:
            if control == 1:
                guitarbot_udp.send_msg_picker(ipickercommand=2, bstartpicker=1, pgain=5000, dgain=50, ipickerpos=10,
                                              ipickervel=7, ipickeracc=100)
            # input()
            elif control == 0:
                guitarbot_udp.send_msg_picker(ipickercommand=2, bstartpicker=0, pgain=5000, dgain=50, ipickerpos=10,
                                              ipickervel=7, ipickeracc=100)