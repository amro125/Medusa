import os
import sys
import time
import csv
import pdb
from queue import Queue
from threading import Thread
import time

robotArr = [0,1,2,3,4]

def cvt(path,rangel):
    return_vector = []
    for i in range(7 * rangel):
        return_vector.append(float(path[i]))
    return return_vector

def playDance(step):
    length = len(step)
    # for a in range (3):
    #     print(2-a)
    #     time.sleep(1)
    for i in range(length):
        start_time = time.time()
        # board.digital[4].mode = pyfirmata.INPUT
        # board.digital[2].mode = pyfirmata.INPUT
        # red = board.digital[4].read()

        j_angles = (step[i])
        # b=6
        # arms[b].set_servo_angle_j(angles=j_angles[(b * 7):((b + 1) * 7)], is_radian=False)

        for b in robotArr:
            arms[b].set_servo_angle_j(angles=j_angles[(b * 7):((b + 1) * 7)], is_radian=False)
        tts = time.time() - start_time
        sleep = 0.006 - tts

        if tts > 0.006:
            sleep = 0

        print(tts)
        time.sleep(sleep)



def readFile(csvFile, rangel):
    flower = []
    with open(csvFile, newline='') as csvfile:
        paths_reader_j = csv.reader(csvfile, delimiter=',', quotechar='|')
        for path in paths_reader_j:
            flower.append(cvt(path,rangel))
    return flower


def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        a.set_servo_angle(angle=[0.0, 0.0, 0.0, 1.57, 0.0, 0, 0.0], wait=False, speed=0.2, acceleration=0.25,
                          is_radian=True)
if __name__ == "__main__":
    ROBOT = "xArms"
    PORT = 5004

    sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
    from xarm.wrapper import XArmAPI


    arm3 = XArmAPI('192.168.1.237')
    #arm4 = XArmAPI('192.168.1.211')#
    #arm5 = XArmAPI('192.168.1.234')#
    arm6 = XArmAPI('192.168.1.203')
    arm7 = XArmAPI('192.168.1.208')#
    arm8 = XArmAPI('192.168.1.226')  #
    arm9 = XArmAPI('192.168.1.244')  #
    #arm10 = XArmAPI('192.168.1.204')


    arms = [arm7, arm8, arm9, arm6, arm3]
    totalArms = len(arms)


    directory = '/home/codmusic/Desktop/xArm/KaylasDance/Peripheral/FiveRobot'
    dances = []
    trajectories = sorted(os.listdir(directory))
    s = time.time()
    for filename in trajectories:
        if filename.endswith(".csv"):
            print(filename)
            currentDance = (readFile(os.path.join(directory, filename), totalArms))
            dances.append(currentDance)
            continue

    e = time.time()
    print("the loading time is", (e-s))
    setup()
    repeat = input("do we need to repeat? [y/n]")
    if repeat == 'y':
        setup()
    for a in arms:
        a.set_mode(1)
        a.set_state(0)


    while True:
        Dance = int(input("What Dance would you like to do?"))
        time.sleep(3)
        playDance(dances[Dance-1])


