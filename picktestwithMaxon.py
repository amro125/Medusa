import os
import sys
import time
import numpy as np
import math
from GuitarBotUDP import GuitarBotUDP
from queue import Queue
from threading import Thread
from xarm.wrapper import XArmAPI
from notes import playnote
from rtpmidi import RtpMidi
from pymidi import server


# def third_poly(q_i, q_f, time):
#     traj_t = np.arange(0, time, 0.005)
#     dq_i = 0
#     dq_f = 0
#     a0 = q_i
#     a1 = dq_i
#     a2 = 3 * (q_f - q_i) / (time ** 2)
#     a3 = 2 * (q_i - q_f) / (time ** 3)
#     traj_pos = a0 + a1 * traj_t + a2 * traj_t ** 2 + a3 * traj_t ** 3
#     return traj_pos

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
            if chn == 1:  # this means its channel 14!!!!!
                if command.command == 'note_on':

                    # print("YEYE S2"
                    #       "TART")
                    key = command.params.key.__int__()
                    velocity = command.params.velocity
                    if velocity == 60:
                        velocity = 0
                    # > 30 strumming
                    # < 30 picking
                    midiq.put([key, velocity])


def soundToP(db):
    pGain = 0.0072 * math.exp(0.1572 * db)
    return pGain


def parser(parse):
    # string = [5, 7, 8, 7, 6, 6, 6, 5, 7, 8, 7, 6, 6, 5, 1, 1, 2, 2, 3, 6, 7, 8, 7, 5, 5, 5, 4]
    # string = [ 1, 2, 3, 4, 5, 6, 6, 5, 4, 3, 2, 1]
    phrase = [1, 1, 7, 7, 1, 1, 7, 1, 1, 2, 2, 3, 3, 2, 2, 2, 1, 1, 7, 7, 1, 1, 7, 2, 2, 1, 1, 1, 1]
    chord1 = [3, 5, 5, 3, 3, 3]
    playchord = [2, 2, 2, 2, 2, 2]
    chord2 = [1, 2, 3, 6, 5, 3]
    chords = [chord1, chord2]

    while True:

        for loop in range(5):
            ifret = [3, 3, 3, 3, 3, 3]
            for s in phrase:
                [key, velocity] = parse.get()

                icommand = [3, 3, 3, 3, 3, 3]

                if key > 34: # strum mode
                    mode = 1
                    chord = chords[key-35]
                    guitarbot_udp.send_msg_left(playchord, chord)
                    strumq.put([pythonSucks, mode])
                    # time.sleep(0.03)

                else: #pick mode
                    mode = 0
                    fret = key - 25
                    command = velocity
                    pythonSucks = s-1
                    ifret[pythonSucks] = fret
                    icommand[pythonSucks] = velocity
                    # print(icommand)
                    if not velocity == 5:
                        strumq.put([pythonSucks, mode])
                    time.sleep(0.1)
                    guitarbot_udp.send_msg_left(icommand, ifret)
            # fingerq.get()




def strumBot(trajx, trajy, trajz):
    for i in range(len(trajx)):
        # run command
        start_time = time.time()
        movepose = [trajx[i], trajy[i], trajz[i], -90, 0, -0]
        arm1.set_servo_cartesian(movepose)
        tts = time.time() - start_time
        sleep = 0.002 - tts
        # print(movepose)
        if tts > 0.002:
            sleep = 0
        time.sleep(sleep)


def chords(chord):
    if chord == 1: #E chord

        ifretnumber = [1, 2, 2, 1, 1, 1]
        iplaycommand = [1, 2, 2, 2, 1, 1]
        guitarbot_udp.send_msg_left(iplaycommand, ifretnumber)

    if chord == 2:  #A chord

        ifretnumber = [1, 1, 2, 2, 2, 1]
        iplaycommand = [3, 1, 2, 2, 2, 1]
        guitarbot_udp.send_msg_left(iplaycommand, ifretnumber)
    if chord == 3: #D chord
        ifretnumber = [1, 1, 1, 2, 3, 2]
        iplaycommand = [3, 3, 1, 2, 2, 2]
        guitarbot_udp.send_msg_left(iplaycommand, ifretnumber)
    if chord == 0: #Open everything
        ifretnumber = [1, 1, 1, 1, 1, 1]
        iplaycommand = [1, 1, 1, 1, 1, 1]
        guitarbot_udp.send_msg_left(iplaycommand, ifretnumber)

    if chord ==5:  # Cool endging
        ifretnumber = [8, 5, 2, 2, 5, 8]
        iplaycommand = [3, 3, 3, 3, 3, 3]
        guitarbot_udp.send_msg_left(iplaycommand, ifretnumber)


def fifth_poly(q_i, q_f, time):
    # np.linspace
    # time/0.005
    traj_t = np.arange(0, time, 0.002)
    dq_i = 0
    dq_f = 0
    ddq_i = 0
    ddq_f = 0
    a0 = q_i
    a1 = dq_i
    a2 = 0.5 * ddq_i
    a3 = 1 / (2 * time ** 3) * (20 * (q_f - q_i) - (8 * dq_f + 12 * dq_i) * time - (3 * ddq_f - ddq_i) * time ** 2)
    a4 = 1 / (2 * time ** 4) * (30 * (q_i - q_f) + (14 * dq_f + 16 * dq_i) * time + (3 * ddq_f - 2 * ddq_i) * time ** 2)
    a5 = 1 / (2 * time ** 5) * (12 * (q_f - q_i) - (6 * dq_f + 6 * dq_i) * time - (ddq_f - ddq_i) * time ** 2)
    traj_pos = a0 + a1 * traj_t + a2 * traj_t ** 2 + a3 * traj_t ** 3 + a4 * traj_t ** 4 + a5 * traj_t ** 5
    return traj_pos


def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        a.set_position(*initial_pose, wait=True)


def rightHand(qplay):
    qplay.get()
    arm1.set_mode(1)
    arm1.set_state(0)

    star = 74
    end = 92
    dp = 10

    # should change with diff tempo
    timet = 0.1500 # 0.25  # time to go to string
    # timet2 = 0.15 # 0.06 #time to do pick motion

    initial = initial_pose[2]
    initialy = initial_pose[1]
    lastSign = 1
    sign = -1



    # outIn = fifth_poly(initial_pose[1], initial_pose[1]+go_out_d, timet2/2)
    # inOut = outIn[::-1]
    # Ndsy = []
    # Ndsyy = []
    # Ndsy = np.append(Ndsy, outIn)
    # Ndsy = np.append(Ndsy, outIn)
    # Ndsyy = np.append(Ndsyy, inOut)
    # print(Ndsy)
    strumtdown = 0.15
    strumtup = 0.2
    dsx = fifth_poly(initial_pose[0], final_pose[0], strumtdown)
    dsy = fifth_poly(initial_pose[1], final_pose[1], strumtdown)
    dsz = fifth_poly(initial_pose[2], final_pose[2], strumtdown)
    usx = fifth_poly(final_pose[0], initial_pose[0], strumtup)
    usy = fifth_poly(final_pose[1], initial_pose[1], strumtup)
    usz = fifth_poly(final_pose[2], initial_pose[2], strumtup)

    ##ellipse

    elps_b = 10
    # uelps_b = 5
    strum_range = initial_pose[2] - final_pose[2]
    delps = (-elps_b / (strum_range / 2)) * np.sqrt(
        ((strum_range / 2) ** 2 - (usz - final_pose[2] - strum_range / 2) ** 2)) + initial_pose[1]
    # uelps = (uelps_b/(strum_range/2)) * np.sqrt(((strum_range / 2) ** 2 - (usz - final_pose[2] - strum_range / 2) ** 2)) + initial_pose[1]
    Nusy = delps

    pick_elps_b = int(go_out_d)
    Pick_range = 10
    while True:

        # input("startagain")
        pp = 15000
        dd = 100


        ###old code
        # pos = 10
        # guitarbot_udp.send_msg_picker(ipickercommand=1, bstartpicker=1, pgain=pp, dgain=dd, ipickerpos=-pos,
        #                               ipickervel=5, ipickeracc=100)  # strum down
        pos = -10 # pick position
        guitarbot_udp.send_msg_picker(ipickercommand=1, bstartpicker=1, pgain=pp, dgain=dd, ipickerpos=pos,
                                      ipickervel=5, ipickeracc=100)  # strum down


        i = 1
        # print("started")

        for repeat in range(100):
            [string, mode] = qplay.get()
            string = int(string)
            mode = int(mode)
            #### If picking do this : ###
            if mode == 0: # we are picking

                ###old code
                # guitarbot_udp.send_msg_picker(ipickercommand=4, bstartpicker=1, pgain=pp, dgain=dd,
                #                               ipickerpos=int(-sign * pos),
                #                               ipickervel=20, ipickeracc=100, isliderpos=-17)

                guitarbot_udp.send_msg_picker(ipickercommand=4, bstartpicker=1, pgain=pp, dgain=dd,
                                              ipickerpos=pos,
                                              ipickervel=20, ipickeracc=100, isliderpos=-17)
                final = strings[string]

                ###### TRAJECTORIES FOR STRIGHT UP AND DOWN #####
                dz = fifth_poly(initial, final, timet)
                dx = np.zeros((1, (len(dz)))) + initial_pose[0]
                dy = np.zeros((1, (len(dz)))) + initial_pose[1]

                # guitarbot_udp.send_msg_picker(ipickercommand=1, bstartpicker=1, pgain=pp, dgain=dd, ipickerpos=int(sign*pos),
                #                               ipickervel=5, ipickeracc=100)

                # print("receive")
                # time.sleep(5)
                strumBot(dx[0], dy[0], dz)
                # strumBot(xio[0], Ndsy, staticz[0])
                ###old code
                # guitarbot_udp.send_msg_picker(ipickercommand=3, bstartpicker=1, pgain=pp, dgain=dd, ipickerpos=int(-sign*pos),
                #                               ipickervel=20, ipickeracc=500)

                # print(-sign*pos)
                # sign = -1 * sign
                pos = -pos
                guitarbot_udp.send_msg_picker(ipickercommand=3, bstartpicker=1, pgain=pp, dgain=dd, ipickerpos=pos,
                                              ipickervel=20, ipickeracc=500)
                print(pos)


            ## String number:
            ## Low E - 1st string
            ## A - 2nd string
            ## D - 3rd string
            ## ...



            ### STRUMMING ####
            # if mode == 1: # We are strumming
                #pos = -pos
                guitarbot_udp.send_msg_picker(ipickercommand=1, bstartpicker=1, pgain=4000, dgain=50, ipickerpos=-20,
                                              ipickervel=20, ipickeracc=200, isliderpos=-7)
                time.sleep(0.06)

                ### old code
                # guitarbot_udp.send_msg_picker(ipickercommand=4, bstartpicker=1, pgain=pp, dgain=dd,
                #                               ipickerpos=int(-pos),
                #                               ipickervel=20, ipickeracc=500, isliderpos=-15)
                guitarbot_udp.send_msg_picker(ipickercommand=4, bstartpicker=1, pgain=pp, dgain=dd, ipickerpos=pos,
                                              ipickervel=20, ipickeracc=500, isliderpos=-15)
                final = initial_pose[2]
                dz = fifth_poly(initial, final, timet)
                dx = np.zeros((1, (len(dz)))) + initial_pose[0]
                dy = np.zeros((1, (len(dz)))) + initial_pose[1]

                strumBot(dx[0], dy[0], dz)

                guitarbot_udp.send_msg_picker(ipickercommand=4, bstartpicker=1, pgain=4000, dgain=50, ipickerpos=-20,
                                              ipickervel=20, ipickeracc=500, isliderpos=-7)

                # strum down
                strumBot(dsx, dsy, dsz) # DOWN strum NOISE

                # pos = -pos
                guitarbot_udp.send_msg_picker(ipickercommand=4, bstartpicker=1, pgain=pp, dgain=dd,
                                              ipickerpos=pos,
                                              ipickervel=20, ipickeracc=500, isliderpos=-15)
                strumBot(usx, Nusy, usz) #Up strum SILENT
                sign = -1

            i+=1

            ##### If strumming do this
            initial = float(final)
            # initialy = float(finaly)
            # sign = -1*sign

            fingerq.put(1)

            # print("up")
            # guitarbot_udp.send_msg_picker(ipickercommand=1, bstartpicker=1, pgain=pp, dgain=10, ipickerpos=30, ipickervel=5, ipickeracc=100)




# in x,y,z order
initial_pose = [684.3, 246.8, 367.7, -90, 0, -0] #287.6-5
# initial_pose = [684.3, 246.8, 337.7, -90, 0, -0]
# [684.3, 287.6, 393.7, -90, 0, -0]
final_pose = [684.3, 246.8, 279.2, -90, 0, -0] # 78.5 delta

go_out_d = 5

if __name__ == "__main__":
    ROBOT = "GuitarBot"
    PORT = 5004
    UDP_IP = "192.168.1.50"
    UDP_PORT = 1001

    guitarbot_udp = GuitarBotUDP(UDP_IP, UDP_PORT)
    sleeptime = 4
    # timet = 0.25  # second
    # timet2 = 0.125

    chords(0)
    input("STOP DUMBAS")
    mq = Queue()
    fingerq = Queue()
    midiq = Queue()
    global arm1
    arm1 = XArmAPI('192.168.1.215')

    arms = [arm1]
    totalArms = len(arms)

    setup()
    print("setup")
    input("get ready to start")




    # print(Ndsy)

    global strings
    global stringsy
    ofs = 16.5
    # OLD VALUES strings = [342.7, 333.5, 322.5, 311.9, 302.4, 292.3]# deltas are 9.2, 20.2, 30.8, 40.3,50.4
    strings = [371.6-ofs, 362.4-ofs, 351.4-ofs, 340.8-ofs, 331.3-ofs, 321.4-ofs, initial_pose[2]] # offset because we calibrated maxon at 0
    # stringsy =[279.8, 279.8, 279.8, 280.1, 280.6, 282.0]
    xArm = Thread(target=rightHand, args=(strumq,))
    parsing = Thread(target=parser, args=(midiq,))

    xArm.start()
    parsing.start()
    strumq.put(1)
    initial = initial_pose[2]
    # chords(1)

    fret = [3, 2, 5, 5, 2, 5, 2, 5, 2, 5, 5, 2, 5]
    input("start arpeggiator")
    rtp_midi = RtpMidi(ROBOT, MyHandler(), PORT)
    print("test")
    rtp_midi.run()
    input("wait)")
    time.sleep(3)
    for x in range(6):
        for y in range(5):
            string = x
            # else:
            strumq.put(string)
            time.sleep(0.1)
        # input("next")
            # initial = strings[string]
    input("sequence")

    #


        # input("pause")
        # fingerq.get()
    time.sleep(5)
    guitarbot_udp.send_msg_picker(ipickercommand=1, bstartpicker=1, pgain=5000, dgain=50, ipickerpos=0,
                                  ipickervel=5, ipickeracc=100)

    chords(0)


