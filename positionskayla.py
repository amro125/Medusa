import numpy as np
import math

########### sine and circle values ##############

def sintrajmaker(IP):
    amp = 40
    speed = 4
    strikes = speed / 4
    tup = np.arange(0, 2 * speed, 0.004)
    # tstrike = np.arange(0, 2 * strikes, 0.004)
    wave = []
    strike = []
    pos = IP
    j_angles = pos.copy()
    traj = []
    for t in tup:
        wave.append(math.cos((math.pi / speed) * (t)))
        strike.append(-math.cos((math.pi / strikes) * (t)))
    for i in range(len(wave)):
        pos = IP
        j_angles[1] = pos[1] + 0.5 * amp * wave[i] - 0.5 * amp
        j_angles[0] = pos[0] + 0.5 * amp2 * strike[i] + 0.5 * amp2
        j_angles[3] = pos[3] + 1.556 * (0.5 * amp * wave[i] - 0.5 * amp)
        buffer = j_angles.copy()
        print(j_angles)
        traj.append(buffer)
    # print(traj)
    return traj

def circletrajmaker(IP):
    amp = 40
    speed = 4
    strikes = speed / 2
    tup = np.arange(0, 2 * speed, 0.004)
    # tstrike = np.arange(0, 2 * strikes, 0.004)
    wave = []
    strike = []
    pos = IP
    j_angles = pos.copy()
    traj = []
    for t in tup:
        wave.append(math.cos((math.pi / speed) * (t)))
        strike.append(-math.cos((math.pi / strikes) * (t)))
    for i in range(len(wave)):
        pos = IP
        j_angles[1] = pos[1] + 0.5 * amp * wave[i] - 0.5 * amp
        j_angles[0] = pos[0] + 0.5 * amp2 * strike[i] + 0.5 * amp2
        j_angles[3] = pos[3] + 1.556 * (0.5 * amp * wave[i] - 0.5 * amp)
        buffer = j_angles.copy()
        print(j_angles)
        traj.append(buffer)
    # print(traj)
    return traj


def utrajmaker(IP):
    amp = uamp

    speed = 0.75
    strikes = speed * 2
    tup = np.arange(0, 2 * speed, 0.004)
    # tstrike = np.arange(0, 2 * strikes, 0.004)
    wave = []
    strike = []
    pos = IP
    j_angles = pos.copy()
    traj = []
    for t in tup:
        wave.append(-math.cos((math.pi / speed) * (t)))
        strike.append(-math.cos((math.pi / strikes) * (t)))
    for i in range(len(wave)):
        pos = IP
        j_angles[1] = pos[1] + 0.5 * amp * wave[i] + 0.5 * amp
        j_angles[0] = pos[0] + 0.5 * baseamp * strike[i] + 0.5 * baseamp
        buffer = j_angles.copy()
        print(j_angles)
        traj.append(buffer)
    # print(traj)
    return traj

def tensetrajmaker(IP):
    pos = IP
    j_angles = pos.copy()
    traj = []
    for i in range(len(pos)):
        j_angles[0] = pos[0] - 22



amp2 = 5
IP0s = [-1-amp2/2, 87.1, -2, 126.5, 0, 51.7, -45]
IP1s = [2.1-amp2/2, 86.3, 0, 127.1, 0, 50.1, -45]
IP2s = [1.5-amp2/2, 81.6, 0.0, 120, 0, 54.2, -45]
IP3s = [-0.2-amp2/2, 83.95, 0, 120, 0, 50.75, -45]
IP4s = pos = [-1.6-amp2/2, 81.85, 0, 120, 0, 50.65, -45]
global IPs
IPs = [IP0s, IP1s, IP2s, IP3s, IP4s]


IP0c = [-1-amp2/2, 87.1, -2, 126.5, 0, 51.7, -45]
IP1c = [2.1-amp2/2, 86.3, 0, 127.1, 0, 50.1, -45]
IP2c = [1.5-amp2/2, 81.6, 0.0, 120, 0, 54.2, -45]
IP3c = [-0.2-amp2/2, 83.95, 0, 120, 0, 50.75, -45]
IP4c = pos = [-1.6-amp2/2, 81.87, 0, 120, 0, 50.65, -45]
global IPc
IPc = [IP0c, IP1c, IP2c, IP3c, IP4c]

global baseamp
global uamp
baseamp = 30
uamp = 10
IP0u = [-1-baseamp/2, 87.1 - uamp, -2, 126.5, 0, 51.7, -45]
IP1u = [2.1-baseamp/2, 86.3 - uamp, 0, 127.1, 0, 50.1, -45]
IP2u = [1.5-baseamp/2, 81.6 - uamp, 0.0, 120, 0, 54.2, -45]
IP3u = [-0.2-baseamp/2, 83.95 - uamp, 0, 120, 0, 50.75, -45] #[-0.2, 83.8, 0, 120, -strumD/2, 50.75, -45]
IP4u = [-1.6-baseamp/2, 81.95 - uamp, 0, 120, 0, 50.65, -45]
global IPu
IPu = [IP0u, IP1u, IP2u, IP3u, IP4u]
sintraj = []
for robots in IPs:
    # trajmaker(robots)
    sintraj.append(sintrajmaker(robots))

circletraj = []
for robots in IPc:
    # trajmaker(robots)
    circletraj.append(circletrajmaker(robots))

utraj = []
for robots in IPu:
    # trajmaker(robots)
    utraj.append(utrajmaker(robots))




    # input("check traj")


############ U traj ########3