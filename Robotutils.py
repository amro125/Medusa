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
