from xarm.wrapper import XArmAPI
import numpy as np
import time


class XArmController:
    def __init__(self):
        self.ROBOT = "xArms"
        self.PORT = 5004

        # Connect to the arms
        self.arm0 = XArmAPI('192.168.1.208')
        self.arm1 = XArmAPI('192.168.1.226')
        self.arm2 = XArmAPI('192.168.1.244')
        self.arm3 = XArmAPI('192.168.1.203')
        self.arm4 = XArmAPI('192.168.1.237')
        self.drumarm1 = XArmAPI('192.168.1.236')
        self.drumarm2 = XArmAPI('192.168.1.204')
        self.arms = [self.arm0, self.arm1, self.arm2, self.arm3, self.arm4, self.drumarm1, self.drumarm2]

        self.strumD = 30
        self.speed = 0.25
        self.notes = np.array([64, 60, 69, 55, 62])

        # Initial position of the arms
        self.IP0 = [-1, 87.1, -2, 126.5, -self.strumD / 2, 51.7, -45]
        self.IP1 = [2.1, 86.3, 0, 127.1, -self.strumD / 2, 50.1, -45]
        self.IP2 = [1.5, 81.6, 0.0, 120, -self.strumD / 2, 54.2, -45]
        self.IP3 = [2.5, 81, 0, 117.7, -self.strumD / 2, 50.5, -45]
        self.IP4 = [-1.6, 81.8, 0, 120, -self.strumD / 2, 50.65, -45]  # [-3.9, 65, 3.5, 100.3, -strumD/2, 42.7, 101.1]
        self.DRUM1 = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0]  # DRUMMMING
        self.DRUM2 = [0.0, 23.1, 0.0, 51.4, 0.0, -60.8, 0.0]  # DRUMMMING
        self.IP = [self.IP0, self.IP1, self.IP2, self.IP3, self.IP4, self.DRUM1, self.DRUM2]

        self.delayarray = np.array([[0.15, 0.15, 0.15, 0.15, 0.15, 0.0, 0.0], [0.1, 0.15, 0.1, 0.15, 0.125, 0.0, 0.0]])

    def setup(self):
        '''
        Enable the arms and set them to the initial position.
        '''
        for a in range(len(self.arms)):
            self.arms[a].set_simulation_robot(on_off=False)
            self.arms[a].motion_enable(enable=True)
            self.arms[a].clean_warn()
            self.arms[a].clean_error()
            self.arms[a].set_mode(0)
            self.arms[a].set_state(0)
            self.currentInitiaPos = self.IP[a]
            self.arms[a].set_servo_angle(angle=self.currentInitiaPos, wait=False, speed=10, acceleration=0.25, is_radian=False)

    def setup_pos(self):
        '''
        Setups the arms to the initial position.
        '''
        for a in range(len(self.arms)):
            self.currentInitiaPos = self.IP[a]
            self.arms[a].set_servo_angle(angle=self.currentInitiaPos, wait=False, speed=10, acceleration=0.25, is_radian=False)

    def fifth_poly(self, q_i, q_f, t):
        '''
        Fifth order polynomial trajectory.
        :param q_i: Initial position
        :param q_f: Final position
        :param t: Time of trajectory
        :return: Incremental trajectory angles
        '''
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


    def move(self, queue, arm):
        '''
        Moves the arm to the desired position.
        :param queue: Queue with the desired positions
        :param arm: Arm to move
        '''
        while True:
            if not queue.empty():
                pos = queue.get()
                arm.set_servo_angle(angle=pos, wait=False, speed=10, acceleration=0.25, is_radian=False)