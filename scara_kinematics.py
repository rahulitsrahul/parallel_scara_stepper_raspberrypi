import math as m
import time


class scara_kinematics(object):
    def __init__(self, L0=29, L1=120, L2=120):
        # L0, L1 and L2 are the link lengths
        self.L0 = L0
        self.L1 = L1
        self.L2 = L2

        # Initialize starting value of thetas where steps of motors are initialized as zero
        self.init_theta_a1 = 0
        self.init_theta_a2 = 0

        self.steps_per_rev = 200 * 8  * 1 # steps per rev * scalar * gear ratio
        self.steps_per_deg = self.steps_per_rev / 360

    def get_link_angles(self, x, y):
        #        t1 = time.time()

        L0 = self.L0
        L1 = self.L1
        L2 = self.L2

        x_plus_L0 = x + L0
        x_minus_L0 = x - L0
        y_sq = y**2
        K_sq = x_plus_L0**2 + y_sq
        S_sq = x_minus_L0**2 + y_sq
        K = m.sqrt(K_sq)
        S = m.sqrt(S_sq)

        gamma_1_num = L1**2 - L2**2 + K_sq
        gamma_1_den = 2 * L1 * K
        gamma_1 = m.acos(gamma_1_num / gamma_1_den)

        gamma_2_num = L1**2 - L2**2 + S_sq
        gamma_2_den = 2 * L1 * S
        gamma_2 = m.acos(gamma_2_num / gamma_2_den)

        # Region_1
        if -self.L0 <= x <= self.L0:
            if x == -self.L0:
                beta_1 = m.pi / 2
                beta_2 = m.pi - m.atan2(y, abs(self.L0 - x))
            elif x == self.L0:
                beta_1 = m.atan2(y, abs(self.L0 + x))
                beta_2 = m.pi / 2
            else:
                beta_1 = m.atan2(y, abs(self.L0 + x))
                beta_2 = m.pi - m.atan2(y, abs(self.L0 - x))

        # Region_2
        elif x > self.L0:
            beta_1 = m.atan2(y, abs(self.L0 + x))
            beta_2 = m.atan2(y, abs(x - self.L0))

        # Region_3
        elif x < -self.L0:
            beta_1 = m.pi - m.atan2(y, abs(self.L0 + x))
            beta_2 = m.pi - m.atan2(y, abs(self.L0 - x))

        theta_a1 = m.degrees(beta_1 + gamma_1)
        theta_a2 = m.degrees(beta_2 - gamma_2)

        #        print("elapsed_kinematics:  ", round(((time.time()-t1)/1e-6), 2), " us")

        return [theta_a1, theta_a2]  # degrees

    def get_steps_for_pos(self, x, y):

        theta_a1, theta_a2 = self.get_link_angles(x, y)

        theta_a1_diff = self.init_theta_a1 - theta_a1
        theta_a2_diff = self.init_theta_a2 - theta_a2

        steps_a1 = self.steps_per_deg * theta_a1_diff
        steps_a2 = self.steps_per_deg * theta_a2_diff

        # print('init_theta_a1: ', self.init_theta_a1)
        # print('theta_a1: ', theta_a1)
        # print("theta_a1_diff: ", theta_a1_diff)
        # print("steps_a1: ", steps_a1)
        # print("steps_a1: ", steps_a1)

        return [round(steps_a1), round(steps_a2)]
