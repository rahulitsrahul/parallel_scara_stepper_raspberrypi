import math
import queue
import threading
import time

import numpy as np
from limit_sw_initialize import *


class robot(object):
    def __init__(self, actuator, kinematics):
        self.actuator = actuator
        self.kinematics = kinematics
        self.is_waiting_for_prev_stpes_completion = False

        """
        when both the links are vertical (90 deg), the x, y positions are [0, 186.6] for L0=50, L1=100, L2= 100
        This may vary for actual robot limit switch positions
        """
        
        print("Initiating ROBOT")
        self.initiate_actuators()
        self.current_pos_xy = [0, 147]
        
        # Set init thetas as same as current pos
        x, y = self.current_pos_xy
        self.kinematics.init_theta_a1, self.kinematics.init_theta_a2 = self.kinematics.get_link_angles(x, y)
        
        # Initialize Queue
        self.steps_to_move_queue = queue.Queue()
        t1 = threading.Thread(
            target=self.actuator.set_target_steps, args=(self.steps_to_move_queue,)
        )
        t1.start()
        
        
        # Set Acceleration/Deceleration Parameters
        self.init_accel_value = 5000
        self.accel_scale_factor = 1.02

    def initiate_actuators(self):
        limit_sw_init = limit_switches()
        limit_sw_init.initialize_actuators()

    def bresenham_line(self, x0, y0, x1, y1):
        print("pts: ", x0, y0, x1, y1)
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx

            if e2 < dx:
                err += dx
                y0 += sy

        return points

    def get_stpr_delay_segments(self, linear_segments, delay_stpr=10):
        stpr_delay_segs = np.ones(len(linear_segments)) * np.inf

        stpr_delay_segs[0] = self.init_accel_value
        stpr_delay_segs[-1] = stpr_delay_segs[0]
        for i in range(1, int(np.floor(len(stpr_delay_segs)) / 2)):
            stpr_delay_segs[i] = max(
                delay_stpr,
                stpr_delay_segs[0] * (math.sqrt(i + 1) - math.sqrt(self.accel_scale_factor * i)),
            )
            stpr_delay_segs[-i - 1] = stpr_delay_segs[i]
        stpr_delay_segs = np.where(
            stpr_delay_segs == np.inf, np.min(stpr_delay_segs), stpr_delay_segs
        )

        return list(stpr_delay_segs * 1e-6)

    def move_robot(self, x, y, delay_stpr=10):

        target_position = [x, y]
        linear_segments = self.bresenham_line(
            self.current_pos_xy[0],
            self.current_pos_xy[1],
            target_position[0],
            target_position[1],
        )
        stpr_delay_segments = self.get_stpr_delay_segments(linear_segments, delay_stpr)
        linear_segments = [["START"]*len(self.actuator.motors)] + linear_segments + [["END"]*len(self.actuator.motors)]
        stpr_delay_segments = [None] + stpr_delay_segments + [None]

        for xy_points, stpr_delay in zip(linear_segments, stpr_delay_segments):
            x, y = xy_points
            # Get the steps required to move to position x, y
            if not x in ["START", "END"]:
                steps_to_move = self.kinematics.get_steps_for_pos(x, y)
            else:
                steps_to_move = [x, y]

            # Move the actuators to reach the steps computed
            self.steps_to_move_queue.put(
                {"steps_to_move": steps_to_move, "stpr_delay": stpr_delay}
            )
            time.sleep(5e-6)
        self.current_pos_xy = target_position

    def move_robot_direct(self, x, y):
        # Get the steps required to move to position x, y
        steps_to_move = self.kinematics.get_steps_for_pos(x, y)

        # Move the actuators to reach the steps computed
        time.sleep(5e-6)
        self.steps_to_move_queue.put(
            {"steps_to_move": steps_to_move, "stpr_delay": 5e-6}
        )
