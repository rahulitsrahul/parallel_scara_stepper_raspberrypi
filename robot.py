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
        self.current_pos_xy = [1, 146]
        
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

    def bresenham_line(self, x1, y1, x2, y2):
        """
        Bresenham's line algorithm to generate points between (x1, y1) and (x2, y2).
        :param x1: Starting x coordinate (in units of 0.1)
        :param y1: Starting y coordinate (in units of 0.1)
        :param x2: Ending x coordinate (in units of 0.1)
        :param y2: Ending y coordinate (in units of 0.1)
        :return: List of points (tuples) representing the line, in units of 0.1
        """
        # Scale coordinates to work with integer units
        scale = 25
        x1, y1 = int(x1 * scale), int(y1 * scale)
        x2, y2 = int(x2 * scale), int(y2 * scale)
    
        points = []
    
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
    
        while True:
            points.append((x1 / scale, y1 / scale))
            if x1 == x2 and y1 == y2:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
    
        return points
    
    
    def circular_interpolation(self, start, end, center, radius, direction='cw'):
        resolution = 0.1
        """
        Generates points along a circular arc with a specified resolution.

        Parameters:
        - start: tuple of (x, y) for the start point of the arc.
        - end: tuple of (x, y) for the end point of the arc.
        - center: tuple of (cx, cy) for the center of the circle.
        - radius: radius of the circle.
        - direction: 'clockwise' or 'counterclockwise' for the direction of the arc.
        - resolution: distance between consecutive points along the arc.

        Returns:
        - List of (x, y) tuples representing points along the arc.
        """
        # Convert inputs to numpy arrays for ease of calculations
        start = np.array(start)
        end = np.array(end)
        center = np.array(center)

        # Compute angles
        def angle_from_center(point):
            return np.arctan2(point[1] - center[1], point[0] - center[0])

        start_angle = angle_from_center(start)
        end_angle = angle_from_center(end)

        # Ensure angles are within the range [0, 2*pi)
        start_angle = start_angle % (2 * np.pi)
        end_angle = end_angle % (2 * np.pi)

        # Compute angular difference
        angular_range = end_angle - start_angle

        # Adjust angular_range for correct direction
        if direction == 'cw':
            if angular_range < 0:
                angular_range += 2 * np.pi
        else:
            if angular_range > 0:
                angular_range -= 2 * np.pi

        # Calculate the number of points required
        arc_length = abs(angular_range) * radius
        num_points = int(np.ceil(arc_length / resolution))

        # Generate points
        angles = np.linspace(start_angle, start_angle + angular_range, num_points)
        points = [(center[0] + radius * np.cos(angle), center[1] + radius * np.sin(angle)) for angle in angles]

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

    def move_robot_linear(self, x, y, delay_stpr=10):

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
        
    def move_robot_circular(self, start, end, center, radius, direction='cw', delay_stpr=10):

        target_position = end
        # linear_segments = self.bresenham_line(
        #     self.current_pos_xy[0],
        #     self.current_pos_xy[1],
        #     target_position[0],
        #     target_position[1],
        # )
        
        linear_segments = self.circular_interpolation(start, end, center, radius, direction)
        
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
