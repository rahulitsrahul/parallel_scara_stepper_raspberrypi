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
        self.current_pos_xy = [3, 147]
        
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
    
    
    # def circular_interpolation(self, start, end, center, radius, direction='cw'):
    #     resolution = 0.1
    #     """
    #     Generates points along a circular arc with a specified resolution.

    #     Parameters:
    #     - start: tuple of (x, y) for the start point of the arc.
    #     - end: tuple of (x, y) for the end point of the arc.
    #     - center: tuple of (cx, cy) for the center of the circle.
    #     - radius: radius of the circle.
    #     - direction: 'clockwise' or 'counterclockwise' for the direction of the arc.
    #     - resolution: distance between consecutive points along the arc.

    #     Returns:
    #     - List of (x, y) tuples representing points along the arc.
    #     """
    #     # Convert inputs to numpy arrays for ease of calculations
    #     start = np.array(start)
    #     end = np.array(end)
    #     center = np.array(center)

    #     # Compute angles
    #     def angle_from_center(point):
    #         return np.arctan2(point[1] - center[1], point[0] - center[0])

    #     start_angle = angle_from_center(start)
    #     end_angle = angle_from_center(end)

    #     # Ensure angles are within the range [0, 2*pi)
    #     start_angle = start_angle % (2 * np.pi)
    #     end_angle = end_angle % (2 * np.pi)

    #     # Compute angular difference
    #     angular_range = end_angle - start_angle

    #     # Adjust angular_range for correct direction
    #     if direction == 'cw':
    #         if angular_range < 0:
    #             angular_range += 2 * np.pi
    #     else:
    #         if angular_range > 0:
    #             angular_range -= 2 * np.pi

    #     # Calculate the number of points required
    #     arc_length = abs(angular_range) * radius
    #     num_points = int(np.ceil(arc_length / resolution))

    #     # Generate points
    #     angles = np.linspace(start_angle, start_angle + angular_range, num_points)
    #     points = [(center[0] + radius * np.cos(angle), center[1] + radius * np.sin(angle)) for angle in angles]

    #     return points
    
    def circular_interpolation(self, command, x, y, i, j, r):
        # Start point of circle
        start_x, start_y = self.current_pos_xy
        # print(f"params: startx: {start_x}, start_y: {start_y}, cmd:{command}, x:{x}, y:{y}, i:{i}, j:{j}, r:{r}")
        full_circle_flag = False
        resolution=0.1
        cw_list = ['G03', 'G3']
        ccw_list = ['G02', 'G2']
    
        if r is not None:
            # Calculate the distance between start and end points
            dx = x - start_x
            dy = y - start_y
            d = math.sqrt(dx**2 + dy**2)
            
    
            if d > 2 * abs(r):
                raise ValueError("Distance between start and end points is greater than the diameter of the circle")
    
            # Calculate the midpoint
            mx, my = (start_x + x) / 2, (start_y + y) / 2
    
            # Calculate the distance from the midpoint to the center of the circle
            h = math.sqrt(r**2 - (d/2)**2)
    
            if command in ccw_list:  # Clockwise
                xc = mx + h * dy / d
                yc = my - h * dx / d
            elif command in cw_list:  # Counterclockwise
                xc = mx - h * dy / d
                yc = my + h * dx / d
    
            radius = r
        else:
            if i == 0 and j == 0:
                # When I and J are both zero, assume full circle
                radius = math.sqrt((x - start_x)**2 + (y - start_y)**2) / 2
                xc = (start_x + x) / 2
                yc = (start_y + y) / 2
                full_circle_flag = True
            else:
                xc = start_x + i
                yc = start_y + j
                radius = math.sqrt(i**2 + j**2)
                full_circle_flag = False
    
        if radius <= 0:
            raise ValueError("Radius must be positive and non-zero")
    
        # Calculate start and end angles
        start_angle = math.atan2(start_y - yc, start_x - xc)
        end_angle = math.atan2(y - yc, x - xc)
    
        if full_circle_flag:
            # Ensure a full circle is made
            if command in ccw_list:  # Clockwise
                end_angle = start_angle - 2 * math.pi
            elif command in cw_list:  # Counterclockwise
                end_angle = start_angle + 2 * math.pi
        else:
            # Adjust the end angle based on the direction
            if command in ccw_list:  # Clockwise
                if end_angle > start_angle:
                    end_angle -= 2 * math.pi
            elif command in cw_list:  # Counterclockwise
                if end_angle < start_angle:
                    end_angle += 2 * math.pi
    
        # Calculate the angular difference
        angular_difference = end_angle - start_angle
    
        # Ensure the number of points is at least one
        if resolution <= 0:
            raise ValueError("Resolution must be positive and non-zero")
        num_points = max(int(abs(angular_difference) / (resolution / radius)), 20)
    
        # Generate the points along the arc
        points = []
        for i in range(num_points + 1):
            angle = start_angle + i * angular_difference / num_points
            px = xc + radius * math.cos(angle)
            py = yc + radius * math.sin(angle)
            points.append([px, py])
    
        # Ensure the last point is exactly the end point, if not a full circle
        if not full_circle_flag:
            points[-1] = [x, y]
    
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
        
    def move_robot_circular(self, command, x_targ, y_targ, i=None, j=None, r=None, delay_stpr=10):
        
        target_position =[x_targ, y_targ]
        if i==0 and j==0: # It makes full circle, target position = start position
            target_position = self.current_pos_xy
        
        
        linear_segments = self.circular_interpolation(command, x_targ, y_targ, i, j, r)
        
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
