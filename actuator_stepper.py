import RPi.GPIO as GPIO  # Import Raspberry Pi GPIO library

GPIO.setmode(GPIO.BOARD)  # Import Raspberry Pi GPIO library
GPIO.setwarnings(False)  # Ignore warning for now
import sys
import time

import numpy as np

sys.setrecursionlimit(10**9)
import queue
import threading
from collections import OrderedDict


class Motor(object):
    master_pulse = 0
    is_running = False

    def __init__(self, mtr_id, step_pin, dir_pin, ena_pin, steps_per_rev=200 * 8  * 1): # steps per rev * scalar * gear ratio
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.ena_pin = ena_pin

        self.initialize_motor()

        self.motor_id = mtr_id
        self.current_pos = 0
        self.target_pos = 0
        self.steps_to_move = 0
        self.R = 1
        self.R_temp = 1
        self.is_running = False
        self.steps_per_rev = steps_per_rev
        self.prev_step_flag = False
        self.counter = 0
        print("Initiated MOTOR")

    def initialize_motor(self):
        """Define the stepper motor step pins and direction pins"""
        GPIO.setup(self.ena_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.step_pin, GPIO.OUT, initial=GPIO.LOW)

        """ Initialize the stepper pins to filp High/Low """
        GPIO.output(self.ena_pin, GPIO.LOW)
        GPIO.output(self.dir_pin, GPIO.HIGH)
        GPIO.output(self.step_pin, GPIO.HIGH)

    def step_low(self):
        GPIO.output(self.step_pin, GPIO.LOW)
        if self.prev_step_flag:
            self.current_pos += +1 if bool(GPIO.input(self.dir_pin)) else -1
        self.prev_step_flag = False

    def step_high(self):
        GPIO.output(self.step_pin, GPIO.HIGH)
        self.prev_step_flag = True

    def set_dir(self):
        GPIO.output(self.dir_pin, GPIO.HIGH) if np.sign(
            self.target_pos - self.current_pos
        ) >= 0 else GPIO.output(self.dir_pin, GPIO.LOW)


class actuator_stepper(object):
    def __init__(self):
        step_pin_0 = 24
        ena_pin_0 = 26
        dir_pin_0 = 22

        step_pin_1 = 18
        ena_pin_1 = 16
        dir_pin_1 = 12

        self.Mtr_0 = Motor(
            mtr_id=0, step_pin=step_pin_0, ena_pin=ena_pin_0, dir_pin=dir_pin_0
        )
        self.Mtr_1 = Motor(
            mtr_id=1, step_pin=step_pin_1, ena_pin=ena_pin_1, dir_pin=dir_pin_1
        )

        # Update self.motors list as per the number of stepper motors used
        self.motors = [self.Mtr_0, self.Mtr_1]

        self.move_steppers_flag = False
        self.stepper_time_delay = 100 * 1e-6
        self.stepper_time_delay_default = 100 * 1e-6
        self.move_stpr_flag = False
        self.t1 = time.time()
        print("Initiated actuator STEPPER")

    def set_target_steps(self, target_steps_queue):
        #        print("STARTED thread target_steps")
        while True:
            if (not target_steps_queue.empty()) and (not Motor.is_running):
                self.t1 = time.time()
                target_steps_dict = target_steps_queue.get()
                t2 = time.time()
                target_steps = target_steps_dict["steps_to_move"]
                self.stepper_time_delay = target_steps_dict["stpr_delay"]

                if not target_steps[0] in ["START", "END"]:
                    Motor.is_running = True
                    for motor, target_step in zip(self.motors, target_steps):
                        motor.target_pos = target_step
                        

#                    print("elapsed_:  ", round(((time.time()-t1)/1e-6), 2), " us")
                    self.move_stpr_flag = True
                    self.move_stpr()
                else:
                    self.move_steppers_flag = (True if target_steps[0] == "START" else False)

    def move_stpr(self):
        run_again = False  # set True if this loop needs recursion
        # Set Motor direction
        for motor in self.motors:
            motor.set_dir()

        # set Motor pins Low
        for motor in self.motors:
            if motor.target_pos != motor.current_pos:
                motor.step_high()
        t2 = time.time()
        time.sleep(max(0, (self.stepper_time_delay - (t2 - self.t1))))
        self.t1 = time.time()

        # Set Motor pins High
        for motor in self.motors:
            if (motor.target_pos != motor.current_pos):
                motor.step_low()
        
        t3 = time.time()
        time.sleep(max(0, self.stepper_time_delay - (t3 - self.t1)))

        for motor in self.motors:
            if motor.target_pos != motor.current_pos:
                run_again = run_again or True
        if run_again:
            self.move_stpr()

        Motor.is_running = False


if __name__ == "__main__":
    actuator = actuator_stepper()
    steps_queue = queue.Queue()

    for i in range(1, (200 * 4), 1):
        steps_queue.put([i, i])
        actuator.set_target_steps(steps_queue)

    print("repeat mode start 10 times")
    time.sleep(2)
    for _ in range(10):
        for i in range(0, (200 * 4), 1):
            steps_queue.put([i, i])
            actuator.set_target_steps(steps_queue)

        for i in range((200 * 4), 0, -1):
            steps_queue.put([i, i])
            actuator.set_target_steps(steps_queue)

    print("done")
    """
    t1 = time.time()
    while not a.empty():
        a.get()
    print("elapsed_:  ", round(((time.time()-t1)/1e-6), 2), " us")
    """