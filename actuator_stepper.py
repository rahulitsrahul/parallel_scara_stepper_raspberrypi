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

    def __init__(self, mtr_id, step_pin, dir_pin, ena_pin, steps_per_rev=200 * 8):
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
        print("Initiated actuator STEPPER")

    def set_target_steps(self, target_steps_queue):
        #        print("STARTED thread target_steps")
        while True:
            if (not target_steps_queue.empty()) and (not Motor.is_running):
                target_steps_dict = target_steps_queue.get()
                target_steps = target_steps_dict["steps_to_move"]
                self.stepper_time_delay = target_steps_dict["stpr_delay"]

                if not target_steps[0] in ["START", "END"]:
                    # t1 = time.time()
                    Motor.is_running = True
                    steps_to_move = []
                    for motor, target_step in zip(self.motors, target_steps):
                        steps_to_move.append(target_step - motor.current_pos)
                    try:
                        #                    print("Steps_to_Move: ", steps_to_move)
                        steps_to_move_temp = np.array(steps_to_move)
                        steps_to_move_temp = np.where(
                            steps_to_move_temp == 0, 1, steps_to_move_temp
                        )
                        R_vals = max(np.abs(steps_to_move_temp)) / np.abs(
                            steps_to_move_temp
                        )
                    except:
                        print("DEBUG")
                        print(steps_to_move)
                        R_vals = [1, 1]

                    Motor.master_pulse = 0
                    for motor, target_step, R in zip(self.motors, target_steps, R_vals):
                        motor.target_pos = target_step
                        motor.R = R
                        motor.R_temp = R

                    # print("elapsed_:  ", round(((time.time()-t1)/1e-6), 2), " us")
                    self.move_stpr(self.motors)
                else:
                    self.move_steppers_flag = (
                        True if target_steps[0] == "START" else False
                    )

    def move_stpr(self, motors):
        run_again = False  # set True if this loop needs recursion
        Motor.master_pulse += 1
        # Set Motor direction
        for motor in motors:
            motor.set_dir()

        # set Motor pins Low
        for motor in motors:
            if motor.target_pos != motor.current_pos:
                motor.step_high()
        time.sleep(self.stepper_time_delay)

        # Set Motor pins High
        for motor in motors:
            if (motor.target_pos != motor.current_pos) & (
                Motor.master_pulse >= motor.R_temp - 1
            ):
                motor.step_low()
                motor.R_temp += motor.R

        #            motor.step_high() if (motor.target_pos != motor.current_pos) & (Motor.master_pulse >= motor.R_temp) else None
        """
        for motor in motors:
            print(motor.R)
        """

        time.sleep(self.stepper_time_delay)

        for motor in motors:
            if motor.target_pos != motor.current_pos:
                run_again = run_again or True
        if run_again:
            self.move_stpr(self.motors)

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
