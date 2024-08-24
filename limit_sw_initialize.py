import RPi.GPIO as GPIO  # Import Raspberry Pi GPIO library

GPIO.setmode(GPIO.BOARD)  # Import Raspberry Pi GPIO library
GPIO.setwarnings(False)  # Ignore warning for now
import time

from actuator_stepper import *


class limit_switches(object):
    def __init__(self):
        print('Initialized limit switches')
        self.lim_sw_pin_left = 35
        self.lim_sw_pin_right = 37
        
        GPIO.setup(self.lim_sw_pin_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.lim_sw_pin_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Initialize motors
        self.steppers = actuator_stepper()
        
        self.actuator_left = self.steppers.Mtr_1
        self.actuator_right = self.steppers.Mtr_0
        
        self.actuator_left.set_dir_cw()
        self.actuator_right.set_dir_cw()
        
        
    def get_values(self):
        while True:
            lim_sw_left_val = GPIO.input(self.lim_sw_pin_left)
            lim_sw_right_val = GPIO.input(self.lim_sw_pin_right)
            
            return {'left': lim_sw_left_val, 'right': lim_sw_right_val}
            
            # print(f"Left: {lim_sw_left_val}, Right: {lim_sw_right_val}")
            # time.sleep(0.01)
            
    def touch_lim_switches(self):
        
        # Set motor direction to rotate towards limit switches
        self.actuator_left.set_dir_cw()
        self.actuator_right.set_dir_cw()
        
        left_flag = False
        right_flag = False
        
        while True:
            lim_sw_vals = self.get_values()
            left_val = lim_sw_vals['left']
            right_val = lim_sw_vals['right']
            
            if left_val ==  0 and left_flag==False:
                self.actuator_left.move_one_step(2000e-6)
            else:
                left_flag = True
            
            if right_val == 0 and right_flag==False:
                self.actuator_right.move_one_step(2000e-6)
            else:
                right_flag = True
            
            if right_flag and left_flag:
                break
            
    
    def goto_home_pos(self):
        # Set motor direction to rotate away limit switches
        self.actuator_left.set_dir_ccw()
        self.actuator_right.set_dir_ccw()
        
        for _ in range(500):
            self.actuator_left.move_one_step(1000e-6)
            
        for _ in range(1000):
            self.actuator_right.move_one_step(1000e-6)
            
        for _ in range(1000):
            self.actuator_left.move_one_step(1000e-6)
            
            
        #--------------------
        for _ in range(280):
            self.actuator_right.move_one_step(1000e-6)
        
        for _ in range(300):
            self.actuator_left.move_one_step(1000e-6)
        
    def initialize_actuators(self):
        self.touch_lim_switches()
        self.goto_home_pos()
    
if __name__ == "__main__":
    limit_sw = limit_switches()
    steppers = actuator_stepper()
    
    actuator_left = steppers.Mtr_1
    actuator_right = steppers.Mtr_0
    
    actuator_left.set_dir_cw()
    actuator_right.set_dir_cw()
    
    # for _ in range(100):
    #     actuator_left.move_one_step()