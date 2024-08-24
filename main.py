import time
from actuator_stepper import *
from robot import *
from scara_kinematics import *

if __name__ == "__main__":

    print("STARTED")
    # Initiate the elements of the Robot (Actuators, kinematics and robot_control)
    actuator = actuator_stepper()
    print("stepper_initiated")
    scara_kin = scara_kinematics(L0=101/2, L1=60, L2=100)
    print("scara_kinematics_initiated")
    robo = robot(actuator, scara_kin)
    print("Robot_Initiated")

    time.sleep(2)
    """
    At this Point robot would be initiated and positioned using limit switches
    For local setup to initiate this, set the stepper motor pointer manually.
    press y to continue
    """
    # while True:
        # start = str(input("Press y if links are initiated: "))
        # if start == "y":
        #     break
        # else:
        #     print("invalid input please provide proper input")

    print("reprap start")
    
    robo.init_accel_value = 10000
    robo.accel_scale_factor = 1.005
    num_reps = 1
    # for i in range(num_reps):
    #     print(f"----------STEP {i+1}/{num_reps}--------------")
    #     robo.move_robot(0, 100, delay_stpr=50)
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
    #     time.sleep(0.2)
        
    #     robo.move_robot(30, 100, delay_stpr=50)
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
    #     time.sleep(0.2)
        
    #     robo.move_robot(30, 130, delay_stpr=50)
        
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
    #     time.sleep(0.2)
        
    #     robo.move_robot(-30, 130, delay_stpr=50)
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
    #     time.sleep(0.2)
        
    #     robo.move_robot(-30, 100, delay_stpr=50)
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
    #     time.sleep(0.2)
        
    #     robo.move_robot(0, 100, delay_stpr=50)
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
time.sleep(0.2)


"""

x = 30
y = 100
for x in range(30, -32, -2):
    robo.move_robot(x, y, delay_stpr=50)
    while robo.actuator.move_steppers_flag:
        time.sleep(0.02)
    for y in [100, 130]:
        robo.move_robot(x, y, delay_stpr=50)
        while robo.actuator.move_steppers_flag:
            time.sleep(0.02)
            
x = -30
y = 130

for y in range(130, 98, -2):
    robo.move_robot(x, y, delay_stpr=50)
    while robo.actuator.move_steppers_flag:
        time.sleep(0.02)
    
    for x in [-30, 30]:
        robo.move_robot(x, y, delay_stpr=50)
        while robo.actuator.move_steppers_flag:
            time.sleep(0.02)


"""
