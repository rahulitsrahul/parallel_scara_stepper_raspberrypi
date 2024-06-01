import time

from actuator_stepper import *
from robot import *
from scara_kinematics import *

if __name__ == "__main__":

    print("STARTED")
    # Initiate the elements of the Robot (Actuators, kinematics and robot_control)
    actuator = actuator_stepper()
    print("stepper_initiated")
    scara_kin = scara_kinematics(L0=29, L1=120, L2=120)
    print("scara_kinematics_initiated")
    robo = robot(actuator, scara_kin)
    print("Robot_Initiated")

    time.sleep(2)
    """
    At this Point robot would be initiated and positioned using limit switches
    For local setup to initiate this, set the stepper motor pointer manually.
    press y to continue
    """
    while True:
        start = str(input("Press y if links are initiated: "))
        if start == "y":
            break
        else:
            print("invalid input please provide proper input")

    print("reprap start")
    for i in range(20):
        robo.move_robot(x=90, y=130, delay_stpr=100)
        while robo.actuator.move_steppers_flag:
            time.sleep(0.02)

        robo.move_robot(90, 180, delay_stpr=100)
        while robo.actuator.move_steppers_flag:
            time.sleep(0.02)

        robo.move_robot(x=-90, y=180, delay_stpr=100)
        while robo.actuator.move_steppers_flag:
            time.sleep(0.02)

        robo.move_robot(-90, 130, delay_stpr=100)
        while robo.actuator.move_steppers_flag:
            time.sleep(0.02)


"""
robo.move_robot(-90, 130)
robo.move_robot(90, 130)
robo.move_robot(-90, 130)
robo.move_robot(90, 130)
robo.move_robot(0, 130)
"""
