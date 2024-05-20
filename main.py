from scara_kinematics import *
from actuator_stepper import *
from robot import *
import time


if __name__ == "__main__":
    
    print("STARTED")
    # Initiate the elements of the Robot (Actuators, kinematics and robot_control)
    actuator = actuator_stepper()
    print("stepper_initiated")
    scara_kin = scara_kinematics(L0=50, L1=100, L2=100)
    print("scara_kinematics_initiated")
    robo = robot(actuator, scara_kin)
    print("Robot_Initiated")
    
    time.sleep(2)
    """
    At this Point robot would be initiated and positioned using limit switches
    For local setup to initiate this, set the stepper motor pointer manually.
    press y to continue
    """
    print("reprap start")
    for i in range(50):
        robo.move_robot(90, 130)
        time.sleep(0.15)
        robo.move_robot(-90, 130)
        time.sleep(0.15)
#    while True:
#        start = str(input("Press y if links are initiated: "))
#        if start == 'y':
#            break
#        else:
#            print("invalid input please provide proper input")

#    # Move the robot to position (x, y)
#    print("Move robot to 50, 130")
#    robo.move_robot_direct(90, 130)
##    while robo.is_moving():
##        pass
#    
#    robo.move_robot_direct(0, 130)
##    while robo.is_moving():
##        pass
#    time.sleep(2)
#    
#    for i in range(10):
#        print("reprap in motion", f"{i} / {10}")
#        # Move the robot horizontal between (-50, 130) and (+50, 130) with the resolution of 2 units
#        x_val = list(range(-90, 90, 1))
#        y_val = [130]*len(x_val)
#
#        for x,y in zip(x_val, y_val):
##             while robo.is_moving():
##                 pass
##             print(f"move robot x:{x} , y:{y}")
#             robo.move_robot_direct(x, y)
#
#        x_val = list(range(90, -90, -1))
#        y_val = [130]*len(x_val)
#
#        for x,y in zip(x_val, y_val):
#            
##             while robo.is_moving():
##                 pass
##             print(f"move robot x:{x} , y:{y}")
#             robo.move_robot_direct(x, y)
#            
##        time.sleep(2)
#        print("done")
#        
#    # Run mtoors with user input of x direction
#    while True:
#        robo.move_robot_direct(0, 130)
#        runs = int(input("Enter num repetations: "))
#        for i in range(runs):
#            print("reprap in motion", f"{i} / {runs}")
#            # Move the robot horizontal between (-50, 130) and (+50, 130) with the resolution of 2 units
#            x_val = list(range(-90, 90, 1))
#            y_val = [130]*len(x_val)
#    
#            for x,y in zip(x_val, y_val):
#    #             while robo.is_moving():
#    #                 pass
#    #             print(f"move robot x:{x} , y:{y}")
#                 robo.move_robot_direct(x, y)
#    
#            x_val = list(range(90, -90, -1))
#            y_val = [130]*len(x_val)
#    
#            for x,y in zip(x_val, y_val):
#                
#    #             while robo.is_moving():
#    #                 pass
#    #             print(f"move robot x:{x} , y:{y}")
#                 robo.move_robot_direct(x, y)
#                
#    #        time.sleep(2)
#            print("done")
        
        

"""
robo.move_robot(-90, 130)
robo.move_robot(90, 130)
robo.move_robot(-90, 130)
robo.move_robot(90, 130)
robo.move_robot(0, 130)

"""
    
