import time
from actuator_stepper import *
from robot import *
from scara_kinematics import *

prev_gcode_params = {'cmd': None, 'x': None, 'y': None , 'i': None, 'j': None, 'r': None, 'f': None}

def parse_gcode(gcode, prev_gcode_params={'cmd': None, 'x': None, 'y': None , 'i': None, 'j': None, 'r': None, 'f': None}):
    # Initialize the parameters
    print(f"Executing: {gcode}")
    command = None
    r = None
    x = prev_gcode_params['x']
    y = prev_gcode_params['y']
    i = prev_gcode_params['i']
    j = prev_gcode_params['j']
    # r = prev_gcode_params['r']
    f = prev_gcode_params['f']

    # Split the gcode string and iterate through the parts
    parts = gcode.split()
    for part in parts:
        if part.startswith('G'):
            command = part
        elif part.startswith('X'):
            x = float(part[1:])
        elif part.startswith('Y'):
            y = float(part[1:])
        elif part.startswith('I'):
            i = float(part[1:])
        elif part.startswith('J'):
            j = float(part[1:])
        elif part.startswith('R'):
            r = float(part[1:])

    return (command, x, y, i, j, r, f)

def execute_motion(robo, command, x, y, i, j, r, f):
    while robo.actuator.move_steppers_flag:
        time.sleep(0.02)
    if command=='G01':
        robo.move_robot_linear(x, y, delay_stpr=50)
    
    elif command=='G02' or command=='G03':
        robo.move_robot_circular(command=command, x_targ=x, y_targ=y, i=i, j=j, r=r, delay_stpr=500)
    
        
def process_gcode(gcode, robo):
    global prev_gcode_params
    command, x, y, i, j, r, f = parse_gcode(gcode=gcode, prev_gcode_params=prev_gcode_params)
    execute_motion(robo, command, x, y, i, j, r, f)
    prev_gcode_params = {'cmd': command, 'x': x, 'y': y , 'i': i, 'j': j, 'r': r, 'f': f}


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
    #     start = str(input("Press y if links are initiated: "))
    #     if start == "y":
    #         break
    #     else:
    #         print("invalid input please provide proper input")
    
    
    
    gcodes = [
            "G01 X0 Y100",
            "G01 X30 Y100",
            "G02 X30 Y130 I0 J15",
            # "G01 X30 Y130",
            "G01 X-30 Y130",
            "G01 X-30 Y100",
            "G01 X0 Y100"
            ]
    
    print("reprap start")
    
    robo.init_accel_value = 10000
    robo.accel_scale_factor = 1.005
    
    for gcode in gcodes:
        process_gcode(gcode, robo)

    
    
    # #--------------------------------------------------------#
    # num_reps = 1
    # for i in range(num_reps):
    #     print(f"----------STEP {i+1}/{num_reps}--------------")
    #     robo.move_robot_linear(0, 100, delay_stpr=50)
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
    #     time.sleep(0.2)
        
    #     robo.move_robot_linear(30, 100, delay_stpr=50)
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
    #     time.sleep(0.2)
        
    #     robo.move_robot_linear(30, 130, delay_stpr=50)
        
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
    #     time.sleep(0.2)
        
    #     robo.move_robot_linear(-30, 130, delay_stpr=50)
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
    #     time.sleep(0.2)
        
    #     robo.move_robot_linear(-30, 100, delay_stpr=50)
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
    #     time.sleep(0.2)
        
    #     robo.move_robot_linear(0, 100, delay_stpr=50)
    #     while robo.actuator.move_steppers_flag:
    #         time.sleep(0.02)
    #     time.sleep(0.2)



"""
robo.move_robot_linear(0, 100, delay_stpr=50)
while robo.actuator.move_steppers_flag:
    time.sleep(0.02)

robo.move_robot_circular(start=[0, 100], end=[0, 130], center=[0, 115], radius=15, direction='cw', delay_stpr=1000)
"""


"""

x = 30
y = 100
for x in range(30, -32, -2):
    robo.move_robot_linear(x, y, delay_stpr=50)
    while robo.actuator.move_steppers_flag:
        time.sleep(0.02)
    for y in [100, 130]:
        robo.move_robot_linear(x, y, delay_stpr=50)
        while robo.actuator.move_steppers_flag:
            time.sleep(0.02)
            
x = -30
y = 130

for y in range(130, 98, -2):
    robo.move_robot_linear(x, y, delay_stpr=50)
    while robo.actuator.move_steppers_flag:
        time.sleep(0.02)
    
    for x in [-30, 30]:
        robo.move_robot_linear(x, y, delay_stpr=50)
        while robo.actuator.move_steppers_flag:
            time.sleep(0.02)


"""
