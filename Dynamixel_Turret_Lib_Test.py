# Dynamixel_Turret_Lib_Test.py
# ECE 3161 - Term Project
# Written by Jacob Krucinski on 03/02/24

# Import custom library 
from Dynamixel_Turret_Lib import *


# Import random library (for random position definition) and math library
import random
import math
import os
import time


# Define exit handling
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
# Import Dyanmixel library again
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Define body and shoulder joint ranges [CW limit, CCW limit], [rad]
BODY_RANGE      = [(1500 - BODY_OFFSET)/ENCODER_RES * 2 * math.pi,
                   (2600 - BODY_OFFSET)/ENCODER_RES * 2 * math.pi]           # 1500 - 2047 (offset) = -547, 2600 - 2047 = 553 encoder counts
SHOULDER_RANGE  = [(788 - BODY_OFFSET)/ENCODER_RES * 2 * math.pi,
                   (3288 - BODY_OFFSET)/ENCODER_RES * 2 * math.pi]           # 788 - 2047 (offset) = , 3288 encoder counts

# Define a function to generate a random VALID position for the joint (within angle limits)
def rand_pos(motor_id):
    if (motor_id == DXL_BODY_ID):
        rand_rad = random.random() * (BODY_RANGE[1] - BODY_RANGE[0]) + BODY_RANGE[0]
    elif (motor_id == DXL_SHOULDER_ID):
        rand_rad =  random.random() * (SHOULDER_RANGE[1] - SHOULDER_RANGE[0]) + SHOULDER_RANGE[0]

    # Return random position [rad]
    return rand_rad


# Main code
if __name__ == "__main__":
    # Call init function
    print("Initializing robot...")
    init_robot()
    time.sleep(3)

    # Main loop
    while (True):
        # Check for ESC key press
        print("Press any key to generate new pose (or press ESC to quit)")
        if getch() == chr(0x1b):
            break

        # Generate random positions
        phi     = rand_pos(DXL_BODY_ID)
        theta   = rand_pos(DXL_SHOULDER_ID)

        # Send goal positions to both body and shoulder joints
        print(f"Phi: {phi}, Theta: {theta}")
        set_pose(phi, theta)