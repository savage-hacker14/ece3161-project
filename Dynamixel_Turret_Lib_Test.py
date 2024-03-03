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

# Define body and shoulder joint ranges [CW limit, CCW limit], [rad]
BODY_RANGE      = [1.687379, 2.791845]          # 1100, 1820 encoder counts
SHOULDER_RANGE  = [1.208777, 5.043729]          # 788, 3288 encoder counts

# Define a function to generate a random VALID position for the joint (within angle limits)
def rand_pos(motor_id):
    if (motor_id == DXL_BODY_ID):
        rand_rad = random.random() * (BODY_RANGE[1] - BODY_RANGE[0]) + BODY_RANGE[0]
    elif (motor_id == DXL_SHOULDER_ID):
        rand_rad =  random.random() * (SHOULDER_RANGE[1] - SHOULDER_RANGE[0]) + SHOULDER_RANGE[0]

    # Convert radian position to encoder position
    return rand_rad * (ENCODER_RES / (2 * math.pi)) 


# Main code
if __name__ == "__main__":
    # Call init function
    init_robot()

    # Main loop
    print("Press ESC to quit!")
    while (True):
        # Check for ESC key press
        if getch() == chr(0x1b):
            break

        # Generate random positions
        phi     = rand_pos(DXL_BODY_ID)
        theta   = rand_pos(DXL_SHOULDER_ID)

        # Send goal positions to both body and shoulder joints
        set_pose(phi, theta)

        # Wait for 5 seconds
        time.sleep(5)