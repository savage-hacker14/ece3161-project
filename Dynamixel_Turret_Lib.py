# Dynamixel_Turret_Lib.py
# ECE 3161 - Term Project
# Written by Jacob Krucinski

# Set up stdin/out
import os

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

# Import main Dynamixel SDK
from dynamixel_sdk import *

# Import rotary encoder library
import rotary_encoder

# Import math libraries
import math
import numpy as np

# Import GPIO library
#import RPi.GPIO as GPIO
import pigpio

# Set target hardware
ON_RASPI = True

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24
ADDR_MX_CW_LIMIT           = 6
ADDR_MX_CCW_LIMIT          = 8
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING             = 46
ADDR_MX_LED                = 25
ADDR_MX_P_GAIN             = 28

# Protocol version
PROTOCOL_VERSION           = 1.0                # Our Dynamixels use Protocol 1.0

# Default setting
DXL_SHOULDER_ID             = 1                 # Dynamixel ID : 1 (shoulder, angle theta) - CHECK THIS
DXL_BODY_ID                 = 2                 # Dynamixel ID : 2 (body, angle phi) - CHECK THIS
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600

# Define communication port
if (not ON_RASPI):
    DEVICENAME              = "COM7"            # Port used for serial communication on Windows
else:
    DEVICENAME              = "/dev/ttyUSB0"    # Linux device port

# Define encoder resolution and torque mode
ENCODER_RES                 = 4095
ENCODER_NXT_RES             = 180
MODE_ENABLE                 = 1                 # Value for enabling the torque
MODE_DISABLE                = 0                 # Value for disabling the torque

# Define hard limits for body and shoulder joints
# Body joint: Fixed to Waffle plate base, controls the "roll" of the turret arm
# Shoulder join: Connected to the body joint, controls the turret firing arc
SHOULDER_CW_LIM             = 788               # CHECK THIS
SHOULDER_CCW_LIM            = 3288              # CHECK THIS
BODY_CW_LIM                 = 1500              # 131.84 deg
BODY_CCW_LIM                = 2600              # 228.52 deg

# Define motor angle offsets (for easier calculation and custom frame of reference)
BODY_OFFSET                 = 2047              # [encoder counts], 180 deg
SHOULDER_OFFSET             = 2047              # [encoder counts], 180 deg

# Define GPIO pins and global GPIO object
GPIO_LIGHT                  = 18
GPIO_MOTOR_IN1              = 24                        # should be PWM compatible
GPIO_MOTOR_IN2              = 23                        # should be PWM compatible
pi_gpio                     = pigpio.pi()

# Define global port handler
portHandler = PortHandler(DEVICENAME)

# Define global packet handler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Kinematics variables
G           = 9.81 # m/s^2
DEG_TO_RAD  = math.pi / 180
#v0=14.27476 # ft/s
HMAX        = 1.04 #1.0541         #  m, maximum height ball reaches when shot straight up
DIST_X_OFFSET = 0.1016 # m, x distance from camera center to shooter
DIST_Y_OFFSET = 0 # m
V0          = np.sqrt(2 * G * HMAX)
MIN_RANGE   = 0.5    # m
MAX_RANGE   = (V0**2) / G
CUP_RADIUS  = 0.0445
SHOULDER_LOOKUP = {0.8636: 0, 
                   1.4478: 15 * DEG_TO_RAD,
                   2.2098: 30 * DEG_TO_RAD, 
                   2.3114: 45 * DEG_TO_RAD,
                   1.9050: 60 * DEG_TO_RAD,
                   0.8636: 80 * DEG_TO_RAD}
SHOULDER_LOOKUP = dict(sorted(SHOULDER_LOOKUP.items()))     # Sorted by key for look-up table
n_firings   = 0      # Total number of firings


# Custom functions
def _error_handler(dxl_comm_result, dxl_error):
    """ Helper method that raises an exception if there is a communication or response error """
    if dxl_comm_result != COMM_SUCCESS:
        raise Exception(f"Comm error: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        raise Exception(f"Hardware error {dxl_error}: {packetHandler.getRxPacketError(dxl_error)}")
    else:
        return None


def _read(n_bytes, motor_id, addr):
    """ Read n bytes from Dynamixel motors"""
    if (n_bytes == 1):
        read_val, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, motor_id, addr)
        _error_handler(dxl_comm_result, dxl_error)
    elif (n_bytes == 2):
        read_val, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, motor_id, addr)
        _error_handler(dxl_comm_result, dxl_error)
    elif (n_bytes == 4):
        read_val, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, motor_id, addr)
        _error_handler(dxl_comm_result, dxl_error)
    else:
        raise ValueError(f"{n_bytes} bytes is not a valid number of bytes to read")
        read_val = None
    
    return read_val


def _write(n_bytes, motor_id, addr, value):
    if (n_bytes == 1):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, addr, value)
        _error_handler(dxl_comm_result, dxl_error)
    elif (n_bytes == 2):
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, addr, value)
        _error_handler(dxl_comm_result, dxl_error)
    elif (n_bytes == 4):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, addr, value)
        _error_handler(dxl_comm_result, dxl_error)
    else:
        raise ValueError(f"{n_bytes} bytes is not a valid number of bytes to write")
        read_value = None


def init_robot():
    """ 
    Initialize robot arm by setting hard limits and moving to a neutral position.
    Also, set up GPIO pins for ring light and turret motor.
    """

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
        
    # Disable torque mode for both joints to set hard limits
    _write(1, DXL_BODY_ID, ADDR_MX_TORQUE_ENABLE, MODE_DISABLE)
    _write(1, DXL_SHOULDER_ID, ADDR_MX_TORQUE_ENABLE, MODE_DISABLE)

    
    # Set hard limits for body and shoulder joints (if not already set
    curr_cw_lim_body      = _read(2, DXL_BODY_ID, ADDR_MX_CW_LIMIT)
    curr_ccw_lim_body     = _read(2, DXL_BODY_ID, ADDR_MX_CCW_LIMIT)
    curr_cw_lim_shoulder  = _read(2, DXL_SHOULDER_ID, ADDR_MX_CW_LIMIT)
    curr_ccw_lim_shoulder = _read(2, DXL_SHOULDER_ID, ADDR_MX_CCW_LIMIT)
    print(curr_cw_lim_body, curr_ccw_lim_body, curr_cw_lim_shoulder, curr_ccw_lim_shoulder)
    if (curr_cw_lim_body != BODY_CW_LIM):
        _write(2, DXL_BODY_ID, ADDR_MX_CW_LIMIT, BODY_CW_LIM)
    if (curr_ccw_lim_body != BODY_CCW_LIM):
        _write(2, DXL_BODY_ID, ADDR_MX_CCW_LIMIT, BODY_CCW_LIM)
    if (curr_cw_lim_shoulder != SHOULDER_CW_LIM):
        _write(2, DXL_SHOULDER_ID, ADDR_MX_CW_LIMIT, BODY_CW_LIM)
    if (curr_ccw_lim_shoulder != SHOULDER_CCW_LIM):
        _write(2, DXL_SHOULDER_ID, ADDR_MX_CCW_LIMIT, BODY_CCW_LIM)


    # Enable torque mode for both joints
    _write(1, DXL_BODY_ID, ADDR_MX_TORQUE_ENABLE, MODE_ENABLE)
    _write(1, DXL_SHOULDER_ID, ADDR_MX_TORQUE_ENABLE, MODE_ENABLE)

    # Set P gain for more gentle movement (original gain P = 32)
    _write(2, DXL_BODY_ID, ADDR_MX_P_GAIN, 4)
    _write(2, DXL_SHOULDER_ID, ADDR_MX_P_GAIN, 4)

    # Go to neutral position (phi = 0, theta = pi rad)
    set_pose(phi_rad=0, theta_rad=0)

    # Set up GPIO 
    pi_gpio.set_mode(GPIO_LIGHT, pigpio.OUTPUT)
    pi_gpio.set_mode(GPIO_MOTOR_IN1, pigpio.OUTPUT)
    pi_gpio.set_mode(GPIO_MOTOR_IN2, pigpio.OUTPUT)


def _rad_to_encoder(motor_id, rad):
    """ Helper function to convert position [rad] to encoder position to send to motor """
    # Compute pre offset encoder position
    pos = rad * (ENCODER_RES / (2 * math.pi))

    # Add offset based on motor id
    if (motor_id == DXL_BODY_ID): 
        pos += BODY_OFFSET
    else:
        pos += SHOULDER_OFFSET

    return int(pos)


def set_position(motor_id, position_rad):
    """ Helper method that command a given motor to a given position [rad] """
    # TODO: Turn on motor_id LED
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_MX_LED, MODE_ENABLE)
    _error_handler(dxl_comm_result, dxl_error)

    # TODO: Write goal position
    encoder_pos = _rad_to_encoder(motor_id, position_rad)
    #print(f"Encoder pos: {encoder_pos}")
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_MX_GOAL_POSITION, encoder_pos)
    _error_handler(dxl_comm_result, dxl_error)

    # TODO: Turn off motor_id LED
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_MX_LED, MODE_DISABLE)
    _error_handler(dxl_comm_result, dxl_error)


def set_pose(phi_rad, theta_rad):
    """ Move the body and shoulder joints to the specified position """
    # Move body joint
    set_position(DXL_BODY_ID, phi_rad)

    # Move shoulder joint
    set_position(DXL_SHOULDER_ID, theta_rad)

# Check if motor is at goal position
def at_goal_pos(motor_id):
    """
    This function reads the correct memory address of the motor to see if it has reached the goal position
    """
    is_moving = _read(1, motor_id, ADDR_MX_MOVING)
    return is_moving == 0


# LED Ring Light Functions
def enable_ring_light():
    """ Turn ON the LED ring light mounted on the camera """
    pi_gpio.write(GPIO_LIGHT, MODE_ENABLE)

def disable_ring_light():
    """ Turn OFF the LED ring light mounted on the camera """
    pi_gpio.write(GPIO_LIGHT, MODE_DISABLE)


# Turret firing methods
def fire_turret():
    """ 
    Spin the NXT motor ~1 rotation to fire the loaded ball and 
    to reload the mechanism 
    """
    # Spin motor backward (check what direction this is)
    global pos
    pos = 0
    def callback(way):
        global pos
        pos += abs(way)
        print(f"Pos: {pos}")

    decoder = rotary_encoder.decoder(pi_gpio, 7, 8, callback)
    p_control = False
    if (p_control):
        # Thought this would improve rotation precision, but it only gets jerky at the end
        # DO NOT USE
        # Rotate motor 1 rotation (should be 180 ticks, seems to be a bit less)
        goal_pos = 146
        freq = 5000
        pi_gpio.set_PWM_frequency(GPIO_MOTOR_IN1, freq)
        pi_gpio.write(GPIO_MOTOR_IN2, MODE_DISABLE)
        while (pos < goal_pos):
            error = goal_pos - pos
            duty = 200 * error/goal_pos + 55
            freq = 1000 
            
            #pi_gpio.set_PWM_dutycycle(GPIO_MOTOR_IN1, duty)
            pi_gpio.set_PWM_dutycycle(GPIO_MOTOR_IN2, duty)
            #pi_gpio.write(GPIO_MOTOR_IN1, MODE_DISABLE)
            #pi_gpio.write(GPIO_MOTOR_IN2, MODE_ENABLE)

        # Stop rotation
        decoder.cancel()
        pi_gpio.set_PWM_frequency(GPIO_MOTOR_IN2, 0)
        pi_gpio.write(GPIO_MOTOR_IN1, MODE_DISABLE)
        pi_gpio.write(GPIO_MOTOR_IN2, MODE_DISABLE)
    else:
        # This method with just fixed speed seems better
        # Every few firings, rotate the motor a bit less
        global n_firings
        if (n_firings % 4 == 0 and n_firings > 0):
            goal_pos = 144
        else:
            goal_pos = 145
        #goal_pos = 145
            
        while (pos < goal_pos):
            pi_gpio.write(GPIO_MOTOR_IN1, MODE_ENABLE)
            pi_gpio.write(GPIO_MOTOR_IN2, MODE_DISABLE)

        # Stop rotation
        decoder.cancel()
        pi_gpio.write(GPIO_MOTOR_IN1, MODE_DISABLE)
        pi_gpio.write(GPIO_MOTOR_IN2, MODE_DISABLE)
    
    n_firings += 1
    

def get_shoulder_angle(distance): # make sure distance input is in m
    """
    This function computes the turret angle in order to shoot the ball into the cup
    """
    distance = distance - DIST_X_OFFSET
    if distance > MAX_RANGE or distance < MIN_RANGE:
        raise ValueError("OBJECT OUT OF RANGE")
    else:
        theta_rad = 0.5 * np.arcsin((distance * G) / V0**2) # returns in radians
        # will likely want to take the larger of two possible solutions (so we enter the cup at a larger angle wrt ground)
        if theta_rad < math.pi / 4: # an angle of 45 will ALWAYS get us max range.
            theta_rad = math.pi/2 - theta_rad
        
    refined_distance=distance-DIST_X_OFFSET*np.cos(theta_rad)
    theta_rad=0.5*np.arcsin((refined_distance*G)/V0**2)
    theta_new=theta_rad
    if theta_new < math.pi / 4: #an angle of 45 will ALWAYS get us max range.
        theta_new = math.pi / 2 - theta_new

    return theta_new
    

def get_shoulder_angle_lookup_table(distance):
    # x_values: Distance (keys)
    # y_values: Shoulder angle (values)
    x_values = np.array(list(SHOULDER_LOOKUP.keys()))
    y_values = np.array(list(SHOULDER_LOOKUP.values()))
    print(f"x: {x_values}")
    print(f"y: {y_values}")
    i = np.searchsorted(x_values, distance)
    if i == 0:
        return y_values[0]
    elif i == len(x_values):
        return y_values[-1]
    else:
        x0, x1 = x_values[i - 1], x_values[i]
        y0, y1 = y_values[i - 1], y_values[i]
        # Perform linear interpolation
        return y0 + (y1 - y0) * (distance - x0) / (x1 - x0)

