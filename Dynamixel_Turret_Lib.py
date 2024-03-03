# Dynamixel_Turret_Lib.py
# ECE 3161 - Term Project
# Written by Jacob Krucinski


# Import main Dynamixel SDK
from dynamixel_sdk import *

# Import math library
import math

# Set target hardware
ON_RASPI = True

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_LED                = 99                 # CHECK THIS

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
MODE_ENABLE                 = 1                 # Value for enabling the torque
MODE_DISABLE                = 0                 # Value for disabling the torque

# Define hard limits for body and shoulder joints
# Body joint: Fixed to Waffle plate base, controls the "roll" of the turret arm
# Shoulder join: Connected to the body joint, controls the turret firing arc
BODY_CW_LIM                 = 788               # CHECK THIS
BODY_CCW_LIM                = 3288              # CHECK THIS
SHOULDER_CW_LIM             = 1100              # CHECK THIS
SHOULDER_CCW_LIM            = 1820              # CHECK THIS

# Define motor angle offsets (for easier calculation and custom frame of reference)
BODY_OFFSET                 = 0                 # [encoder counts]
SHOULDER_OFFSET             = 0                 # [encoder counts]

# Define global port handler
portHandler = PortHandler(DEVICENAME)

# Define global packet handler
packetHandler = PacketHandler(PROTOCOL_VERSION)



# Custom functions
def init_robot():
    # TODO: Set hard limits for body and shoulder joints


    # TODO: Go to neutral position (phi = 0, theta = pi rad)
    pass

def _rad_to_encoder(motor_id, rad):
    # Compute pre offset encoder position
    pos = rad * (ENCODER_RES / (2 * math.pi))

    # Add offset based on motor id
    if (motor_id == DXL_BODY_ID): 
        pos += BODY_OFFSET
    else:
        pos += SHOULDER_OFFSET

    return pos


def _error_handler(dxl_comm_result, dxl_error):
    if dxl_comm_result != COMM_SUCCESS:
        raise AttributeError(f"Comm error: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        raise AttributeError(f"Hardware error {dxl_error}: {packetHandler.getRxPacketError(dxl_error)}")


def _set_positon(motor_id, position_rad):
    # TODO: Turn on motor_id LED
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_MX_LED, MODE_ENABLE)
    _error_handler(dxl_comm_result, dxl_error)

    # TODO: Write goal position
    encoder_pos = _rad_to_encoder(position_rad, motor_id)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_MX_GOAL_POSITION, encoder_pos)
    _error_handler(dxl_comm_result, dxl_error)

    # TODO: Turn off motor_id LED
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_MX_LED, MODE_DISABLE)
    _error_handler(dxl_comm_result, dxl_error)


def set_pose(phi_rad, theta_rad):
    """ Move the body and shoulder joints to the specified position """

    # Move body joint
    _set_positon(DXL_BODY_ID, phi_rad)

    # Move shoulder joint
    _set_positon(DXL_SHOULDER_ID, theta_rad)
