'''
Author: Jacob Krucinski
ECE 3161 Term Project
Date last modified: 03/18/24

Description: Have robot shoulder joint TRACK the detected ArUco tag
'''

# Import image progressing libraries and GPIO for ring light
import cv2
import scipy.io as sio
import numpy as np
import math
import RPi.GPIO as GPIO


# Import custom turret library 
from Dynamixel_Turret_Lib import *

# Load camera parameters from MATLAB
camParams = sio.loadmat("raspi_camera_params_640x480_opencv_v2.mat")
cameraMatrix = camParams['cameraMatrix']
distCoeffs = camParams['distortionCoefficients']

# Start camera
camera = cv2.VideoCapture(0)         # USB camera
camera.set(3, 640)
camera.set(4, 480)

# Set up the ArUco dictionary and detector object
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
markerLength = 94       # [mm] update based on cup tag size later 

print("Reading from camera...\n")

# To keep track of saved images
i = 0

# Define tracking status (as a boolean)
TRACKING  		= False
already_fired 	= False
COL_RED   		= (0, 0, 255)       # BGR format
COL_GREEN 		= (0, 255, 0)

# ** Scaling factor testing
USE_SCALING = False
if (USE_SCALING):
    scale = 1400 / 279.4
else:
    scale = 1
    
# Define function to convert tvec to polar coordinates
# Axes: X - To the left of the camera (to the right in image frame)
#       Y - Down towards the floor
#       Z - Straight out from the camera lens
def tvec_to_polar(tvec):
    x, y, z = tvec
    r = math.sqrt(x**2 + z**2)
    theta = math.atan2(x, z)   # y, x order
    
    return [r, theta]

# Init robot and GPIO
init_robot()

# Main vision control loop
while True:
    success, image = camera.read()

    # Make sure to rotate frame 180 deg
    image = cv2.rotate(image, cv2.ROTATE_180)

    if (success):
        s = image.shape
        #image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # First we detect all markers in the frame
        (corners, ids, rejected) = detector.detectMarkers(image)
    
        if len(corners) > 0: # we have detected something
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(image, corners, ids) # draw outlines for all 

            # For every detected marker, we do pose estimation using its corners and find the rotational and translational vectors
            for (markerCorner, markerID) in zip(corners, ids):
                reshapedCorners = markerCorner.reshape((4, 2))
                (tL, tR, bR, bL) = reshapedCorners
                topLeft = [int(tL[0]), int(tL[1])]
                
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, markerLength, cameraMatrix, distCoeffs)
                rvec = rvec[0][0]
                tvec = tvec[0][0]

                # Convert to polar coordinates
                d, theta = tvec_to_polar(tvec)
                
                # Set body position
                if (TRACKING):
                    set_position(DXL_BODY_ID, -theta)

                # Set shoulder position
                if (TRACKING and at_goal_pos(DXL_BODY_ID)):
                    d_m = d / 1000
                    try:
                        phi = get_shoulder_angle(d_m)
                        #phi = get_shoulder_angle_lookup_table(d_m)
                        print(f"d: {d_m} m, phi: {phi * 180/math.pi} deg")
                        set_position(DXL_SHOULDER_ID, phi)
                    except ValueError as e:
                        raise e

                # Fire once at goal turret angle
                if (TRACKING and at_goal_pos(DXL_SHOULDER_ID) and not already_fired):
                    fire_turret()
                    already_fired = True

                # Printing distance on the image
                cv2.putText(image, f"{round(d / 1000, 2)} m", (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_RED, 2)
                #print("Marker detected! ID: {}, RVEC: {}, TVEC: {}".format(str(markerID), rvec, tvec))
                #print(f"TVEC: {tvec}")
                #print(f"TVEC Polar: r={tvec_polar[0]}, theta={tvec_polar[1] * (180/math.pi)} deg")
                
            # Enable/disable tracking via space bar (ONLY if a tag is detected)
            if cv2.waitKey(1) == ord(' '):
                TRACKING = not TRACKING
                if (not TRACKING):
                    already_fired = False
        else:
            TRACKING = False
            already_fired = False       # Reset already fired flag

        # Add tracking mode text to top left corner of frame - ONLY WHEN TARGET IS DETECTED
        if (TRACKING):
            cv2.putText(image, "Tracking: ON", (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_GREEN, 2)
            enable_ring_light()
        else:
            cv2.putText(image, "Tracking: OFF", (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_RED, 2)
            disable_ring_light()


        # Fire turret when ENTER key is pressed
        if cv2.waitKey(1) == ord('f') and TRACKING:
            fire_turret()

        # Show final image in window
        cv2.imshow("ArUco Detection", image)


    if cv2.waitKey(1) == 27: # ESC key to exit
        break

# Terminate program
print("Camera terminated. Finished reading!\n")
camera.release()
cv2.destroyAllWindows()
disable_ring_light()
pi_gpio.stop()
