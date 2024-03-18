'''
Author: Jacob Krucinski
ECE 3161 Term Project
Date last modified: 03/18/24

Description: Have robot shoulder joint TRACK the detected ArUco tag
'''

# Import image progressing libraries
import cv2
import scipy.io as sio
import numpy as np

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
markerLength = 54       # [mm] update based on cup tag size later 

print("Reading from camera...\n")

# To keep track of saved images
i = 0

# Define tracking status (as a boolean)
TRACKING  = False
COL_RED   = (255, 0, 0)
COL_GREEN = (0, 255, 0)

# ** Scaling factor testing
USE_SCALING = False
if (USE_SCALING):
    scale = 1400 / 279.4
else:
    scale = 1

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

                # Printing distance on the image
                cv2.putText(image, str(round(tvec[2] / scale, 2)), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_RED, 2)
                print("Marker detected! ID: {}, RVEC: {}, TVEC: {}".format(str(markerID), rvec, tvec))

        # Add tracking mode text to top left corner of frame
        if (TRACKING):
            cv2.putText(image, "Tracking: ON", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_GREEN, 1)
        else:
            cv2.putText(image, "Tracking: OFF", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_RED, 1)

        # Show final image in window
        cv2.imshow("ArUco Detection", image)

        # Enable/disable tracking via space bar (ONLY if a tag is detected)
        if cv2.waitKey(1) == ord(' '):
            TRACKING = not TRACKING


    if cv2.waitKey(1) == 27: # ESC key to exit
        break

print("Camera terminated. Finished reading!\n")
cv2.destroyAllWindows()
