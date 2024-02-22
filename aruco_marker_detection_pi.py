'''
Author: Arjun Viswanathan, John Dale
ECE 566 Final Project
Date created: 11/21/23
Date last modified: 102/21/24

Description: base code for detecting ArUco markers off a live camera feed on RASPBERRY PI
'''

import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import scipy.io as sio
import numpy as np

# Load camera parameters from MATLAB
camParams = sio.loadmat("raspi_camera_params_opencv.mat")
cameraMatrix = camParams['cameraMatrix']
distCoeffs = camParams['distortionCoefficients']

# Start camera
camera = PiCamera()
camera.resolution = (1920, 1080)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)
time.sleep(0.1)

# Set up the ArUco dictionary and detector object
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
markerLength = 25       # [mm] update based on cup tag size later 

print("Reading from camera...\n")

# To keep track of saved images
i = 0

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Grab the raw NumPy array representing the image
    image = frame.array
        
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
            cv2.putText(image, str(round(tvec[2], 2)), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("Marker detected! ID: {}, RVEC: {}, TVEC: {}".format(str(markerID), rvec, tvec))

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    cv2.imshow("ArUco Detection", image)

    if cv2.waitKey(1) == 27: # ESC key to exit
        print("Exiting...")
        break