import time
import cv2
import numpy as np

MARKER_LENGTH_M = 0.20
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

def main():
    cap = cv2.VideoCapture(0) # Cap is the video capture object, 0 is usually the default camera (so webcam)
    # Check if the camera opened successfully
    if not cap.isOpened():
        raise RuntimeError("Could not open video device")
    
    # Now we can read frames from the camera in a loop
    while True:
        check, frame = cap.read() # Read a frame from the camera, check is just a ture or false if any frame was read
if __name__ == "__main__":
    main()   