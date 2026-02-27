"""
This is a simple script to test the detetion of Aruco markers using a webcam, rather than the OAK D Pro, which is what we will be using for the actual localization.
The code is based on the OpenCV Aruco marker detection tutorial, and is meant to be a simple test of detection logic
It doesnt contain any localization logic yet, just detection and annotation of the markers in the video feed.
"""
import time
import cv2
import numpy as np

# Most of this is used for the localizzation part, not needed yet
MARKER_LENGTH_M = 0.20 # M is for meters
ARUCO_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICTIONARY, ARUCO_PARAMS)

def webcam_aruco():
    cap = cv2.VideoCapture(0) # Cap is the video capture object, 0 is usually the default camera (so webcam)
    # Check if the camera opened successfully
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video device {cap}")
    
    # Now we can read frames from the camera in a loop
    while True:
        ### CAMERA FRAME READING AND MARKER DETECTION
        check, frame = cap.read() # Reads a frame from the camera, check is just a ture or false if any frame was read
        if not check:
            print("Failed to capture frame")
            break

        frame_g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert the frame to grayscale for marker detection
        corners, ids, rejected = ARUCO_DETECTOR.detectMarkers(frame_g) # Detect markers in the frame , um rejected is just the corners that were rejected as markers (shrugging)
        # Note: OpenCV automatically works on gray scale for marker detection, so we don't need to convert the frame to grayscale before detection. But would be faster too I believe

        ### ANNOTATION OF FRAME
        if ids is not None and len(ids) > 0: #     
            # Draw boxes around markers (Green is good)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Make a list of seen ID's useful for localization
            id_list = [int(x[0]) for x in ids]
            id_log = "Found IDs: " + ", ".join(map(str, id_list))

            # Add an ID indicator on screen
            cv2.putText(frame, id_log, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2) #Use color blue

            # Show the list of ID's in the terminal (updates very loop, can show multiple ID's)
            #print(id_log) 

        else:
            # In case theres like a really distant marker, best to add something to say that
            cv2.putText(frame, "No markers detected", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (225, 0, 0), 2) #Use color blue

        ### VIDEO DISPLAY
        cv2.imshow("Aruco Marker Detection", frame) # Show the frame with the detected markers and ID's

        ### SAFE EXIT
        key = cv2.waitKey(1) & 0xFF # waits 1 millisecond
        if key == ord('q'):
            break # Exit the loop if 'q' is pressed (On the display), or ctrl+c in the terminal

    cap.release() # Release the video capture object when done
    cv2.destroyAllWindows() # Close all OpenCV windows

if __name__ == "__main__":
    webcam_aruco()   