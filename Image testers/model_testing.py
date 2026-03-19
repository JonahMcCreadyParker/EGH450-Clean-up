"""
This is a simple script to test the detection of Aruco markers using a webcam, rather than the OAK D Pro, which is what we will be using for the actual localization.
The code is based on the OpenCV Aruco marker detection tutorial, and is meant to be a simple test of detection logic
It doesnt contain any localization logic yet, just detection and annotation of the markers in the video feed.

Refer to me if ya have questions - Taj
"""
import time
import cv2
import numpy as np
from ultralytics import YOLO

# Most of this is used for the localization part, not needed yet
MARKER_LENGTH_M = 0.20 # M is for meters
ARUCO_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICTIONARY, ARUCO_PARAMS)

MODEL_PATH = "/Users/tajfoley/Library/CloudStorage/OneDrive-Personal/Documents/Uni/QUT/4th Year QUT/EGB349/EGB349_Project_HTI_eldrone/Image testers/best.pt"

print("Lets Go Ahead...")

### YOLO MODEL LOADING ###
def load_yolo_model():
    model = YOLO(MODEL_PATH)
    return model


### YOLO DETECTION AND ANNOTATION LOGIC ###
def draw_yolo_detections(frame, results, model):
    boxes = results[0].boxes

    # Check if any boxes were detected, if not add some text to indicate that
    if boxes is None or len(boxes) == 0:
        cv2.putText(frame, "No YOLO targets detected", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
        return frame

    # Loop through the detected boxes and draw them on the frame
    for box in boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        x1, y1, x2, y2 = box.xyxy[0].tolist()

        # Convert coordinates to integers for drawing
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)

        class_name = model.names[cls_id]

        # Blue box for YOLO detections
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

        label = f"{class_name}: {conf:.2f}"
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

    return frame


### ARUCO DETECTION AND ANNOTATION LOGIC ###
def webcam_aruco():
    model = load_yolo_model()
    print(model.names)
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

        ### YOLO DETECTION
        yolo_results = model(frame, verbose=False)
        frame = draw_yolo_detections(frame, yolo_results, model)

        ### ANNOTATION OF FRAME
        if ids is not None and len(ids) > 0: #     
            # Draw boxes around markers (Green is good)
            cv2.aruco.drawDetectedMarkers(frame, corners, borderColor=(0,255,0))

            # Make a list of seen ID's useful for localization
            id_list = [int(x[0]) for x in ids]
            id_log = "Found IDs: " + ", ".join(map(str, id_list))

            # Add an ID indicator on top of screen
            cv2.putText(frame, id_log, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2) #Use color green

            # Show the list of ID's in the terminal (updates very loop, can show multiple ID's)
            #print(id_log) 

            #This adds the ID to the Top left corner of the marker
            for i,c in enumerate(corners):
                aruco_id = int(ids[i][0])
                top_left = c[0][0] # Get the top left corner of the marker
                x, y = int(top_left[0,]), int(top_left[1]) 
                cv2.putText(frame, f"ID: {aruco_id}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 3) #green
        else:
            # Good to add something to indicate that no markers were seen
            cv2.putText(frame, "No markers detected", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3) #red

        ### VIDEO DISPLAY
        cv2.imshow("Aruco Marker Detection + YOLO Detection", frame) # Show the frame with the detected markers and ID's

        ### SAFE EXIT
        key = cv2.waitKey(1) & 0xFF # waits 1 millisecond
        if key == ord('q'):
            break # Exit the loop if 'q' is pressed (On the display), or ctrl+c in the terminal

    cap.release() # Release the video capture object when done
    cv2.destroyAllWindows() # Close all OpenCV windows


if __name__ == "__main__":
    webcam_aruco()   

"""
The main changes that would be added here for a complete simulation solution are:
- Pose estimation logic and conversion
- Displaying a coordinate frame on the marker to show orientation
- A filtering system to help prevent false positives and buffering 

For the EGB349 Proper Implementation
- Change the video capture to use the OAK D Pro's camera feed instead of the webcam
- Adjust processing and encoding logic to be done on the OAK D Pro's VPU, Myriad X
- Add the camera intrinsics and distortion coefficients for the OAK D Pro to the pose estimation logic
- Change Video Display to show on GCS Operator Interface
- Add ROS integration to publish detected marker information to the rest of the system
  - Specifically for a log in the GCS, and for the NAV system to use for navigation, and payload targeting

Expected for EGH450:
- Will need to be integrated with the AI model for the other targets
- Will need to make compensations for frame rate and processing time

Little things to note:
- Th
"""