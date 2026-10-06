# EGB349-Project_HTI_eldrone-2026
A group of genious Aerospace Engineers designing a Drone for QUT's EGB349 and EGB450 in 2026. A project focuses on the planning, design and creation of eldrone

This repository contains the software developed for the QUT EGB349/EGH450 Search and Rescue (SAR) UAS project.

The system is designed to autonomously search an indoor area, detect and localise targets, display mission information to an operator, and trigger payload deployment mid-air when required.

The SAR dataset collected can be found here:

https://app.roboflow.com/egh455-vbb5h/sar-mission-ucsok/models

## Operator Interface User Guide for Mission

A detailed HTML user guide is available for the **Pi 400 Ground Control Station and Pi 4B ROS Master**, including the demo sequence, RViz/rqt setup, navigation, payload testing, shutdown procedure, and troubleshooting.

**[Download the UAS_SAR_G8 Operator Interface User Guide here](UAS_SAR_G8_Operator_Interface_User_Guide.html)**

## System overview
### UAV Hardware
- Holybro X500 V2 airframe
- Pixhawk 6X autopilot
- Raspberry Pi 4B onboard computer / ROS master
- OAK-D Pro RGB-D camera
- Optitrack external localisation
- Dual servo payload deployment system

### Ground Control Station
- Raspberry Pi 400
- Rviz
- RQT
- QGroundControl
- Manual RC controller
- Distributed ROS & SSH connection to the onboard Raspberry Pi 4B

### Software
- Ubuntu 20.04
- ROS Noetic
- PX4
- MAVROS
- QUTAS ROS packages
- DepthAI
- OpenCV
- Python3

## Repository Structure
The main ROS workspace is contained within:

```text
 catkin_ws_G6/
```
The main packages used by the current system include:
```text
catkin_ws_G6/src/
├── breadcrumb/
├── depthai_publisher/
├── egb349_group8_vision/
├── egh450_image_processor/
├── egh450_target_solvepnp/
├── qutas_lab_450/
├── rqt_eyedropper/
├── rqt_generic_hud/
├── rqt_mavros_gui/
└── spar/
```
Other useful project folders include:

```text
Image testers/
Recordings/
models/
old_catkin_ws/
```
## Prerequisites
- Ubuntu 20.04
- ROS Noetic
- Python 3
- PX4
- MAVROS
- QGroundControl
- DepthAI
- OpenCV
- tmux
- RViz
- RQT

## Hardware Setup
- Airframe: Holybro X500 V2
- Autopilot: Pixhawk 6X
- On-board Computer: Raspberry Pi 4B
- Ground Control Station: Raspberry Pi 4B
- Camera: Luxonis OAK-D Pro
- Localisation: OptiTrack
- Payload: Dual servo payload release system
- Manual Control: RC transmitter

## Installation and Workspace Setup
### 1. Clone Repository
Clone the project repository onto the Raspberry Pi 4B:

```text
git clone https://github.com/notajet/EGB349_Project_eldrone.git
cd EGB349_Project_eldrone
```
### 2. Build ROS workspace
Build the main catkin workspace:

```text
cd ~/EGB349_Project_eldrone/catkin_ws_G6
catkin_make
source devel/setup.bash
```
The workspace should be sourced before running project ROS nodes.

## Distributed ROS setup
The Raspberry Pi 4B acts as the ROS master and runs the main UAV software.
The Raspberry Pi 400 acts as the Ground Control Station and is used for:
- RViz
- RQT
- telemetry monitoring
- camera viewing
- operator control
Both Raspberry Pis must be connected to the same network
### 1. Start the Raspberry Pi 4B ROS Master
From the Raspberry Pi 400, SSH into the Raspberry Pi 4B:
```text
ssh uavteam6@<PI_4B_IP>
```
The ```<PI_4B_IP>``` can be obtained through a webhook on a private discord channel, sent each time the Pi is turned on by a bash.rc script.
Then load the main tmux script:

```text
source ~/run349.sh
run349
```
This starts the primary tmux session on the Raspberry Pi 4B. ROS core is started automatically as part of this process.

### 2. Connect Raspberry Pi 400 to the ROS Master
Open a seperate terminal on the Raspberry Pi 400 and run:
```text
disros <PI_4B_IP>
```
Confirm that the Raspberry Pi 400 can communicate with the ROS master:
```text
rostopic list
```
If the ROS topics are displayed, distributed ROS is operating correctly.
The Pi 4B is the ROS master, while the Pi 400 is used for visualisation and operator tools.

## RViz Setup
After running:
```text
disros <PI_4B_IP>
```
launch Rviz from the Raspberry Pi 400 using:
```text
rviz -d ~/349.rviz
```
The saved RViz configuration is used to display:
- UAV position
- OptiTrack coordinate frames
- Camera Imagery
- Detected ArUco markers
- Target pose information
- Other mission information
If RViz opens but some displays are missing, check the current ROS topics using:
```text
rostopic list
```
and updated the affected RViz topic selections if required.


## Vision System
The vision system uses the OAK-D Pro camera to run both YOLO target detection and ArUco marker detection.

The current vision system uses:
- UAS_SAR_v26 YOLO model
- 416x416 camera input
- 15 FPS camera feed
- Person and backpack detection
- ArUco 5x5 marker detection
- Landing ArUco ID 16

The vision system is launched using:

```text
roslaunch depthai_publisher dai_yolo_publisher.launch
```

This launch file starts both the DepthAI YOLO publisher and the ArUco detector.

The main camera topics are:

```text
/depthai_node/image/compressed
/yolo/image/compressed
/processed_vision/image/compressed
```

The raw camera image is published on:

```text
/depthai_node/image/compressed
```

The YOLO annotated image is published on:

```text
/yolo/image/compressed
```

The final processed image containing both YOLO and ArUco annotations is published on:

```text
/processed_vision/image/compressed
```

The processed image can be viewed using:

```text
rqt_image_view
```

Then select `/processed_vision/image/compressed` from the topic list.

## YOLO Target Detection
The YOLO model is used to detect people and backpacks within the search area.

The current model files are stored in:

```text
models/UAS_SAR_v26_RVC2/
```

The active model and its detection settings are configured inside:

```text
catkin_ws_G6/src/depthai_publisher/launch/dai_yolo_publisher.launch
```

The current settings are:
- Model: UAS_SAR_v26
- Input size: 416x416
- Confidence threshold: 0.89
- IoU threshold: 0.50
- Camera FPS: 15
- JPEG quality: 90

YOLO detections are published on:

```text
/yolo/detections
```

The detected image positions are converted into world positions using the camera calibration and current UAV pose supplied by OptiTrack.

The target position and type are then published to the guidance system using:

```text
/guidance/roi
/guidance/roi_type
```

The guidance system enables and disables target diversions using:

```text
/guidance/roi_enable
```

Targets may still appear on the processed camera feed while ROI detection is disabled, but they will not interrupt the flight path.

## ArUco Detection
ArUco markers are used to locate targets and perform the final landing approach.

The detector currently uses:

```text
DICT_5X5_100
```

The physical marker size used by the detector is:

```text
0.200 m
```

The default landing marker is ArUco ID 16.

The landing marker ID can be changed when launching the vision system:

```text
roslaunch depthai_publisher dai_yolo_publisher.launch landing_aruco_id:=16
```

ArUco information is published on:

```text
/aruco/pose_log
/aruco/world_position
/aruco/detection_log_text
```

The detector requires a current OptiTrack pose before it can convert a marker position into world coordinates.

ArUco detection works best when:
- The complete marker is visible
- The image is not affected by motion blur
- The marker border can be clearly seen
- There are no strong reflections over the marker
- The UAV is moving slowly over the marker

## Guidance System
The autonomous mission is controlled by the guidance node.

The guidance configuration is stored in:

```text
catkin_ws_G6/src/spar/spar_node/config/guidance.yaml
```

This file contains:
- Flight height
- Flight speeds
- Takeoff and landing speeds
- Position and yaw tolerances
- Payload deployment height
- Battery limits
- Search waypoints
- ROS topic names

The guidance node is started through:

```text
catkin_ws_G6/src/spar/spar_node/launch/guidance.launch
```

The mission follows this general sequence:

```text
Takeoff
Search waypoints
Detect target
Divert to target
Deploy payload
Return to search
Complete the remaining waypoints
Move towards the landing ArUco
Land
```

The guidance node initially waits for the SPAR flight server.

The SPAR flight server becomes available after:
- MAVROS is connected
- A valid UAV pose is available
- The UAV is armed
- OFFBOARD mode is selected

Once these conditions are met, the autonomous mission begins.

For any extra information regarding AUT-specfic mission logic information, refer to:

```text
catkin_ws_G6/src/spar/spar_node/EGH450_AUT_Architecture_Handover.md
```

and/or:

```text
catkin_ws_G6/src/spar/spar_node/EGH450_AUT_Architecture_Handover.pdf
```

## Search Pattern
The search pattern is stored inside:

```text
catkin_ws_G6/src/spar/spar_node/config/guidance.yaml
```

The current search uses a lawnmower pattern across the assessed flight area.

All waypoint coordinates are relative to the OptiTrack world origin. Before flying in a different area, check:
- The location of the world origin
- The positive X and Y directions
- The flight boundaries
- The flight height
- The distance between each search pass
- The final return position

The guidance system currently enables target diversions after waypoint 2. This allows the UAV to begin the search before responding to targets.

## Payload Deployment
The payload deployment system uses two servos to release the required payload.

The two payload commands are:
- `T` for the tracker
- `E` for the EpiPen

The target to payload mapping is:
- Backpack detection → Tracker
- Person detection → EpiPen

The payload system is started through:

```text
catkin_ws_G6/src/egh450_payload_deploy/launch/pdd_launch.launch
```

The guidance system sends a payload command using:

```text
/payload/nav_clearance
```

The payload system returns confirmation using:

```text
/payload/deploy_confirmation
```

Only one payload of each type can be deployed during a mission.

The servo and payload system should be tested without the propellers installed before each assessed flight.

## Important ROS Topics
The main topics used during the mission are:

```text
/mavros/state
/mavros/local_position/pose
/mavros/vision_pose/pose
/mavros/battery

/depthai_node/image/compressed
/depthai_node/camera/camera_info

/yolo/image/compressed
/yolo/detections

/processed_vision/image/compressed

/guidance/roi
/guidance/roi_type
/guidance/roi_enable

/aruco/pose_log
/aruco/world_position

/payload/nav_clearance
/payload/deploy_confirmation
```

These topics can be checked using:

```text
rostopic list
```

## Pre-Flight Checks
Before starting a flight, check:
- The UAV battery is charged
- The RC controller is connected
- Manual control has been tested
- The propellers are correctly installed
- The payloads are installed in the correct positions
- The OAK-D Pro is connected
- The Pixhawk is connected to `/dev/ttyUSB0`
- Both Raspberry Pis are connected to the same network
- The Raspberry Pi 4B IP address is known
- OptiTrack is tracking the `eldrone` rigid body
- MAVROS is connected
- UAV position data is updating
- Camera images are being published
- The YOLO model is running
- ArUco detection is running
- RViz is receiving the correct topics
- There is enough storage available for rosbag recording
- The flight area is clear

Check MAVROS using:

```text
rostopic echo /mavros/state
```

Check the local UAV position using:

```text
rostopic echo /mavros/local_position/pose
```

Check the OptiTrack pose using:

```text
rostopic echo /mavros/vision_pose/pose
```

Check the battery using:

```text
rostopic echo /mavros/battery
```

Check the raw camera frame rate using:

```text
rostopic hz /depthai_node/image/compressed
```

Check the final processed vision frame rate using:

```text
rostopic hz /processed_vision/image/compressed
```

## Running the Mission
Before starting the mission:
1. Complete the pre-flight checks
2. Start rosbag recording
3. Confirm that the flight area is clear
4. Confirm the UAV position in RViz
5. Confirm that the camera feed is visible
6. Confirm that the payloads are installed correctly
7. Confirm that the pilot is ready to take manual control

The pilot can then:
1. Arm the UAV
2. Confirm that the UAV remains stable
3. Select OFFBOARD mode
4. Monitor the autonomous mission

After OFFBOARD mode is selected and the SPAR server becomes available, the guidance system begins the mission.

During the flight, monitor:
- UAV position
- Battery level
- MAVROS connection
- OptiTrack pose
- Camera frame rate
- YOLO detections
- ArUco detections
- Payload deployment
- Current mission state

The RC pilot must remain ready to take manual control during the entire flight.

## Recording Flights
Rosbags are used to record flight information and camera images for later testing.

To record all ROS topics:

```text
rosbag record -a
```

To create a rosbag with the date and time in its name:

```text
rosbag record -O flight_$(date +%Y%m%d_%H%M%S).bag -a
```

To record only the main camera topics:

```text
rosbag record -O images_$(date +%Y%m%d_%H%M%S).bag /depthai_node/image/compressed /yolo/image/compressed /processed_vision/image/compressed
```

Stop the recording using:

```text
Ctrl+C
```

The rosbag should be stopped correctly before shutting down the Raspberry Pi. This reduces the chance of the rosbag becoming corrupted.

## Mission Shutdown
After the UAV has landed:
1. Confirm that the UAV is disarmed
2. Stop the rosbag recording
3. Stop the guidance and vision nodes
4. Close RViz and RQT
5. Shut down the onboard Raspberry Pi
6. Disconnect the UAV battery
7. Remove any remaining payloads
8. Save the rosbag and screen recording

The UAV must be disarmed before anybody approaches it.

## Common Problems
### No ROS topics on the Raspberry Pi 400
Check:
- Both Raspberry Pis are on the same network
- `disros` was run using the correct IP address
- ROS core is running on the Raspberry Pi 4B

Run:

```text
rostopic list
```

### MAVROS is not connected
Check:
- The Pixhawk USB connection
- `/dev/ttyUSB0` exists
- The baud rate is set to 921600
- The Pixhawk is powered
- The target system ID is correct

### No OptiTrack position
Check:
- The `eldrone` rigid body is enabled
- OptiTrack streaming is running
- The UAV is visible to the OptiTrack cameras
- `/vicon/eldrone/pose` is being published
- `/mavros/vision_pose/pose` is updating

### No camera images
Check:
- The OAK-D Pro USB connection
- The vision terminal for any DepthAI errors
- The model and configuration file paths
- No other program is already using the camera

### YOLO is running but ArUco is not detecting
Check:
- The complete marker is visible
- The correct ArUco dictionary is being used
- The marker is large enough in the image
- The landing marker ID is correct
- The image is not blurred
- Camera calibration information is being published

Check the camera information using:

```text
rostopic echo /depthai_node/camera/camera_info
```

### The mission does not begin
Check:
- MAVROS is connected
- A valid UAV pose is available
- The UAV is armed
- OFFBOARD mode is selected
- The `/spar/flight` action server is available

### A target is detected but the UAV does not divert
Check:

```text
rostopic echo /guidance/roi_enable
rostopic echo /guidance/roi
rostopic echo /guidance/roi_type
```

Target diversion is disabled during the beginning of the search and during parts of the mission where an interruption would be unsafe.

### The payload does not deploy
Check:

```text
rostopic echo /payload/nav_clearance
rostopic echo /payload/deploy_confirmation
```

Also check the servo power, connections and payload positions.

## Known Limitations
- ArUco detection becomes less reliable at greater heights
- Fast UAV movement can cause motion blur
- The system requires a valid OptiTrack pose
- The search waypoints are configured for the assessed flight area
- Some launch files contain absolute paths for the `uavteam6` Raspberry Pi
- Camera performance depends on the lighting and USB connection
- The current vision system uses a 416x416 image
- Older scripts and model versions remain in the repository for testing and reference

## Main Project Packages
### `depthai_publisher`
Runs the OAK-D Pro camera and YOLO model. It publishes the raw image, YOLO detections, annotated image and camera information.

### `egb349_group8_vision`
Contains the ArUco detector and older payload and servo development files.

### `egh450_payload_deploy`
Controls the payload servos and reports when a payload has been deployed.

### `spar`
Provides the flight motion action server and contains the main autonomous guidance node.

### `qutas_lab_450`
Contains the QUT flight environment, area map and supporting flight packages.

### `breadcrumb`
Contains the supplied A* path planning system used with the occupancy grid.

## Current Demo Configuration
The current assessed flight configuration is:
- Flight Height: Adjustable
- Horizontal speed: 0.4 m/s
- Vertical speed: 0.3 m/s
- Takeoff speed: 0.2 m/s
- Landing speed: 0.2 m/s
- Payload Diversion Speed: 0.3 m/s
- Payload deployment height: 1.1 m (0.5m above target - from base of UAS)
- Battery warning: 20%
- Automatic low battery landing: 10%
- YOLO model: UAS_SAR_v26
- Vision input: 416x416
- Camera FPS: 15
- Landing ArUco ID: Adjustable

These values can be changed through:

```text
catkin_ws_G6/src/spar/spar_node/config/guidance.yaml
catkin_ws_G6/src/depthai_publisher/launch/dai_yolo_publisher.launch
```

The settings should be checked before each assessed flight.

