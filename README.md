# EGB349-Project_HTI_eldrone-2026
A group of genious Aerospace Engineers designing a Drone for QUT's EGB349 and EGB450 in 2026. A project focuses on the planning, design and creation of eldrone

This repository contains the software developed for the QUT EGB349/EGH450 Search and Rescue (SAR) UAS project.

The system is designed to autonomously search an indoor area, detect and localise targets, display mission information to an operator, and trigger payload deployment mid-air when required.

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







