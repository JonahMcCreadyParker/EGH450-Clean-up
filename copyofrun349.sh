# This isn't configured as ours is within a repo
# function run349(){
#     echo "Starting tmux sessions for EGH450 UAV Control"
#     sleep 2
#     tmux new-session -s egh450\; set -g mouse on\; send-keys "roscore" C-m\; split-window -h -p 85\; split-window -v\;      \
#     send-keys "htop" C-m\; select-pane -t 2\; split-window -h\; select-pane -t 2\;                                          \
#     send-keys "ls -l" C-m\; split-window -v -p 75\;                                                                         \
#     send-keys "sleep 5; roslaunch ~/catkin_ws/launch/control.launch"\; split-window -v -p 60\;                        \
#     send-keys "sleep 10; roslaunch qutas_lab_450 environment.launch"\; split-window -v \;                               \
#     send-keys "rosrun depthai_publisher dai_publisher" \; select-pane -t 6\;                                  \
#     send-keys "rosbag record -a" \; split-window -v -p 85\;                                                                        \
#     send-keys "rostopic echo /mavros/local_position/pose" \; split-window -v -p 80\; select-pane -t 7\; split-window -h\;\
#     send-keys "rostopic echo /mavros/vision_position/pose"\; select-pane -t 9\;                                         \
#     send-keys "rosrun depthai_publisher aruco_subscriber" \; split-window -v -p 10\;                                        \
#     send-keys "tmux kill-session" \;

# }

# export -f run349

# One were using for the aruco detector node instead of the depthai subscriber.

# function run349(){
#     echo "Starting tmux sessions for EGB349 UAV Control"
#     sleep 2
#     tmux new-session -s egh450\; set -g mouse on\; send-keys "roscore" C-m\; split-window -h -p 85\; split-window -v\;      \
#     send-keys "htop" C-m\; select-pane -t 2\; split-window -h\; select-pane -t 2\;                                          \
#     send-keys "ls -l" C-m\; split-window -v -p 75\;                                                                         \
#     send-keys "sleep 5; roslaunch ~/catkin_ws/launch/control.launch fcu_url:=/dev/ttyUSB0:921600"\; split-window -v -p 60\;                        \
#     send-keys "sleep 10; roslaunch qutas_lab_450 environment.launch"\; split-window -v \;                               \
#     send-keys "rosrun depthai_publisher dai_publisher" \; select-pane -t 6\;                                  \
#     send-keys "rosbag record -a" \; split-window -v -p 85\;                                                                        \
#     send-keys "rostopic echo /mavros/local_position/pose" \; split-window -v -p 80\; select-pane -t 7\; split-window -h\;\
#     send-keys "rostopic echo /mavros/vision_position/pose"\; select-pane -t 9\;                                         \
#     send-keys "rosrun egb349_group8_vision aruco_detector_node.py" \; split-window -v -p 10\;                                        \
#     send-keys "tmux kill-session" \;
# }

# export -f run349











function run349(){
    echo "Starting tmux sessions for EGB349 UAV Control"
    sleep 2
    tmux new-session -s egh450\; set -g mouse on\; send-keys "roscore" C-m\; split-window -h -p 85\; split-window -v\;      \
    send-keys "htop" C-m\; select-pane -t 2\; split-window -h\; select-pane -t 2\;                                          \
    send-keys "cd ~/EGB349_Project_eldrone/catkin_ws_G6; ls -l" C-m\; split-window -v -p 75\;                              \
    send-keys "roslaunch ~/EGB349_Project_eldrone/catkin_ws_G6/launch/control.launch fcu_url:=/dev/ttyUSB0:921600"\; split-window -v -p 60\;                        \
    send-keys "roslaunch qutas_lab_450 environment.launch"\; split-window -v \;                                  \
    send-keys "rosrun depthai_publisher dai_publisher" \; select-pane -t 6\;                                               \
    send-keys "rosbag record -a" \; split-window -v -p 85\;                                                               \
    send-keys "rostopic echo /mavros/local_position/pose" \; split-window -v -p 80\; select-pane -t 7\; split-window -h\; \
    send-keys "rostopic echo /mavros/vision_pose/pose"\; select-pane -t 9\;                                            \
    send-keys "rosrun egb349_group8_vision aruco_detector_node.py" \; split-window -v -p 10\;                              \
    send-keys "tmux kill-session" \;
}

export -f run349







# function run349(){
#     echo "Starting tmux sessions for EGB349 UAV Control"
#     sleep 2
#     tmux new-session -s egh450\; set -g mouse on\; send-keys "roscore" C-m\; split-window -h -p 85\; split-window -v\;      \
#     send-keys "htop" C-m\; select-pane -t 2\; split-window -h\; select-pane -t 2\;                                          \
#     send-keys "cd ~/EGB349_Project_eldrone/catkin_ws_G6; ls -l" C-m\; split-window -v -p 75\;                              \
#     send-keys "sleep 5; roslaunch ~/EGB349_Project_eldrone/catkin_ws_G6/launch/control.launch fcu_url:=/dev/ttyUSB0:921600"\; split-window -v -p 60\;                        \
#     send-keys "sleep 10; roslaunch qutas_lab_450 environment.launch"\; split-window -v \;                                  \
#     send-keys "rosrun depthai_publisher dai_publisher" \; select-pane -t 6\;                                               \
#     send-keys "rosbag record -a" \; split-window -v -p 85\;                                                               \
#     send-keys "rostopic echo /mavros/local_position/pose" \; split-window -v -p 80\; select-pane -t 7\; split-window -h\; \
#     send-keys "rostopic echo /mavros/vision_position/pose"\; select-pane -t 9\;                                            \
#     send-keys "rosrun depthai_publisher aruco_subscriber" \; split-window -v -p 10\;                              \
#     send-keys "tmux kill-session" \;
# }

# export -f run349


## BACKUP Aruco Subscriber Version: 

# function run349(){
#     echo "Starting tmux sessions for EGB349 UAV Control"
#     sleep 2
#     tmux new-session -s egh450\; set -g mouse on\; send-keys "roscore" C-m\; split-window -h -p 85\; split-window -v\;      \
#     send-keys "htop" C-m\; select-pane -t 2\; split-window -h\; select-pane -t 2\;                                          \
#     send-keys "ls -l" C-m\; split-window -v -p 75\;                                                                         \
#     send-keys "sleep 5; roslaunch ~/catkin_ws/launch/control.launch"\; split-window -v -p 60\;                        \
#     send-keys "sleep 10; roslaunch qutas_lab_450 environment.launch"\; split-window -v \;                               \
#     send-keys "rosrun depthai_publisher dai_publisher" \; select-pane -t 6\;                                  \
#     send-keys "rosbag record -a" \; split-window -v -p 85\;                                                                        \
#     send-keys "rostopic echo /mavros/local_position/pose" \; split-window -v -p 80\; select-pane -t 7\; split-window -h\;\
#     send-keys "rostopic echo /mavros/vision_position/pose"\; select-pane -t 9\;                                         \
#     send-keys "rosrun depthai_publisher aruco_subscriber" \; split-window -v -p 10\;                                        \
#     send-keys "tmux kill-session" \;
# }

# export -f run349