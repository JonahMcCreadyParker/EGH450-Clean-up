cmake_minimum_required(VERSION 3.0.2)
project(egb349_group8_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/aruco_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)