cmake_minimum_required(VERSION 3.0.2)

project(spar_node)


# ============================================================
# Compiler Configuration
# ============================================================

add_compile_options(-std=c++11)


# ============================================================
# Catkin Dependencies
# ============================================================

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  sensor_msgs
  spar_msgs
  std_msgs
)


# ============================================================
# Catkin Package
# ============================================================

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    mavros_msgs
    roscpp
    rospy
    sensor_msgs
    spar_msgs
    std_msgs
)


# ============================================================
# Include Directories
# ============================================================

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# ============================================================
# SPAR Library
# ============================================================

add_library(${PROJECT_NAME}_spar
  src/${PROJECT_NAME}/spar.cpp
)

add_dependencies(
  ${PROJECT_NAME}_spar
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_spar
  ${catkin_LIBRARIES}
)


# ============================================================
# SPAR Node Executable
# ============================================================

add_executable(
  ${PROJECT_NAME}
  src/spar_node.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_spar
)


# ============================================================
# Python Scripts
# ============================================================
#
# The original scripts are retained for reference / rollback
# while the new consolidated guidance node is commissioned.
#

catkin_install_python(
  PROGRAMS

    scripts/demo_wp
    scripts/goto
    scripts/takeoff
    scripts/land

    # New consolidated autopilot / guidance node
    scripts/guidance

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# ============================================================
# Install Compiled SPAR Node
# ============================================================

install(
  TARGETS ${PROJECT_NAME}

  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# ============================================================
# Install Launch and Configuration Files
# ============================================================
#
# This installs:
#
#   launch/spar.launch
#   launch/guidance.launch
#   launch/demo_wp_roi.launch (retained for reference)
#
# and:
#
#   config/guidance.yaml
#

install(
  DIRECTORY
    launch
    config

  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)