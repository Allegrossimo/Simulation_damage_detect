cmake_minimum_required(VERSION 3.0.2)
project(damage_detect)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  nav_msgs
)

## System dependencies
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport nav_msgs
  DEPENDS OpenCV Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Build the recorder node
add_executable(airsim_recorder_node
  src/airsim_recorder_node.cpp
)

target_link_libraries(airsim_recorder_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/defect_detector_node.py
  scripts/tracker.py
  scripts/utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install files
install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".git" EXCLUDE
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".git" EXCLUDE
)
