cmake_minimum_required(VERSION 2.8.3)
project(door_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  doorCoords.srv
  doorText.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

