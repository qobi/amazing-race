cmake_minimum_required(VERSION 2.8.3)
project(follow_directions)

find_package(catkin REQUIRED 
  rospy 
  std_msgs
  message_generation
)

add_service_files(
  FILES
  directionsContent.srv
  commandNode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES 
  CATKIN_DEPENDS rospy std_msgs message_runtime
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

