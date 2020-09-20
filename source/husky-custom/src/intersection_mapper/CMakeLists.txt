cmake_minimum_required(VERSION 2.8.3)
project(intersection_mapper)

find_package(catkin REQUIRED COMPONENTS
  rospy 
  std_msgs
  message_generation
)

add_service_files(
  FILES
  singleRoute.srv
  recoveryRoutes.srv
)

add_message_files(
  FILES
  Routes.msg
  Routes2.msg
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

