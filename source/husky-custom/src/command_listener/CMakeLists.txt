cmake_minimum_required(VERSION 2.8.3)
project(command_listener)

find_package(catkin REQUIRED rospy std_msgs)

catkin_package(
  LIBRARIES start_command_listener
  CATKIN_DEPENDS rospy std_msgs
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

