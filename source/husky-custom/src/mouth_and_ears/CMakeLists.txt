cmake_minimum_required(VERSION 2.8.3)
project(mouth_and_ears)

find_package(catkin REQUIRED rospy std_msgs)

catkin_package(
  LIBRARIES start_mouth_and_ears
  CATKIN_DEPENDS rospy std_msgs
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

