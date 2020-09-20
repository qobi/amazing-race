cmake_minimum_required(VERSION 2.8.3)
project(rviz_marker_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

