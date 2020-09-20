cmake_minimum_required(VERSION 2.8.3)
project(my_axis_camera)

find_package(catkin REQUIRED
  COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_generation
  rospy
  std_msgs
)

# ROS message generation
add_message_files(FILES Axis.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
generate_dynamic_reconfigure_options(cfg/PTZ.cfg)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS rospy std_msgs
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  nodes/axis.py
  nodes/axis_ptz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



