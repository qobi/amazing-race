cmake_minimum_required(VERSION 2.8.3)
project(husky_pur22_bringup)

find_package(catkin REQUIRED COMPONENTS
  axis_camera
  velodyne_driver
  velodyne_pointcloud
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES husky_pur22_bringup
#  CATKIN_DEPENDS axis_driver velodyne_driver velodyne_pointcloud
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

