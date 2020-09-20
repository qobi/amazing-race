cmake_minimum_required(VERSION 2.8.3)
project(husky_pur22_description)

find_package(catkin REQUIRED COMPONENTS
  husky_description
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES husky_pur22_description
#  CATKIN_DEPENDS husky_description
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
