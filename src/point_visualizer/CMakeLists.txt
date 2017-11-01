cmake_minimum_required(VERSION 2.8.3)
project(point_visualizer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  keyboard_reader
  mouse_reader
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES point_visualizer
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
#  include
#    ${catkin_INCLUDE_DIRS}
#)

add_executable(position_publisher src/position_publisher.cpp)
target_link_libraries(position_publisher ${catkin_LIBRARIES})
#add_dependencies(position_publisher point_visualizer_generate_messages_cpp)

add_executable(position_publisher_keyboard src/position_publisher_keyboard.cpp)
target_link_libraries(position_publisher_keyboard ${catkin_LIBRARIES})
#add_dependencies(position_publisher_keyboard point_visualizer_generate_messages_cpp)

add_executable(position_publisher_mouse src/position_publisher_mouse.cpp)
target_link_libraries(position_publisher_mouse ${catkin_LIBRARIES})


