cmake_minimum_required(VERSION 2.8.3)
project(mouse_reader)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MouseMotion.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mouse_reader
#  CATKIN_DEPENDS message_generation roscpp rospy std_msg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(mouse_reader src/mouse_reader.cpp)
target_link_libraries(mouse_reader ${catkin_LIBRARIES})
add_dependencies(mouse_reader ${catkin_EXPORTED_TARGETS})

add_executable(mouse_event_publisher src/mouse_event_publisher.cpp)
target_link_libraries(mouse_event_publisher ${catkin_LIBRARIES} mouse_reader)
add_dependencies(mouse_event_publisher mouse_reader_gencpp ${catkin_EXPORTED_TARGETS})
