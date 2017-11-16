cmake_minimum_required(VERSION 2.8.3)
project(planning_tutorial)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  moveit_ros_planning_interface
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning_tutorial
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(planning_tutorial src/planning_tutorial.cpp)
target_link_libraries(planning_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(sia5d_moving src/sia5d_moving.cpp)
target_link_libraries(sia5d_moving ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS sia5d_moving DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
