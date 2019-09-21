cmake_minimum_required(VERSION 2.8.3)
project(high_level_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  gazebo_msgs
  geometry_msgs
  mavros
  default_cfg_fkie
  master_discovery_fkie
  master_sync_fkie
  multimaster_msgs_fkie
  node_manager_fkie
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PythonInterp REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MavAction.msg
  MissionState.msg
  SubregionData.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GuiPathData.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  mavros_msgs
  gazebo_msgs
  multimaster_msgs_fkie
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES high_level_control
  CATKIN_DEPENDS gazebo_msgs geometry_msgs mavros roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/high_level_control.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(high_level_controller src/high_level_runner.cpp src/high_level_control_interface.cpp)
add_executable(ground_station src/ground_station_runner.cpp src/ground_station.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(high_level_controller ${catkin_LIBRARIES})
target_link_libraries(ground_station ${catkin_LIBRARIES})
