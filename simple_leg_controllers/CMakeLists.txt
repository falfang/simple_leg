cmake_minimum_required(VERSION 3.0.2)
project(simple_leg_controllers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hardware_interface
  controller_interface
  controller_manager
  pluginlib
  simple_leg_msgs
  angles
  gazebo_ros_control
  joint_limits_interface
)

find_package(gazebo REQUIRED) # Need for build RobotHWSim


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
  INCLUDE_DIRS include
  LIBRARIES simple_leg_controllers
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_interface
    hardware_interface 
    pluginlib
    angles
    gazebo_ros_control
    joint_limits_interface
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
  ${GAZEBO_INCLUDE_DIRS}
)

# simple_leg_controllers
add_library(simple_leg_controllers src/leg_position_controller.cpp)
add_dependencies(simple_leg_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_leg_controllers ${catkin_LIBRARIES})


add_subdirectory(src/simple_leg)