cmake_minimum_required(VERSION 3.0.2)
project(dwr_rrp_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  gazebo_ros
  gazebo_ros_control
)

## catkin_package() must be called before any install() that uses CATKIN_*_DESTINATION variables
catkin_package()

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
   scripts/dwr_rrp_controller.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (launch, config, worlds, models)
install(DIRECTORY models worlds config launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
