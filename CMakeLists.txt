cmake_minimum_required(VERSION 3.0)
project(autolign)

## Compiler flags
# none needed yet?

## Find catkin macros and libraries
find_package(catkin REQUIRED
  genmsg  
  roscpp
  rospy
  roslib
  std_msgs
  message_generation
  auto_bridge
  )

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
    FILES
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    auto_bridge
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
    CATKIN_DEPENDS std_msgs message_runtime auto_bridge
)
