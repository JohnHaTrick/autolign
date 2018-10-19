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
#    std_msgs
#    auto_bridge
)

# http://docs.ros.org/indigo/api/catkin/html/howto/format2/installing_python.html
catkin_python_setup()

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

# https://cmake.org/cmake/help/v3.0/command/add_dependencies.html
#add_executable(node_autolign scripts/node_autolign.py)

# https://answers.ros.org/question/206257/catkin-use-ros-message-from-another-package/
#add_dependencies(node_autolign ${catkin_EXPORTED_TARGETS})

