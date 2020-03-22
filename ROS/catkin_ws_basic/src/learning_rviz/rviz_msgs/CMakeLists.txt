cmake_minimum_required(VERSION 2.8.3)
project(rviz_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -O1 -std=gnu++0x")
set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

add_message_files(
  FILES
  Object.msg
  ObjectArray.msg
)

#generate_messages必须在catkin_package前面
generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_handle_node
CATKIN_DEPENDS roscpp geometry_msgs std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)