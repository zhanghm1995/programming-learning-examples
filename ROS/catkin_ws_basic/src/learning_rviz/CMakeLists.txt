cmake_minimum_required(VERSION 2.8.3)
project(learning_rviz)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#  INCLUDE_DIRS include
#  LIBRARIES learning_tf
#  CATKIN_DEPENDS roscpp rospy tf turtlesim
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

add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes
  ${catkin_LIBRARIES})
  
add_executable(points_and_lines src/points_and_lines.cpp)
target_link_libraries(points_and_lines
  ${catkin_LIBRARIES})
  
add_executable(path_plot src/path_plot.cpp)
target_link_libraries(path_plot
  ${catkin_LIBRARIES})
 
add_executable(path_plot2 src/path_plot2.cpp)
target_link_libraries(path_plot2
  ${catkin_LIBRARIES})
 
add_executable(path_multi_plot src/path_multi_plot.cpp)
target_link_libraries(path_multi_plot
  ${catkin_LIBRARIES})
