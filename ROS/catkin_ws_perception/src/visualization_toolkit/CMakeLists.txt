cmake_minimum_required(VERSION 3.0.2)
project(visualization_toolkit)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -O1 -std=gnu++0x")
set(CMAKE_BUILD_TYPE "Release")

set(PACKAGE_DEPENDENCIES
  sensor_msgs
  std_msgs
  roscpp
  roslib
  iv_object_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.9 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system program_options)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
   src
   ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} 
   ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR}
)

set (THIRD_PARTY_LIB ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})
###########
## Build ##
###########

add_executable(object_visualizer_node src/app/object_visualizer_node.cpp src/rviz_3d_object_visualizer.cpp)
target_link_libraries(object_visualizer_node ${catkin_LIBRARIES} ${THIRD_PARTY_LIB})
