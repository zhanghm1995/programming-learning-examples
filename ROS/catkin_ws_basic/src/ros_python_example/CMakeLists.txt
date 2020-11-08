cmake_minimum_required(VERSION 2.8.3)
project(ros_python_example)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -O1 -std=gnu++0x")
set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.9 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system program_options)
find_package(OpenCV REQUIRED)


add_definitions(${PCL_DEFINITIONS})
###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_handle_node
CATKIN_DEPENDS  roscpp sensor_msgs std_msgs rospy
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

###########
## Build ####
###########
# add_executable(test_node src/test_node.cpp)
# target_link_libraries(test_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

catkin_install_python(PROGRAMS scripts/python_publisher.py
                                              DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})