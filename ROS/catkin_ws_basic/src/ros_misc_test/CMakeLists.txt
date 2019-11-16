cmake_minimum_required(VERSION 2.8.3)
project(ros_misc_test)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -O1 -std=gnu++0x")
set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
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
CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

###########
## Build ##
###########
add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(cloud_subscriber_publisher src/cloud_subscriber_publisher.cpp)
target_link_libraries(cloud_subscriber_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
