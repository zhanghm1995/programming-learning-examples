cmake_minimum_required(VERSION 2.8.3)
project(ros_misc_test)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -O1 -std=c++11")
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
  message_generation
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.9 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system program_options)
find_package(OpenCV REQUIRED)


add_definitions(${PCL_DEFINITIONS})
###################################
## catkin specific configuration ##
###################################

add_message_files(
  FILES
  TestMsg.msg
)  

#generate_messages必须在catkin_package前面
generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_handle_node
CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime
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

add_executable(image_subscriber_publisher src/image_subscriber_publisher.cpp)
target_link_libraries(image_subscriber_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(topic_callback_test src/topic_callback_test.cpp)
target_link_libraries(topic_callback_test ${catkin_LIBRARIES})

add_executable(ros_basic_tutorial src/ros_basic_tutorial.cpp)
target_link_libraries(ros_basic_tutorial ${catkin_LIBRARIES})

add_executable(message_definition src/message_definition.cpp)
target_link_libraries(message_definition ${catkin_LIBRARIES})

add_executable(occupancy_grid_map src/occupancy_grid_map.cpp)
target_link_libraries(occupancy_grid_map ${catkin_LIBRARIES})

add_executable(customized_point_cloud_operation src/customized_point_cloud_operation.cpp)
target_link_libraries(customized_point_cloud_operation ${catkin_LIBRARIES})

add_executable(point_cloud_reader src/point_cloud_reader.cpp)
target_link_libraries(point_cloud_reader ${catkin_LIBRARIES})

add_executable(point_cloud_subscriber src/point_cloud_subscriber.cpp)
target_link_libraries(point_cloud_subscriber ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(srv_server src/srv_server.cpp)
target_link_libraries(srv_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(srv_client src/srv_client.cpp)
target_link_libraries(srv_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})