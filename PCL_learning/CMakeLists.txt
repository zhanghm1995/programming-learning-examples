cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCL_Learning)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SRC_LIST main.cpp hdl_grabber.h hdl_grabber.cpp)
add_executable(ICP registration.cpp)
target_link_libraries(ICP ${PCL_LIBRARIES})

add_executable(ICP_incremental registration_continues.cpp)
target_link_libraries(ICP_incremental ${PCL_LIBRARIES})

add_executable(pcl_transformation pcl_transformation.cpp)
target_link_libraries(pcl_transformation ${PCL_LIBRARIES})

add_executable(cloud_clustering cloud_clustering.cpp)
target_link_libraries(cloud_clustering ${PCL_LIBRARIES})


add_executable(euclidean_clustering euclidean_clustering.cpp)
target_link_libraries(euclidean_clustering ${PCL_LIBRARIES})

add_executable(pcl_visualizer_demo pcl_visualizer_demo.cpp)
target_link_libraries(pcl_visualizer_demo ${PCL_LIBRARIES})

add_executable(statistical_removal statistical_removal.cpp)
target_link_libraries(statistical_removal ${PCL_LIBRARIES})

add_executable(cloud_extraction cloud_extraction.cpp)
target_link_libraries(cloud_extraction ${PCL_LIBRARIES})

add_executable(pcl_template pcl_template.cpp)
target_link_libraries(pcl_template ${PCL_LIBRARIES})

add_subdirectory(customized_point_type)
