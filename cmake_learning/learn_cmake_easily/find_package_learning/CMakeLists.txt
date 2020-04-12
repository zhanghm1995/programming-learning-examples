cmake_minimum_required(VERSION 2.8)
project(find_package_learning)

# set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 3 REQUIRED)

message(STATUS "OpenCV_DIR = ${OpenCV_DIR}")
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

message(STATUS "CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")

include_directories(${OPENCV_INCLUDE_DIRS})  
add_executable(opencv_test opencv_test.cpp)  
target_link_libraries(opencv_test ${OpenCV_LIBS})