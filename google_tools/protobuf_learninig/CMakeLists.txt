cmake_minimum_required(VERSION 2.8.12)
project(google_test)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")

#find packages
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)

#include headers
include_directories(${OpenCV_INCLUDE_DIRS})

#target generation
add_executable(main main.cpp student.pb.cc)

#link libraries
target_link_libraries(main ${OpenCV_LIBS} ${PROTOBUF_LIBRARY} pthread)
