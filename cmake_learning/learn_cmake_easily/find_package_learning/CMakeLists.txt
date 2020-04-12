cmake_minimum_required(VERSION 2.8)
project(find_package_learning)
find_package(OpenCV 3 REQUIRED)

include_directories(${OPENCV_INCLUDE_DIRS})  
add_executable(opencv_test opencv_test.cpp)  
target_link_libraries(opencv_test ${OpenCV_LIBS})