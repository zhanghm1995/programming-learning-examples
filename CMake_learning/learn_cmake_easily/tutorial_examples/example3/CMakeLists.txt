cmake_minimum_required(VERSION 3.4)
project(example3)

include_directories(include)
add_executable(example3 src/main.cpp src/math.cpp src/utils/utils.cpp)