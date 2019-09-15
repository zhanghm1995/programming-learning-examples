cmake_minimum_required(VERSION 2.8.12)
project(google_test)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")

IF(IS_DIRECTORY "/usr/src/gtest/")
  MESSAGE(STATUS "Found google test sources in /usr/src/gtest/")

  ADD_SUBDIRECTORY(/usr/src/gtest/ gtest)

  # mimick the behaviour of find_package(GTest)
  SET(GTEST_FOUND TRUE)
  SET(GTEST_BOTH_LIBRARIES gtest gtest_main)
  SET(GTEST_LIBRARIES gtest)
  SET(GTEST_MAIN_LIBRARIES gtest_main)
ELSE()
  find_package(GTest)
ENDIF()

#find packages
find_package(OpenCV REQUIRED)

#include headers
include_directories(${OpenCV_INCLUDE_DIRS})

#target generation
add_executable(main main.cpp)

#link libraries
target_link_libraries(main ${OpenCV_LIBS} ${GTEST_MAIN_LIBRARIES} pthread)
