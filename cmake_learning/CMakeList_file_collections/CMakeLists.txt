cmake_minimum_required(VERSION 2.8)
project(Homography)
############ enable C++11 compiler###############
## Method 1
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(WARNING "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
    message(STATUS "Please use a different C++ compiler.")
endif()
## Method 2
set(CMAKE_CXX_STANDARD 11)

#############设置cmake目标编译选项##############
set(CMAKE_BUILD_TYPE "Release")
###########设置默认编译模式，并将设置结果显示在GUI界面上
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)
###########设置默认编译模式，并将设置结果显示在GUI界面上

# enable warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall")
