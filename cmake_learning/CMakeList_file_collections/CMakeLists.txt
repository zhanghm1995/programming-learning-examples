## This is a standard CMakeLists.txt template,including
## how to enable C++11 compiler option and how to set 'Release'
## target compile option

cmake_minimum_required(VERSION 2.8)
project(cmake_template)

############# Use message function to print information ############
message(STATUS "CMake version: ${CMAKE_VERSION}")

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


############ #enable warnings##################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall")


#############设置cmake目标编译选项##############
## Method 1
set(CMAKE_BUILD_TYPE "Release")

## Method 2 设置默认编译模式，并将设置结果显示在cmake GUI界面上
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

################## Some useful arguments ##########################
set(CMAKE_INCLUDE_CURRENT_DIR ON)

##########
##Build###
##########
add_library(lib hello.cpp)

add_executable(main main.cpp)
target_link_libraries(main ${LINK_LIBRARIES})


