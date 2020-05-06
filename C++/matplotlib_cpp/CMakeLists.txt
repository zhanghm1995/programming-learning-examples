cmake_minimum_required(VERSION 2.8)
project(test)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O0 -g")
set(CMAKE_BUILD_TYPE "Release")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(PythonLibs 2.7)
include_directories(${PYTHON_INCLUDE_DIRS})

add_executable(minimal_test examples/minimal.cpp)
target_link_libraries(minimal_test ${PYTHON_LIBRARIES})

add_executable(basic_test examples/basic.cpp)
target_link_libraries(basic_test ${PYTHON_LIBRARIES})

add_executable(modern_test examples/modern.cpp)
target_link_libraries(modern_test ${PYTHON_LIBRARIES})

add_executable(surface_test examples/surface.cpp)
target_link_libraries(surface_test ${PYTHON_LIBRARIES})