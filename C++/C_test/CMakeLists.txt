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

# set(OpenCV_DIR "/usr/local/opencv3/share/OpenCV")
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system program_options)
include_directories(${OPENCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

add_subdirectory(macro_test)
#############
####Build####
#############
add_executable(base_conversion base_conversion.cpp)

add_executable(c_test c_test.cpp )  
target_link_libraries(c_test ${OpenCV_LIBS})

add_executable(class_learning class_learning.cpp)

add_executable(anonymous_namespace_test include_test/anonymous_namespace_test.cpp
                                                                                                 include_test/test.cpp
                                                                                                 include_test/test1.cpp)

add_executable(stl_learning STL_learning.cpp)  
target_link_libraries(stl_learning ${OpenCV_LIBS}) 

add_executable(stl_test stl_test.cpp)

add_executable(stl_algorithm STL_algorithm.cpp)

add_executable(string_usages string_usages.cpp)

add_executable(data_structure data_structure.cpp)  
target_link_libraries(data_structure ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(read_file read_file.cpp)
  
add_executable(boost_signal boost_signal.cpp)
target_link_libraries(boost_signal ${Boost_LIBRARIES})

add_executable(function_test function_test.cpp)

add_executable(shared_ptr_test shared_ptr_test.cpp)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
