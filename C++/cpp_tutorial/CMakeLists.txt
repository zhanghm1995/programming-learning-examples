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
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system program_options)
include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

add_subdirectory(argument_parse)
add_subdirectory(macro_test)
add_subdirectory(input_output_operations)
add_subdirectory(class_examples)
add_subdirectory(c++11)
add_subdirectory(stl)
add_subdirectory(utils)
add_subdirectory(logging)

#############
####Build####
#############
add_executable(base_conversion base_conversion.cpp)

# add_executable(c_test c_test.cpp )  
# target_link_libraries(c_test ${OpenCV_LIBS})

add_executable(anonymous_namespace_test include_test/anonymous_namespace_test.cpp
                                                                                                 include_test/test.cpp
                                                                                                 include_test/test1.cpp)

add_executable(boost_signal boost_signal.cpp)
target_link_libraries(boost_signal ${Boost_LIBRARIES})

add_executable(function_test function_test.cpp)

add_executable(shared_ptr_test shared_ptr_test.cpp)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

add_executable(time_utils_test time_utils_test.cpp)

add_executable(misc_learning misc_learning.cpp)

add_executable(numerical_calculation numerical_calculation.cpp)

add_executable(math_test math_test.cpp)

add_executable(basic_data_type basic_data_type.cpp)

add_executable(exception_learning exception_learning.cpp)

add_executable(random_generator random_generator.cpp)