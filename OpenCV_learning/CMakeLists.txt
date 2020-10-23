cmake_minimum_required(VERSION 2.8)
project(test)
if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic-errors --std=gnu++14" CACHE STRING COMPILE_FLAGS FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -march=native -mtune=native --fast-math -ffast-math -funroll-loops -Wall -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -march=native -mtune=native -Wall -DDEBUG" CACHE STRING COMPILE_FLAGS FORCE)
elseif (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /W4 -DGTL_STATIC" CACHE STRING COMPILE_FLAGS FORCE)
endif()
#set(OpenCV_DIR "/usr/local/opencv3/share/OpenCV")
# set(OpenCV_DIR "/usr/share/OpenCV")

set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV_DIR = ${OpenCV_DIR}")
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")

include_directories(${OPENCV_INCLUDE_DIRS}) 

add_executable(opencv_misc opencv_misc.cpp)
target_link_libraries(opencv_misc ${OpenCV_LIBS})

add_executable(optical_flow optical_flow.cpp)  
target_link_libraries(optical_flow ${OpenCV_LIBS})

add_executable(LK_track LK_track.cpp)  
target_link_libraries(LK_track ${OpenCV_LIBS})

add_executable(invisible_cloak invisible_cloak.cpp)  
target_link_libraries(invisible_cloak ${OpenCV_LIBS})

add_executable(image_mask_test image_mask_test.cpp)  
target_link_libraries(image_mask_test ${OpenCV_LIBS})

add_executable(polygon_drawing polygon_drawing.cpp)  
target_link_libraries(polygon_drawing ${OpenCV_LIBS})

add_executable(color_handle color_handle.cpp)  
target_link_libraries(color_handle ${OpenCV_LIBS})

add_executable(image_operations image_operations.cpp)  
target_link_libraries(image_operations ${OpenCV_LIBS})

add_executable(mat_learning mat_learning.cpp)  
target_link_libraries(mat_learning ${OpenCV_LIBS})

add_subdirectory(basic_drawing)
add_subdirectory(utils)