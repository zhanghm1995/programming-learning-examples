file(GLOB_RECURSE SRC "*.cpp" )
message("geometry SRC is ----------------------------------------------------------------${SRC}")

add_executable(polygon2d_test ${SRC})
target_link_libraries(polygon2d_test glog)