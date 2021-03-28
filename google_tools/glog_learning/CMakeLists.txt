cmake_minimum_required (VERSION 3.0.2)
project (glog_learning VERSION 1.0)

find_package (glog REQUIRED)

add_executable (glog_basic glog_basic.cpp)
target_link_libraries (glog_basic glog)