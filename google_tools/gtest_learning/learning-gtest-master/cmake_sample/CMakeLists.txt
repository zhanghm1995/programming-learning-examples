cmake_minimum_required(VERSION 2.8.4)

project(cmake_sample)

set(GTEST_DIR "gtest")

add_subdirectory(${GTEST_DIR})

add_subdirectory(test)
