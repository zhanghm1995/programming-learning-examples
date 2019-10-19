cmake_minimum_required(VERSION 2.8.4)

include_directories(${gtest_SOURCE_DIR}/include)

set(TEST_SOURCES sample1_unittest.cc sample1.cc)

add_executable(sample_tests ${TEST_SOURCES})

target_link_libraries(sample_tests gtest_main)
