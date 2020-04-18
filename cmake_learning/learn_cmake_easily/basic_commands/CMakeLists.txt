cmake_minimum_required(VERSION 2.8)
project(basic_commands)

###############################################
##############常用变量总结######################
###############################################
message(STATUS "---------------------------------------------------------------------------")
message(STATUS "CMAKE_INSTALL_PREFIX ===== ${CMAKE_INSTALL_PREFIX}")

## 由GNUInstallDirs模块定义的变量
include(GNUInstallDirs)
message(STATUS "CMAKE_INSTALL_INCLUDEDIR ===== ${CMAKE_INSTALL_INCLUDEDIR}")
message(STATUS "CMAKE_INSTALL_DATADIR ===== ${CMAKE_INSTALL_DATADIR}")
message(STATUS "CMAKE_INSTALL_LIBDIR ===== ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "CMAKE_INSTALL_BINDIR ===== ${CMAKE_INSTALL_BINDIR}")
message(STATUS "---------------------------------------------------------------------------")

###############################################
##############常用函数总结######################
###############################################
## 文件操作 https://cmake.org/cmake/help/v3.0/command/get_filename_component.html
get_filename_component(FOOBAR_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
message(STATUS "FOOBAR_CMAKE_DIR ===== ${FOOBAR_CMAKE_DIR}")