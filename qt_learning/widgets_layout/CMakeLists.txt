cmake_minimum_required(VERSION 2.8.12)
project(cmake_qt)

set(CMAKE_CXX_STANDARD 11)

set(QT_SOURCES
    main.cpp
    helloworld.cpp
    )
set(QT_HEADERS
    helloworld.h
    )
set(QT_UI_FILES
    helloworld.ui
    )

find_package(Qt5 REQUIRED COMPONENTS Core Xml OpenGL Gui Widgets)
include_directories(${Qt5Core_INCLUDE_DIRS}
                      ${Qt5Xml_INCLUDE_DIRS}
                      ${Qt5Gui_INCLUDE_DIRS}
                      ${Qt5Widgets_INCLUDE_DIRS}
                      ${Qt5OpenGL_INCLUDE_DIRS})
set(MY_QT_LIBRARIES
      ${Qt5Widgets_LIBRARIES}
      ${Qt5Core_LIBRARIES}
      ${Qt5Gui_LIBRARIES}
      ${Qt5Xml_LIBRARIES}
      ${Qt5OpenGL_LIBRARIES})

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_SOURCE_DIR})

message(STATUS "QT LIBS: ${MY_QT_LIBRARIES}")
      
if (Qt4_FOUND)
  QT4_WRAP_CPP(QT_HEADERS_MOC ${QT_HEADERS})
  QT4_WRAP_UI(QT_UI_FILES_MOC ${QT_UI_FILES})
elseif(Qt5_FOUND)
  QT5_WRAP_CPP(QT_HEADERS_MOC ${QT_HEADERS})
  QT5_WRAP_UI(QT_UI_FILES_MOC ${QT_UI_FILES})
else()
  message(FATAL_ERROR "Qt not found. So cannot moc.")
endif()

add_executable(helloworld
            ${QT_SOURCES}
            ${QT_UI_FILES_MOC}
            ${QT_HEADERS_MOC})
target_link_libraries(helloworld ${MY_QT_LIBRARIES})
