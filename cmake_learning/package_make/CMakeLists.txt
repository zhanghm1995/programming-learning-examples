cmake_minimum_required(VERSION 3.4)
project(FooBar)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(EXAMPLE_VERSION_MAJOR 0)
set(EXAMPLE_VERSION_MINOR 1)
set(EXAMPLE_VERSION_PATCH 0)
set(EXAMPLE_VERSION_STRING ${EXAMPLE_VERSION_MAJOR}.${EXAMPLE_VERSION_MINOR}.${EXAMPLE_VERSION_PATCH})

include_directories(include)
add_library(${PROJECT_NAME} SHARED
    src/foo_bar.cpp
    )

set(ConfigPackageLocation "${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME}")

set(CONFIG_INCLUDE_DIRS "\${FooBar_CMAKE_DIR}/include")
configure_package_config_file(
    ${PROJECT_NAME}Config.cmake.in
    ${PROJECT_NAME}Config.cmake                                            
    INSTALL_DESTINATION "${ConfigPackageLocation}"                       
    PATH_VARS CMAKE_INSTALL_PREFIX
    )
    
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${EXAMPLE_VERSION_STRING}
    COMPATIBILITY AnyNewerVersion
    )

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # This is for Windows
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    )

install(
    DIRECTORY include/ 
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
    )

install(EXPORT ${PROJECT_NAME}Targets
  FILE
    ${PROJECT_NAME}Targets.cmake
  DESTINATION
    ${ConfigPackageLocation}
)

install(
  FILES
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
)

