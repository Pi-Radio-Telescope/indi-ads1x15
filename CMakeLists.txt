cmake_minimum_required(VERSION 3.10)

# define the project name
project(indi-ads1x15 C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

# add our cmake_modules folder
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# find our required packages
find_package(INDI REQUIRED)
find_package(Nova REQUIRED)
find_package(ZLIB REQUIRED)
find_package(GSL REQUIRED)
find_library(RT rt REQUIRED)

add_compile_options(-Wall -Wextra -Wno-shadow -Wpedantic -O3)


# these will be used to set the version number in config.h and our driver's xml file
set(CDRIVER_VERSION_MAJOR 0)
set(CDRIVER_VERSION_MINOR 1)

# do the replacement in the config.h
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

# do the replacement in the driver's xml file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_ads1x15.xml.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/indi_ads1x15.xml
)

# set our include directories to look for header files
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${NOVA_INCLUDE_DIR})
include_directories( ${EV_INCLUDE_DIR})

#include(CMakeCommon)

set(SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/i2cdevice.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ads1115.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ads1115_measurement.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/indi_ads1x15.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/connection_i2c.cpp"
)

set(HEADER_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/i2cdevice.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/ads1115.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/ads1115_measurement.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/indi_ads1x15.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/utility.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/connection_i2c.h"
)

# tell cmake to build our executable
add_executable(
    indi_ads1x15
	${SOURCE_FILES}
)

# and link it to these libraries
target_link_libraries(
    indi_ads1x15
    ${INDI_LIBRARIES}
    ${NOVA_LIBRARIES}
    ${GSL_LIBRARIES}
    rt
    pthread
)

# tell cmake where to install our executable
install(TARGETS indi_ads1x15 RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/indi_ads1x15.xml
    DESTINATION ${INDI_DATA_DIR}
)

add_subdirectory(config)
add_subdirectory(macros)
