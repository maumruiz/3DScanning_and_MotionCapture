cmake_minimum_required(VERSION 3.6)

# set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(surfaces)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs CACHE PATH "Path to lib folder")
set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/Eigen/ CACHE PATH "Path to Eigen source folder")

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)


# Define header and source files
set(HEADERS
    Eigen.h
    ImplicitSurface.h
    MarchingCubes.h
    Volume.h
)

set(SOURCES
    main.cpp
    Volume.cpp
)

add_executable(surfaces ${HEADERS} ${SOURCES})
target_include_directories(surfaces PUBLIC ${Eigen3_INCLUDE_DIR} ${FreeImage_INCLUDE_DIR})

# Visual Studio properties
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT surfaces)
set_property(TARGET surfaces PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
