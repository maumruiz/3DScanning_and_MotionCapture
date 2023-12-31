cmake_minimum_required(VERSION 3.6)

project(coarse_alignment)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs CACHE PATH "Path to lib folder")
set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/Eigen/ CACHE PATH "Path to Eigen source folder")

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)


# Define header and source files
set(HEADERS
    Eigen.h
    ProcrustesAligner.h
    SimpleMesh.h
)

set(SOURCES
    main.cpp
)

add_executable(coarse_alignment ${HEADERS} ${SOURCES})
target_include_directories(coarse_alignment PUBLIC ${Eigen3_INCLUDE_DIR} ${FreeImage_INCLUDE_DIR})

# Visual Studio properties
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT coarse_alignment)
set_property(TARGET coarse_alignment PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
