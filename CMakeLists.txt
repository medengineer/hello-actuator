cmake_minimum_required(VERSION 3.10)
project(ActuatorDemo VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add executable
add_executable(actuator_demo
    src/actuator.cpp
    tests/main.cpp
)

# Add include directories
target_include_directories(actuator_demo
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Copy calibration file to build directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/data/calibration.csv
              ${CMAKE_CURRENT_BINARY_DIR}/calibration.csv
              COPYONLY)

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Add compiler warnings
if(MSVC)
    target_compile_options(actuator_demo PRIVATE /W4)
else()
    target_compile_options(actuator_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Optional: Enable testing
option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif() 