cmake_minimum_required(VERSION 3.10)
project(tinysim)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the include directories
include_directories(${CMAKE_SOURCE_DIR}/include)  # Assuming include directory is in the project root
# You need to do this in case you want to put .h files next to the .cpp files:
#include_directories(${CMAKE_SOURCE_DIR}/src/controls)  # Include control directory
#include_directories(${CMAKE_SOURCE_DIR}/src/parameters)  # Include parameters directory

# Specify the build directory
set(BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")

# Add all source files in the src directory
#file(GLOB SOURCES src/*.cpp)
#file(GLOB CONTROL_SOURCES src/controls/*.cpp)
#file(GLOB PARAMETER_SOURCES src/parameters/*.cpp)

# Append the source files from the subdirectories to the main SOURCES variable
#list(APPEND SOURCES ${CONTROL_SOURCES} ${PARAMETER_SOURCES})

# Add all source files in the src directory
file(GLOB SOURCES "src/*.cpp"
                  "src/parameters/*.cpp"
                  "src/physics/*.cpp"
                  "src/sensors/*.cpp"
                  "src/controls/*.cpp"
                  "src/filters/*.cpp"
                  "src/fusion/*.cpp"
                  "src/geometry/*.cpp")

# Create the executable
add_executable(tinysim ${SOURCES})

# Set the output directory
set_target_properties(tinysim PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${BUILD_DIR}
)

# Clean rule. This only make sense if you are not keeping it all under a specific folder.
add_custom_target(clean_files
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${BUILD_DIR}
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/Makefile
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/cmake_install.cmake
)