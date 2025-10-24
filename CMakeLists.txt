cmake_minimum_required(VERSION 3.15)
project(PlatinumNavigation VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -pedantic)
endif()

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Source files
set(NAVIGATION_SOURCES
    src/navigation/config.cpp
    src/navigation/buoy_detection.cpp
    src/navigation/advanced_algorithms.cpp
    src/navigation/dynamic_kdtree.cpp
    src/navigation/dynamic_navigation.cpp
    src/navigation/aimm_icc_competition.cpp
)

# Create library
add_library(navigation_lib STATIC ${NAVIGATION_SOURCES})
target_include_directories(navigation_lib PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Main executable
add_executable(platinum_navigation src/main.cpp)
target_link_libraries(platinum_navigation PRIVATE navigation_lib)

# Installation
install(TARGETS platinum_navigation DESTINATION bin)
install(TARGETS navigation_lib DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Testing (optional, can be extended)
enable_testing()
add_test(NAME BasicRun COMMAND platinum_navigation)

# Build information
message(STATUS "Building Platinum Navigation System")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER_ID}")
