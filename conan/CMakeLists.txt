cmake_minimum_required(VERSION 2.8.12)
project(lanelet2_conan)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Make a redundant call of `conan install` to suport CMake-first environments eg IDEs.
execute_process(COMMAND
    conan install ${CMAKE_SOURCE_DIR} --install-folder ${CMAKE_BINARY_DIR} --build=missing --generator=cmake)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_core.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_io.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_projection.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_traffic_rules.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_routing.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_validation.cmake")
