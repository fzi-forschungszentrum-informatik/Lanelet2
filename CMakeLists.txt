cmake_minimum_required(VERSION 2.8.12)
project(lanelet2_conan)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_core/CMakeLists_conan.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_io/CMakeLists_conan.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_projection/CMakeLists_conan.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_traffic_rules/CMakeLists_conan.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_routing/CMakeLists_conan.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_validation/CMakeLists_conan.cmake")
