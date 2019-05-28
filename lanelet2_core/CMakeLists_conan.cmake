cmake_minimum_required(VERSION 2.8.12)
project(lanelet2_core)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0")

set(lanelet2_core_lib_name ${PROJECT_NAME})

############################
## read source code files ##
############################
file(GLOB_RECURSE lanelet2_core_source_list "${CMAKE_CURRENT_LIST_DIR}/src/**.cpp")

###########
## Build ##
###########
add_library(
    ${lanelet2_core_lib_name}
        ${lanelet2_core_source_list})
target_include_directories(
    ${lanelet2_core_lib_name}
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/include"
        "${CMAKE_CURRENT_LIST_DIR}/include/${lanelet2_core_lib_name}")
target_link_libraries(
    ${lanelet2_core_lib_name}
    PUBLIC
        CONAN_PKG::boost
        CONAN_PKG::eigen
        CONAN_PKG::geographiclib)
