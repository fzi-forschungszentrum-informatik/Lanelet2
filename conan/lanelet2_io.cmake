cmake_minimum_required(VERSION 2.8.12)
project(lanelet2_io)

set(lanelet2_io_lib_name ${PROJECT_NAME})

############################
## read source code files ##
############################
file(GLOB_RECURSE lanelet2_io_source_list "${CMAKE_CURRENT_LIST_DIR}/../lanelet2_io/src/**.cpp")

###########
## Build ##
###########
add_library(
    ${lanelet2_io_lib_name}
        ${lanelet2_io_source_list})
target_include_directories(
    ${lanelet2_io_lib_name}
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/../lanelet2_io/include"
        "${CMAKE_CURRENT_LIST_DIR}/../lanelet2_io/include/${lanelet2_io_lib_name}")
target_link_libraries(
    ${lanelet2_io_lib_name}
    PUBLIC
        CONAN_PKG::pugixml
        ${lanelet2_core_lib_name})
