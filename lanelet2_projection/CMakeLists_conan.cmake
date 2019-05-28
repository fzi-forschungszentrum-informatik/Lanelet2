cmake_minimum_required(VERSION 2.8.12)
project(lanelet2_projection)

set(lanelet2_projection_lib_name ${PROJECT_NAME})

############################
## read source code files ##
############################
file(GLOB_RECURSE lanelet2_projection_source_list "${CMAKE_CURRENT_LIST_DIR}/src/**.cpp")

###########
## Build ##
###########
add_library(
    ${lanelet2_projection_lib_name}
        ${lanelet2_projection_source_list})
target_include_directories(
    ${lanelet2_projection_lib_name}
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/include"
        "${CMAKE_CURRENT_LIST_DIR}/include/${lanelet2_projection_lib_name}")
target_link_libraries(
    ${lanelet2_projection_lib_name}
    PUBLIC
        ${lanelet2_io_lib_name})
