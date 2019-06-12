cmake_minimum_required(VERSION 2.8.12)
project(lanelet2_validation)

set(lanelet2_validation_lib_name ${PROJECT_NAME})

############################
## read source code files ##
############################
file(GLOB_RECURSE lanelet2_validation_source_list "${CMAKE_CURRENT_LIST_DIR}/../lanelet2_validation/src/**.cpp")

###########
## Build ##
###########
add_library(
    ${lanelet2_validation_lib_name}
        ${lanelet2_validation_source_list})
target_include_directories(
    ${lanelet2_validation_lib_name}
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/../lanelet2_validation/include"
        "${CMAKE_CURRENT_LIST_DIR}/../lanelet2_validation/include/${lanelet2_validation_lib_name}")
target_link_libraries(
    ${lanelet2_validation_lib_name}
    PUBLIC
        ${lanelet2_projection_lib_name}
        ${lanelet2_routing_lib_name})
