# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 3.5.1)
project(lanelet2_matching)

###################
## Find packages ##
###################
find_package(ament_cmake REQUIRED)

# You can add a custom.cmake in order to add special handling for this package. E.g. you can do:
# list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
# To remove libs which cannot be found automatically. You can also "find_package" other, custom dependencies there.
# You can also set PROJECT_INSTALL_FILES to install files that are not installed by default.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
endif()


############################
## Read source code files ##
############################
file(GLOB_RECURSE PROJECT_SOURCE_FILES_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

###########
## Build ##
###########
# Declare a cpp library
add_library(${PROJECT_NAME}    
    ${PROJECT_SOURCE_FILES_SRC}
    )
target_include_directories(${PROJECT_NAME}  
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")

find_package(lanelet2_core  REQUIRED)      
ament_target_dependencies(${PROJECT_NAME} PUBLIC lanelet2_core)          

find_package(lanelet2_traffic_rules  REQUIRED)      
ament_target_dependencies(${PROJECT_NAME} PUBLIC lanelet2_traffic_rules)    
#############
## Install ##
#############
install(
  DIRECTORY include/lanelet2_matching
  DESTINATION include/
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############
#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB TEST_SOURCES "test/*.cpp")
  # Add ament_gtest for your test file
  foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    ament_add_gtest(test_${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(test_${TEST_NAME}
      ${PROJECT_NAME}
      ${ament_LIBRARIES}
    )
    find_package(lanelet2_io  REQUIRED)      
    ament_target_dependencies(test_${TEST_NAME} lanelet2_io)   
    find_package(lanelet2_projection  REQUIRED)      
    ament_target_dependencies(test_${TEST_NAME} lanelet2_projection)  
  endforeach()
endif()

ament_package()
