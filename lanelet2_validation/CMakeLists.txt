# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 3.5.1)
project(lanelet2_validation)

###################
## Find packages ##
###################
find_package(ament_cmake REQUIRED)
find_package(Boost COMPONENTS program_options)

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
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Boost::program_options)     
target_include_directories(${PROJECT_NAME}  
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")

find_package(lanelet2_core  REQUIRED)      
find_package(lanelet2_io  REQUIRED)    
find_package(lanelet2_traffic_rules  REQUIRED)    
find_package(lanelet2_routing  REQUIRED)    
find_package(lanelet2_projection  REQUIRED)    
ament_target_dependencies(${PROJECT_NAME} 
    PUBLIC lanelet2_core lanelet2_io lanelet2_traffic_rules lanelet2_routing lanelet2_projection)  

#############
## Install ##
#############
install(
  DIRECTORY include/lanelet2_validation
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
  endforeach()
endif()

ament_package()