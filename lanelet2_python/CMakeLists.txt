# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 3.12)    # for find_package(Python ...)
project(lanelet2_python)

###################
## Find packages ##
###################
find_package(ament_cmake REQUIRED)
find_package(Boost COMPONENTS python)
find_package(Python COMPONENTS Development)
include_directories(${Python_INCLUDE_DIRS})

########################
## Add python modules ##
########################
# This adds a python module if located under src/{PROJECT_NAME)
file(GLOB_RECURSE PROJECT_SOURCE_FILES_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/python_api/*.cpp
)

find_package(lanelet2_core  REQUIRED)      
find_package(lanelet2_io  REQUIRED)      
find_package(lanelet2_projection  REQUIRED)      
find_package(lanelet2_routing  REQUIRED)      
find_package(lanelet2_matching  REQUIRED)      
find_package(lanelet2_traffic_rules  REQUIRED)      

foreach(PROJECT_SOURCE_FILE ${PROJECT_SOURCE_FILES_SRC})
    get_filename_component(SOURCE_NAME ${PROJECT_SOURCE_FILE} NAME_WE)
    add_library(${SOURCE_NAME} ${PROJECT_SOURCE_FILE})
    # prevent cmake from creating a "lib" prefix
    set_target_properties(${SOURCE_NAME} PROPERTIES PREFIX "")
    target_link_libraries(${SOURCE_NAME}
      ${ament_LIBRARIES}
    )
    target_link_libraries(${SOURCE_NAME} PRIVATE 
        Boost::python) 
    target_include_directories(${SOURCE_NAME} PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
    ament_target_dependencies(${SOURCE_NAME} PUBLIC lanelet2_core)  
    ament_target_dependencies(${SOURCE_NAME} PUBLIC lanelet2_io)  
    ament_target_dependencies(${SOURCE_NAME} PUBLIC lanelet2_projection)  
    ament_target_dependencies(${SOURCE_NAME} PUBLIC lanelet2_routing)  
    ament_target_dependencies(${SOURCE_NAME} PUBLIC lanelet2_matching)  
    ament_target_dependencies(${SOURCE_NAME} PUBLIC lanelet2_traffic_rules)  
    install(
        TARGETS ${SOURCE_NAME}
        LIBRARY DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/lanelet2
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
    )
endforeach()


#############
## Install ##
#############
install(
    FILES
        scripts/create_debug_routing_graph.py
        scripts/make_ids_positive.py
        scripts/print_ids.py
    DESTINATION lib/${PROJECT_NAME}
)
install(
    FILES __init__.py
    DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/lanelet2/  #same folder as the *.so files
)

#############
## Testing ##
#############
# Add test targets for python tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  file(GLOB TEST_SOURCES "test/*.py")
  # Add ament_gtest for your test file
  foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    ament_add_gtest(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME}
      ${ament_LIBRARIES}
    )
  endforeach()
endif()

ament_package()
