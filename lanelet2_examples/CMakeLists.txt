# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 3.5.1)
project(lanelet2_examples)

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

###########
## Build ##
###########
file(GLOB SUBFOLDERS LIST_DIRECTORIES true "src/*")

find_package(lanelet2_core  REQUIRED)      
find_package(lanelet2_io  REQUIRED)   
find_package(lanelet2_projection  REQUIRED)      
find_package(lanelet2_routing  REQUIRED)      
find_package(lanelet2_matching  REQUIRED)      
find_package(lanelet2_traffic_rules  REQUIRED)  

# Iterate over the subfolders and print their names
foreach(SUBFOLDER IN LISTS SUBFOLDERS)
    if(IS_DIRECTORY ${SUBFOLDER})
        get_filename_component(SUBFOLDER_NAME ${SUBFOLDER} NAME)
        message("Subfolder: ${SUBFOLDER_NAME}")
        add_executable(${SUBFOLDER_NAME}
            src/${SUBFOLDER_NAME}/main.cpp
        )
        target_include_directories(${SUBFOLDER_NAME}  PUBLIC
            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
            "$<INSTALL_INTERFACE:include>")
        ament_target_dependencies(${SUBFOLDER_NAME} PUBLIC lanelet2_core)  
        ament_target_dependencies(${SUBFOLDER_NAME} PUBLIC lanelet2_io)  
        ament_target_dependencies(${SUBFOLDER_NAME} PUBLIC lanelet2_projection)  
        ament_target_dependencies(${SUBFOLDER_NAME} PUBLIC lanelet2_routing)  
        ament_target_dependencies(${SUBFOLDER_NAME} PUBLIC lanelet2_matching)  
        ament_target_dependencies(${SUBFOLDER_NAME} PUBLIC lanelet2_traffic_rules)  
        install(
            TARGETS ${SUBFOLDER_NAME}
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
        )
    endif()
endforeach()

#############
## Install ##
#############
# Install all targets, headers by default and scripts and other files if specified (folders or files).
# This command also exports libraries and config files for dependent packages and this supersedes catkin_package.
install(
    FILES scripts/tutorial.py
    DESTINATION lib/${PROJECT_NAME}  
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
