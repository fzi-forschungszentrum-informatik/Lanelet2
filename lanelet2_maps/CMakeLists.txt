# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 3.5.1)
project(lanelet2_maps)

###################
## Find packages ##
###################
find_package(ament_cmake REQUIRED)

############################
## Read source code files ##
############################


###########
## Build ##
###########
# Declare a cpp library
add_library(${PROJECT_NAME} INTERFACE   
    )

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY res/
  DESTINATION share/${PROJECT_NAME}/res
)

ament_package()