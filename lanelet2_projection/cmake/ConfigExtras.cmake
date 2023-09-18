# https://bugs.launchpad.net/ubuntu/+source/geographiclib/+bug/1805173
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
  set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
endif()