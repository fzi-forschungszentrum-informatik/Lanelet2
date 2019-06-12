## Conan usage

### Building

`conan create path/to/this/conan/dir my_user/my_channel`

### Publishing

`conan push --remote my_remote --all lanelet2/my_version@my_user/my_channel`

### Using

#### In an ordinary cmake project

```cmake
execute_process(COMMAND
    conan install ${CMAKE_SOURCE_DIR} --install-folder ${CMAKE_BINARY_DIR} --build=missing --generator=cmake)
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

add_executable(myexec main.cxx)
target_include_directories(myexec
    PUBLIC ...)
target_link_libraries(myexec
    PUBLIC
        CONAN_PKG::spdlog
        CONAN_PKG::yaml-cpp)
```

#### In a ros project

In src/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 2.8.3)
execute_process(COMMAND
    conan install ${CMAKE_SOURCE_DIR} --install-folder "/opt/my_org/share/cmake/Modules" --build=missing --generator=cmake_find_package)
include(/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake)
```

In each ros package's CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(myrospack)

list(APPEND CMAKE_MODULE_PATH "/opt/my_org/share/cmake/Modules")

find_package(spdlog REQUIRED)

add_executable(myexec main.cxx)
target_include_directories(myexec
    PUBLIC ...)
target_link_libraries(myexec
    PUBLIC
        spdlog::spdlog
        spdlog::yaml-cpp)
```
