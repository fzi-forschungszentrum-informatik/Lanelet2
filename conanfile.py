import os
import sys
import xml.etree.ElementTree as ET
from conans import ConanFile, CMake, tools
from distutils.sysconfig import get_python_lib

find_mrt_cmake="""
set(mrt_cmake_modules_FOUND True)
include(${CMAKE_CURRENT_LIST_DIR}/mrt_cmake_modules-extras.cmake)
"""

cmake_lists="""
cmake_minimum_required(VERSION 3.5)
project(lanelet2)
if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW) # allows to do target_link_libraries on targets from subdirs
endif()
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
set(BoostPython_FOUND Yes)
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(SKIP_STD)

# hint to gtest
set(MRT_GTEST_DIR ${CMAKE_CURRENT_LIST_DIR})
enable_testing()

# declare dependencies
include_directories(lanelet2_core/include lanelet2_io/include lanelet2_projection/include lanelet2_traffic_rules/include
    lanelet2_routing/include lanelet2_validation/include)
add_subdirectory(lanelet2_core)
add_subdirectory(lanelet2_io)
add_subdirectory(lanelet2_projection)
add_subdirectory(lanelet2_traffic_rules)
add_subdirectory(lanelet2_routing)
add_subdirectory(lanelet2_validation)
add_subdirectory(lanelet2_examples)
add_subdirectory(lanelet2_python)
add_subdirectory(lanelet2_maps)
# declare dependencies
target_link_libraries(lanelet2_io PUBLIC lanelet2_core)
target_link_libraries(lanelet2_projection PUBLIC lanelet2_core)
target_link_libraries(lanelet2_traffic_rules PUBLIC lanelet2_core)
target_link_libraries(lanelet2_routing PUBLIC lanelet2_core lanelet2_traffic_rules)
target_link_libraries(lanelet2_validation PUBLIC lanelet2_core lanelet2_io lanelet2_routing lanelet2_traffic_rules lanelet2_projection)
target_link_libraries(lanelet2_examples_compiler_flags INTERFACE lanelet2_core lanelet2_io lanelet2_routing lanelet2_traffic_rules lanelet2_projection)
target_link_libraries(lanelet2_python_compiler_flags INTERFACE lanelet2_core lanelet2_io lanelet2_routing lanelet2_traffic_rules lanelet2_projection)
"""

def read_version():
    package = ET.parse('lanelet2_core/package.xml')
    return package.find('version').text

def get_py_version():
    return "{}.{}".format(sys.version_info.major, sys.version_info.minor)

def get_py_exec():
    return "python3" if sys.version_info.major == 3 else "python"

class Lanelet2Conan(ConanFile):
    name = "lanelet2"
    version = read_version()
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"
    license = "BSD"
    url = "https://github.com/fzi-forschungszentrum-informatik/lanelet2"
    description = "Map handling framework for automated driving"
    options = {"shared": [True, False], "fPIC": [True]}
    default_options = {"shared": False, "fPIC": True, "boost:python_version": get_py_version(), "boost:without_python": False, "python_dev_config:python": get_py_exec()}

    requires = ("python_dev_config/0.6@bincrafters/stable",
                "boost/1.69.0@conan/stable",
                "eigen/3.3.7@conan/stable",
                "geographiclib/1.49@bincrafters/stable",
                "pugixml/1.9@bincrafters/stable")

    exports_sources = "*"
    exports = "lanelet2_core/package.xml"

    proj_list = [
        'lanelet2_core',
        'lanelet2_io',
        'lanelet2_projection',
        'lanelet2_traffic_rules',
        'lanelet2_routing',
        'lanelet2_validation'
    ]

    def _configure_cmake(self):
        cmake = CMake(self)
        cmake.definitions["PYTHON_VERSION"] = get_py_version()
        cmake.definitions["MRT_CMAKE_ENV"] = "sh;env;PYTHONPATH=" + os.path.join(self.package_folder, self._pythonpath())
        cmake.configure()
        return cmake

    def _pythonpath(self):
        return os.path.relpath(get_python_lib(prefix=self.package_folder), start=self.package_folder)

    def source(self):
        if not os.path.exists("mrt_cmake_modules"):
            self.run("git clone https://github.com/KIT-MRT/mrt_cmake_modules.git")
        mrt_cmake_dir = os.path.join(os.getcwd(), "mrt_cmake_modules")
        with open("mrt_cmake_modules/cmake/mrt_cmake_modules-extras.cmake.in") as f:
            extras = f.read().replace("@DEVELSPACE@", "True").replace("@PROJECT_SOURCE_DIR@", mrt_cmake_dir).replace("@CMAKE_CURRENT_SOURCE_DIR@", mrt_cmake_dir)
        with open("mrt_cmake_modules-extras.cmake", "w") as f:
            f.write(extras)
        with open("Findmrt_cmake_modules.cmake", "w") as f:
            f.write(find_mrt_cmake)
        with open("CMakeLists.txt", "w") as f:
            f.write(cmake_lists)
        if not os.path.exists("googletest"):
            self.run("git clone https://github.com/google/googletest.git")

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()
        cmake.test() # not working as long as the pythonpath is not adapted first
        cmake.install()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = list(reversed(self.proj_list))
        self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, self._pythonpath()))
