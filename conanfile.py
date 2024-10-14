import os
import sys
import xml.etree.ElementTree as ET
from distutils.sysconfig import get_python_lib
from io import StringIO
from pathlib import Path

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import copy

find_mrt_cmake = """
set(mrt_cmake_modules_FOUND True)
include(${CMAKE_CURRENT_LIST_DIR}/mrt_cmake_modules-extras.cmake)
"""

cmake_lists = """
cmake_minimum_required(VERSION 3.5)
project(lanelet2)
if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW) # allows to do target_link_libraries on targets from subdirs
endif()
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
# mrt_cmake_modules inofficially support conan, but only conan1. So we have to set these:
set(CONAN_PACKAGE_NAME lanelet2)
macro(conan_define_targets)
    add_library(${PROJECT_NAME}_conan_deps INTERFACE)
    target_include_directories(${PROJECT_NAME}_conan_deps INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    set(CONAN_TARGETS ${PROJECT_NAME}_conan_deps)
endmacro()

# hint to gtest
set(GOOGLETEST_VERSION 1.0.0)
set(MRT_GTEST_DIR ${CMAKE_CURRENT_LIST_DIR})
enable_testing()

# find thirdparty
find_package(mrt_cmake_modules REQUIRED)
find_package(Boost REQUIRED)
find_package(BoostPython REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pugixml REQUIRED)
add_library(conan INTERFACE IMPORTED)
set_target_properties(
    conan
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BoostPython_INCLUDE_DIRS}")
target_link_libraries(conan INTERFACE ${BoostPython_LIBRARIES} boost::boost pugixml::pugixml GeographicLib::GeographicLib Eigen3::Eigen)

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
add_subdirectory(lanelet2_matching)
# declare dependencies
target_link_libraries(lanelet2_core PUBLIC conan)
target_link_libraries(lanelet2_io PUBLIC lanelet2_core)
target_link_libraries(lanelet2_projection PUBLIC lanelet2_core)
target_link_libraries(lanelet2_traffic_rules PUBLIC lanelet2_core)
target_link_libraries(lanelet2_routing PUBLIC lanelet2_core lanelet2_traffic_rules)
target_link_libraries(lanelet2_matching PUBLIC lanelet2_core lanelet2_traffic_rules lanelet2_io lanelet2_projection lanelet2_maps)
target_link_libraries(lanelet2_validation PUBLIC lanelet2_core lanelet2_io lanelet2_routing lanelet2_traffic_rules lanelet2_projection)
target_link_libraries(lanelet2_examples_compiler_flags INTERFACE lanelet2_core lanelet2_io lanelet2_routing lanelet2_traffic_rules lanelet2_projection lanelet2_matching)
target_link_libraries(lanelet2_python_compiler_flags INTERFACE lanelet2_core lanelet2_io lanelet2_routing lanelet2_traffic_rules lanelet2_projection lanelet2_matching)
"""


def read_version():
    package = ET.parse("lanelet2_core/package.xml")
    return package.find("version").text


def get_py_version():
    return "{}.{}".format(sys.version_info.major, sys.version_info.minor)


class Lanelet2Conan(ConanFile):
    name = "lanelet2"
    version = read_version()
    settings = "os", "compiler", "build_type", "arch"
    license = "BSD"
    url = "https://github.com/fzi-forschungszentrum-informatik/lanelet2"
    description = "Map handling framework for automated driving"
    options = {
        "shared": [True, False],
        "fPIC": [True],
        "python": ["ANY"],
        "build_wheel": [True, False],
        "platform": ["ANY"],
    }
    generators = "CMakeDeps", "VirtualRunEnv"
    default_options = {
        "shared": True,
        "fPIC": True,
        "build_wheel": False,
        "python": sys.executable,
        "platform": "manylinux_2_31_x86_64",
        "boost/*:shared": True,
        "boost/*:python_version": get_py_version(),
        "boost/*:without_python": False,
    }

    virtualrunenv = True

    requires = (
        "boost/1.81.0" if sys.version_info.minor > 9 else "boost/1.75.0",
        "eigen/3.4.0",
        "geographiclib/1.52",
        "pugixml/1.13",
    )

    exports_sources = "*"
    exports = "lanelet2_core/package.xml"

    proj_list = [
        "lanelet2_core",
        "lanelet2_io",
        "lanelet2_matching",
        "lanelet2_projection",
        "lanelet2_traffic_rules",
        "lanelet2_routing",
        "lanelet2_validation",
    ]

    def layout(self):
        cmake_layout(self)

    def generate(self):
        # This generates "conan_toolchain.cmake" in self.generators_folder
        tc = CMakeToolchain(self)
        output = StringIO()
        py_exec = str(self.options.python)
        output = StringIO()
        self.run(
            "{0} -c \"from sys import *; print('%d.%d' % (version_info[0],version_info[1]))\"".format(
                py_exec
            ),
            stdout=output,
        )
        py_version = output.getvalue().strip()
        tc.variables["PYTHON_VERSION"] = py_version
        tc.variables["PYTHON_EXECUTABLE"] = py_exec
        tc.variables["MRT_CMAKE_ENV"] = "sh env PYTHONPATH=" + py_exec
        tc.generate()
        self._set_env(self.runenv)

    def _configure_cmake(self):
        cmake = CMake(self)
        mrt_env = "sh;env;PYTHONPATH=" + os.path.join(
            self.package_folder, self._pythonpath()
        )
        cmake.configure(
            variables={"PYTHON_VERSION": get_py_version(), "MRT_CMAKE_ENV": mrt_env}
        )
        return cmake

    def _pythonpath(self):
        return os.path.relpath(
            get_python_lib(prefix=self.package_folder), start=self.package_folder
        )

    def source(self):
        if not os.path.exists("mrt_cmake_modules"):
            self.run("git clone https://github.com/KIT-MRT/mrt_cmake_modules.git")
        mrt_cmake_dir = os.path.join(os.getcwd(), "mrt_cmake_modules")
        with open("mrt_cmake_modules/cmake/mrt_cmake_modules-extras.cmake.in") as f:
            extras = (
                f.read()
                .replace("@DEVELSPACE@", "True")
                .replace("@PROJECT_SOURCE_DIR@", mrt_cmake_dir)
                .replace("@CMAKE_CURRENT_SOURCE_DIR@", mrt_cmake_dir)
            )
        with open("mrt_cmake_modules-extras.cmake", "w") as f:
            f.write(extras)
        with open("Findmrt_cmake_modules.cmake", "w") as f:
            f.write(find_mrt_cmake)
        with open("CMakeLists.txt", "w") as f:
            f.write(cmake_lists)
        if not os.path.exists("googletest"):
            self.run("git clone https://github.com/google/googletest.git")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        cmake.test(
            cli_args=["-v"]
        )  # not working as long as the pythonpath is not adapted first
        if self.options.build_wheel:
            with (
                Path(self.source_folder) / "lanelet2_python" / "setup.py.template"
            ).open() as f:
                setup_template = f.read()
            setup_py = setup_template.replace("{{ version }}", self.version)
            with (Path(self.build_folder) / "setup.py").open("w") as f:
                f.write(setup_py)

    def package(self):
        cmake = CMake(self)
        cmake.install()
        if self.options.build_wheel:
            whl_tmp = os.path.join(self.package_folder, "wheel-incomplete")
            whl_out = os.path.join(self.package_folder, "wheel")
            copy(self, "setup.py", self.build_folder, whl_tmp)
            copy(
                self,
                "*",
                os.path.join(self.package_folder, self._pythonpath()),
                whl_tmp,
            )
            self.run(f"pip wheel -w {whl_tmp} {whl_tmp}")
            self.run(
                f"export LD_LIBRARY_PATH={os.path.join(self.package_folder, self.cpp_info.libdir)}:$LD_LIBRARY_PATH && auditwheel repair -w {whl_out} --plat {self.options.platform} {whl_tmp}/*.whl",
                scope="conanrun",
            )

    def _set_env(self, env_info):
        if not self.package_folder:
            return
        execs = ("lanelet2_examples", "lanelet2_validation", "lanelet2_python")
        env_info.define_path(
            "PYTHONPATH", os.path.join(self.package_folder, self._pythonpath())
        )
        for libname in execs:
            env_info.append_path(
                "PATH", os.path.join(self.package_folder, "lib", libname)
            )

    def package_info(self):
        self.cpp_info.libs = list(reversed(self.proj_list))
        self._set_env(self.runenv_info)
