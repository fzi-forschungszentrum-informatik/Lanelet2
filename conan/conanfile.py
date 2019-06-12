import os

from conans import ConanFile, CMake, tools

def read_version():
    with open('version.txt') as f:
        line = next(f)
    return line.rstrip()

class Lanelet2Conan(ConanFile):
    name = "lanelet2"
    version = read_version()
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"

    requires = ("boost/1.64.0@conan/stable",
                "eigen/3.3.7@conan/stable",
                "geographiclib/1.49@bincrafters/stable",
                "pugixml/1.9@bincrafters/stable")

    exports_sources = "../*"
    exports = "version.txt"

    proj_list = [
        'lanelet2_core',
        'lanelet2_io',
        'lanelet2_projection',
        'lanelet2_traffic_rules',
        'lanelet2_routing',
        'lanelet2_validation'
    ]

    def build(self):
        cmake = CMake(self)
        cmake.configure(source_folder='conan')
        cmake.build()

    def package(self):
        for proj in self.proj_list:
            self.copy('*.h', dst='include', src=os.path.join(proj, 'include'))

        self.copy("*.a", dst="lib", keep_path=False)

    def package_info(self):
        self.cpp_info.libs = list(reversed(self.proj_list))
