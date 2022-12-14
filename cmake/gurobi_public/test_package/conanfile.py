from conans import ConanFile, tools
from conan.tools.cmake import CMake, CMakeToolchain


class GurobiTestPackageConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps", "VirtualBuildEnv", "VirtualRunEnv"
    options = {}

    def _configure_cmake(self):
        cmake = CMake(self)
        cmake.configure()
        return cmake

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()

    def test(self):
        if not tools.cross_building(self):
            cmake = self._configure_cmake()
            cmake.test()

