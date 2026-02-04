from conan import ConanFile
from conan.tools.cmake import cmake_layout


class ProjectRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        self.requires("spdlog/[~1.15]")
        self.requires("eigen/[~5]")

    def configure(self):
        self.options["spdlog"].use_std_fmt = True
        self.options["spdlog"].shared = True

        self.options["eigen"].MPL2_only = True

    def layout(self):
        cmake_layout(self)
