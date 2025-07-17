from conan import ConanFile
from conan.tools.cmake import cmake_layout


class ProjectRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        self.requires("spdlog/[~1.15]")
        self.requires("stduuid/[~1.2]")
        self.requires("argparse/3.2")

    def configure(self):
        self.options["stduuid"].with_cxx20_span = True

        self.options["spdlog"].use_std_fmt = True
        self.options["spdlog"].shared = True  # Add this

    def layout(self):
        cmake_layout(self)
