cmake_minimum_required(VERSION 4.1)

project(2d_geometry_constraint_solver)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)

message(STATUS "Setting CMAKE_DEVENV_PROFILE to $ENV{DEVENV_PROFILE}")
set(CMAKE_DEVENV_PROFILE $ENV{DEVENV_PROFILE})

find_package(spdlog REQUIRED)
find_package(ogdf REQUIRED)
find_package(stduuid REQUIRED)
find_package(argparse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Python COMPONENTS Development Interpreter REQUIRED)

######################## Downloading packages that are currently not available ########################

include(FetchContent)

set(AUTODIFF_VERSION "v1.1.2")
set(AUTODIFF_BUILD_TESTS OFF CACHE BOOL "Turn off autodiff test build" FORCE)
set(AUTODIFF_BUILD_EXAMPLES OFF CACHE BOOL "Turn off autodiff example build" FORCE)
set(AUTODIFF_BUILD_DOCS OFF CACHE BOOL "Turn off autodiff doc build" FORCE)
set(AUTODIFF_BUILD_PYTHON OFF CACHE BOOL "Turn off autodiff python bind generation" FORCE)

FetchContent_Declare(
    autodiff
    GIT_REPOSITORY https://github.com/autodiff/autodiff/
    GIT_TAG ${AUTODIFF_VERSION}
)

FetchContent_MakeAvailable(autodiff)

######################## Downloading packages that are currently not available ########################


######################## Optionally setting up nanobind for python binding generation ########################

option(GENERATE_PYTHON_BINDINGS "Enable nanobind python bindings generation" ON)

if(GENERATE_PYTHON_BINDINGS)
    if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    endif()

    # Detect the installed nanobind package and import it into CMake
    execute_process(
      COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)

    message(STATUS "The nanobind root is ${nanobind_ROOT}")
    find_package(nanobind CONFIG REQUIRED)

    # Constraint Graph Handle binding (opaque handle pattern for simplified API)
    nanobind_add_module(constraint_graph_handle_binding NOMINSIZE
                        src/python_bindings/constraint_graph_handle_binding.cpp
                        src/python_bindings/constraint_graph_handle.cpp
                        src/constraint_solver/geometric_constraint_system.cpp
                        src/constraint_solver/constraint_equation_solver.cpp
                        src/constraint_solver/elements.cpp
                        src/constraint_solver/constraints.cpp
                        src/constraint_solver/json_constraint_graph_loader.cpp
                        src/common/common/common_uuid.cpp)
    target_include_directories(constraint_graph_handle_binding PRIVATE
                src/common
                src/utils
                ${spdlog_INCLUDE_DIRS}
                ${ogdf_INCLUDE_DIRS}
                ${stduuid_INCLUDE_DIRS}
                ${autodiff_INCLUDE_DIRS}
                ${Eigen3_INCLUDE_DIRS})

    target_compile_definitions(constraint_graph_handle_binding PRIVATE SPDLOG_USE_STD_FORMAT LIBUUID_CPP20_OR_GREATER)
    target_link_libraries(constraint_graph_handle_binding PRIVATE spdlog::spdlog ogdf::ogdf stduuid::stduuid autodiff::autodiff Eigen3::Eigen nlohmann_json::nlohmann_json)
endif()

######################## Optionally setting up nanobind for python binding generation ########################


# IMPORTANT NOTE: Currently this project outputs only one binary. There is no
# modularization in respect to the logical modules via archives or shared
# libraries. DO NOT expect source file to work by themselves without the
# correct linking

add_executable(${PROJECT_NAME}  src/main.cpp
                                src/constraint_solver/constraint_equation_solver.cpp
                                src/constraint_solver/geometric_constraint_system.cpp
                                src/constraint_solver/elements.cpp
                                src/constraint_solver/constraints.cpp
                                src/constraint_solver/json_constraint_graph_loader.cpp
                                src/common/common/common_uuid.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ./include
            src/common
            src/utils
            ${spdlog_INCLUDE_DIRS}
            ${ogdf_INCLUDE_DIRS}
            ${argparse_INCLUDE_DIRS}
            ${stduuid_INCLUDE_DIRS}
            ${autodiff_INCLUDE_DIRS}
            ${Eigen3_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME} PRIVATE SPDLOG_USE_STD_FORMAT LIBUUID_CPP20_OR_GREATER)
target_link_libraries(${PROJECT_NAME} PRIVATE   stduuid::stduuid argparse::argparse ogdf::ogdf
                                                spdlog::spdlog autodiff::autodiff Eigen3::Eigen nlohmann_json::nlohmann_json)

# Explanation why these flags are used:
# -ggdb: if debugging use it for full support of gdb
# -pedantic-errors: only support standard c++ features
# The rest turns on several useful warnings, the `Weffc++` flag turns on checks
# coming from Scott Meyers series
# TODO: make sure that the flags are correct for different compilers such as gcc and clang
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:-ggdb>
-pedantic-errors -Wall -Wextra -Wconversion -Wsign-conversion)
