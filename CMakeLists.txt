cmake_minimum_required(VERSION 3.31)

project(2d_geometry_constraint_solver)

set(CMAKE_CXX_STANDARD 20 CACHE STRING "C++ standard" FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)

message(STATUS "Setting CMAKE_DEVENV_PROFILE to $ENV{DEVENV_PROFILE}")
set(CMAKE_DEVENV_PROFILE $ENV{DEVENV_PROFILE})

configure_file(${CMAKE_CURRENT_LIST_DIR}/pyproject.toml.template ${CMAKE_CURRENT_LIST_DIR}/pyproject.toml)


find_package(spdlog REQUIRED)
find_package(stduuid REQUIRED)
find_package(argparse REQUIRED)
find_package(Eigen3 REQUIRED)
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

    nanobind_add_module(equation_system_solver_binding NOMINSIZE
                        src/python_bindings/equation_system_solver_binding.cpp
                        src/constraint_solver/constraint_equation_solver.cpp)
    target_include_directories(equation_system_solver_binding PRIVATE
                src/common
                ${spdlog_INCLUDE_DIRS}
                ${autodiff_INCLUDE_DIRS}
                ${Eigen3_INCLUDE_DIRS})

    target_compile_definitions(equation_system_solver_binding PRIVATE SPDLOG_USE_STD_FORMAT)
    target_link_libraries(equation_system_solver_binding PRIVATE spdlog::spdlog autodiff::autodiff Eigen3::Eigen)
endif()

######################## Optionally setting up nanobind for python binding generation ########################


# IMPORTANT NOTE: Currently this project outputs only one binary. There is no
# modularization in respect to the logical modules via archives or shared
# libraries. DO NOT expect source file to work by themselves without the
# correct linking

add_executable(${PROJECT_NAME}  src/main.cpp
                                src/utils/graph.cpp
                                src/constraint_solver/constraint_equation_solver.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ./include
            src/common
            ${spdlog_INCLUDE_DIRS}
            ${argparse_INCLUDE_DIRS}
            ${stduuid_INCLUDE_DIRS}
            ${autodiff_INCLUDE_DIRS}
            ${Eigen3_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME} PRIVATE SPDLOG_USE_STD_FORMAT LIBUUID_CPP20_OR_GREATER)
target_link_libraries(${PROJECT_NAME} PRIVATE   stduuid::stduuid argparse::argparse
                                                spdlog::spdlog autodiff::autodiff Eigen3::Eigen)

# Explanation why these flags are used:
# -ggdb: if debugging use it for full support of gdb
# -pedantic-errors: only support standard c++ features
# The rest turns on several useful warnings, the `Weffc++` flag turns on checks
# coming from Scott Meyers series
# TODO: make sure that the flags are correct for different compilers such as gcc and clang
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:-ggdb>
-pedantic-errors -Wall -Weffc++ -Wextra -Wconversion -Wsign-conversion)
